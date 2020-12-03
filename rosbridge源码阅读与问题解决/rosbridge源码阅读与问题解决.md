# rosbridge源码阅读与问题解决



## 0.碰到的问题

### 1）大数据发送不全；

### 2）发送的数据出现交叉的情况（大数据中会插入小数据，导致解析错误）。

## 1.安装方法

具体方法参考官网教程，这里假定已经跑通rosbridge，已经可以获得如下log：

```shell
//...略去部分
registered capabilities (classes):
 - rosbridge_library.capabilities.call_service.CallService
 - rosbridge_library.capabilities.advertise.Advertise
 - rosbridge_library.capabilities.publish.Publish
 - rosbridge_library.capabilities.subscribe.Subscribe
 - <class 'rosbridge_library.capabilities.defragmentation.Defragment'>
 - rosbridge_library.capabilities.advertise_service.AdvertiseService
 - rosbridge_library.capabilities.service_response.ServiceResponse
 - rosbridge_library.capabilities.unadvertise_service.UnadvertiseService
trying to start rosbridge TCP server..

[INFO] [WallTime: 1331933370.827223] Rosbridge TCP server started on port 9090
[INFO] [WallTime: 1331933371.376914] [Client 0] connected. 1 client total.
[INFO] [WallTime: 1331933371.953258] [Client 0] Subscribed to /pose
[INFO] [WallTime: 1331933371.962872] [Client 0] Subscribed to /map
```



## 2.源码阅读

### 1）先从rosbridge_tcp.launch开始

port：端口，默认9090；
host：host ip，默认为空，0.0.0.0会绑定本机所有ip；

incoming_buffer：接收buffer，默认65536；
socket_timeout：接收超时时间，默认10，10s内接收不到消息会打印debug，可以不用理会；
retry_startup_delay：尝试重启延迟，默认5，启动socket失败后5s重启；

fragment_timeout：片段超时时间，默认600，目前没发现有啥用；
delay_between_messages：消息之间的延迟，默认0；
max_message_size：消息的最大大小，默认None。

### 2）rosbridge_tcp
```python
            #...略去部分，前面各种初始化RosbridgeTcpSocket
    
    		# Server host is a tuple ('host', port)
            # empty string for host makes server listen on all available interfaces
            SocketServer.ThreadingTCPServer.allow_reuse_address = True
            server = SocketServer.ThreadingTCPServer((host, port), RosbridgeTcpSocket)
            on_shutdown(partial(shutdown_hook, server))

            loginfo("Rosbridge TCP server started on port %d", port)

            server.serve_forever()
            loaded = True
        except Exception, e:
            time.sleep(retry_startup_delay)
    print "server loaded"
```

这里使用了SocketServer，RosbridgeTcpSocket继承自SocketServer.BaseRequestHandler类，在每次有新的client连接时都会创建一个新的线程，并依次调用RosbridgeTcpSocket的init()、setup()、handle()、finish()方法。

### 3）tcp_handler.py

该文件中定义了class RosbridgeTcpSocket(SocketServer.BaseRequestHandler)：

- \__init__(self, request, client_address, server)：定义在**父类**中，初始化socket信息，如`self.request`，该变量最后一行self.request.send(message+"\n")有使用；
- setup()： 在handle()之前执行，用户自定义配置，定义了rosbridge中**最重要的变量**`self.protocol = RosbridgeProtocol(cls.client_id_seed, parameters=parameters)`，每次有新的client连接时都会定义一个`self.protocol`，传入的参数为client的序号和之前传入的部分参数，重新定义了该类outgoing方法`self.protocol.outgoing = self.send_message`；
- handle()： 定义了如何处理每一个连接，收到数据后处理数据`self.protocol.incoming(data.strip(''))`；
- finish()：在handle()之后执行；
- send_message(self, message=None)：使用`self.request.send(message+"\n")`发送数据，send()方法不一定发送所有的数据，返回值是已发送的数据，可以看出大的数据发送不全很可能问题就出在这，可以写个循环判断发送，或者简单地改成sendall()。



```python
import rospy
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol

import SocketServer

class RosbridgeTcpSocket(SocketServer.BaseRequestHandler):
    """
    TCP Socket server for rosbridge
    """

    busy = False
    queue = []
    client_id_seed = 0
    clients_connected = 0

    # list of parameters
    incoming_buffer = 65536                 # bytes
    socket_timeout = 10                     # seconds
    # The following are passed on to RosbridgeProtocol
    # defragmentation.py:
    fragment_timeout = 600                  # seconds
    # protocol.py:
    delay_between_messages = 0              # seconds
    max_message_size = None                 # bytes

    def setup(self):
        cls = self.__class__
        parameters = {
            "fragment_timeout": cls.fragment_timeout,
            "delay_between_messages": cls.delay_between_messages,
            "max_message_size": cls.max_message_size
        }

        try:
            self.protocol = RosbridgeProtocol(cls.client_id_seed, parameters=parameters)
            self.protocol.outgoing = self.send_message
            cls.client_id_seed += 1
            cls.clients_connected += 1
            self.protocol.log("info", "connected. " + str(cls.clients_connected) + " client total.")
        except Exception as exc:
            rospy.logerr("Unable to accept incoming connection.  Reason: %s", str(exc))

    def handle(self):
        """
        Listen for TCP messages
        """
        cls = self.__class__
        self.request.settimeout(cls.socket_timeout)
        while 1:
            try:
              data = self.request.recv(cls.incoming_buffer)
              # Exit on empty string
              if data.strip() == '':
                  break
              elif len(data.strip()) > 0:
                  self.protocol.incoming(data.strip(''))	#当有数据来后，会调用RosbridgeProtocol类的incoming()方法
              else:
                  pass
            except Exception, e:
                pass
                self.protocol.log("debug", "socket connection timed out! (ignore warning if client is only listening..)")

    def finish(self):
        #...略去部分

    def send_message(self, message=None):
        """
        Callback from rosbridge
        """
        self.request.send(message+"\n")

```

已验证当数据为135940时，实际发送只有46336，除了前面说改为`sendall()`外，还可以修改`max_message_size`，具体方法可以视情况而定。

```shell
---------start-----------
135940
46336
---------end-----------
```

### 4）rosbridge_protocol.py

RosbridgeProtocol类继承自Protocol类。

`self.add_capability(capability_class)`

这里遍历rosbridge_capabilities将每个capability添加到RosbridgeProtocol类中。add_capability()是在父类Protocol类中定义的。

```python
def add_capability(self, capability_class):
    self.capabilities.append(capability_class(self))
```

每个capability都继承自Capability类，Capability类中又保存了Protocol类`self.protocol = protocol`。在append(capability_class(self))的过程中，会实例化每个capability，实例化capability的过程中又会注册（register_operation）该capability的operation到Protocol类中。

```python
from rosbridge_library.protocol import Protocol
from rosbridge_library.capabilities.call_service import CallService
from rosbridge_library.capabilities.advertise import Advertise
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.capabilities.subscribe import Subscribe
# imports for defragmentation
from rosbridge_library.capabilities.defragmentation import Defragment
# imports for external service_server
from rosbridge_library.capabilities.advertise_service import AdvertiseService
from rosbridge_library.capabilities.service_response import ServiceResponse
from rosbridge_library.capabilities.unadvertise_service import UnadvertiseService



class RosbridgeProtocol(Protocol):
    """ Adds the handlers for the rosbridge opcodes """
    rosbridge_capabilities = [CallService, Advertise, Publish, Subscribe, Defragment, AdvertiseService, ServiceResponse, UnadvertiseService]

    print "registered capabilities (classes):"
    for cap in rosbridge_capabilities:
        print " -", str(cap)

    parameters = None

    def __init__(self, client_id, parameters = None):
        self.parameters = parameters
        Protocol.__init__(self, client_id)
        for capability_class in self.rosbridge_capabilities:
            self.add_capability(capability_class)
```

### 5）protocol.py

这里有**最重要**的类Protocol。

初始化完成后，每个capability的操作都会注册到self.operations{}，当有数据来时，RosbridgeTcpSocket类的handle()方法中会调用Protocol类的incoming()方法，这里会调用`self.operations[op](msg)`，后续我们假设调用的是subscribe操作。

```python
#删去了对本文不重要的内容
class Protocol:
    def __init__(self, client_id):
        self.client_id = client_id
        self.capabilities = []	#将会保存8个capability，如Subscribe
        self.operations = {}	#每个capability实例化过程中会注册操作到这里，如subscribe、unsubscribe

        if self.parameters:
            self.fragment_size = self.parameters["max_message_size"]
            self.delay_between_messages = self.parameters["delay_between_messages"]

    def incoming(self, message_string=""):
        # now try to pass message to according operation
        try:
            self.operations[op](msg)
        except Exception as exc:
            self.log("error", "%s: %s" % (op, str(exc)), mid)

    def outgoing(self, message):	#RosbridgeTcpSocket类的setup()方法中重定义该方法
        pass

    def send(self, message, cid=None):
        #...略去部分，使用serialize()得到序列化后的数据serialized
        self.outgoing(serialized)
        time.sleep(self.delay_between_messages)

    def finish(self):

    def serialize(self, msg, cid=None):	#序列化数据

    def deserialize(self, msg, cid=None):	#反序列化数据

    def register_operation(self, opcode, handler):	#每个capability使用该方法注册操作
        self.operations[opcode] = handler

    def unregister_operation(self, opcode):	#注销操作
        if opcode in self.operations:
            del self.operations[opcode]

    def add_capability(self, capability_class):	#将capability添加到capabilities[]中
        self.capabilities.append(capability_class(self))

    def log(self, level, message, lid=None):

```



### 6）subscribe.py

上文调用了Subscribe类中的subscribe()方法，改方法主要分为两步：1.如果self._subscriptions{}中没有该topic则创建该topic的Subscription类的实例；2.调用该topic对应Subscription类的subscribe()方法。Subscription类的subscribe()方法又调用了`manager.subscribe(self.client_id, self.topic, self.on_msg, msg_type)`，这里将在 *8）subscribers.py* 中讲解。这里一个Subscribe类可以有多个Subscription类，就是说一个client可以获取多个topic的数据。Subscribe类中有个publish()方法，我们在这里加个锁就可以解决问题2（数据交叉问题）。

```python
#删去了对本文不重要的内容
class Subscription():

    def __init__(self, client_id, topic, publish):
        self.client_id = client_id
        self.topic = topic
        self.publish = publish

        self.clients = {}

        self.handler = MessageHandler(None, self._publish)
        self.handler_lock = Lock()
        self.update_params()

    def unregister(self):
        """ Unsubscribes this subscription and cleans up resources """
        manager.unsubscribe(self.client_id, self.topic)
        with self.handler_lock:
            self.handler.finish()
        self.clients.clear()

    def subscribe(self, sid=None, msg_type=None, throttle_rate=0,
                  queue_length=0, fragment_size=None, compression="none"):
        client_details = {
            "throttle_rate": throttle_rate,
            "queue_length": queue_length,
            "fragment_size": fragment_size,
            "compression": compression
        }

        self.clients[sid] = client_details

        self.update_params()

        # Subscribe with the manager. This will propagate any exceptions
        manager.subscribe(self.client_id, self.topic, self.on_msg, msg_type)

    def unsubscribe(self, sid=None):

    def is_empty(self):

    def _publish(self, message):
        self.publish(message, self.fragment_size, self.compression)

    def on_msg(self, msg):
        with self.handler_lock:
            self.handler.handle_message(msg)

    def update_params(self):

class Subscribe(Capability):

    subscribe_msg_fields = [(True, "topic", (str, unicode)), (False, "type", (str, unicode)),
        (False, "throttle_rate", int), (False, "fragment_size", int),
        (False, "queue_length", int), (False, "compression", (str, unicode))]
    unsubscribe_msg_fields = [(True, "topic", (str, unicode))]

    def __init__(self, protocol):
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("subscribe", self.subscribe)	#注册了subscribe操作
        protocol.register_operation("unsubscribe", self.unsubscribe)	#注册了unsubscribe操作

        self._subscriptions = {}	#每个不同的topic都保存了一个Subscription类的实例

    def subscribe(self, msg):
        # Pull out the ID
        sid = msg.get("id", None)
        
        # Check the args
        self.basic_type_check(msg, self.subscribe_msg_fields)

        # Make the subscription
        topic = msg["topic"]
        if not topic in self._subscriptions:	#self._subscriptions{}中没有该topic则创建该topic的subscription
            client_id = self.protocol.client_id
            cb = partial(self.publish, topic)
            self._subscriptions[topic] = Subscription(client_id, topic, cb)

        # Register the subscriber
        subscribe_args = {
          "sid": sid,
          "msg_type": msg.get("type", None),
          "throttle_rate": msg.get("throttle_rate", 0),
          "fragment_size": msg.get("fragment_size", None),
          "queue_length": msg.get("queue_length", 0),
          "compression": msg.get("compression", "none")
        }
        self._subscriptions[topic].subscribe(**subscribe_args)	#

        self.protocol.log("info", "Subscribed to %s" % topic)

    def unsubscribe(self, msg):

    def publish(self, topic, message, fragment_size=None, compression="none"):
        # TODO: fragmentation, proper ids
        outgoing_msg = {"op": "publish", "topic": topic, "msg": message}
        if compression=="png":
            outgoing_msg_dumped = dumps(outgoing_msg)
            outgoing_msg = {"op": "png", "data": encode(outgoing_msg_dumped)}
        self.protocol.send(outgoing_msg)

    def finish(self):

```



### 7）capability.py

```python
#删去了对本文不重要的内容
class Capability:
    def __init__(self, protocol):
        self.protocol = protocol	#这里可以看到每个capability都保存了与其对应的Protocol类

    def handle_message(self, message):
    def finish(self):
    def basic_type_check(self, msg, types_info):
```

### 8）subscribers.py

文件中定义了`manager = SubscriberManager()`，所以只会有一个SubscriberManager类的实例manager 。上文中调用了SubscriberManager类的subscribe()方法，该方法分两步：1.没有该topic时，则增加该topic对应的MultiSubscriber`self._subscribers[topic] = MultiSubscriber(topic, msg_type)`；2.调用该topic对应MultiSubscriber类的subscribe()方法`self._subscribers[topic].subscribe(client_id, callback)`。

MultiSubscriber类初始化时，就会向rospy中订阅该topic，回调函数为callback()，当ros系统中有消息来时，会调用这个回调函数，在这个回调函数中又会调用每个client向rosbridge订阅的相同topic的回调函数。MultiSubscriber类的subscribe()方法中，将新client的callback加入`self.subscriptions[client_id] = callback`。

这里实现了多个client订阅了同个topic，rosbridge也只会向ros订阅一个topic。

```python
#删去了对本文不重要的内容
from rospy import Subscriber, logerr

""" Manages and interfaces with ROS Subscriber objects.  A single subscriber
is shared between multiple clients
"""

class MultiSubscriber():
    """ Handles multiple clients for a single subscriber.

    Converts msgs to JSON before handing them to callbacks.  Due to subscriber
    callbacks being called in separate threads, must lock whenever modifying
    or accessing the subscribed clients. """

    def __init__(self, topic, msg_type=None):
        # Create the subscriber and associated member variables
        self.subscriptions = {}	#存放不同client的相同topic的callback
        self.lock = Lock()
        self.topic = topic
        self.msg_class = msg_class
        self.subscriber = Subscriber(topic, msg_class, self.callback)

    def unregister(self):

    def verify_type(self, msg_type):

    def subscribe(self, client_id, callback):
        with self.lock:
            self.subscriptions[client_id] = callback	#将新client的callback加入
            # If the topic is latched, add_callback will immediately invoke
            # the given callback.
            #如果第一次订阅，下述操作将马上获得第一个数据
            self.subscriber.impl.add_callback(self.callback, [callback])
            self.subscriber.impl.remove_callback(self.callback, [callback])

    def unsubscribe(self, client_id):

    def has_subscribers(self):
        """ Return true if there are subscribers """
        with self.lock:
            ret = len(self.subscriptions) != 0
            return ret

    def callback(self, msg, callbacks=None):	#如果callbacks为None则全部self.subscriptions中的callback都发一遍，否则只发callbacks中的callback
        # Get the callbacks to call
        if not callbacks:
            with self.lock:
                callbacks = self.subscriptions.values()

        # Pass the JSON to each of the callbacks
        for callback in callbacks:
            try:
                callback(json)
            except Exception as exc:
                # Do nothing if one particular callback fails except log it
                logerr("Exception calling subscribe callback: %s", exc)
                pass


class SubscriberManager():
    """
    Keeps track of client subscriptions
    """

    def __init__(self):
        self._subscribers = {}	#每个topic都会有一个MultiSubscriber类的实例

    def subscribe(self, client_id, topic, callback, msg_type=None):
        if not topic in self._subscribers:
            self._subscribers[topic] = MultiSubscriber(topic, msg_type)	#没有该topic时，则增加该topic对应的MultiSubscriber

        if msg_type is not None:
            self._subscribers[topic].verify_type(msg_type)

        self._subscribers[topic].subscribe(client_id, callback)	#调用该topic对应MultiSubscriber类的subscribe()方法

    def unsubscribe(self, client_id, topic):

manager = SubscriberManager()
```

## 3.总结

- rosbridge_tcp：启动socket；

- tcp_handler.py：RosbridgeTcpSocket类，传入message（`self.protocol.incoming(data.strip(''))`），重定义发送；
- rosbridge_protocol.py：定义了RosbridgeProtocol类，父类为Protocol；
- protocol.py：定义了Protocol类，保存capability，capability将其操作注册到Protocol类中；
- subscribe.py：实现了一个capability，一个client对应一个Subscribe类的实例，Subscribe类中根据topic的不同又保存了多个Subscription；
- subscribers.py：SubscriberManager类是rosbridge中，订阅topic总的管理类，SubscriberManager类中根据topic的不同又保存了多个MultiSubscriber，当ros系统中有新消息时，会调用MultiSubscriber类中的回调函数，该回调函数又会调用多个client的回调函数。当数据较大时，多个线程向一个socket发送消息会导致数据交叉。

---------

参考：

官方wiki：

http://wiki.ros.org/rosbridge_suite

协议说明：
 https://github.com/RobotWebTools/rosbridge_suite/blob/groovy-devel/ROSBRIDGE_PROTOCOL.md

源码介绍：

https://blog.csdn.net/XCCCCZ/article/details/86773609

socketserver介绍:

https://www.cnblogs.com/progor/p/8617042.html

https://www.cnblogs.com/lixiaoliuer/p/6739528.html