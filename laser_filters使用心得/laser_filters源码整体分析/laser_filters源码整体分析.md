`laser_filters` 也用了挺久，感觉还是蛮好用的，最近看了下源码（ `laser_filters`包为kinetic-devel， `filters`包为hydro-devel），在这里写个流水账。
  
## laser_filters
总的来说 `laser_filters`包 调用了 `filters`包，下面来详细阅读下代码。  
```xml
<launch>
<node pkg="laser_filters" type="scan_to_scan_filter_chain" output="screen" name="laser_filter">
      <rosparam command="load" file="$(find laser_filters)/examples/speckle_filter_example.yaml" />
</node>
</launch>
```
```yaml
scan_filter_chain:
- name: speckle_filter
  type: laser_filters/LaserScanSpeckleFilter
  params:
    # Select which filter type to use.
    # 0: Range based filtering (distance between consecutive points)
    # 1: Euclidean filtering based on radius outlier search
    filter_type: 0

    # Only ranges smaller than this range are taken into account
    max_range: 2.0

    # filter_type[0] (Distance): max distance between consecutive points
    # filter_type[1] (RadiusOutlier): max distance between points
    max_range_difference: 0.1

    # filter_type[0] (Distance): Number of consecutive ranges that will be tested for max_distance
    # filter_type[1] (RadiusOutlier): Minimum number of neighbors
    filter_window: 2
```
先看launch文件，启动了 `scan_to_scan_filter_chain` 并导入了 yaml文件。 `scan_to_scan_filter_chain` 可执行文件是由 [scan_to_scan_filter_chain.cpp](https://github.com/ros-perception/laser_filters/blob/ros2/src/scan_to_scan_filter_chain.cpp) 生成的，scan_to_scan_filter_chain.cpp 中使用了 class scan_to_scan_filter_chain ， class scan_to_scan_filter_chain 中又使用了 `filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;` 。在类初始化时使用了 `filter_chain_.configure("scan_filter_chain", private_nh_);` 配置 `filter_chain_` ，在激光数据到来时会使用 `filter_chain_.update(*msg_in, msg_)` 滤波。  
下面再继续看 `filters`包 中的 `filter_chain` 。
  
## filters
`filters`包 使用了[ROS的插件机制](http://wiki.ros.org/pluginlib)，这个插件机制实现还是挺神奇的，以后有空可以详细了解下，开始阅读代码[filter_chain.h](https://github.com/ros/filters/blob/lunar-devel/include/filters/filter_chain.h)。 
```cpp
bool configure(std::string param_name, ros::NodeHandle node = ros::NodeHandle())
{
  XmlRpc::XmlRpcValue config;
  if(node.getParam(param_name + "/filter_chain", config))
  {
    std::string resolved_name = node.resolveName(param_name).c_str();
    ROS_WARN("Filter chains no longer check implicit nested 'filter_chain' parameter.  This node is configured to look directly at '%s'.  Please move your chain description from '%s/filter_chain' to '%s'", resolved_name.c_str(), resolved_name.c_str(), resolved_name.c_str());
  }
  else if(!node.getParam(param_name, config))
  {
    ROS_DEBUG("Could not load the filter chain configuration from parameter %s, are you sure it was pushed to the parameter server? Assuming that you meant to leave it empty.", param_name.c_str());
    configured_ = true;
    return true;
  }
  return this->configure(config, node.getNamespace());
}
```
`filter_chain_.configure("scan_filter_chain", private_nh_);` 传参为"scan_filter_chain"，再根据上文中的yaml文件，所以在 `else if(!node.getParam(param_name, config))` 这里可以获取到参数服务器上的数据，最后又执行了这个 `this->configure(config, node.getNamespace())`。  
```cpp
bool configure(XmlRpc::XmlRpcValue& config, const std::string& filter_ns)
{
  ......//省略了检查数据的部分，前面检查数据结构，后面检查name不能重复，然后把type存起来

  bool result = true;
  for (int i = 0; i < config.size(); ++i)
  {
    boost::shared_ptr<filters::FilterBase<T> > p(loader_.createInstance(config[i]["type"]));
    if (p.get() == NULL)
      return false;
    result = result &&  p.get()->configure(config[i]);    
    reference_pointers_.push_back(p);
    std::string type = config[i]["type"];
    std::string name = config[i]["name"];
    ROS_DEBUG("%s: Configured %s:%s filter at %p\n", filter_ns.c_str(), type.c_str(),
              name.c_str(),  p.get());
  }
  
  if (result == true)
  {
    configured_ = true;
  }
  return result;
};
```
这里就是将所有的type（滤波插件）实例化后，再配置一遍，params也都用上了。  
还用到一个 `XmlRpc::XmlRpcValue` ，这个是跟ros的底层通信有关，参考中列了一些链接。  
最后 `bool update(const T& data_in, T& data_out)` 就是根据配置依次滤波。
还有个需要注意下，只用到了 类FilterBase（所有的滤波插件都继承了该类），类MultiChannelFilterBase 和 类MultiChannelFilterChain 都没用到，不要看错了。（不要问为什么会有这个提醒o(╥﹏╥)o）  
  
---
## 参考
[laser_filters wiki](http://wiki.ros.org/laser_filters)  
[filters wiki](http://wiki.ros.org/filters)   
[ros_comm](https://github.com/ros/ros_comm)  
[一文搞懂XML、Json、Protobuf序列化协议](https://blog.csdn.net/Jiangtagong/article/details/119656782)  
[探索ROS中的XML](https://www.dazhuanlan.com/heraclitus/topics/1235795)