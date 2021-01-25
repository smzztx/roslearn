http://ceres-solver.org/
计算视觉SLAM十四讲 ch6
https://blog.csdn.net/qq_42138662/article/details/109289129
g2o catkin_make error
```sh
CMakeFiles/g2o_curve_fitting_node.dir/src/g2o_curve_fitting.cpp.o: In function `main':
g2o_curve_fitting.cpp:(.text.startup+0x43e): undefined reference to `g2o::OptimizableGraph::addVertex(g2o::HyperGraph::Vertex*, g2o::OptimizableGraph::Data*)'
collect2: error: ld returned 1 exit status
make[2]: *** [/home/txcom-ubuntu64/catkin_ws/devel/lib/curve_fitting/g2o_curve_fitting_node] Error 1
make[1]: *** [roslearn/curve_fitting/CMakeFiles/g2o_curve_fitting_node.dir/all] Error 2
make[1]: *** Waiting for unfinished jobs....
[ 81%] Built target dual_serial_restart
make: *** [all] Error 2
Invoking "make -j2 -l2" failed
```