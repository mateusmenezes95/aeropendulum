# rqt_gui_aeropendulum
A GUI to control an Aeropendulum using ROS/Rosserial and Arduino board

# Instructions to run the GUI

Download this package in the source of catkin workspace or run 

```sh
cd catkin_ws/src/
git clone https://github.com/mateusmenezes95/rqt_gui_aeropendulum.git
```

This package uses special messages, so you need to download or clone [ros_tests](https://github.com/mateusmenezes95/ros_tests) package and put it in your catkin workspace

Now, run these commands in your terminal
```sh
cd catkin_ws/
catkin_make
source devel/setup.bash
``` 
