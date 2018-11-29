# obst_avoid18
This contains multiple ROS packages for obstacle avoidance and vualization thereof in Duckietown.

## Prerequisites
- Desktop-Full installation of ROS
- [duckietown-world](https://github.com/duckietown/duckietown-world)

## Installing
From the `src` directory of your ROS Workspace, run
```
$ git clone https://github.com/lgulich/Software/
```
From your workspace directory, run
```
$ git submodule init
$ git submodule update
$ catkin build 
```
Run `catkin_make` instead if you don't use `python-catkin-tools`.

Next, source your workspace using
```
$ source devel/setup.bash
```

## Running the obstacle avoidance and visualization
Run the visualization of the `robotarium1` map, which is currently the default 
by using
```
$ roslaunch obst_avoid obst_avoid_withviz.launch
```

You can specify different map names to be loaded according to the maps in 
`duckietown-world`. For example,
```
$ roslaunch obst_avoid obst_avoid_withviz.launch map_name:="small_loop"
```

Also different rviz configuration files can be specified.
```
$ roslaunch duckietown_visualization publish_map.launch map_name:="small_loop" rviz_config:="path/to/myconfig.rviz"
```


## Further Information
This package largely depends on
- [duckietown-world](https://github.com/duckietown/duckietown-world) (required as dependency) 
- [duckietown-fplan](https://github.com/duckietown/duckietown-fplan) (is a submodule)
- [duckietown-visualization](https://github.com/surirohit/duckietown-visualization) (is a submodule)

See their respective documentations for further insights.

