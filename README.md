# obst-avoid
This contains multiple ROS packages for obstacle avoidance and visualization thereof for Duckietown.
[![CircleCI](https://circleci.com/gh/duckietown/duckietown-mplan.svg?style=shield)](https://circleci.com/gh/duckietown/duckietown-mplan)

[![Coverage Status](https://coveralls.io/repos/github/duckietown/duckietown-mplan/badge.svg?branch=master)](https://coveralls.io/github/duckietown/duckietown-mplan?branch=master18)

## Prerequisites
- [Desktop-Full installation of ROS](http://wiki.ros.org/kinetic/Installation)
- [duckietown-world](https://github.com/duckietown/duckietown-world)

## Installing
Make sure you have the prerequisites installed. We recommend running the whole setup in a virtual environment using `python2.7`.

setup virtual environment
```
$ virtualenv -p python2.7 venv
```
activate the virtual environment
```
$ source venv/bin/activate
```


Clone this repo with the following command. Make sure you are inside the `src` folder of a catkin workspace. If you do not have a catkin workspace setup follow these [instructions](https://github.com/duckietown/duckietown-mplan/wiki/Setting-up-a-catkin-workspace).
```
$ git clone https://github.com/duckietown/duckietown-mplan.git
```

Enter the repo
```
$ cd Software/catkin_ws
```

Install the additional requirements using
```
$ pip install -r requirements.txt 
```

Load the submodules and build the workspace
```
$ git submodule init
$ git submodule update
$ catkin build 
```
Run `catkin_make` instead if you don't use `python-catkin-tools`.

Next, source your workspace using
```
$ source ../devel/setup.bash
```

## Running the obstacle avoidance and visualization
Run the visualization with
```
$ roslaunch obst_avoid obst_avoid_withviz.launch
```

You can specify different map names to be loaded according to the maps in 
`duckietown-world`. For example,
```
$ roslaunch obst_avoid obst_avoid_withviz.launch map_name:="small_loop"
```

If you prefer to launch the obstacle avoider without visualization run
```
$ roslaunch obst_avoid obst_avoid_noviz.launch
```


## Further Information
This package largely depends on
- [duckietown-world](https://github.com/duckietown/duckietown-world) (required as dependency) 
- [duckietown-fplan](https://github.com/duckietown/duckietown-fplan) (is a submodule)
- [duckietown-visualization](https://github.com/surirohit/duckietown-visualization) (is a submodule)

See their respective documentations for further insights and instructions for installation.

