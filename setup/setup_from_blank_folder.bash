#!/bin/bash

# copy this file in a folder in which you want all the demo stuff for mplan
# use this command to do the setup: ". ./setup_from_blank_folder.bash"

# prerequesits
# - ssh key for github
# - pip
# - virtualenv
# - catkin_build


# virtualenv
virtualenv -p python2.7 venv

source venv/bin/activate

# duckietown world
git clone git@github.com:duckietown/duckietown-world.git

cd duckietown-world

pip install -r requirements.txt

python setup.py develop --no-deps

cd ..

# our package
mkdir -p dt_catkin_ws/src

cd dt_catkin_ws

catkin init

cd src

git clone https://github.com/duckietown/duckietown-mplan.git

cd duckietown-mplan

# current fix, as our proper branch is not yet the master
git checkout restart

pip install -r requirements.txt

git submodule init

git submodule update

cd duckietown-fplan

git submodule init

git submodule update

cd ../../..

catkin build

source devel/setup.bash

# launch default demo
roslaunch obst_avoid obst_avoid_withviz_demo.launch demo_num:=5
