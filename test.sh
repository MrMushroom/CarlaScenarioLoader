source ~/catkin_ws/devel/setup.sh
source ~/temp-ros-geometry/devel/setup.sh

python3 scenario_loader.py Carla 172.20.1.70 2000 2.0 OpenScenario scenarios/database/

# # get tf for python3
# mkdir catkin_ws
# cd catkin_ws
# mkdir src
# cd src
# git clone https://github.com/ros/geometry
# git clone https://github.com/ros/geometry2
# cd ..
# virtualenv -p /usr/bin/python3 venv
# source venv/bin/activate
# pip install catkin_pkg pyyaml empy rospkg numpy
# catkin_make
# source devel/setup.bash
