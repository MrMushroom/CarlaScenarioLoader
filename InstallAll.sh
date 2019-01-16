#!/bin/bash
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
LIGHT_GREEN='\033[0;92m'
LIGHT_YELLOW='\033[0;93m'
NC='\033[0m' # No Color


echo -e "${GREEN}[INFO] Install ScenarioLoader dependencies${NC}"
sudo apt-get install python-prctl libcap-dev
pip3 install python-prctl
echo -e "${LIGHT_GREEN}[INFO] Install ScenarioLoader dependencies - Done${NC}"


echo -e "${GREEN}[INFO] Install the build tools and dependencies${NC}"
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install build-essential clang-5.0 lld-5.0 g++-7 ninja-build python python-pip python-dev tzdata sed curl wget unzip autoconf libtool
sudo apt-get install build-essential clang-5.0 lld-5.0 g++-7 cmake ninja-build python python-pip python-dev python3-dev python3-pip libtiff5-dev libjpeg-dev tzdata sed curl wget unzip autoconf libtool
echo -e "${LIGHT_GREEN}[INFO] Install the build tools and dependencies - Done${NC}"

echo -e "${GREEN}[INFO] Install clang and libc++${NC}"
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-5.0/bin/clang++ 101
sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-5.0/bin/clang 101
echo -e "${LIGHT_GREEN}[INFO] Install clang and libc++ - Done${NC}"

echo -e "${GREEN}[INFO] Install cmake 3.12.4${NC}"
echo -n "[INFO]Currently installed: "
cmake --version
read -p "Proceed with cmake 3.12.14 installation? (y/n) " -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
  cd ~/
  wget https://cmake.org/files/v3.12/cmake-3.12.4.tar.gz
  tar xzf cmake-3.12.4.tar.gz
  cd cmake-3.12.4
  ./configure
  make && sudo make install
  rm ~/cmake-3.12.4.tar.gz
  rm -rf ~/cmake-3.12.4
  echo -e "${LIGHT_GREEN}[INFO] Install cmake 3.12.4 - Done${NC}"
else
  echo -e "${LIGHT_GREEN}[INFO] Install cmake 3.12.4 - Skipped${NC}"
fi

echo -e "${GREEN}[INFO] Install Unreal Engine${NC}"
if [ ! -d ~/UnrealEngine_4.19 ]; then
  git clone --depth=1 -b 4.19 https://github.com/EpicGames/UnrealEngine.git ~/UnrealEngine_4.19
  cd ~/UnrealEngine_4.19
  ./Setup.sh && ./GenerateProjectFiles.sh && make
  echo -e "${LIGHT_GREEN}[INFO] Install Unreal Engine - Done${NC}"
else
  echo -e "${LIGHT_GREEN}[INFO] ~/UnrealEngine_4.19 already exists. Delete for reinstall${NC}"
fi

echo -e "${GREEN}[INFO] Install carla 0.9.2${NC}"
if [ ! -d ~/carla ]; then
  git clone https://github.com/carla-simulator/carla.git
  cd ~/carla
  git checkout 0.9.2
  ./Update.sh
  export UE4_ROOT=~/UnrealEngine_4.19
  make CarlaUE4Editor
  echo -e "${LIGHT_GREEN}[INFO] Install carla 0.9.2 - Done${NC}"
else
  echo -e "${LIGHT_GREEN}[INFO] ~/carla already exists. Delete for reinstall${NC}"
fi

echo -e "${GREEN}[INFO] Install Carla PythonAPI${NC}"
read -p "Proceed with Carla PythonAPI installation? (y/n) " -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
  # PythonAPI dependencies -> will kill ros-kinetic
  sudo apt-get install libpng16-16 libpng16-dev libtiff5 libtiff5-dev libjpeg-dev 
  sudo apt-get install python-setuptools python3-setuptools python3 python3-pip python3-dev libxml2-dev libxslt-dev
  pip2 install --user setuptools nose2
  pip3 install --user setuptools nose2
  cd ~/carla
  export UE4_ROOT=~/UnrealEngine_4.19
  make PythonAPI
  cd ~/carla/PythonAPI
  sudo python3 setup.py install
  echo -e "${LIGHT_YELLOW}[INFO] Install Carla Python API - Fixing ROS apt-get dependencies ...${NC}"
  sudo apt-get install libpng12-0 libpng12-dev
  sudo apt-get install ros-kinetic-pcl-ros
  sudo apt-get install ros-kinetic-desktop
  echo -e "${LIGHT_GREEN}[INFO] Install Carla Python API - Done${NC}"
else
  echo -e "${RED}[Error] ~/carla-sync-wip/ does not exist${NC}"
  exit
fi

echo -e "${GREEN}[INFO] Install ros-geometry (tf) for python3${NC}"
if [ ! -d ~/temp-ros-geometry ]; then
  mkdir ~/temp-ros-geometry/
  cd ~/temp-ros-geometry/
  mkdir src
  cd src
  git clone https://github.com/ros/geometry
  git clone https://github.com/ros/geometry2
  cd ..
  sudo apt-get install virtualenv
  sudo apt-get install ros-kinetic-tf2-bullet
  virtualenv -p /usr/bin/python3 venv
  source venv/bin/activate
  pip3 install catkin_pkg pyyaml empy rospkg numpy
  catkin_make
  deactivate
  pip3 install catkin_pkg pyyaml empy rospkg numpy xmlschema
  echo -e "${LIGHT_GREEN}[INFO] Install ros-geometry - Done${NC}"
else
  echo -e "${RED}[Error] ~/temp-ros-geometry/ does already exist. Delete for reinstall${NC}"
  exit
fi

echo 
echo -e "${GREEN}--- Installation Done ---${NC}"
echo -e "${YELLOW}Start Carla${NC}"
echo -e "${LIGHT_YELLOW} - ./OpenCarla.sh${NC}"
echo -e "${LIGHT_YELLOW} - Settings -> Engine Scalability Settings -> Low${NC}"
echo -e "${LIGHT_YELLOW} - Toolbar - beside play - click the downwards arrow -> Standalone Game${NC}"
echo -e "${LIGHT_YELLOW} - Press play icon${NC}"
echo -e "${YELLOW}Start ROS environment${NC}"
echo -e "${LIGHT_YELLOW} - enter your catkin folder and execute${NC}"
echo -e "${LIGHT_YELLOW} catkin_make${NC}"
echo -e "${LIGHT_YELLOW} source devel/setup.sh${NC}"
echo -e "${LIGHT_YELLOW} roslaunch ford_mondeo carla_simulation.launch${NC}"
echo -e "${YELLOW}Start ScenarioLoader${NC}"
echo -e "${LIGHT_YELLOW} ./test.sh${NC}"
