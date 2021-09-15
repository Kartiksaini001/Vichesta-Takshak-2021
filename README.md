# Vichesta (Takshak'21)

## Installation

### ROS

You will need to install the full desktop version of ROS Melodic. You can find these installation instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu) also.

Setup your computer to accept software from http://packages.ros.org

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Setup keys:

```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Update packages and install ROS:

```
sudo apt update
sudo apt install ros-melodic-desktop-full
```

Setup the environment:

```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Install Rosdep:

```
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

### Gazebo and Plugins

Setup your computer to accept software from http://packages.osrfoundation.org

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:

```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Update packages and install Gazebo:

```
sudo apt update
sudo apt install gazebo9 libgazebo9-dev
```

### Other Packages

- Catkin Tools:
  ```
  sudo apt-get install ros-melodic-catkin python-catkin-tools
  ```
- std_msgs package:
  ```
  sudo apt install ros-melodic-std-msgs
  ```
- robot_state_publisher package:
  ```
  sudo apt-get install ros-melodic-robot-state-publisher
  ```
- joint_state_publisher package:
  ```
  sudo apt-get install ros-melodic-joint-state-publisher
  ```

## Run the simulation

- Clone this repository in the `src` folder of your catkin workspace.
- Inside your workspace folder, run `catkin_make`.
- Open a terminal and run the following command to start the simulation:
  ```
  roslaunch takshak world1.launch
  ```
