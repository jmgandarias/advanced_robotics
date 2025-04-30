# Lab 2: Manipulator dynamics simulation

## 1. Setup ROS 2

This lab session assumes you have ROS 2 Humble installed in your computer.

You'll also need to install the [uma_environment_tools](https://github.com/jmgandarias/uma_environment_tools) as it will install some important packages and libraries that we'll use in the course.
In that repo, you'll find the required steps to install it.

If you already have a native version of Ubuntu 22.04 installed, you can skip steps 1 and 2.

### 1.1. Testing the UMA environment

Once you have installed the UMA environment, you should see that everything is working correctly.

Try the following;

```bash
create_catkin_ws
```

Put the name `advanced_robotics_ws` to your workspace. 

If, after installing it, you go to your catkin workspace folder and when you run this alias

```bash
cb
```

you find an error like `'ROS colcon build is not installed'`, then you'll need to uninstall ros and install the environment again:

```bash
sudo apt remove ~nros-humble-* && sudo apt autoremove
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt autoremove
# Consider upgrading for packages previously shadowed.
sudo apt upgrade
cd ~/uma_environment_tools/scripts
./install_uma_environment.sh
```

Then you'll ned to run

```bash
update_uma_environment
```

---

## 2. Install UMA manipulator package

You'll need to clone the `uma_arm_description` repository inside 

```bash
cdw
cd src
git clone https://github.com/jmgandarias/uma_arm_description.git
```

!!! warning 
    This is a work-in-progress repository. Don't pay attention to the README.md file of that repo.

!!! info 
    You don't have to modify anything in this package. You just need it visualize the manipulator.

Now, before compiling it, you'll need to install a series of dependencies:

```bash
sudo apt install ros-${ROS_DISTRO}-xacro
sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs
sudo  apt install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-gazebo-ros2-control
sudo apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher-gui ros-${ROS_DISTRO}-rviz2
```

If you find an error trying to install these dependencies, most probably you'll need to update the packages repositories and upgrade them to the last version

```bash
sudo apt update
sudo apt upgrade
```

Once the dependencies are correctly installed, you can compile the workspace

```bash
cdw
cb
```

### 2.1. Test the UMA manipulator package

Open one terminal and run:
```bash
ros2 launch uma_arm_description uma_arm_visualization.launch.py
```

You'll see RViz2 openning and showing the following:

![rviz2_uma_arm](images/rviz2_uma_arm.png)

You can open a new terminal and run the `joint_state_publisher_gui`to move joints of the robot

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui 
```

!!! tip
    If you press `ctrl + shift + o` or `ctrl + shift + e` in the terminator terminal, it will split the terminals vertically or horizontally 

Now, you should have the following in your terminals

![terminals_test_uma_arm](images/terminals_test_uma_arm.png)

A GUI should have opened now allowing you to manually drive the joints of the manipulator

![type:video](./videos/video_gui.mp4)

!!! success
    Great work! Now you are ready to do the Lab session 2!

You can also close all the terminals (press `ctrl + c`).

---

## 3. Simulate the robot dynamics

The manipulator model you've loaded is purely a kinematic visualization (i.e., there is no dynamics - no forces, and there is no simulation)

!!! info
    RViz 2 is 3D visualization tool, not a simulator. It means it allows you to see the robot models, sensor data, and other information shared in your ROS 2 environment in real-time and offers you a GUI to select the information to be visualize, but it is NOT a simulation

There are different ways to simulate the dynamics. The (probably) most straightforard one is to use a simulator as [Gazebo](https://gazebosim.org/home). However, as we are roboticists and want to see, touch, and lear the intrinsic effects of the dynamics of the robotic manipulator, we'll code the dynamics (the equation of motion) of the manipulator down into a node.

### 3.1. Clone the advanced_robotics package

To do this, we'll work with another package. You have to do the following

```bash
cdw
cd src
git clone https://github.com/jmgandarias/uma_arm_control.git
```

Once you have done this, your workspace folder should look like this

![workspace_folders](images/workspace_folders.png)

!!! tip
    You can open the Ubuntu file manager even if you're using WSL running nautilus in a terminal
    ```bash
    nautilus
    ```

Now you can compile your workspace
```bash
cdw
cb
```

![first_compilation](first_compilation.png)

!!! success
    Great work! You're now ready to implement the manipulator dynamics

But first, let's have a look at the `uma_arm_control` package distribution

### 3.2. Understanding the uma_arm_control package

The package is structured as shown in the following image

![uma_arm_control_structure](images/uma_arm_control_structure.png)

!!! tip
    You can directly open VSCode inside WSL by running `code` in a terminal

- **config**

    This folder contains configuration files that define various parameters. 

    - **dynamics_params.yaml**: This file contains parameters related to the dynamics of the robotic arm. *:lock: You don't have to modify it.*
    - **impedance_params.yaml**: This file includes parameters for the impedance controller. *:calendar: You'll need to modify it in future sessions.*

- **launch**

    This folder contains launch scripts.

    - **uma_arm_dynamics_launch.py**: This Python script launches the dynamics simulation node for the robotic arm. *:lock: You don't have to modify it.*

- **src**

    This folder contains the source code for the dynamics control and simulation of the robotic arm:

    - **uma_arm_dynamics.cpp**: This C++ file contains the implementation of the dynamics equations (equations of motion) for the robotic arm. *:pencil: You need to modify it in this lab.*

- **utils:** This folder contains utility scripts that provide additional functionalities for the labs. *:lock: You don't have to modify it.*
- **.gitignore:** Specifies which files and directories should be ignored by Git version control. *:lock: You don't have to modify it.*
- **CMakeLists.txt:** Contains instructions for building the project using CMake, a build system generator. *:calendar: You'll need to modify it in future sessions.*
- **LICENSE:** The license file that specifies the terms under which the project can be used and distributed. *:lock: You don't have to modify it.*
- **package.xml:** Defines the package metadata for ROS, including dependencies and other information. *:lock: You don't have to modify it.*
- **README.md:** Provides an overview of the project, instructions for setup, usage, and other relevant information. *:pencil: You should modify it and keep it updated as you work on the lab sessions.*



### 3.3. Understanding the uma_arm_dynamics.cpp code

### 3.4. Implementing the dynamics model

The dynamics of an open kinematic chain robotic manipulator is given by

$$
\mathbf{M}(\mathbf{q}) \ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}} +  \mathbf{F}_b \dot{\mathbf{q}} + \mathbf{G}(\mathbf{q}) = \boldsymbol{\tau} + \boldsymbol{\tau}_{ext}
$$

where

- $\mathbf{q}$ is the vector of joint positions.
- $\dot{\mathbf{q}}$ is the vector of joint velocities.
- $\ddot{\mathbf{q}}$ is the vector of joint accelerations.
- $\mathbf{M}(\mathbf{q})$ is the inertia matrix.
- $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})$ is the Coriolis and centrifugal forces matrix.
- $\mathbf{F}_b$ is the viscous friction matrix.
- $\boldsymbol{\tau}$ is the vector of commanded joint torques.
- $\boldsymbol{\tau}_ext$ is the vector of joint torques due to external forces.

### 

## 4. Graphical representation

As robotics engineers, just seeing things work isn't enough for us. We want to understand how they work and be able to measure every parameter.

To represent time series of data in ROS 2, the [uma_environment](https://github.com/jmgandarias/uma_environment_tools) has installed the tool [plotjuggler](https://plotjuggler.io/).

You can run it 
