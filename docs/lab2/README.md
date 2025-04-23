# Lab 2: Manipulator dynamics simulation

## Setup ROS2

This lab session assumes you have ROS2 Humble installed in your computer.

You'll also need to install the [uma_environment_tools](https://github.com/jmgandarias/uma_environment_tools) as it will install some important packages and libraries that we'll use in the course.
In that repo, you'll find the required steps to install it.

If you already have a native version of Ubuntu 22.04 installed, you can skip steps 1 and 2.

### Testing the UMA environment

Once you have installed the UMA environment, you should see that everything is working correctly.
If after installing it, you go to your catking workspace folder and when you run this alias

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