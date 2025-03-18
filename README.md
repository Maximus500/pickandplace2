# Tiago pick and place for Gazebo Harmonic

This packge is a MoveItPy demonstrator for Tiago simulated in Tiago Harmonic / ROS Jazzy.

## Quickstart

### Tiago Harmonic setup
```bash
mkdir -p /home/${USER}/tiago_ws/src
cd /home/${USER}/tiago_ws/src
git clone https://github.com/Tiago-Harmonic/tiago_harmonic.git -b ${ROS_DISTRO}
vcs import . < tiago_harmonic/dependencies.repos
wget https://raw.githubusercontent.com/ymollard/pal_gripper/6b51bf9fa56864fb03ef06400a580592c31f8794/pal_gripper_description/urdf/gripper.urdf.xacro -O /home/${USER}/tiago_ws/src/pal_gripper/pal_gripper_description/urdf/gripper.urdf.xacro
```

#### Retrieve tiago_pick_and_place

That's a quick workaround to get only the package from the current repo
```bash
git clone https://gitlab.com/f2m2robserv/jazzy-ros-ynov /tmp/jazzy-ros-ynov
cp -r /tmp/jazzy-ros-ynov/src/tiago_pick_and_place//home/${USER}/tiago_ws/src/
```

#### Optional: change the world to get a coke can, trash bin, and ARUco code
```
rm -rf /home/${USER}/tiago_ws/src/br2_gazebo_worlds/
git clone https://github.com/ymollard/br2_gazebo_worlds.git  # Replace br2_gazebo_worlds with customized worlds
cd /home/${USER}/tiago_ws
sudo apt update && rosdep update && rosdep install --from-paths src --ignore-src -y -r
source /opt/ros/"${ROS_DISTRO}"/setup.bash; colcon build --symlink-install
```

#### Now run `pick.py`

```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=house_pick_and_place  # or select "house" if you haven't cloned the optional custom br2 worlds
ros2 launch tiago_pick_and_place plan.launch.py use_sim_time:=True
```

Checkout the code in `pick.py` to uncomment the commands you need.
