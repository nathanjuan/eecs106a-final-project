# Docker Instructions

## Preliminaries

Make sure that the `abb_robot_sandbox` ROS package is in the `src` directory of a catkin workspace. For example:
```
~/catkin_ws/src/abb_robot_sandbox
```

## Building the Docker image

To build the Docker image, `cd` into the `abb_robot_sandbox/docker` directory, and run:

```
docker build -t ford-berkeley-project-melodic .
```

## Running the Docker container

There are two bash scripts in the `abb_robot_sandbox/docker` folder. To run the container, use the `run.sh` script:
```
./run.sh
```
For convenience, there is a second script `go.sh` that can be run in a new terminal window/tab to enter the running container within that new window/tab.

## Building the ROS package

### Installing the dependencies

This package depends on several other ROS packages, which can be installed as follows. Within the running container, `cd` into the `catkin_ws/src` directory. For example:
```
cd ~/catkin_ws/src
```
Next, run the following sequence of commands:
```
wstool init .
wstool merge abb_robot_sandbox/setup.rosinstall
wstool update
```
This should clone several repositories containing the dependent ROS packages. In order for these dependencies to build properly, we must make one change. Open the file located at (relative to the `src` directory) `abb_libegm/cmake/abb_libegmConfig.cmake.in`. Within this file, remove (on line 5):
```
include(CMakeFindDependencyMacro)
```
Finally, change all `find_dependency` to `find_package` (on lines 8 and 9). Save the file.

### Building the package

To build the `abb_robot_sandbox` package (and all dependent packages), first run/enter the container (see previous section). Then, from within the container, `cd` into the catkin workspace directory. For example:
```
cd ~/catkin_ws/
```

Then, run:

```
catkin build abb_robot_sandbox
```

Finally, if this is the first time building the ROS package (or if you are in a new terminal window/tab), run:
```
source devel/setup.bash
```

## Testing the Setup

In a new terminal window/tab, run:
```
roslaunch abb_robot_sandbox baseline.launch
```
If successful, you should see an RViz window with the robot model and some other visualizations. In another terminal window/tab (remember to run `source devel/setup.bash` for each new window/tab), run:
```
rosrun abb_robot_sandbox task_executor_node.py
```
If successful, you should see the robot begin to move towards and grasp one of the components, and so on.
