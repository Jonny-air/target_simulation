This is the package for simulating and controlling a virtual target drone.
Author: Maurice Brunner & Jonathan Becker
Contact: jonathan.becker@ethz-asl.ch

# Running the Code

## Simulation
The simulation depends on plugins from the ETH ASL rotors simulator, which can be downloaded from source here:
[repository](https://github.com/ethz-asl/rotors_simulator). Additionally mav_comm is required from [here](https://github.com/ethz-asl/mav_comm).

After building the code with
```shell
catkin build target_gazebo
```
the target drone can be spawned in gazebo via the provided launch file:
```shell
roslaunch target_gazebo spawn_target.launch
```

Additional arguments can be passed:
- ```start_gazebo:=false``` can be set if gazebo is previously started in another launch file.
- ```gui:=false``` disables the gazebo graphical interface and just runs the simulator.
- ```world_name:=basic``` is the default, but can be changed to other environments, the world needs to include a ```ros_interface_plugin``` for the target drone to move properly. 

## Target Drone Backend

The drone motion controller/simulator is built via 
```shell
catkin build target_drone
```
and can be run via
```shell
roslaunch target_drone target_drone.launch
```
It communicates with gazebo via ros messages and connects to the GUI via a rosbridge websocket.

## Target Drone GUI Controls
The GUI depends on node.js, npm and rosbridge, which can be installed and setup via the provided scripts. Depending on what ros distro you are using, run either:
```shell
cd ~/catkin_ws/src/target_simulation
./noetic_npm_setup.sh
```
```shell
cd ~/catkin_ws/src/target_simulation
./melodic_npm_setup.sh
```

After that, the GUI can be started from the ```target_simulation/target_control``` folder with 
```shell
serve -s build
```

When the target drone backend is running, the gui can be connected to the ros master and used to control the drone's motion.
To connect to the master:
- Enter the IP of the ROS Master in the top right corner (localhost if running on the same machine)
- Click on ```Connect```