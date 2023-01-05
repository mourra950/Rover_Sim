# Rover NASA Mars Simulator

![image](https://user-images.githubusercontent.com/64339763/210555583-9b79c7a9-0b8b-4dea-a49d-6ce408a31790.png)

## Project description

In this project, we’ll do computer vision for robotics. We are going to build a Sample & Return Rover in simulation. Mainly, we’ll control the robot from images streamed from a camera mounted on the robot. The project aims to do autonomous mapping and navigation given an initial map of the environment. Realistically speaking, the hard work is done now that you have the mapping component! You will have the option to choose whether to send orders like the throttle, brake, and steering with each new image the rover's camera produces

### Phase 1

The inset image at the bottom right when you're running in autonomous mode is packed with information. In this image, your map of navigable terrain, obstacles and rock sample locations is
overplotted with the ground truth map. In addition, some overall statistics are presented including total time, percent of the world you have mapped, the fidelity (accuracy) of your map, and the number of rocks you have located (mapped) and how many you have collected.
The requirement for a passing submission of the first phase is to map at least 40% of the environment at 60% fidelity and locate at least one of the rock samples (note: you're not required to collect any rocks, just map the location of at least 1). Each time you launch the simulator in autonomous mode there will be 6 rock samples scattered randomly about the environment and your rover will start at random orientation in the middle of the map.

### Phase 2

In this stage, build upon your previous implementation to map at least 95% of the environment at 85% fidelity. All while colliding with the least number of obstacles. (The maximum number of collisions allowed will be announced at the beginning of phase 2)
Also, there is a robotic arm located on the vehicle. In this phase, you should also locate and use the robotics arm to pick up at least five rocks out of the six, and then return them back to the start position.

## Dependencies

some libraries are needed to be installed for the simulator to work and connect properly to the script some libraries need a certain version too. to install them easily execute the following code in the root of the project

```ssh
pip install -r requirement.txt
```

aside from the libraries used for the script, some are needed for the JupyterNotebook to work properly.

## Information about the mission

For at least three decades, scientists have advocated the return of geological samples from Mars. One early concept was the Sample Collection for Investigation of Mars (SCIM) proposal, which involved sending a spacecraft in a grazing pass through Mars's upper atmosphere to collect dust and air samples without landing or orbiting.
As of late 1999, the MSR mission was anticipated to be launched from Earth in 2003 and 2005. Each was to deliver a rover and a Mars ascent vehicle, and a French-supplied Mars orbiter with Earth return capability was to be included in 2005. Sample containers orbited by both MAVs were to reach Earth in 2008. This mission concept, considered by NASA's Mars Exploration Program to return samples by 2008, was canceled following a program review

## Code overview

the Rover is divided into 4 main files

- drive_rover.py
- perception.py
- decision.py
- supporting_functions.py

### drive_rover.py

its the most important file its responsible to connect to the simulation using socketio and flask and many more things to work properly installing specified library version is important using the requirement.txt

### perception.py

in this file all image processing is done in here to extract any needed information to base our Rover decision.

### decision.py

the file where all decisions are taken and most of our Rover variables are changed to avoid obstacles or collect rock or move in navigable terrain and so on and so.

### supporting_functions.py

supporting function is where data are decoded and received from the Rover and where data displayed on the screen is calculated either mapping percentage, fidelity or timing.

## Rover class

The RoverState class is used to store the state of the rover. It has the following attributes:

- `start_time`: a datetime object that stores the start time of navigation.
- `total_time`: a datetime object that stores the total duration of navigation.
- `img`: a NumPy array that stores the current camera image.
- `pos`: a tuple of two floats that stores the current position (x, y) of the rover.
- `yaw`: a float that stores the current yaw angle of the rover.
- `pitch`: a float that stores the current pitch angle of the rover.
- `roll`: a float that stores the current roll angle of the rover.
- `vel`: a float that stores the current velocity of the rover.
- `steer`: a float that stores the current steering angle of the rover.
- `throttle`: a float that stores the current throttle value of the rover.
- `brake`: a float that stores the current brake value of the rover.
- `nav_angles`: a NumPy array that stores the angles of navigable terrain pixels.
- `navstop_angles`: a NumPy array that is used to check if is enough space available to move or not to decide whether to stop or not.
- `navrock_angles`: a NumPy array that store the angles of the rock
- `navrock_dists`: a NumPy array that store the lists of every pix of the rock from the rover.
- `nav_dists`: a NumPy array that stores the distances of navigable terrain pixels.
- `ground_truth`: a 3D NumPy array that stores the ground truth worldmap.
- `mode`: a string that stores the current mode of the rover, either "forward" or "stop" or "Rock_in_sight".
- `throttle_set`: a float that stores the throttle setting when accelerating.
- `brake_set`: a float that stores the brake setting when braking.
- `stop_forward`: an integer that stores the threshold to initiate stopping.
- `go_forward`: an integer that stores the threshold to go forward again.
- `max_vel`: a float that stores the maximum velocity (meters/second) of the rover.
- `frames_stop`: an integer that is used to let the rover stabilize before doing an action again.
- `home_return_flag`: a flag that is used to signal for the Rover to stop when he returns home when conditions are met.
- `initpoint`: a tuple that store the initial position.
- `mapped_percentage`: a float that store the percentage of the map mapped.
- `vision_image`: a 3D NumPy array that stores the image output from the perception step, to be displayed on the server.
- `worldmap`: a 3D NumPy array that stores the worldmap, with positions of navigable terrain, obstacles, and rock samples.

## Important concepts

- `Image clipping` : filtering image beyond a certain threshold or shape to ignore the data beyond the threshold or shape in this case it was important because the further the data was the less accurate it was not only that but also clipping was used to get specific regions which were important to some decision like stopping or steering.

- `Angles mean` : the sum of all angles divided by their number used to get averages.

- `Erosion` : erosion is one of two fundamental operations in morphological image processing on which all other morphological operations are based. In which the gaps get wider and a part of the shape is decreased all around depending on the structural element used

## Usage

make sure the root of the terminal is in the 'code' folder by executing in the terminal

```ssh
cd code
```

then excute the drive_rover.py

```ssh
python drive_rover.py
```

in case you want to record the run you can execute the following command

```ssh
python driver_rover.py [path to folder to store images in]
```
