# Rover NASA Mars Simulator

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
