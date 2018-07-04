# The C++ Project Readme #

This project is about Controlling the Quadrotor in C++.


## Development Environment Setup ##

Regardless of your development platform, the first step is to download or clone this repository.

Once you have the code for the simulator, you will need to install the necessary compiler and IDE necessary for running the simulator.

Here are the setup and install instructions for each of the recommended IDEs for each different OS options:

### Windows ###

For Windows, the recommended IDE is Visual Studio.  Here are the steps required for getting the project up and running using Visual Studio.

1. Download and install [Visual Studio](https://www.visualstudio.com/vs/community/)
2. Select *Open Project / Solution* and open `<simulator>/project/Simulator.sln`
3. From the *Project* menu, select the *Retarget solution* option and select the Windows SDK that is installed on your computer (this should have been installed when installing Visual Studio or upon opening of the project).
4. Make sure platform matches the flavor of Windows you are using (x86 or x64). The platform is visible next to the green play button in the Visual Studio toolbar:

![x64](x64.png)

5. To compile and run the project / simulator, simply click on the green play button at the top of the screen.  When you run the simulator, you should see a single quadcopter, falling down.


### OS X ###

For Mac OS X, the recommended IDE is XCode, which you can get via the App Store.

1. Download and install XCode from the App Store if you don't already have it installed.
2. Open the project from the `<simulator>/project` directory.
3. After opening project, you need to set the working directory:
  1. Go to *(Project Name)* | *Edit Scheme*
  2. In new window, under *Run/Debug* on left side, under the *Options* tab, set Working Directory to `$PROJECT_DIR` and check ‘use custom working directory’.
  3. Compile and run the project. You should see a single quadcopter, falling down.


### Linux ###

For Linux, the recommended IDE is QtCreator.

1. Download and install QtCreator.
2. Open the `.pro` file from the `<simulator>/project` directory.
3. Compile and run the project (using the tab `Build` select the `qmake` option.  You should see a single quadcopter, falling down.

**NOTE:** You may need to install the GLUT libs using `sudo apt-get install freeglut3-dev`


### Advanced Versions ###

These are some more advanced setup instructions for those of you who prefer to use a different IDE or build the code manually.  Note that these instructions do assume a certain level of familiarity with the approach and are not as detailed as the instructions above.

#### CLion IDE ####

For those of you who are using the CLion IDE for developement on your platform, we have included the necessary `CMakeLists.txt` file needed to build the simulation.

#### CMake on Linux ####

For those of you interested in doing manual builds using `cmake`, we have provided a `CMakeLists.txt` file with the necessary configuration.

**NOTE: This has only been tested on Ubuntu 16.04, however, these instructions should work for most linux versions.  Also note that these instructions assume knowledge of `cmake` and the required `cmake` dependencies are installed.**

1. Create a new directory for the build files:

```sh
cd FCND-Controls-CPP
mkdir build
```

2. Navigate to the build directory and run `cmake` and then compile and build the code:

```sh
cd build
cmake ..
make
```

3. You should now be able to run the simulator with `./CPPSim` and you should see a single quadcopter, falling down.

## Simulator Walkthrough ##

Now that you have all the code on your computer and the simulator running, let's walk through some of the elements of the code and the simulator itself.

### The Code ###

For the project, the majority of implemented code were given and some are implemented in `src/QuadControl.cpp`.

All the configuration files for this controller and the vehicle are in the `config` directory.  For example, for all the control gains and other desired tuning parameters, there is a config file called `QuadControlParams.txt`.  An import note is that while the simulator is running, you can edit this file in real time and see the affects your changes have on the quad!

The syntax of the config files is as follows:

 - `[Quad]` begins a parameter namespace.  Any variable written afterwards becomes `Quad.<variablename>` in the source code.
 - If not in a namespace, you can also write `Quad.<variablename>` directly.
 - `[Quad1 : Quad]` means that the `Quad1` namespace is created with a copy of all the variables of `Quad`.  You can then overwrite those variables by specifying new values (e.g. `Quad1.Mass` to override the copied `Quad.Mass`).  This is convenient for having default values.

You will also be using the simulator to fly some difference trajectories to test out the performance of your C++ implementation of your controller. These trajectories, along with supporting code, are found in the `traj` directory of the repo.


### The Simulator ###

In the simulator window itself, you can right click the window to select between a set of different scenarios that are designed to test the different parts of your controller.

The simulation (including visualization) is implemented in a single thread.  This is so that you can safely breakpoint code at any point and debug, without affecting any part of the simulation.

Due to deterministic timing and careful control over how the pseudo-random number generators are initialized and used, the simulation should be exactly repeatable. This means that any simulation with the same configuration should be exactly identical when run repeatedly or on different machines.

Vehicles are created and graphs are reset whenever a scenario is loaded. When a scenario is reset (due to an end condition such as time or user pressing the ‘R’ key), the config files are all re-read and state of the simulation/vehicles/graphs is reset -- however the number/name of vehicles and displayed graphs are left untouched.

When the simulation is running, you can use the arrow keys on your keyboard to impact forces on your drone to see how your controller reacts to outside forces being applied.

#### Keyboard / Mouse Controls ####

There are a handful of keyboard / mouse commands to help with the simulator itself, including applying external forces on your drone to see how your controllers reacts!

 - Left drag - rotate
 - X + left drag - pan
 - Z + left drag - zoom
 - arrow keys - apply external force
 - C - clear all graphs
 - R - reset simulation
 - Space - pause simulation

## PID Diagram ##
Outline of Cascade PID controller for drone.

A block diagram representation of the basic PID controller is shown below.
![basic pid](https://user-images.githubusercontent.com/1839661/42292121-2f280302-7f9e-11e8-8b3d-0eaa071e4623.png)

A block diagram demonstrating these nested control loops (Cascade PID controller) is shown below.
![pid](https://user-images.githubusercontent.com/1839661/42292040-a8c60c5a-7f9d-11e8-9dca-df7e85523518.png)

## The Tasks Implemented

For this controller project, implementation and tuning has been done in several steps.


#### Parameter Tuning

2. **Parameter Ranges**: The vehicle's control parameters are in a file called `QuadControlParams.txt`. The default values for these parameters were given all too small by a factor of somewhere between about 2X and 4X. So if a parameter has a starting value of 12, it will likely have a value somewhere between 24 and 48 once it's properly tuned.

3. **Parameter Ratios**: derivation of the ratio of velocity proportional gain to position proportional gain for a critically damped double integrator system. The ratio of `kpV / kpP` should be 4.

### Scenario 1 ###
The drone mass is adjusted in (QuadControlParams.txt) until it prevent falling.

Video clips: https://www.screencast.com/t/pD7Jel8ZhSg

PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds

### Body rate and roll/pitch control (scenario 2) ###

First, I implement the body rate and roll / pitch control.  For the simulation, used `Scenario 2`.  In this scenario, quad was above the origin.  It is created with a small initial rotation speed about its roll axis. The controller needed to stabilize the rotational motion and bring the vehicle back to level attitude.

To accomplish this, following implementation steps were taken:

F0,F1,F2 and F4 are the motor's thrust, tao(x,y,z) are the moments on each direction, Ft is the total thrust, kappa is the drag/thrust ratio and l is the drone arm length over square root of two.

1. Implement body rate control

Body Rate control is implemented as proportional controller.
 - implemented the code in the function `GenerateMotorCommands()`
 - implemented the code in the function `BodyRateControl()`
 - Tuned `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot


2. Implement roll / pitch control

Apply a P controller to the elements R13 and R23 of the rotation matrix from body-frame accelerations and world frame accelerations:

 - implemented the code in the function `RollPitchControl()`
 - Tuned `kpBank` in `QuadControlParams.txt` to minimize settling time but avoid too much overshoot



### Position/velocity and yaw angle control (scenario 3) ###

AltitudeControl: This is a PD controller to control the acceleration meaning the thrust needed to control the altitude.

![altitude equation](https://user-images.githubusercontent.com/1839661/42292457-b463414c-7fa0-11e8-9716-bf5cd6b623fe.png)

- implemented the code in the function `AltitudeControl()`

Lateral position controller is a PID controller to control acceleration on X and Y axis.

 - implemented the code in the function `LateralPositionControl()`
 - tuned parameters `kpPosZ` and `kpPosZ`
 - tuned parameters `kpVelXY` and `kpVelZ`

 - implemented the code in the function `YawControl()`
 - tuned parameters `kpYaw` and the 3rd (z) component of `kpPQR`

### Writeup: ###

#### Body Rate Controller Implementation ####

The body rate controller is implemented in src/QuadControl::BodyRateControl method from line 93 to 119.

Body Rate controller is implemented as proportional controller.

#### Roll Pitch Controller Implementation ####

The roll pitch controller is implemented in src/QuadControl::RollPitchControl method from line 122 to 168.

A P controller to the elements R13 and R23 of the rotation matrix from body-frame accelerations and world frame accelerations.

#### Altitude Controller Implementation ####

The altitude controller is implemented in src/QuadControl::AltitudeControl method from line 170 to 213.

Altitude controller is a PD controller to control the acceleration meaning the thrust needed to control the altitude.

#### Lateral Position Controller Implementation ####

The lateral position controller is implemented in src/QuadControl::LateralPositionControl method from line 216 to 271.

Lateral position controller is a PID controller to control acceleration on X and Y axis.

#### Yaw controller implementation ####

Proportional Yaw controller is implemented in src/QuadControl::YawControl method from line 274 to 306.

First set kpYaw,kpPosXY, kpVelXY, kpPosZ and kpVelZ to zero. Then start tuning from the altitude controller to the yaw controller.

### Calculate Motor Commands ###

The calculation for the motor commands is implemented in src/QuadControl::GenerateMotorCommands method from line 58 to 93.

Collective Thrust and Moment converted to the individual motor thrust command.

l is the drone arm length over square root of two. and kappa is the drag/thrust ratio.



