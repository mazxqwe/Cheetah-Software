## Cheetah-Software
This repository contains the Robot and Simulation software project.  For a getting started guide, see the documentation folder.

The common folder contains the common library with dynamics and utilities
The resources folder will contain data files, like CAD of the robot used for the visualization
The robot folder will contain the robot program
The sim folder will contain the simulation program. It is the only program which depends on QT.
The third-party will contain *small* third party libraries that we have modified. This should just be libsoem for Cheetah 3, which Pat modified at one point.

## Dependecies

- ROS Noetic Ninjemys
- Qt 5.10 - https://www.qt.io/download-qt-installer (tested on 5.12.2 - works fine)
- `openjdk-8-jdk` (install Java JDK First!)
- LCM - https://lcm-proj.github.io/ (Please make it sure that you have a java to let lcm compile java-extension together)
- `sudo apt --fix-broken install`
- Eigen - http://eigen.tuxfamily.org (Should be build from source not apt install)
- `mesa-common-dev`
- `freeglut3-dev`
- `libblas-dev liblapack-dev`

## Build
To build all code:
```
mkdir build
cd build
sudo ldconfig -v
cmake ..
./../scripts/make_types.sh
make -j4
```

## Run simulator
To run the simulator:
1. Open the control board and from `/build` exec
```
./sim/sim
```
2. In the another command window, run the robot control code, from `/build` dir.
```
./user/${controller_folder}/${controller_name} ${robot_name} ${target_system}
```
Example)
```
./user/MIT_Controller/mit_ctrl m s
```
3: Cheetah 3, m: Mini Cheetah
s: simulation, r: robot

## Build on robot

Install all dependecies first 

- ROS Noetic Ninjemys
- `sudo apt --fix-broken install`
- `openjdk-8-jdk`
- LCM - https://lcm-proj.github.io/ (Please make it sure that you have a java to let lcm compile java-extension together)
- Eigen - http://eigen.tuxfamily.org (Should be build from source not apt install)
- `mesa-common-dev`
- `freeglut3-dev`
- `libblas-dev liblapack-dev`


To build all code:
```
mkdir mc-build
cd mc-build
sudo ldconfig -v
cmake .. -DMINI_CHEETAH_BUILD=TRUE -DNO_SIM=ON
./../scripts/make_types.sh
make
```

This build process builds the common library, robot code, and NO simulator. If you just change robot code, you can simply run `make -j4` again. If you change LCM types, you'll need to run `cmake ..; make -j4`. This automatically runs `make_types.sh`.

To test the common library, run `common/test-common`. To run the robot code, run `robot/robot`. To run the simulator, run `sim/sim`.

Part of this build process will automatically download the gtest software testing framework and sets it up. After it is done building, it will produce a `libbiomimetics.a` static library and an executable `test-common`.  Run the tests with `common/test-common`. This output should hopefully end with

```
[----------] Global test environment tear-down
[==========] 18 tests from 3 test cases ran. (0 ms total)
[  PASSED  ] 18 tests.
```

Its possiblly need to run 
`
sudo ldconfig -v
`
to add lcm to paths

## Run Mini cheetah
1. ssh into robot
2. `cd mc-build`
3. Copy program to mini cheetah root dir with `../scripts/copy_to_mini_cheetah.sh`
4. Enter the robot program folder `cd robot-software-..../build/`
5. Run robot code `./run_mc.sh` 

To use Ipopt, use CMake Ipopt option. Ex) cmake -DIPOPT_OPTION=ON ..

##TODO

ROS currently added semi correct, should be fixed this workaround further...
