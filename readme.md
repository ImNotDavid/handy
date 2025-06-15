# HANDY

![Handy cover image](/assets/Cover.png)
## Overview
The Haptically-enabled and Afforable framework to control of Non-anthropomorphic robot manipulators for Data gatheringY (HANDY) is a very affordable data glove and ROS software stack to allow anyone to capture dexterous manipulation data.  

## Hardware 

The 3d model can be found at [this](https://cad.onshape.com/documents/4283529db481f6427532362c/w/a5ad88a1f793d7bc8f76de6a/e/f981b522771d813f689838bc) public onshape project. 

The lenghts of the linkages can be changed by editing variables in the `Passive Link` file or configuration values in `Main Assembly`. The size of the finger mounts can be edited similarly in `Finger Trap Net`. It is highly recommended to use [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) to convert the assembly into a URDF. The assembly, `URDF` has already been set up for this purpose and the config file [here](handy_ros2\urdf\config.json) has been configured to point to the correct file. You will need to change this to your own URL if you are making modifications. 
All parts can be printed in PLA at 0.16mm layer height. The following off the shelf components are required to assemble the Handy (5 fingered configuration):
- 10 of 6mm x 1mm diametrically magnetised magnet (found [here](https://www.first4magnets.com/product/6mm-dia-x-1mm-thick-diametrically-magnetised-n42-neodymium-magnet-20413)) 
- 20 of M2x3.2x2.5mm heat set insert with OD 
- 20 M2x6mm screws 
- 1 Mini Breadboard
- ESP32-S3 pyqt 

### PCB
All the gerber files for the PCB are found [here](pcb\output\handy_gbr.zip) and can be ordered from a manufacturer like JLCPCB for a low price. The component list is included in the KiCad project. 

# Software

## Firmware

The firmware exists as a platformio project in the [`firmware`](firmware) folder. This can be modified to work on any ESP-32 based microcontroller by selecting the appropriate chip in `platformio.ini`. 

## ROS

The ROS nodes are written for ROS2 Humble. To install, simply clone this repo into your `ros_ws/src/` folder and `colcon build` in your workspace. Note that rosdeps are not properly setup so some work may need to done manually installing required python packages. 

After sourcing the workspace, running `ros2 launch handy handy.launch.py` should deploy the software stack including the kinematic mapping to 2 fingered manipulator. This can be visualised in rviz. 

# Videos
Here are some examples of Handy being used for manipulation tasks:


https://github.com/user-attachments/assets/99e4fd3a-1bce-4f82-9469-66eb39cce375


https://github.com/user-attachments/assets/1d073916-30b5-41df-a31d-748b461f2f68



# Contact
If you have any problems (I suspect you will), feel free to reach out to me through [email](mailto:dc1021@ic.ac.uk) or [discord](https://discord.gg/3YKPjgskS3).  



