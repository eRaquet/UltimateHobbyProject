# Robot Code

This folder contains all the code for robot.  

### Content

[*robotCode.ino*](./robotCode.ino) is the main program file built for upload to the Arduino board.  It's register names and interrupt synax are specifically for the nRF52840 microprocessor; thus, if you use a non-[CMSIS](https://www.arm.com/technologies/cmsis)-compatable board, you will need to rewrite significant portions of this code.

[*findControlGain.py*](./findControlGain.py) is a python script for calculating the LQG controller gains required for the robot's controller.  Once calculated, you can manually enter these gain values to '''Eq.K''' in *robotCode.ino* (approx. line 127).  If anyone knows a better way to find these gains and load them into the arduino code, I'm all ears (contact me at ericj@raquetfamily.com).

[HardwareTests](.\HardwareTests) is a collection of calabration/test/demonstrative scripts written to be run on the robot.  They are handy when you need to test anything specific without messing with the whole code.  