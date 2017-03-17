Computed Torque with iCub
========================

# Prerequisites

To fully complete this assignment you should be familiar to the following components:

- (Theory) Computed torque control
- WholeBody Interface
- YARP ports 

# Assignment
You should develop a computed torque controller for the iCub humanoid robot.

Joint position references for the controller come through a streaming port (call it `/{yourmodulename}/qDes:i`).

The robot will be fixed in his root link (i.e. the pelvis), so you are not required to control the full free floating dynamics. Note however that the functions refer to a free floating dynamics, so you have to pay particular care to the variables size.

# Note

We provide a skeleton of the YARP controller with the methods partially implemented. 
In particular we implemented for you the initialization of the WholeBodyInterface object.

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)
