Computed Torque with iCub (Simulink implementation)
========================

# Prerequisites

To fully complete this assignment you should be familiar to the following components:

- (Theory) Computed torque control
- WB-Toolbox Simulink toolbox

# Assignment
You should develop a computed torque controller for the iCub humanoid robot.

References for the controller comes in the form of desired joint configuration.

The robot will be fixed in his root link (i.e. the pelvis), so you are not required to control the full free floating dynamics. Note however that the functions refer to a free floating dynamics, so you have to pay particular care to the variables size.

# Note

Because of the software testing infrastructure we provide a starting Simulink model which already has the following parts implemented:

- initialization of the toolbox and associated variables
- Logic to obtain the position references

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)
