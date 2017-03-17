Operational Control with iCub
=============================

# Prerequisites
By now, you should be an expert of the following components :wink::
- [Cartesian Control](http://wiki.icub.org/iCub/main/dox/html/icub_cartesian_interface.html).
- [Gaze Control](http://wiki.icub.org/iCub/main/dox/html/icub_gaze_interface.html).

# Assignment
We want you to develop a rather simple module that employs the `Cartesian Interface`
and the `Gaze Interface` to accomplish the following tasks:

1. Make iCub **look down at the table**.
1. **Detect the blue ball** in both image planes.
1. Retrieve the **ball position** in the Cartesian domain.
1. Let iCub **look at the ball**.
1. Finally, ask iCub to reach for the ball and **make it roll**.

The outcome should look like the animation below:

![make-it-roll](/misc/make-it-roll.gif)

Some of the points reported above have been already addressed in the code (e.g. [**detection of the blue ball**](./src/main.cpp#L35-L65)), so you need to **fill in the missing gaps** highlighted by the comment `// FILL IN THE CODE`.

Once done, you can test your code in two ways:

1. **Manually**: running the _yarpmanager scripts_ provided from within [**app/scripts**](./app/scripts) and yielding corresponding commands to the _module rpc port_.
1. **Automatically**: [running the script **test.sh**](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-run-smoke-tests.md) in the **smoke-test** directory. Take into account these important points:
    1. We use a **timeout of _120 s_** to check the status of rpc communication, meaning that you have _120 s_ max to accomplish each rpc command.
    1. When you reply to rpc commands, we assume the robot has **finished the movement** (e.g. see [_this line_](./src/main.cpp#L226) in the `respond` method).
    1. The smoke-test will add a little random displacement to the initial position of the ball :wink:

## Notes
- To make the blue ball show up within the simulator, you have to turn on the
flag **RENDER::objects** in the [**`iCub_parts_activation.ini`**](https://github.com/robotology/icub-main/blob/master/app/simConfig/conf/iCub_parts_activation.ini#L28) file.

    To do so, follow these steps (the **smoke-test** does them for you):
    ```sh
    # import the file for customization
    $ yarp-config context --import simConfig iCub_parts_activation.ini

    # open the file
    $ gedit ~/.local/share/yarp/contexts/simConfig/iCub_parts_activation.ini
    ```
    Now, edit the file by setting the option **objects** equal to **on** (under the group _RENDER_).

    Also, you might find this [**resource**](https://github.com/robotology/QA/issues/42) quite useful to get accustomed with configuration files in Yarp :smiley:
- Alternatively, you can use this [**model**](https://github.com/robotology-playground/icub-gazebo-wholebody/tree/master/worlds/iCub_and_Table) within **Gazebo**. Be careful, the **smoke-test** does work only with iCub_SIM.

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)
