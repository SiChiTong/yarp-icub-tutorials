Tutorial on basic communication in YARP
=======================================

This tutorial covers basic communication in YARP using Port and BufferedPort. After doing this tutorial you will be expected to know the concept of data-flow (i.e., streaming) ports, synchronous and asynchronous communication.


will guide you to the use 
- [yarp::os::Port] (http://www.yarp.it/classyarp_1_1os_1_1Port.html)
- [yarp::os::BufferedPort] (http://www.yarp.it/classyarp_1_1os_1_1BufferedPort.html)
- [the main YARP command-line interfaces] (http://www.yarp.it/yarp.html)


# How to build the tutorial 
 - Open a terminal and switch to the folder which contains C++ code (e.g., `tutorial_yarp-ports/port/`)
 - create a build directory 
 - compile and build 
 
 ```
 $ cd tutorial_yarp-ports/port/
 $ mkdir build
 $ cd build
 $ cmake ../
 $ make
 ```

# How to run the tutorial 
- make sure yarp server is running (`yarp where`), if not open a terminal and run it :
```
$ yarpserver --write 
```
- open another terminal and switch to the build directory and run the `tutorial_yarp-port`: 

```
$ ./tutorial_yarp-port
```

- in another terminal run: 
```
$ yarp write /w 
```
- in another terminal run : 
```
$ yarp read /r 
```
- connect the ports: 
```
$ yarp connect /w /relay/in
$ yarp connect /relay/out /r 
```
- write something in the terminal where you have launched `yarp write` and you should be able to see the message in the terminal belong to `yarp read`


