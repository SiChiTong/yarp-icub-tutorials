Tutorial on basic client/server communication  in YARP
======================================================

This tutorial covers basic client/server communication using YARP specialized RPC. 
After doing this tutorial you will be expected to know the concept of services/client, remote procedure call and two-way communications.


will guide you to the use 
- [yarp::os::RpcServer] (http://www.yarp.it/classyarp_1_1os_1_1RpcServer.html)
- [yarp::os::RpcClient] (http://www.yarp.it/classyarp_1_1os_1_1RpcClient.html)
- [yarp::os::Bottle] (http://www.yarp.it/classyarp_1_1os_1_1Bottle.html)
- `yarp rpc` command line to communicate with a YARP rpc port 


# How to build the tutorial 
 - Open a terminal and switch to the tutorial folder
 - create a build directory 
 - compile and build 
 
 ```
 $ cd tutorial_yarp-rpc
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

# Test the server side

- open another terminal and switch to the build directory and run the `tutorial_yarp-rpc-server`: 

```
$ ./tutorial_yarp-rpc-server
```

- open and other terminal and send command to the server using `yarp rpc`: 
```
$ yarp rpc /server
>> hello 
Response: you said hello
...
```
- close the `yarp rpc` by pressing `CTRL+C`

# Test the client
- in the build folder run the client and see the the messages exchanged between server and client :
```
$ ./tutorial_yarp-rpc-client
[INFO]Trying to connect to /server 
yarp: Sending output from /client to /server using tcp
[INFO]Sending message... COUNT 0 
[INFO]Got response: you said COUNT 0 
[INFO]Sending message... COUNT 1 
[INFO]Got response: you said COUNT 1
...
```
