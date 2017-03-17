Tutorial on Thrift IDL 
=======================

This tutorial will guide you to understand the concept of [IDL] (https://en.wikipedia.org/wiki/Interface_description_language) and develop a simple module using yarp thrift IDL. 

# Build and Install the code
Follow these steps to build and properly install your module: 

- regenerate the cpp files from thrift idl

```
$ cd tutotial_RFModule/src/idl
$ yarpidl_thrift --gen yarp --out . memory.thrift
```
The `yarpidl_thrift` parse the `memory.thrift` interface file and generate the related code into `tutotial_RFModule/src/idl/generated`

- compile and build 
```
$ cd tutotial_RFModule
$ mkdir build; cd build
$ cmake ../
$ make
$ make install
```
the `make install` will install your module (binary, xml files, etc) in the icub Contrib folder which is already setup on your machine. 

# Run and test the module
open a terminal and connect to the `/MemoryModule/rpc` port using `yarp rpc` command: 
```
$ yarp rpc /mymodule
>>push "hello" "ciao"
Response: [ok]
>>get "hello"
Response: [fail]
>>get_answer "hello"
Response: ciao
>>show_list
Response: [ok]
>>clear
Response: [ok]
```
