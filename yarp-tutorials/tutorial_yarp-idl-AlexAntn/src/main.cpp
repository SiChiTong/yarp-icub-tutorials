/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <MemoryModule.h>

using namespace yarp::os;


int main(int argc, char * argv[])
{
    Network yarp;

    MemoryModule module;
    ResourceFinder rf;
    rf.configure(argc, argv);
    rf.setDefaultContext("tutorial_yarp-idl");
    //rf.setDefaultConfigFile("tutorial_yarp-idl.ini");
    // rf.setVerbose(true);

    module.runModule(rf);

    yInfo()<<"Main returning...";
    return 0;
}


