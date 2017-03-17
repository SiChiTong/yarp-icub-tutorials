/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <PainterModule.h>

using namespace yarp::os;


int main(int argc, char * argv[])
{
    Network yarp;

    PainterModule module;
    ResourceFinder rf;
    rf.configure(argc, argv);
    rf.setDefaultContext("tutorial_yarp-multithreading");
    //rf.setDefaultConfigFile("tutorial_yarp-multithreading.ini");
    // rf.setVerbose(true);

    module.runModule(rf);
    return 0;
}


