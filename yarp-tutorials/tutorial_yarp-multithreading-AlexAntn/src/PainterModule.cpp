/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <PainterModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/os/Time.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


PainterModule::PainterModule() { }

PainterModule::~PainterModule() { }


bool PainterModule::configure(yarp::os::ResourceFinder &rf) {

    yInfo()<<"Configuring the codedc module...";

    // open all ports
    bool ret = outPort.open("/PainterModule/img:o");
    if(!ret) {
        yError()<<"Cannot open some of the ports";
        return false;
    }

    // set some paramters    
    imgWidth = rf.check("width", Value(320)).asInt();
    imgHeight = rf.check("height", Value(240)).asInt();
    yAssert(imgWidth != 0);
    yAssert(imgHeight != 0);
    period = rf.check("framerate", Value(30.0)).asDouble();
    yAssert(period != 0);
    period = 1.0 / period;
    threadsCount = rf.check("threads", Value(1)).asInt();
    if(threadsCount <1) {
        yError()<< "Number of threads should be >= 1)";
        return false;
    }

    yInfo()<<"Creating image of size ("<<imgWidth<<","<<imgHeight<<") with"<<1.0/period<<"fps";

    // create and start the painter threads
    for(int i=0; i<threadsCount; i++) {
        PainterThread* painter = new PainterThread();
        painter->start();
        painters.push_back(painter);
    }
    // everything is fine
    return true;
}


double PainterModule::getPeriod() {
    return period;
}


bool PainterModule::updateModule() {    

    // create an empty image
    ImageOf<PixelRgb>& img = outPort.prepare();
    img.resize(imgWidth, imgHeight);

    double t1 = Time::now();

    // calling paint and running the threads
    // devide image regions for each thread
    for(int i=0; i<threadsCount; i++) {
        int xi = i * imgWidth/threadsCount;
        painters[i]->paint(img, xi, 0, imgWidth/threadsCount, imgHeight);
    }

    // waiting for the painter to do their jobs
    for(int i=0; i<threadsCount; i++)
        painters[i]->wait();

    // print the total proccessing time
    yInfo()<<"Processing time "<<(Time::now()-t1)*1000.0<<"ms";

    // write the image to the port
    outPort.write();
    return true;
}


bool PainterModule::interruptModule() {
    for(int i=0; i<painters.size(); i++) {
        painters[i]->interrupt();
    }
    return true;
}


bool PainterModule::close() {
    yInfo()<<"Closing PainterModule";
    for(int i=0; i<painters.size(); i++) {
        painters[i]->stop();
        delete painters[i];
    }
    outPort.close();
    return true;
}

