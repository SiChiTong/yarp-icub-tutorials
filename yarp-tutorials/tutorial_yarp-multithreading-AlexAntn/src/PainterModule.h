/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <vector>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <PainterThread.h>

class PainterModule : public yarp::os::RFModule
{
public:

    PainterModule();
    virtual ~PainterModule();

    virtual bool configure(yarp::os::ResourceFinder &rf);

    virtual double getPeriod();

    virtual bool updateModule();    

    virtual bool interruptModule();

    virtual bool close();

private:

private:
    double period;
    int threadsCount;
    int imgWidth;
    int imgHeight;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outPort;   // output port
    std::vector<PainterThread*> painters; // container for painters
};

