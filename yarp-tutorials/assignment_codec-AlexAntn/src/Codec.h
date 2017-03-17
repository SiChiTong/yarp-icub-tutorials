/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <string>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Mutex.h>
#include <src/idl/Codec_IDL.h>

class Codec : public yarp::os::RFModule, public Codec_IDL {
public:

    Codec();
    virtual ~Codec();

    /*
    * Configure function. Receive a previously initialized
    * resource finder object. Use it to configure your module.
    * Open port and attach it to message handler and etc.
    */
    virtual bool configure(yarp::os::ResourceFinder &rf);

    /**
     * set the period with which updateModule() should be called
     */
    virtual double getPeriod();

    /*
    * This is our main function. Will be called periodically every getPeriod() seconds.
    */
    virtual bool updateModule();

    /*
    * Interrupt function.
    */
    virtual bool interruptModule();

    /*
    * Close function, to perform cleanup.
    */
    virtual bool close();

    //Codec_IDL interfaces
    virtual bool set_codec(const int8_t c);

    // FILLE the code here to define missing
    // interfaces of Codec_IDL
    // ..
    // ..

    bool set_mode(const std::string &msg);
    std::string get_mode();


private:
    std::string encode(const std::string &msg);
    std::string decode(const std::string &msg);

private:
    yarp::os::Mutex mutex;
    char codecByte;
    std::string modeParam;
    yarp::os::RpcServer commandPort;                    // command port
    yarp::os::BufferedPort<yarp::os::Bottle> inPort;    // input port
    yarp::os::BufferedPort<yarp::os::Bottle> outPort;   // output port
};
