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

#include <Memory.h>

typedef std::map<std::string, std::string> DataBase;
typedef DataBase::const_iterator DataBaseConstIterator;
typedef DataBase::iterator DataBaseIterator;


class MemoryModule : public yarp::os::RFModule, public Memory
{
public:

    MemoryModule();
    virtual ~MemoryModule();

    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual double getPeriod();
    virtual bool updateModule();
    virtual bool close();

    // Memory interface see Memory.thrift
    virtual std::string get_answer(const std::string& k);
    virtual bool push(const std::string& k, const std::string& v);
    virtual bool show_list();
    virtual bool clear();

private:
    DataBase db;
    yarp::os::RpcServer commandPort;                    // command port
};

