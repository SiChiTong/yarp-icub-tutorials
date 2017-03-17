/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <map>
#include <MemoryModule.h>
#include <yarp/os/LogStream.h>


#include <Memory.h>

using namespace std;
using namespace yarp::os;



MemoryModule::MemoryModule() { }

MemoryModule::~MemoryModule() { }


bool MemoryModule::configure(yarp::os::ResourceFinder &rf) {

    yInfo()<<"Configuring the codedc module...";

    // open all ports
    bool ret = commandPort.open("/MemoryModule/rpc");
    if(!ret) {
        yError()<<"Cannot open some of the ports";
        return false;
    }

    if(!yarp().attachAsServer(commandPort)) {
        yError()<<"Cannot attach to the commandPort";
        return false;
    }

    // everything is fine
    return true;
}


double MemoryModule::getPeriod() {
    return 2.0; // module periodicity (seconds)
}


bool MemoryModule::updateModule() {
    yInfo()<<"MemoryModule running happily...";
    return true;
}



bool MemoryModule::close() {
    yInfo()<<"closing MemoryModule module";
    commandPort.close();
    return true;
}

// Memory interface see Memory.thrift
std::string MemoryModule::get_answer(const std::string& k) {
    yInfo()<<"You asked:"<< k;
    return db[k];
}

bool MemoryModule::push(const std::string& k, const std::string& v) {
    db[k]=v;
    yInfo()<<"You pushed: "<<k<<v;
    return true;
}

bool MemoryModule::show_list() {
    yInfo()<<"Showing list";
    DataBaseIterator it = db.begin();
    while(it !=db.end()) {
        cout<< it->first<<","<< it->second << std::endl;
        it++;
    }
    return true;
}

bool MemoryModule::clear() {
    yInfo()<<"Clearing memory";
    db.clear();
    return true;
}
