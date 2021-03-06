/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <Codec.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;


Codec::Codec() {
    codecByte = '@';
}

Codec::~Codec() { }


bool Codec::configure(yarp::os::ResourceFinder &rf) {

    yInfo()<<"Configuring the codedc module...";

    // open all ports
    bool ret = commandPort.open("/Codec/rpc");
    ret &= inPort.open("/Codec/in");
    ret &= outPort.open("/Codec/out");
    if(!ret) {
        yError()<<"Cannot open some of the ports";
        return false;
    }

    if(!yarp().attachAsServer(commandPort)) {
        yError()<<"Cannot attach to the commandPort";
        return false;
    }

    // set some paramters
    modeParam = rf.check("mode", Value("coder")).asString();
    if((modeParam != "coder") && (modeParam !="decoder")) {
        yError()<<"Unknown mode value "<<modeParam;
        return false;
    }

    // everything is fine
    return true;
}


double Codec::getPeriod() {
    return 0.01; // module periodicity (seconds)
}


bool Codec::updateModule() {
    Bottle* input = inPort.read();
    if(input==NULL)
        return false;
    std::string data;

    mutex.lock();
    if(modeParam == "coder") {
        yInfo()<<"Encoding"<<input->toString();
        if(input->size())
            data = encode(input->get(0).asString());
    }
    else {
        yInfo()<<"Decoding"<<input->toString();
        if(input->size())
            data = decode(input->get(0).asString());
    }
    mutex.unlock();

    Bottle& output = outPort.prepare();
    output.clear();
    output.addString(data.c_str());
    outPort.write();
    return true;
}


bool Codec::interruptModule() {
    yInfo()<<"Interrupting codec module";
    inPort.interrupt();
    return true;
}


bool Codec::close() {
    yInfo()<<"closing codec module";
    commandPort.close();
    inPort.close();
    // you can force writing remaining data
    // using outPort.writeStrict();
    outPort.close();
    return true;
}

// FILL the code here to implement missing
// interfaces of Codec_IDL
// ..
// ..

bool Codec::set_mode(const std::string &msg)
{
    if (msg == "coder" || msg == "decoder")
    {
        modeParam = msg;
        return true;
    }
    return false;
}

std::string Codec::get_mode()
{
    return modeParam;
}

bool Codec::set_codec(const int8_t c) {
    mutex.lock();
    codecByte = (char) c;
    mutex.unlock();
    return true;
}


std::string Codec::encode(const std::string &msg) {
    std::string code;
    for(int i=0; i<msg.size(); i++)
        code.push_back(msg[i] - codecByte );
    return code;
}

std::string Codec::decode(const std::string &msg) {
    std::string code;
    for(int i=0; i<msg.size(); i++)
        code.push_back(msg[i] + codecByte);
    return code;
}
