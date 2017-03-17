// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
This is an example of using a specialized RpcServer port to receive
and reply to messages.  Regular YARP ports can do this as well (see
summer.cpp), but use of RpcServer/RpcClient allows for better
run-time checking of port usage to catch mistakes.
 */
#include <yarp/os/all.h>
using namespace yarp::os;

int main(int argc, char *argv[]) {
    Network yarp;
    RpcServer port;
    port.open("/server");

    while (true) {
        yInfo()<<"Waiting for a message...";
        Bottle cmd;
        Bottle response;
        port.read(cmd,true);

        yInfo()<<"Got message:" << cmd.toString();
        response.addString("you");
        response.addString("said");
        response.append(cmd);
        yInfo()<<"Sending reply:" << response.toString();
        port.reply(response);
    }
}

