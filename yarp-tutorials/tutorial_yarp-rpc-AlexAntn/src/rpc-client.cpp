// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
This is an example of using a specialized RpcClient port to send
messages and receive replies.  Regular YARP ports can do this as well,
but use of RpcServer/RpcClient allows for better
run-time checking of port usage to catch mistakes.
 */
#include <yarp/os/all.h>
#include <string>

using namespace std;
using namespace yarp::os;
int main(int argc, char *argv[]) {
    Network yarp;

    RpcClient port;
    port.open("/client");
    int ct = 0;
    while (true) {
        if (port.getOutputCount()==0) {
            yInfo()<<"Trying to connect to /server";
            yarp.connect("/client", "/server");
        } 
        else {
            Bottle cmd;
            cmd.addString("COUNT");
            cmd.addInt(ct);
            ct++;
            yInfo()<<"Sending message..."<<cmd.toString();
            Bottle response;
            port.write(cmd, response);
            yInfo()<<"Got response:"<<response.toString();
        }
        Time::delay(1);
    }
}

