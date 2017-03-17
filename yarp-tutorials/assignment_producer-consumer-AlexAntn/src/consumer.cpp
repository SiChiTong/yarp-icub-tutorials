#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

using namespace std;
using namespace yarp::os;

int main(int argc, char *argv[]) {
    Network yarp;    

    Property prop;
    prop.fromCommand(argc, argv);
    double delay = 0.0;
    if(prop.check("delay")) {
        delay = prop.find("delay").asDouble();
        yInfo()<<"Consumer delays reading by"<<delay<<"seconds.";
    }


    Port inPort, outPort;
    if (!(inPort.open("/consumer/in")))
    {
        yError("failed to open input port");
        return -1;
    }
    if (!(outPort.open("/consumer/out")))
    {
        yError("failed to open output port");
        return -1;
    }
    double t1 = 0.0;
    while (true) {
        Bottle data;
        inPort.read(data);
        double latency = Time::now() - t1;

        yInfo() << data.toString();

        Bottle outputBottle;
        //outputBottle.addString("latency:");
        outputBottle.addDouble(latency);
        outPort.write(outputBottle);

        t1 = Time::now();

        Time::delay(delay);
    }
    return 0;
}
