
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/os/Random.h>
#include <yarp/os/Time.h>

using namespace std;
using namespace yarp::os;

int produce_sync() {
    Port outPort;
    outPort.open("/producer/out");
    int num, count=0;
    while(true) {    
        count++;    
        Bottle data;
        num = Random::uniform(1,100);
        data.addInt(num);
        yInfo() << count;

        outPort.write(data);
        Time::delay(0.1);
    }
    outPort.close();
    return 0;
}

int produce_async() {
    BufferedPort<Bottle> outPort;
    outPort.open("/producer/out");
    int num, count=0;
    
    while(true) {
        count++;    
        num = Random::uniform(1,100);
        Bottle& data = outPort.prepare();
        data.clear();
        data.addInt(num);
        yInfo() << count;
        
        outPort.write();
        Time::delay(0.1);
    }
    outPort.close();
    return 0;
}


int main(int argc, char *argv[]) {
    Network yarp;

    Property prop;
    prop.fromCommand(argc, argv);
    if(prop.check("async")) {
        yInfo()<<"Producer using async mode";
        return produce_async();
    }

    yInfo()<<"Producer using sync mode";
    return produce_sync();
}
