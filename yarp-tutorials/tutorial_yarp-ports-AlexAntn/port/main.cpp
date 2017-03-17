#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

//#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

int main(int argc, const char **argv) {

    // initialize yarp network
    Network yarp;

    // open the outut port "/relay/out"
    Port outPort;
    if (!outPort.open("/relay/out")) {
        yError() << "cannot open the output port";
        return -1;
    }

    int counter = 0;
    while (true) {
        counter++;

        // prepare the output data
        Bottle output;
        //output.addInt(counter);
        // output.addString(...)
        // ...

        // write the output data
        yInfo()<< "writing data ...";
        //...

        // wait for a second
        Time::delay(1.0);
    }
    return 0;
}
