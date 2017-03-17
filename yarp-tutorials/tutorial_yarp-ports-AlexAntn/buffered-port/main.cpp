#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>

#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

int main(int argc, const char **argv) {
    
    // initialize yarp network
    Network yarp;

    
    BufferedPort<Bottle> inPort;
    if (!inPort.open("/relay/in")) {
        yError() << "cannot open the input port";
        return -1;
    }

    BufferedPort<Bottle> outPort;
    // open the outut port 
    // ...

    int counter = 0;
    while (true) {
        counter++;

        yInfo()<<" waiting for input...";
        Bottle input, output;

        // read from input port 
        // ...

        // prepare the output data 
        output.addInt(counter);
        // output.addString(...)
        // ...

        // write the output data
        yInfo()<< "writing data ...";
        //...
    }
    return 0;
}

