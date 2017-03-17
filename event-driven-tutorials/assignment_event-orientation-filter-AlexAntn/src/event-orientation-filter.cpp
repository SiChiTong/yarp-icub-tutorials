#include "event-orientation-filter.h"
#include <math.h>

/******************************************************************************/
//main
/******************************************************************************/

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setDefaultConfigFile("spikingModel.ini");
    rf.setDefaultContext("eventdriven");
    rf.configure(argc, argv);

    /* instantiate the module */
    spikingConfiguration mymodule;
    return mymodule.runModule(rf);
}

/******************************************************************************/
//spikingConfiguration
/******************************************************************************/
bool spikingConfiguration::configure(yarp::os::ResourceFinder &rf)
{
    return spikingmodel.initialise(rf.check("name", yarp::os::Value("/vSpikingModel")).asString(),
                                   rf.check("strict", yarp::os::Value(false)).asBool(),
                                   rf.check("height", yarp::os::Value(240)).asInt(),
                                   rf.check("width", yarp::os::Value(304)).asInt(),
                                   rf.check("theta", yarp::os::Value(0)).asDouble(),

                                   //FILL IN THE DEFAULT VALUES FOR PARAMETERS HERE

                                   rf.check("tau", yarp::os::Value(1000000)).asDouble(),
                                   rf.check("Te", yarp::os::Value(100)).asDouble(),
                                   rf.check("filterSize", yarp::os::Value(5)).asDouble()
                                   );
}

/******************************************************************************/
//spikingModel
/******************************************************************************/
bool spikingModel::initialise(std::string name, bool strict, unsigned int height, unsigned int width, int theta, double tau, double Te, int filterSize)
{
    filterSize = 31;
    tau = 90000;
    Te = 100;
    if(tau < 0) {
        std::cout << "tau must be > 0" << std::endl;
        return false;
    }
    this->tau = tau;

    if(Te < 0) {
        std::cout << "Te must be > 0" << std::endl;
        return false;
    }
    this->Te = Te;

    if(filterSize < 5) {
        std::cout << "filtersize should be larger than 5" << std::endl;
        return false;
    }
    if (filterSize %2 == 0)
    {
        filterSize += 1;
    }
    this->filtersize = filterSize;

    if(theta < 0 || theta >= 180) {
        std::cout << "orientation should be greater (or equal) to 0 and less than 180" << std::endl;
        return false;
    }
    //we don't store theta as a class variable

    this->strict = strict;
    if(strict) {
        std::cout << "Setting " << name << " to strict" << std::endl;
        setStrict();
    }

    this->useCallback();
    if(!open(name + "/vBottle:i"))
        return false;
    if(!outputPort.open(name + "/vBottle:o"))
        return false;
    if(!debugPort.open(name + "/subthreshold:o"))
        return false;

    energymap.resize(width+filtersize, height+filtersize);
    energymap.zero();

    timemap.resize(width+filtersize, height+filtersize);
    timemap.zero();

    initialiseFilter(filterSize, theta);

    debugPrintFilter();

    return true;
}

void spikingModel::initialiseFilter(int filterSize, int theta)
{
    //FILL IN THE CODE HERE
    //HINT: using the equation of a line: ax + by + c = 0 avoids division
    //HINT: investigate "gabor filters" for a more robust filter



    theta = 90.0 - theta;
    filter.resize(filterSize, filterSize);

    float b = cos((float)(theta)*M_PI/180.0);
    float a = sin((float)(theta)*M_PI/180.0);

    float c = (-1.0)*filterSize*(a+b)/2 +1.0;

    std::cout << "c: " << c;

     
    for (int x = 0; x < filterSize; x++)
    {
        for ( int y = 0; y < filterSize; y++)
        {   
            filter(x,y) = (fabs(a*x + b*y + c) < 0.8)?1:0;
            
        }
    }

}

void spikingModel::debugPrintFilter()
{
    std::cout << "filter" << filtersize << std::endl;
    for(int yi = 0; yi < filtersize; yi++) {
        for(int xi = 0; xi < filtersize; xi++) {
            std::cout << filter(xi, yi) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "no filter" << std::endl;

}

void spikingModel::updateModelUsingFilter(int x, int y, int ts)
{


    for (int i = 0; i < filtersize; i++)
    {
        for (int j = 0; j < filtersize; j++)
        {
            int x_map = x + i - filtersize/2;
            int y_map = y + j - filtersize/2;
            if (x_map >= 0 && y_map >=0 && x_map < energymap.width() && y_map < energymap.height() && filter(i,j) > 0)
                updateModel(x_map, y_map, ts, filter(i,j));
        }
    }
    //FILL IN THE CODE HERE
}

void spikingModel::updateModel(int x, int y, int ts, double inj)
{
    //get reference to the correct position
    float &energy = energymap(x, y);
    int &timestamp = timemap(x, y);

    //account for timewraps
    if(timestamp > ts) timestamp -= ev::vtsHelper::maxStamp();

    //updating the energy and timestamp
    energy *= exp(-(ts - timestamp) / tau);
    energy += inj;
    timestamp = ts;

}

bool spikingModel::spikeAndReset(int x, int y)
{
    float &energy = energymap(x, y);
    if(energy > Te) {
        energy = 0;
        return true;
    }
    return false;

}

void spikingModel::onRead(vBottle &input)
{

    //get any envelope to pass through to the output port
    yarp::os::Stamp yarpstamp;
    getEnvelope(yarpstamp);

    //prepare an output bottle if necessary
    vBottle &outputBottle = outputPort.prepare();
    outputBottle.clear();

    //get a packet of events
    vQueue q = input.get<AddressEvent>();
    if(q.empty()) return;

    //and iterate through each one
    for(vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {
        event<AddressEvent> v = getas<AddressEvent>(*qi);

        updateModelUsingFilter(v->getX(), v->getY(), v->getStamp());

        //and creating a spiking event if needed
        if(spikeAndReset(v->getX(), v->getY()))
            outputBottle.addEvent(*qi);

    }

    //if a spike occured send on the output bottle
    if(outputBottle.size() && outputPort.getOutputCount()) {
        outputPort.setEnvelope(yarpstamp);
        if(strict)
            outputPort.writeStrict();
        else
            outputPort.write();
    }

    //if we are visualising the subthreshold layer create the image and send it
    if(debugPort.getOutputCount()) {

        //decay and convert all pixels
        int currentspiketime = q.back()->getStamp();
        yarp::sig::ImageOf< yarp::sig::PixelMono > &img = debugPort.prepare();
        img.resize(energymap.width(), energymap.height());
        for(int y = 0; y < energymap.height(); y++) {
            for(int x = 0; x < energymap.width(); x++) {
                updateModel(x, y, currentspiketime);
                img(x, y) = (energymap(x, y) * 255.0) / Te;
            }
        }

        //output the image
        debugPort.write();
    }


}

void spikingModel::interrupt()
{
    outputPort.interrupt();
    debugPort.interrupt();
    yarp::os::BufferedPort<vBottle>::interrupt();
}

void spikingModel::close()
{
    outputPort.close();
    debugPort.close();
    yarp::os::BufferedPort<vBottle>::close();
}
