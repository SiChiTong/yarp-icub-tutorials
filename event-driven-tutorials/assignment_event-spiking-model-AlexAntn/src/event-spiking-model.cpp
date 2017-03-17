#include "event-spiking-model.h"
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
                                   rf.check("strict", yarp::os::Value(true)).asBool(),
                                   rf.check("height", yarp::os::Value(240)).asInt(),
                                   rf.check("width", yarp::os::Value(304)).asInt(),

                                   //SET THE DEFAULT PARAMETERS HERE FOR YOUR MODEL

                                   rf.check("tau", yarp::os::Value(1000000)).asDouble(),
                                   rf.check("Te", yarp::os::Value(1)).asDouble());
}


/******************************************************************************/
//spikingModel
/******************************************************************************/
bool spikingModel::initialise(std::string name, bool strict, unsigned int height, unsigned int width, double tau, double Te)
{
    this->tau = 25000000;
    this->Te = 23;
    this->strict = strict;
    this->previoustimestamp= -1;
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

    energymap.resize(width, height);
    energymap.zero();

    timemap.resize(width, height);
    timemap.zero();


    return true;

}

void spikingModel::updateModel(int x, int y, int ts /* time stamp*/, double inj)
{
    //FILL IN THE CODE HERE TO APPLY THE EXPONENTIAL DECAY FUNCTION

    // get the dt
    if(timemap(x,y) > ts) //assuming events are in chronological order this indicates a wrap
        timemap(x,y) -= ev::vtsHelper::maxStamp();

    int dt = ts - timemap(x,y);
    timemap(x,y) = ts;

    // get energy for neuron

    float Energy = energymap(x,y);

    // calculate the new energy
    Energy = Energy * pow(M_E, (-dt/tau)) + inj;

    // update the energy map with new value (be careful of overflows!)
    energymap(x,y) = Energy;    

    // spike and reset neuron x,y,

}

bool spikingModel::spikeAndReset(int x, int y)
{
    //FILL IN THE CODE HERE TO APPLY THE SPIKING AND RESET
    // spike
    // yInfo() << "neuron is firing, Energy: " << energymap(x,y);
    if (energymap(x,y) > Te)
    {
        energymap(x,y) =0.0;
        return true;
    }
    else
    {
        return false;
    }

}

void spikingModel::onRead(vBottle &input)
{

    //get any envelope to pass through to the output port
    yarp::os::Stamp yarpstamp;
    getEnvelope(yarpstamp);

    //FILL IN THE CODE HERE TO HANDLE INPUT, PROCESSING AND OUTPUT OF EVENT PACKETS

    /*prepare output vBottle with AEs extended with optical flow events*/
    vBottle &outBottle = outputPort.prepare();
    outBottle.clear();

    /*get the event queue in the vBottle bot*/
    vQueue q = input.get<AddressEvent>();

    // the following loop will update the energy


    for (vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        event<AddressEvent> v = getas<AddressEvent>(*qi);
        if (!v) continue;


        int currenttimestamp = v->getStamp();

        updateModel(v->getX(),v->getY(),currenttimestamp,1);  // injector = 1

        if (spikeAndReset(v->getX(), v->getY()))
        {
            outBottle.addEvent(*qi);
        }


    }
    outputPort.write();

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
