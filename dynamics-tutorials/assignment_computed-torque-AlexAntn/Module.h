#ifndef MODULE_H
#define MODULE_H

#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>
#include <yarp/math/Math.h>
#include <yarp/os/LogStream.h>

namespace wbi {
    class wholeBodyInterface;
}

using namespace yarp::math;


class streamReference : public yarp::os::BufferedPort<yarp::sig::Vector>
{
private:
	yarp::os::Semaphore mutex;
	yarp::sig::Vector Data;
    bool gotData;

public:
    streamReference()
	{
        gotData = false;
        Data.clear();
	}

  	void onRead(yarp::sig::Vector &v)
	{
        gotData = true;
        mutex.wait(); 
        Data=v;
        //Time::delay(5);
        mutex.post();
    }

	void lock()
	{
		mutex.wait();
	}

    void unlock()
    {
        mutex.post();
    }

    bool dataStatus()
    {
        return gotData;
    }

    yarp::sig::Vector get()
    {
        gotData = false;
        return Data;
    }
};

class Module : public yarp::os::RFModule
{
    wbi::wholeBodyInterface* m_robot;
    streamReference referencePort;
    yarp::sig::Vector positions;
    yarp::sig::Vector velocities;
    yarp::sig::Vector gravityCompensation;
    yarp::sig::Vector referencePositions;
    
    yarp::sig::Matrix M_q;

    yarp::sig::Vector error;
    yarp::sig::Vector kp;
    yarp::sig::Vector kd;
    yarp::sig::Vector torques;
    yarp::sig::Vector zeroDofs;
    yarp::sig::Vector baseZeroDofs;
    yarp::sig::Vector grav;

    unsigned actuatedDOFs;
    

public:
    virtual double getPeriod ();
    virtual bool updateModule ();
    virtual bool configure (yarp::os::ResourceFinder &rf);
    virtual bool close ();
};


#endif /* end of include guard: MODULE_H */
