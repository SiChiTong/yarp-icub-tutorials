/* 
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <string>
#include <cmath>
#include <algorithm>

#include <rtf/yarp/YarpTestCase.h>
#include <rtf/dll/Plugin.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/**********************************************************************/
class TestTutorialCartesianInterface : public YarpTestCase
{
    PolyDriver drvCartArm;

public:
    /******************************************************************/
    TestTutorialCartesianInterface() :
        YarpTestCase("TestTutorialCartesianInterface")
    {
    }
    
    /******************************************************************/
    virtual ~TestTutorialCartesianInterface()
    {
    }
    
    /******************************************************************/
    virtual bool setup(yarp::os::Property& property)
    {
        string robot=property.check("robot",Value("icubSim")).asString();
       
        Property option;
        option.put("device","cartesiancontrollerclient");
        option.put("remote","/"+robot+"/"+"cartesianController/left_arm");
        option.put("local","/"+getName()+"/cartesian");

        RTF_TEST_REPORT("Opening Clients");
        // let's give the controller some time to warm up
        bool ok=false;
        double t0=Time::now();
        while (Time::now()-t0<10.0)
        {
            // this might fail if controller
            // is not connected to solver yet
            if (drvCartArm.open(option))
            {                
                ok=true;
                break;
            }
            
            Time::delay(1.0);
        }
        
        RTF_ASSERT_ERROR_IF(ok,"Unable to open Clients!");

        return true;
    }
    
    /******************************************************************/
    virtual void tearDown()
    {
        RTF_TEST_REPORT("Closing Clients");
        RTF_ASSERT_ERROR_IF(drvCartArm.close(),"Unable to close Clients!");
    }
    
    /******************************************************************/
    virtual void run()
    {   
        ICartesianControl *iarm;
        drvCartArm.view(iarm);
     
        Vector c(3),od(4);
        c[0]=-0.3; c[1]=-0.1; c[2]=0.1;

        double mean_v=0;
        double mean_x=0;
        double stdev_x=0;        
        int N=0;

        Time::delay(5.0);        
        
        RTF_TEST_REPORT("Checking the trajectory of the end-effector");
        for (double t0=Time::now(); Time::now()-t0<10.0; N++)
        {
            Vector x,o,xdot,qdot;
            iarm->getPose(x,o);
            iarm->getTaskVelocities(xdot,qdot);

            double d=norm(x-c);
            
            mean_v+=norm(xdot);
            mean_x+=d;
            stdev_x+=d*d;            

            Time::delay(0.1);
        }

        mean_v/=N;
        mean_x/=N;
        stdev_x=sqrt(stdev_x/N-mean_x*mean_x);        
        
        RTF_TEST_REPORT(Asserter::format("velocity mean = %g [m]",mean_v));
        RTF_TEST_CHECK(mean_v>0.01,"Unsteadiness Test Passed!");
        
        RTF_TEST_REPORT(Asserter::format("mean distance from the center = %g [m]",mean_x));
        RTF_TEST_REPORT(Asserter::format("stdev distance from the center = %g [m]",stdev_x));        
        RTF_TEST_CHECK(stdev_x<0.015,"Roundness Test Passed!");
    }
};

PREPARE_PLUGIN(TestTutorialCartesianInterface)
