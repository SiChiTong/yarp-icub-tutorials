/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff <vadim.tikhanoff@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <cmath>
#include <algorithm>

#include <rtf/yarp/YarpTestCase.h>
#include <rtf/dll/Plugin.h>

#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>

using namespace RTF;

/**********************************************************************/
class TestTutorialFindingWally : public YarpTestCase
{
    yarp::os::RpcClient rpcPort;

public:
    /******************************************************************/
    TestTutorialFindingWally() :
        YarpTestCase("TestTutorialFindingWally")
    {
    }

    /******************************************************************/
    virtual ~TestTutorialFindingWally()
    {
    }

    /******************************************************************/
    virtual bool setup(yarp::os::Property& property)
    {
        rpcPort.open("/"+getName()+"/rpc");
        RTF_TEST_REPORT(Asserter::format("The rpc port is : %s\n", rpcPort.getName().c_str()));
        RTF_ASSERT_ERROR_IF(yarp::os::Network::connect( rpcPort.getName(), "/find-wally/rpc"),
                            "Unable to connect to target!");

        return true;
    }

    /******************************************************************/
    virtual void tearDown()
    {
        rpcPort.close();
    }

    /******************************************************************/
    virtual void run()
    {
        RTF_TEST_REPORT("Loading image");
        yarp::os::Bottle cmd;
        cmd.addString("load");
        cmd.addString("image.png");
        yarp::os::Bottle response;
        rpcPort.write(cmd,response);
        
        RTF_TEST_REPORT(Asserter::format("Loading of image replied: %s\n", response.toString().c_str()));
        
        RTF_TEST_REPORT("Loading template image");
        cmd.clear(); response.clear();
        cmd.addString("templateMatch");
        cmd.addString("wally.png");
        cmd.addInt(5);
        rpcPort.write(cmd,response);
        
        RTF_TEST_REPORT(Asserter::format("Staring template traking replied: %s\n", response.toString().c_str()));
        
        RTF_TEST_REPORT("Check position");
        cmd.clear(); response.clear();
        cmd.addString("getLocation");
        rpcPort.write(cmd,response);
        
        
        RTF_TEST_REPORT(Asserter::format("Check position replied: %s\n", response.toString().c_str()));
    
        double x = 90.0;
        double y = 942.0;
        
        double responseX = response.get(0).asList()->get(0).asDouble();
        double responseY = response.get(0).asList()->get(1).asDouble();
        
        RTF_TEST_REPORT(Asserter::format("RESPONSE IS : %lf %lf\n", responseX, responseY));
        
        bool xOK = false;
        bool yOK = false;
        
        if ( (responseX - x) < 10.0 && (responseX - x) >= 0.0)
            xOK = true;
        if ( (responseY - y) < 10.0 && (responseY - y) >= 0.0)
            yOK = true;
        
        double diffx = responseX - x;
        double diffy = responseY - y;
        
        RTF_TEST_REPORT(Asserter::format("DIFF IS : %lf %lf\n", diffx, diffy));
        
        RTF_TEST_CHECK( xOK && yOK, "Checking location validity");
    }
};

PREPARE_PLUGIN(TestTutorialFindingWally)
