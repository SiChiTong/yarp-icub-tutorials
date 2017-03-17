/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/
#include <string>

#include <rtf/yarp/YarpTestCase.h>
#include <rtf/dll/Plugin.h>

#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>

using namespace std;
using namespace RTF;

/**********************************************************************/
class TestAssignmentEventOpticalFlow : public YarpTestCase
{

private:

    yarp::os::RpcClient playercontroller;


public:
    /******************************************************************/
    TestAssignmentEventOpticalFlow() :
        YarpTestCase("TestAssignmentEventOpticalFlow")
    {
    }

    /******************************************************************/
    virtual ~TestAssignmentEventOpticalFlow()
    {
    }

    /******************************************************************/
    virtual bool setup(yarp::os::Property& property)
    {

        //we need to load the data file into yarpdataplayer
        std::string cntlportname = "/playercontroller/rpc";

        RTF_ASSERT_ERROR_IF(playercontroller.open(cntlportname),
                            "Could not open RPC to yarpdataplayer");

        RTF_ASSERT_ERROR_IF(yarp::os::Network::connect(cntlportname, "/yarpdataplayer/rpc:i"),
                            "Could not connect RPC to yarpdataplayer");

        RTF_TEST_REPORT("Ports successfully open and connected");

        return true;
    }

    /******************************************************************/
    virtual void tearDown()
    {
        RTF_TEST_REPORT("Closing Clients");
        playercontroller.close();
    }

    /******************************************************************/
    virtual void run()
    {

        //play the dataset
        yarp::os::Bottle cmd, reply;
        cmd.addString("play");
        playercontroller.write(cmd, reply);
        RTF_ASSERT_ERROR_IF(reply.get(0).asString() == "ok", "Did not successfully play the dataset");

        yarp::os::Time::delay(15);

        cmd.clear();
        cmd.addString("stop");
        playercontroller.write(cmd, reply);
        RTF_ASSERT_ERROR_IF(reply.get(0).asString() == "ok", "Did not successfully stop the dataset");

    }
};

PREPARE_PLUGIN(TestAssignmentEventOpticalFlow)
