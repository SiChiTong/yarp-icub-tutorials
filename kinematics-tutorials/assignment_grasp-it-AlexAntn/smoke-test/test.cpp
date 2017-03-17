/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <string>

#include <rtf/yarp/YarpTestCase.h>
#include <rtf/dll/Plugin.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

using namespace std;
using namespace RTF;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

/**********************************************************************/
class TestAssignmentGraspIt : public YarpTestCase,
                              public PortReader
{
    RpcClient portBall;
    RpcClient portGI;
    Port      portHandR;
    Port      portHandL;

    Vector ballPosRobFrame;
    bool hit;

    /******************************************************************/
    Vector getBallPosition()
    {
        Bottle cmd,reply;
        cmd.addString("world");
        cmd.addString("get");
        cmd.addString("ball");
        RTF_ASSERT_ERROR_IF(portBall.write(cmd,reply),"Unable to talk to world");
        RTF_ASSERT_ERROR_IF(reply.size()>=3,"Invalid reply from world");

        Vector pos(3);
        pos[0]=reply.get(0).asDouble();
        pos[1]=reply.get(1).asDouble();
        pos[2]=reply.get(2).asDouble();

        return pos;
    }

    /******************************************************************/
    bool setBallPosition(const Vector& pos)
    {
        if (pos.length()>=3)
        {
            Bottle cmd,reply;
            cmd.addString("world");
            cmd.addString("set");
            cmd.addString("ball");
            cmd.addDouble(pos[0]);
            cmd.addDouble(pos[1]);
            cmd.addDouble(pos[2]);
            RTF_ASSERT_ERROR_IF(portBall.write(cmd,reply),"Unable to talk to world");
            return true;
        }
        else
            return false;
    }

public:
    /******************************************************************/
    TestAssignmentGraspIt() :
        YarpTestCase("TestAssignmentGraspIt"),
        hit(false)
    {
    }

    /******************************************************************/
    virtual ~TestAssignmentGraspIt()
    {
    }

    /******************************************************************/
    virtual bool setup(yarp::os::Property& property)
    {
        string robot=property.check("robot",Value("icubSim")).asString();
        float rpcTmo=(float)property.check("rpc-timeout",Value(240.0)).asDouble();

        string robotPortRName("/"+robot+"/cartesianController/right_arm/state:o");
        string robotPortLName("/"+robot+"/cartesianController/left_arm/state:o");

        string portBallName("/"+getName()+"/ball:rpc");
        string portGIName("/"+getName()+"/gi:rpc");
        string portHandRName("/"+getName()+"/hand/right:i");
        string portHandLName("/"+getName()+"/hand/left:i");

        portBall.open(portBallName);
        portGI.open(portGIName);
        portHandR.open(portHandRName);
        portHandL.open(portHandLName);

        RTF_TEST_REPORT(Asserter::format("Set rpc timeout = %g [s]",rpcTmo));
        portBall.asPort().setTimeout(rpcTmo);
        portGI.asPort().setTimeout(rpcTmo);        

        Time::delay(5.0);

        RTF_TEST_REPORT("Connecting Ports");
        RTF_ASSERT_ERROR_IF(Network::connect(portBallName,"/icubSim/world"),
                            "Unable to connect to /icubSim/world");
        RTF_ASSERT_ERROR_IF(Network::connect(portGIName,"/service"),
                            "Unable to connect to /service");
        RTF_ASSERT_ERROR_IF(Network::connect(robotPortRName,portHandRName),
                            Asserter::format("Unable to connect to %s",
                                             robotPortRName.c_str()));
        RTF_ASSERT_ERROR_IF(Network::connect(robotPortLName,portHandLName),
                            Asserter::format("Unable to connect to %s",
                                             robotPortLName.c_str()));

        Rand::init();

        return true;
    }

    /******************************************************************/
    virtual void tearDown()
    {
        RTF_TEST_REPORT("Closing Ports");
        portBall.close();
        portGI.close();
        portHandL.close();
        portHandR.close();
    }

    /******************************************************************/
    virtual bool read(ConnectionReader& reader)
    {
        if (!hit)
        {
            Bottle data;
            data.read(reader);

            Vector x(3);
            x[0]=data.get(0).asDouble();
            x[1]=data.get(1).asDouble();
            x[2]=data.get(2).asDouble();

            double d=norm(ballPosRobFrame-x);
            if (d<0.15)
            {
                RTF_TEST_REPORT(Asserter::format("Great! We're at %g [m] from the ball",d));
                hit=true;
            }
        }

        return true;
    }

    /******************************************************************/
    virtual void run()
    {
        Time::delay(5.0);
        
        RTF_TEST_REPORT("Retrieving initial ball position");
        Vector initialBallPos=getBallPosition();
        RTF_TEST_REPORT(Asserter::format("initial ball position = (%s) [m]",
                                         initialBallPos.toString(3,3).c_str()));

        Vector min(3,0.0),max(3,0.0);
        min[0]= 0.0;  max[0]=0.4;   // x-axis
        min[1]= 0.0;  max[1]=0.0;   // y-axis
        min[2]=-0.02; max[2]=0.02;  // z-axis

        RTF_TEST_REPORT("Setting new initial ball position");
        initialBallPos+=Rand::vector(min,max);
        setBallPosition(initialBallPos);
        RTF_TEST_REPORT(Asserter::format("new ball position = (%s) [m]",
                                         initialBallPos.toString(3,3).c_str()));

        // compute ball position in robot's root frame
        Matrix T=zeros(4,4);
        T(0,1)=-1.0;
        T(1,2)=1.0;  T(1,3)=0.5976;
        T(2,0)=-1.0; T(2,3)=-0.026;
        T(3,3)=1.0;
        Vector initBallPosHomog=initialBallPos;
        initBallPosHomog.push_back(1.0);
        ballPosRobFrame=SE3inv(T)*initBallPosHomog;
        ballPosRobFrame.pop_back();

        Bottle cmd,reply;
        cmd.addString("look_down");
        RTF_ASSERT_ERROR_IF(portGI.write(cmd,reply),"Unable to talk to GI");
        RTF_ASSERT_ERROR_IF(reply.get(0).asString()=="ack","Unable to look_down");
        cmd.clear(); reply.clear();

        RTF_TEST_REPORT("Proximity check is now active");
        portHandR.setReader(*this);
        portHandL.setReader(*this);

        cmd.addString("grasp_it");
        RTF_ASSERT_ERROR_IF(portGI.write(cmd,reply),"Unable to talk to GI");
        RTF_ASSERT_ERROR_IF(reply.get(0).asString()=="ack","Unable to grasp_it");
        cmd.clear(); reply.clear();

        RTF_TEST_REPORT("Retrieving final ball position");
        Vector finalBallPos=getBallPosition();
        RTF_TEST_REPORT(Asserter::format("final ball position = (%s) [m]",
                                         finalBallPos.toString(3,3).c_str()));

        double d=norm(finalBallPos-initialBallPos);
        RTF_TEST_CHECK(hit,"We grasped the ball!");
        RTF_TEST_CHECK(d>0.01,Asserter::format("Ball has moved for at least %g [m]!",d));
    }
};

PREPARE_PLUGIN(TestAssignmentGraspIt)
