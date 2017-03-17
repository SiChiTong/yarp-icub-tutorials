/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <string>

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

#define TEST_MESSAGE          "hello"
#define TEST_MESSAGE_CODED    "(%,,/"

class BottleReader : public BufferedPort<Bottle> {
    std::string msg;
    Semaphore sem;

public:
    BottleReader(): msg("timeout"), sem(0) { }

  virtual void onRead(Bottle &bt) {
        if(bt.size() && bt.get(0).isString())
            msg = bt.get(0).asString();
        else
            msg = "wrong data";
        sem.post();
    }

    const std::string getMessage() {
        sem.waitWithTimeout(5.0);
        return msg;
    }
};

/**********************************************************************/
class TestAssignmentCoded : public YarpTestCase
{
private:
        BufferedPort<Bottle> portOut;
        BottleReader portIn;
        RpcClient rpc;
public:
    /******************************************************************/
    TestAssignmentCoded() :
        YarpTestCase("TestAssignmentCoded") { }

    /******************************************************************/
    virtual ~TestAssignmentCoded() {
    }

    /******************************************************************/
    virtual bool setup(yarp::os::Property& property) {

        RTF_ASSERT_ERROR_IF(portOut.open("/TestAssignmentCoded/o"), "Cannot open the output port");
        RTF_ASSERT_ERROR_IF(portIn.open("/TestAssignmentCoded/i"), "Cannot open the output port");
        RTF_ASSERT_ERROR_IF(rpc.open("/TestAssignmentCoded/rpc"), "Cannot open the output port");

        RTF_ASSERT_ERROR_IF(NetworkBase::connect("/coder/Codec/out", portIn.getName()),
                            "Cannot connect to /coder/Codec/out");
        RTF_ASSERT_ERROR_IF(NetworkBase::connect(portOut.getName(), "/coder/Codec/in"),
                            "Cannot connect to /coder/Codec/in");
        RTF_ASSERT_ERROR_IF(NetworkBase::connect(rpc.getName(), "/coder/Codec/rpc"),
                            "Cannot connect to /coder/Codec/rpc");

        portIn.useCallback();
        return true;
    }

    /******************************************************************/
    virtual void tearDown() {
        RTF_TEST_REPORT("Tearing down TestAssignmentCoded");
        NetworkBase::disconnect("/coder/Codec/in", portIn.getName());
        NetworkBase::disconnect(portOut.getName(), "/coder/Codec/out");
        NetworkBase::disconnect(rpc.getName(), "/coder/Codec/rpc");
        portIn.interrupt();
        portIn.close();
        portOut.close();
        rpc.close();
    }

    /******************************************************************/
    virtual void run()
    {
        Bottle cmd, reply;

        RTF_TEST_REPORT("Setting coder mode");
        // setting decoder mode
        cmd.clear();
        reply.clear();
        cmd.addString("set_mode");
        cmd.addString("coder");
        rpc.write(cmd, reply);
        RTF_ASSERT_FAIL_IF(reply.toString()=="[ok]", "setting decoder mode using set_mode()");

        RTF_TEST_REPORT("Testing coded");
        // setting codec
        cmd.clear();
        reply.clear();
        cmd.addString("set_codec");
        cmd.addInt(64);
        rpc.write(cmd, reply);
        RTF_TEST_CHECK(reply.toString()=="[ok]", "setting codec using set_codec()");
        // checking message
        RTF_TEST_REPORT(Asserter::format("Writing test message %s", TEST_MESSAGE));
        Bottle& msg = portOut.prepare();
        msg.clear();
        msg.addString(TEST_MESSAGE);
        portOut.write(true);
        RTF_TEST_REPORT("Reading decoded test message");
        string str = portIn.getMessage();
        RTF_TEST_CHECK(str == string(TEST_MESSAGE_CODED),
                       Asserter::format("encoded message (%s == %s)", str.c_str(), TEST_MESSAGE_CODED) );

        RTF_TEST_REPORT("Setting decoder mode");
        // setting decoder mode
        cmd.clear();
        reply.clear();
        cmd.addString("set_mode");
        cmd.addString("decoder");
        rpc.write(cmd, reply);
        RTF_ASSERT_FAIL_IF(reply.toString()=="[ok]", "setting decoder mode using set_mode()");
        // checking message
        RTF_TEST_REPORT(Asserter::format("Writing test message %s", TEST_MESSAGE_CODED));
        msg = portOut.prepare();
        msg.clear();
        msg.addString(TEST_MESSAGE_CODED);
        portOut.write(true);
        RTF_TEST_REPORT("Reading coded test message");
        str = portIn.getMessage();
        RTF_TEST_CHECK(str == string(TEST_MESSAGE),
                       Asserter::format("encoded message (%s == %s)", str.c_str(), TEST_MESSAGE) );


    }

    /******************************************************************/
    virtual void interrupt() {
        portIn.interrupt();
    }
};

PREPARE_PLUGIN(TestAssignmentCoded)
