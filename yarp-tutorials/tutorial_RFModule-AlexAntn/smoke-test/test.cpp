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

#define TEST_MESSAGE    "Hello Tutorial"

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
class TestTutorialRFModule : public YarpTestCase
{
private:
        BufferedPort<Bottle> portOut;
        BottleReader portIn;
public:
    /******************************************************************/
    TestTutorialRFModule() :
        YarpTestCase("TestTutorialRFModule") { }
    
    /******************************************************************/
    virtual ~TestTutorialRFModule() {
    }
    
    /******************************************************************/
    virtual bool setup(yarp::os::Property& property) {       

        RTF_ASSERT_ERROR_IF(portOut.open("/TestTutorialRFModule/o"), "Cannot open the output port");
        RTF_ASSERT_ERROR_IF(portIn.open("/TestTutorialRFModule/i"), "Cannot open the output port");

        RTF_ASSERT_ERROR_IF(NetworkBase::connect("/decoder/Codec/out", portIn.getName()),
                            "Cannot connect to /decoder/Codec/out");

        RTF_ASSERT_ERROR_IF(NetworkBase::connect(portOut.getName(), "/coder/Codec/in"),
                            "Cannot connect to /coder/Codec/in");
        portIn.useCallback();
        return true;
    }
    
    /******************************************************************/
    virtual void tearDown() {
        RTF_TEST_REPORT("Tearing down TestTutorialRFModule");
        NetworkBase::disconnect("/decoder/Codec/in", portIn.getName());
        NetworkBase::disconnect(portOut.getName(), "/coder/Codec/out");
        portIn.interrupt();
        portIn.close();
        portOut.close();
    }
    
    /******************************************************************/
    virtual void run()
    {   
        RTF_TEST_REPORT(Asserter::format("Writing test message %s", TEST_MESSAGE));
        Bottle& msg = portOut.prepare();
        msg.clear();
        msg.addString(TEST_MESSAGE);
        portOut.write(true);
        RTF_TEST_REPORT("Reading decoded test message");
        string str = portIn.getMessage();
        RTF_TEST_CHECK(str == string(TEST_MESSAGE),
                       Asserter::format("encoded/decoded message (%s == %s)", str.c_str(), TEST_MESSAGE) );
    }

    /******************************************************************/
    virtual void interrupt() {
        portIn.interrupt();
    }
};

PREPARE_PLUGIN(TestTutorialRFModule)
