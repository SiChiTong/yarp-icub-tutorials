/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <string>
#include <sstream>
#include <vector>

#include <rtf/yarp/YarpTestCase.h>
#include <rtf/dll/Plugin.h>

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

using namespace RTF;
using namespace yarp::os;
using namespace yarp::sig;

struct JointError{
    unsigned index;
    double value;
    double expected;
    double error;
};

/**********************************************************************/
class TestAssignmentComputedTorque : public YarpTestCase
{
    BufferedPort<Vector> portReference;
    Vector references;
    yarpWbi::yarpWholeBodyInterface *m_robot;


public:
    /******************************************************************/
    TestAssignmentComputedTorque() :
        YarpTestCase("TestAssignmentComputedTorque"), m_robot(0)
    {
    }

    /******************************************************************/
    virtual ~TestAssignmentComputedTorque()
    {
    }
    
    /******************************************************************/
    virtual bool setup(yarp::os::Property& property)
    {
        double firstTime = Time::now();
        double maxTimeout = 60; //seconds
        while (true) {
            if (NetworkBase::exists("/computed-torque/qDes:i")) {
                RTF_TEST_REPORT("Controller found. Starting test");
                break;
            }
            if ((Time::now() - firstTime) > maxTimeout) {
                RTF_ASSERT_FAIL("Could not find controller.");
            }
            Time::delay(1);
            
        }
        
        // RTF_TEST_REPORT("Waiting for matlab");
        // Time::delay(15.0);
        // RTF_TEST_REPORT("Finished waiting");
        
        ResourceFinder rf = ResourceFinder::getResourceFinderSingleton();

        Property wbiProperties;
        std::string wbiConfFile = property.check("wbi_config_file", Value("yarpWholeBodyInterface.ini"), "Checking wbi configuration file").asString();

        if (!wbiProperties.fromConfigFile(rf.findFile(wbiConfFile))) {
            RTF_ASSERT_FAIL("Not possible to load WBI properties from file.");
            return false;
        }
        wbiProperties.fromString(property.toString(), false);

        //retrieve the joint list
        std::string wbiList = property.check("wbi_list", Value("ROBOT_TORQUE_CONTROL_JOINTS_WITHOUT_PRONOSUP"), "Looking for wbi list").asString();

        wbi::IDList iCubMainJoints;
        if (!yarpWbi::loadIdListFromConfig(wbiList, wbiProperties, iCubMainJoints)) {
            RTF_ASSERT_FAIL("Cannot find joint list");
            return false;
        }

        unsigned actuatedDOFs = iCubMainJoints.size();
        RTF_TEST_REPORT(Asserter::format("DOFS: %d", actuatedDOFs));

        //create an instance of wbi
        m_robot = new yarpWbi::yarpWholeBodyInterface("TestAssignmentComputedTorque", wbiProperties);
        if (!m_robot) {
            RTF_ASSERT_FAIL("Could not create wbi object.");
            return false;
        }

        m_robot->addJoints(iCubMainJoints);
        if (!m_robot->init()) {
            RTF_ASSERT_FAIL("Could not initialize wbi object.");
            return false;
        }
        
        if (!portReference.open("/TestAssignmentComputedTorque/qDes:o")) {
            RTF_ASSERT_FAIL("Could not open reference port.");
            return false;
        }
        
        references.resize(actuatedDOFs, 0.0);
        // get initial configuration
        m_robot->getEstimates(wbi::ESTIMATE_JOINT_POS, references.data());
        // as refernce sum 40 degs to the first joint of the torso
        references(0) += ((40.0 * M_PI) / 180.0);

        RTF_ASSERT_FAIL_IF(Network::connect("/TestAssignmentComputedTorque/qDes:o", "/computed-torque/qDes:i"), "Failed to connect ports");
        return true;
    }

    /******************************************************************/
    virtual void tearDown()
    {
        RTF_TEST_REPORT("Closing Ports");
        portReference.close();
        m_robot->close();
    }
    
    /******************************************************************/

    /******************************************************************/
    virtual void run()
    {
        RTF_ASSERT_FAIL_IF(m_robot, "WBI object is null. Test internal failure");
        
        // sending references to the port
        Vector& outReference = portReference.prepare();
        outReference = references;
        portReference.write();

        RTF_TEST_REPORT("Waiting the controller to adapt to the new reference");        
        Time::delay(10.0);

        Vector currentConfigurationVector(references.size(), 0.0);
     
        RTF_ASSERT_ERROR_IF(m_robot->getEstimates(wbi::ESTIMATE_JOINT_POS, currentConfigurationVector.data()), "Failed to retrieve robot configuration");
        
        double maxJointError = 5.0 * M_PI / 180.0;
        
        using namespace Eigen;
        Map<VectorXd> currentConfiguration(currentConfigurationVector.data(), currentConfigurationVector.size());
        Map<VectorXd> referenceConfiguration(references.data(), references.size());
        
        VectorXd jointsError = (currentConfiguration - referenceConfiguration).cwiseAbs(); //.array().abs();

        std::vector<JointError> errors;
        
        for (unsigned i = 0; i < jointsError.size(); ++i) {
            if (jointsError(i) > maxJointError) {
                JointError error;
                error.index = i;
                error.value = currentConfiguration(i);
                error.expected = referenceConfiguration(i);
                error.error = jointsError(i);
                errors.push_back(error);
            }
        }

        for (std::vector<JointError>::const_iterator it = errors.begin();
            it != errors.end(); ++it) {
                RTF_TEST_REPORT(Asserter::format("Joint[%d]. Expected %lf - Actual %lf [rad]",
                                                 it->index, it->expected, it->value));
            }

        RTF_ASSERT_ERROR_IF(errors.empty(), "Error in tracking reference");

    }
};

PREPARE_PLUGIN(TestAssignmentComputedTorque)
