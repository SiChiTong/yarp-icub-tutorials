#include <Module.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

double Module::getPeriod () { return 0.01; }

bool Module::updateModule ()
{
    // Implement the Computed Torque controller
    wbi::Frame w_H_b; //identity + zero vector

    //read state
    m_robot->getEstimates(wbi::ESTIMATE_JOINT_POS, positions.data());
    m_robot->getEstimates(wbi::ESTIMATE_JOINT_VEL, velocities.data());
   // m_robot->computeGravityBiasForces(positions.data(), w_H_b, grav.data(), gravityCompensation.data());
    m_robot->computeGeneralizedBiasForces(positions.data(), w_H_b, velocities.data(),
                                        baseZeroDofs.data(), grav.data(), gravityCompensation.data());


    M_q.resize(actuatedDOFs+6, actuatedDOFs+6);
    
    m_robot->computeMassMatrix(positions.data(), w_H_b, M_q.data());
    /*m_robot->computeCentroidalMomentum(positions.data(), w_H_b, zeroDofs.data(), baseZeroDofs.data(), C_q.data());*/

                                        // std::cout << gravityCompensation.toString() <<  "\n";
    //compute control
    //std::cout << "calculated coefs" << std::endl;
    if (referencePort.dataStatus())
    {
        referencePort.lock();
        referencePositions = referencePort.get();
        yInfo() << "got reference: " << referencePositions.toString();
        yInfo() << "position: " << positions.toString();
        referencePort.unlock();
    }


    yarp::sig::Matrix M_q_d = eye(actuatedDOFs);
    yarp::sig::Matrix M_q_p = eye(actuatedDOFs);

    yarp::sig::Matrix M_q_redux = eye(actuatedDOFs);

    for (int i = 0; i < actuatedDOFs; i++)
    {
        M_q_d(i,i) = -1 * kd(i+6);
        M_q_p(i,i) = -1 * kp(i+6);
    }

    //std::cout << "kp and kd calculated " << std::endl;

    for (int i = 0; i < actuatedDOFs; i++)
    {
        for (int j = 0; j < actuatedDOFs; j++)
        {
            M_q_redux(i,j) = M_q(i+6, j+6);
        }
    }
    //M_q_redux = M_q.submatrix(6, 6+actuatedDOFs, 6, 6+actuatedDOFs);

    //std::cout << "computed aux vectors" << std::endl;

    yarp::sig::Vector gravityShort;

    gravityShort.resize(actuatedDOFs,0.0);

    //gravityShort = gravityCompensation.subVector(6,6+actuatedDOFs); 
    for (int i = 0; i < gravityShort.size(); i++)
    {
        gravityShort(i) = gravityCompensation(i+6);
    }
/*
    std::cout << "size of gravity now" << gravityShort.size() << std::endl;
    std::cout << "size of torques" << torques.size() << std::endl;
    std::cout << "size of positions" << positions.size() << std::endl;
    std::cout << "size of referencePositions" << referencePositions.size() << std::endl;
    std::cout << "size of M_q_p" << M_q_p.rows() << std::endl;
    std::cout << "size of M_q_d" << M_q_d.rows() << std::endl;
    std::cout << "size of M_q_redux" << M_q_redux.rows() << std::endl;
    std::cout << "size of velocities" << velocities.size() << std::endl;

    yarp::sig::Vector error = positions - referencePositions;

    std::cout << "size of error" << error.size() << std::endl;

    error = M_q_p * error;

    std::cout << "size of error * M_q_p" << error.size() << std::endl;

    error = M_q_d * velocities;
    
    std::cout << "size of velocities * M_q_d" << error.size() << std::endl;

    error = M_q_redux * error;
    
    std::cout << "size of velocities * M_q_d * M_q" << error.size() << std::endl;
*/

    torques = M_q_redux * (M_q_p * (positions - referencePositions) + M_q_d * velocities) + gravityShort;

    
    //std::cout << "computed torque" << std::endl;


    m_robot->setControlReference(torques.data());
    
    return true;
}

bool Module::configure (yarp::os::ResourceFinder &rf)
{
    using namespace yarp::os;
    using namespace yarp::sig;

    referencePort.open("/computed-torque/qDes:i");
    referencePort.useCallback();

    Property wbiProperties;
    std::string wbiConfFile = rf.check("wbi_config_file", Value("yarpWholeBodyInterface.ini"), "Checking wbi configuration file").asString();

    if (!wbiProperties.fromConfigFile(rf.findFile(wbiConfFile))) {
        std::cout << "Not possible to load WBI properties from file.\n";
        return false;
    }
    wbiProperties.fromString(rf.toString(), false);

    //retrieve the joint list
    std::string wbiList = rf.check("wbi_list", Value("ROBOT_TORQUE_CONTROL_JOINTS_WITHOUT_PRONOSUP"), "Looking for wbi list").asString();

    wbi::IDList iCubMainJoints;
    if (!yarpWbi::loadIdListFromConfig(wbiList, wbiProperties, iCubMainJoints)) {
        std::cout << "Cannot find joint list\n";
        return false;
    }

    actuatedDOFs = iCubMainJoints.size();

    //create an instance of wbi
    m_robot = new yarpWbi::yarpWholeBodyInterface("computed_torque", wbiProperties);
    if (!m_robot) {
        std::cout << "Could not create wbi object.\n";
        return false;
    }

    m_robot->addJoints(iCubMainJoints);
    if (!m_robot->init()) {
        std::cout << "Could not initialize wbi object.\n";
        return false;
    }
    
    //Creating and configuring a wbi instance.
    //We use the yarp implementation.

    //Load information about the joints and global properties for the wbi
    //this file also contains references to the urdf model
    
    std::cout << "Number of DOFs: " << actuatedDOFs << "\n";


    //Any further initialization

    positions.resize(actuatedDOFs, 0.0);
    velocities.resize(actuatedDOFs, 0.0);
    gravityCompensation.resize(actuatedDOFs + 6, 0.0);
    referencePositions.resize(actuatedDOFs, 0.0);
    
    m_robot->getEstimates(wbi::ESTIMATE_JOINT_POS, referencePositions.data());
    
    int index = -1;
    if (!m_robot->getJointList().idToIndex("l_shoulder_pitch", index)) {
        std::cout << "Could not retrieve index of l_shoulder_pitch.\n";
        return false;
    }
    referencePositions(index) += 1.5;
    
    if (!m_robot->getJointList().idToIndex("l_elbow", index)) {
        std::cout << "Could not retrieve index of l_elbow.\n";
        return false;
    }
    referencePositions(index) += 1.5;
    
    if (!m_robot->getJointList().idToIndex("r_shoulder_pitch", index)) {
        std::cout << "Could not retrieve index of r_shoulder_pitch.\n";
        return false;
    }
    referencePositions(index) += 1.5;
    
    if (!m_robot->getJointList().idToIndex("r_elbow", index)) {
        std::cout << "Could not retrieve index of r_elbow.\n";
        return false;
    }
    referencePositions(index) += 1.5;

    std::cout << "initial position configured" << std::endl;
    
    //write
    error.resize(actuatedDOFs, 0.0);

    kp.resize(actuatedDOFs, 16.0);
    kd.resize(actuatedDOFs, 8.0);
    
    torques.resize(actuatedDOFs, 0.0);
    
    zeroDofs.resize(actuatedDOFs, 0.0);
    baseZeroDofs.resize(6, 0.0);
    
    grav.resize(3, 0.0);
    grav(2) = -9.81;
    
    m_robot->setControlMode(wbi::CTRL_MODE_TORQUE);


    std::cout << "got reference: " << referencePositions.toString() << "\n";
    
    std::cout << "robot mode set" << std::endl;
    
    
    return true;
}

bool Module::close ()
{
    referencePort.close();
    m_robot->setControlMode(wbi::CTRL_MODE_POS);
    //cleanup stuff
    m_robot->close();
    delete m_robot;
    m_robot = 0;
    return true;
}

