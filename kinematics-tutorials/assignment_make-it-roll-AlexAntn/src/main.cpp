// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/***************************************************/
class CtrlModule: public RFModule
{
protected:
    PolyDriver drvArm, drvGaze;
    ICartesianControl *iarm;
    IGazeControl      *igaze;

    BufferedPort<ImageOf<PixelRgb> > imgLPortIn,imgRPortIn;
    BufferedPort<ImageOf<PixelRgb> > imgLPortOut,imgRPortOut;
    RpcServer rpcPort;

    Mutex mutex;
    Vector cogL,cogR;
    bool okL,okR;

    /***************************************************/
    bool getCOG(ImageOf<PixelRgb> &img, Vector &cog)
    {
        int xMean=0;
        int yMean=0;
        int ct=0;

        for (int x=0; x<img.width(); x++)
        {
            for (int y=0; y<img.height(); y++)
            {
                PixelRgb &pixel=img.pixel(x,y);
                if ((pixel.b>5.0*pixel.r) && (pixel.b>5.0*pixel.g))
                {
                    xMean+=x;
                    yMean+=y;
                    ct++;
                }
            }
        }

        if (ct>0)
        {
            cog.resize(2);
            cog[0]=xMean/ct;
            cog[1]=yMean/ct;
            return true;
        }
        else
            return false;
    }

    /***************************************************/
    Vector retrieveTarget3D(const Vector &cogL, const Vector &cogR)
    {
        Vector targetpoint;
        igaze->triangulate3DPoint(cogL, cogR, targetpoint);
        return targetpoint;
    }

    /***************************************************/
    void fixate(const Vector &x)
    {
        igaze->lookAtFixationPoint(x);
        //igaze->setTrackingMode(true);    // We have decided not to track it!!
        igaze->waitMotionDone();
    }

    /***************************************************/
    Vector computeHandOrientation()
    {
        Matrix Rot(3,3);
        Rot(0,0)=-1.0; Rot(0,1)= 0.0; Rot(0,2)= 0.0; 
        Rot(1,0)= 0.0; Rot(1,1)= 0.0; Rot(1,2)=-1.0; 
        Rot(2,0)= 0.0; Rot(2,1)=-1.0; Rot(2,2)= 0.0; 
        return dcm2axis(Rot);
    }

    /***************************************************/
    void approachTargetWithHand(const Vector &x, const Vector &o)
    {
        Vector dof(10,1.0), dummy;
        iarm->setDOF(dof, dummy);
        Vector approach = x;
        approach[1]+=0.1; // 10cm away
        iarm->goToPoseSync(approach,o);
        iarm->waitMotionDone();
    }

    /***************************************************/
    void roll(const Vector &x, const Vector &o)
    {
        iarm->setTrajTime(0.4);
        Vector target = x;
        target[1]-=0.1; // 10cm away
        iarm->goToPoseSync(target,o);
        iarm->waitMotionDone();
    }

    /***************************************************/
    void look_down()
    {
        /*Vector xd(3);
        xd[0] = -0.5;
        xd[1] = 0.0;
        xd[2] = -0.5;*/
        Vector ang(3,0.0);
        ang[1] = -40.0;  // elevation (deg)
        igaze->lookAtAbsAngles(ang);
        igaze->waitMotionDone();
    }

    /***************************************************/
    void make_it_roll(const Vector &cogL, const Vector &cogR)
    {
        yInfo()<<"detected cogs = ("<<cogL.toString(0,0)<<") ("<<cogR.toString(0,0)<<")";

        Vector x=retrieveTarget3D(cogL,cogR);
        yInfo()<<"retrieved 3D point = ("<<x.toString(3,3)<<")";

        fixate(x);
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        Vector o=computeHandOrientation();
        yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

        approachTargetWithHand(x,o);
        yInfo()<<"approached";

        roll(x,o);
        yInfo()<<"roll!";
    }

    /***************************************************/
    void home()
    {
        Vector xd(3);
        xd[0] = -0.3;
        xd[1] = 0.2;
        xd[2] = 0.0;
        iarm->goToPositionSync(xd);
        iarm->waitMotionDone();
    }

public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        Property optArm;
        optArm.put("device","cartesiancontrollerclient");
        optArm.put("remote","/icubSim/cartesianController/right_arm");
        optArm.put("local","/cartesian_client/right_arm");

        // let's give the controller some time to warm up
        bool ok=false;
        double t0=Time::now();
        while (Time::now()-t0<10.0)
        {
            // this might fail if controller
            // is not connected to solver yet
            if (drvArm.open(optArm))
            {
                ok=true;
                break;
            }

            Time::delay(1.0);
        }

        if (!ok)
        {
            yError()<<"Unable to open the Cartesian Controller";
            return false;
        }

        Property optGaze;
        optGaze.put("device","gazecontrollerclient");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/gaze_client");


        if (!drvGaze.open(optGaze))
        {
            drvArm.close();
            yError()<<"Unable to open the Gaze Controller";
            return false;
        }

        drvArm.view(iarm);
        drvGaze.view(igaze);

        imgLPortIn.open("/imgL:i");
        imgRPortIn.open("/imgR:i");

        imgLPortOut.open("/imgL:o");
        imgRPortOut.open("/imgR:o");

        rpcPort.open("/service");
        attach(rpcPort);

        return true;
    }

    /***************************************************/
    bool interruptModule()
    {
        imgLPortIn.interrupt();
        imgRPortIn.interrupt();
        return true;
    }

    /***************************************************/
    bool close()
    {
        drvArm.close();
        drvGaze.close();
        imgLPortIn.close();
        imgRPortIn.close();
        imgLPortOut.close();
        imgRPortOut.close();
        rpcPort.close();
        return true;
    }

    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();
        if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- look_down");
            reply.addString("- make_it_roll");
            reply.addString("- home");
            reply.addString("- quit");
        }
        else if (cmd=="look_down")
        {
            look_down();
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Yep! I'm looking down now!");
        }
        else if (cmd=="make_it_roll")
        {
            mutex.lock();
            bool go=okL && okR;
            Vector cogL = this->cogL;
            Vector cogR = this->cogR;
            mutex.unlock();

            if (go)
            {
                make_it_roll(cogL,cogR);
                // we assume the robot is not moving now
                reply.addString("ack");
                reply.addString("Yeah! I've made it roll like a charm!");
            }
            else
            {
                reply.addString("nack");
                reply.addString("I don't see any object!");
            }
        }
        else if (cmd=="home")
        {
            home();
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("I've got the hard work done! Gone home.");
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);

        return true;
    }

    /***************************************************/
    double getPeriod()
    {
        return 0.0;     // sync upon incoming images
    }

    /***************************************************/
    bool updateModule()
    {
        // get fresh images
        ImageOf<PixelRgb> *imgL=imgLPortIn.read();
        ImageOf<PixelRgb> *imgR=imgRPortIn.read();

        // interrupt sequence detected
        if ((imgL==NULL) || (imgR==NULL))
            return false;

        // compute the center-of-mass of pixels of our color
        mutex.lock();
        okL=getCOG(*imgL,cogL);
        okR=getCOG(*imgR,cogR);
        mutex.unlock();

        PixelRgb color;
        color.r=255; color.g=0; color.b=0;

        if (okL)
            draw::addCircle(*imgL,color,(int)cogL[0],(int)cogL[1],5);

        if (okR)
            draw::addCircle(*imgR,color,(int)cogR[0],(int)cogR[1],5);

        imgLPortOut.prepare()=*imgL;
        imgRPortOut.prepare()=*imgR;

        imgLPortOut.write();
        imgRPortOut.write();

        return true;
    }
};


/***************************************************/
int main()
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    CtrlModule mod;
    ResourceFinder rf;
    return mod.runModule(rf);
}
