/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
 * email:  vadim.tikhanoff@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Image.h>
#include <yarp/os/RpcClient.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "closestBlob_IDL.h"

using namespace cv;

/********************************************************/
class Processing : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >
{
    std::string moduleName;

    yarp::os::RpcServer handlerPort;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   inPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   outPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   cropOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle>  targetPort;

    yarp::os::RpcClient rpc;

    int32_t lowBound;
    int32_t highBound;

    int dilate_niter;
    int erode_niter;
    int gaussian_size;

public:
    /********************************************************/

    Processing( const std::string &moduleName )
    {
        this->moduleName = moduleName;
    }

    /********************************************************/
    ~Processing()
    {

    };

    /********************************************************/
    bool open(){

        this->useCallback();

        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::open( "/" + moduleName + "/disparity:i" );
        inPort.open("/"+ moduleName + "/image:i");
        outPort.open("/"+ moduleName + "/image:o");
        cropOutPort.open("/" + moduleName + "/crop:o");
        targetPort.open("/"+ moduleName + "/target:o");


        lowBound = 70;
        
        highBound = 255;

        dilate_niter = 4;
        erode_niter = 0;
        gaussian_size = 9;

        return true;
    }

    /********************************************************/
    void close()
    {
        inPort.close();
        outPort.close();
        targetPort.close();
        cropOutPort.close();
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::close();
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::interrupt();
    }

    /********************************************************/
    void onRead( yarp::sig::ImageOf<yarp::sig::PixelMono> &dispImage )
    {
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImage  = outPort.prepare();
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &cropOutImage  = cropOutPort.prepare();
        yarp::os::Bottle &outTargets = targetPort.prepare();
        
        yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImage = inPort.read();

        outImage.resize(dispImage.width(), dispImage.height());
        cropOutImage.resize(dispImage.width(), dispImage.height());
        
        outImage.zero();
        cropOutImage.zero();
        
        cv::Mat inColour_cv = cv::cvarrToMat((IplImage *)inImage->getIplImage());  // prepare the image ports and targets
        cv::Mat inDisp_cv = cv::cvarrToMat((IplImage *)dispImage.getIplImage());
        cv::Mat output_roi = cv::cvarrToMat((IplImage *)cropOutImage.getIplImage());
        
        
        cv::Mat disp = inDisp_cv.clone();

        
        //FILL IN THE CODE
        
        // Apply image processing techniques on the disparity image to smooth things out 

        cv::GaussianBlur(inDisp_cv, inDisp_cv, cv::Size(gaussian_size, gaussian_size), 2, 2);

        cv::dilate(inDisp_cv, inDisp_cv, cv::Mat(), cv::Point(-1,-1), dilate_niter, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

        // !!!! cv::inRange(disp, lowBound, highBound, redBallOnly);
    
        // Apply some threshold on the image to remove background:
        // have a look at cv::threshold function

        cv::threshold(inDisp_cv, disp, lowBound, highBound, cv::THRESH_TOZERO); 	

        // Find the max value and its position

        cv::Mat result;
        double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;

        minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

        //....

        //Find the contour of the closest objects with moments and mass center
        //

        std::vector<std::vector<Point> > contours;
        std::vector<Vec4i> hierarchy;
       
        /// Find contours
        findContours( disp, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

        yInfo() << "countour found";

        /// Get the moments 
        std::vector<Moments> mu(contours.size() );
        for( int i = 0; i < contours.size(); i++ )
           { mu[i] = moments( contours[i], false ); }

        yInfo() << "moments calculated";

        ///  Get the mass centers:
        std::vector<Point2f> mc( contours.size() );
        for( int i = 0; i < contours.size(); i++ )
           { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

        yInfo() << "center of mass found";

        /// Draw contours
        // Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
        for( int i = 0; i< contours.size(); i++ )
        {
           Scalar color = Scalar( 0, 255, 0 );
           drawContours( disp, contours, i, color, 2, 8, hierarchy, 0, Point() );
           circle( disp, mc[i], 4, color, -1, 8, 0 );
        }

        yInfo() << "countour drawn";

        //....

        // optional hint: you could use pointPolygonTest and the previous maxvalue location to compare with all contours found and get the actual brightest one
        
        int brightContour=0;
        for (int i = 0; i < contours.size(); i++)
        {
            if (pointPolygonTest(contours[i], maxLoc, false) > 0)
            {
                brightContour = i;
                break;
            }
        }

        yInfo() << "Polygon test passed, n_contours: " << contours.size() << "brightest: " << brightContour;

        //....

        // Use the result of pointPolygonTest or your own technique as the closest contour to:
        // 1 - draw it on the disparity image
        // 2 - create a cropped image containing the rgb roi
        // 3 - fill in a yarp bottle with the bounding box

        //be aware that the expected Bottle should be a list containing:
        // (tl.x tl.y br.x br.y)
        //where tl is top left and br - bottom right
        outTargets.clear();
        cvtColor(inDisp_cv, inDisp_cv, CV_GRAY2RGB);
        if (contours.size() > 0)
        {
            drawContours(inDisp_cv, contours, brightContour, Scalar( 0, 255, 0 ), 3, 8, hierarchy, 0);
            circle(inDisp_cv, mc[brightContour], 3, Scalar(0, 255, 0), -1, 8, 0);




            yInfo() << "contour drawn";
            //Use the previous location to fill in the bottle
            double x_pos = maxLoc.x;
            double y_pos = maxLoc.y;

            cv::Rect roi = boundingRect(contours[brightContour]);

            yarp::os::Bottle &t = outTargets.addList();

            //Point topLeftRoi = roi.tl;
            //Point botRightRoi = roi.tl;
            
            t.addDouble(roi.tl().x);
            t.addDouble(roi.tl().y);
            t.addDouble(roi.br().x);
            t.addDouble(roi.br().y);

            yInfo() << "rectangle obtained";

            Mat input_roi = inColour_cv(roi);

            input_roi.copyTo(output_roi(roi));
              


            yInfo() << "rectangle copied";
        
        }
        if (outTargets.size() >0 )
            targetPort.write();          

        IplImage out = inDisp_cv;
        outImage.resize(out.width, out.height);
        cvCopy( &out, (IplImage *) outImage.getIplImage());
        outPort.write();

        IplImage crop = output_roi;
        cropOutImage.resize(crop.width, crop.height);
        cvCopy( &crop, (IplImage *) cropOutImage.getIplImage());
        cropOutPort.write();
    }
};

/********************************************************/
class Module : public yarp::os::RFModule, public closestBlob_IDL
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;

    Processing                  *processing;
    friend class                processing;

    bool                        closing;

    /********************************************************/
    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

public:

    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
        this->rf=&rf;
        std::string moduleName = rf.check("name", yarp::os::Value("closest-blob"), "module name (string)").asString();
        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")).c_str());

        closing = false;

        processing = new Processing( moduleName );

        /* now start the thread to do the work */
        processing->open();

        attach(rpcPort);

        return true;
    }

    /**********************************************************/
    bool close()
    {
        processing->interrupt();
        processing->close();
        delete processing;
        return true;
    }

    /**********************************************************/
    bool quit(){
        closing = true;
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /********************************************************/
    bool updateModule()
    {
        return !closing;
    }
};

/********************************************************/
int main(int argc, char *argv[])
{
    yarp::os::Network::init();

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    Module module;
    yarp::os::ResourceFinder rf;

    rf.setVerbose();
    rf.configure(argc,argv);

    return module.runModule(rf);
}
