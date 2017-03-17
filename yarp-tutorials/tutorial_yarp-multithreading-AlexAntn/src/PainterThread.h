/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <string>
#include <yarp/os/Thread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Image.h>



class PainterThread : public yarp::os::Thread
{
public:

    PainterThread();
    virtual ~PainterThread();

    /**
     * @brief run thread main function
     */
    virtual void run();

    /**
     * @brief paint sets the variable and signal the thread function
     * to proccess
     * @param image
     * @param x
     * @param y
     * @param w
     * @param h
     */
    void paint(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image,
               int x, int y, int w, int h);

    /**
     * @brief wait suspends the calling thread until the current thread
     * finishes proccessing the data
     */
    void wait();

    /**
     * @brief interrupt any ongoing proccess/waiting
     */
    void interrupt();

private:
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* image;
    int x;
    int y;
    int w;
    int h;
    bool interrupted;
    yarp::os::Semaphore semStart;
    yarp::os::Semaphore semDone;
};

