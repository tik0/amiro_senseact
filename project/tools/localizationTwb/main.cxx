/* itoa example */
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <iomanip>
#include <vector>
#include <iCVCDriver.h>
#include <iCVCImg.h>
#include <iCVGenApi.h>
#include <CVCError.h>
#include <iCVCUtilities.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/time.h>



//

#ifdef _WIN32
#include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#ifndef __APPLE__
#include <GL/gl.h>
#include <GL/glut.h>
#else
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#endif
#include <AR/gsub.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>
#include "object.h"




// #include "iBayerToRGB.h"

// openCV
// #include "opencv.hpp"
// #include "core/core.hpp"
// #include "core/types_c.h"
// #include "highgui/highgui.hpp"
// #include "opencv2/imgproc/imgproc.hpp"

// #include "stdafx.h"
#include <string>




#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
// 
// #include <rsb/Event.h>
// #include <rsb/Factory.h>
// #include <rsb/Handler.h>


using namespace std;
using namespace cv;
using namespace boost;

const int KEY_ESC = 27;
const int DELAY = 30;
static const std::size_t DRIVERPATHSIZE = 256;
//const string WIN_NAME = "Camera View";

/* Object Data */
char            *model_name = "Data/object_data";
ObjectData_T    *object;
int             objectnum;

//
// Camera configuration.
//

                // First get a factory instance that is used to create RSB domain
    // objects.
//     rsb::Factory& factory = rsb::getFactory();

    // Create an informer that is capable of sending events containing
    // string data on the scope "/example/informer".
//     rsb::Informer<string>::Ptr informer = factory.createInformer<string> ("/murox/roboterlocation");
#ifdef _WIN32
char                    *vconf = "Data\\WDM_camera_flipV.xml";
#else
char                    *vconf = "";
#endif

int             xsize, ysize;
int             thresh = 100;
int             count = 0;

char           *cparam_name    = "Data/camera_para.dat";
ARParam         cparam;

// Processing the camera hCamera and write the image to frame
void process(IMG hCamera,cv::Mat &frame);
// Process the frame and to multi marker tracking
void tracking(cv::Mat &frame);
// init the AR Toolkit
static void initARToolkit(int xsize,int ysize);
// Cleanup AR Toolkit
static void   cleanup(void);
// AR stuff
static int  draw(ObjectData_T *object, int objectnum);
static int  draw_object( int obj_id, double gl_para[16] );


static void initARToolkit( int xsize, int ysize ) {
    ARParam  wparam;

    //print("Image size (x,y) = (%d,%d)\n", xsize, ysize);

    /* set the initial camera parameters */
    if( arParamLoad(cparam_name, 1, &wparam) < 0 ) {
        //print("Camera parameter load error !!\n");
        exit(0);
    }
    arParamChangeSize( &wparam, xsize, ysize, &cparam );
    arInitCparam( &cparam );
    //print("*** Camera Parameter ***\n");
    arParamDisp( &cparam );

    /* load in the object data - trained markers and associated bitmap files */
    if( (object=read_ObjData(model_name, &objectnum)) == NULL ) exit(0);
    //print("Objectfile num = %d\n", objectnum);

    /* open the graphics window */
    argInit( &cparam, 1.0, 0, 0, 0, 0 );
}

/* cleanup function called when program exits */
static void cleanup(void)
{
    arVideoCapStop();
    arVideoClose();
    argCleanup();
}


IMG hCamera;
// Image from the camera
        cv::Mat frame;


static void processEverything(void){
        // wait for next image to be acquired
        // (returns immediately if unprocessed image are in the ring buffer)
        cvbres_t result = G2Wait(hCamera);
        if(result < 0)
        {
                std::cout << " Error with G2Wait";
        }
        else
        {
                process(hCamera,frame);
                tracking(frame);
        }
}


int main(int argc, char **argv)
{
//        CONSOLE_FONT_INFOEX font;
//        font.dwFontSize.X = 20;
        // Init glut
        glutInit(&argc, argv);

        

        
        char driverPath[DRIVERPATHSIZE] = { 0 };
        // Get Driver from arguments
        TranslateFileName("%CVB%\\Drivers\\GenICam.vin",driverPath,DRIVERPATHSIZE);

        // load the camera
        // IMG hCamera;
        cvbbool_t success = LoadImageFile(driverPath, hCamera);

        if (success == 0) {
                std::cout << "Fail to load the camera" << std::endl;
                return -1;
        }

        // Height & Width of the camera image
        long IMGheight = ImageHeight(hCamera); 
        long IMGwidth = ImageWidth(hCamera);
        // Allocation of the framedata
        frame.create(IMGheight, IMGwidth, CV_8UC4);
        // Init AR Toolkit
        initARToolkit( IMGwidth, IMGheight);

        std::cout << "Height: " << IMGheight << std::endl << "Width:  " << IMGheight << std::endl;
        
        if(!success)
        {
                std::cout << "Error loading driver";
                sleep(5000);
                return 0;
        }

        // start grab with ring buffer
        cvbres_t result = G2Grab(hCamera); 

        argMainLoop( NULL, NULL, processEverything );

        std::cout << "Stop acquisition";
        // stop the grab (kill = true: wait for ongoing frame acquisition to stop)
        result = G2Freeze(hCamera, true); 

        // free camera
        ReleaseObject(hCamera); 

        // free AR Toolkit
        cleanup();

        std::system("pause");

        return 0;
}

void tracking(cv::Mat &frame) {

    ARUint8         *dataPtr;
    ARMarkerInfo    *marker_info;
    int             marker_num;
    int             j, k;
        int                             i; //Anzahl der Pattern/später Roboter
        int                             a, b;
        double                  rotat[2];
        double                  xt;
        double                  yt;
        double                  zt;
        double                  grad;
        double cosa, sina, atana;
        int count = 0;
        std::string roboterkoord = "";
        
    /* grab a vide frame */
        dataPtr = (ARUint8 *) frame.data;

    if( count == 0 ) arUtilTimerReset();
    count++;
        //draw the video
    argDrawMode2D();
    argDispImage( dataPtr, 0,0 );

    /* detect the markers in the video frame */
    if( arDetectMarker(dataPtr, thresh, &marker_info, &marker_num) < 0 ) {
        cleanup();
        exit(0);
    }
        for( i = 0; i < marker_num; i++ ) {
                //glColor3f(0.0, 0.0, 1.0);
                //argDrawSquare(marker_info[i].vertex,0,0);
        }
        //std::system("cls");
        //printf(" ID  X-Wert  Y-Wert  Orientierung\n\n");
        /* check for known patterns */
    for( i = 0; i < objectnum; i++ ) {
                k = -1;
                for( j = 0; j < marker_num; j++ ) {
                        // std::cout << "Objekt i:"  << object[i].id << " Marker j:" << marker_info[j].id << std::endl;
                if( object[i].id == marker_info[j].id) {
                                
                                /* you've found a pattern */
                                //malt den grünen kreis um den marker
                                glColor3f( 0.0, 0.0, 1.0 );
                                argDrawSquare(marker_info[j].vertex,0,0);
                                if( k == -1 ) k = j;
                        else /* make sure you have the best pattern (highest confidence factor) */
                                        if( marker_info[k].cf < marker_info[j].cf ) k = j;
                        }
                }
                if(marker_info[k].cf < 0.7) k = -1;
                if( k == -1 ) {

                        object[i].visible = 0;
                        continue;
                }
                glColor3f( 1.0, 1.0, 0.0 );
                argDrawSquare(marker_info[k].vertex,0,0);
                /* calculate the transform for each marker */
                if( object[i].visible == 0 ) {
            arGetTransMat(&marker_info[k],
                          object[i].marker_center, object[i].marker_width,
                          object[i].trans);
        }
        else {
            arGetTransMatCont(&marker_info[k], object[i].trans,
                          object[i].marker_center, object[i].marker_width,
                          object[i].trans);

        }

                rotat[0] = object[i].trans[0][0];
                rotat[1] = object[i].trans[0][1];


                cosa = acos(rotat[0]);
                sina = asin(rotat[1]);
                if(sina < 0)
                {
                        atana = 3.14159265359 * 2 - cosa;
                }else
                {
                        atana = cosa;
                }
                grad = atana * 360 / (3.14159265359 * 2);
                object[i].marker_grad = grad;
                object[i].marker_pos[0] = marker_info[k].pos[0];
                object[i].marker_pos[1] = marker_info[k].pos[1];
                object[i].visible = 1;
                //printf(" %d   %4.1f   %4.1f   %4.1f %4.2f\n\n",object[i].id, object[i].marker_pos[0],object[i].marker_pos[1],object[i].marker_grad, marker_info[k].cf);
                //printf(" %d, %3.0f, %3.0f, %3.0f, \n",object[i].id, object[i].marker_pos[0],object[i].marker_pos[1],object[i].marker_grad);
                int id = object[i].id;
                object[i].marker_pos[0] = object[i].marker_pos[0] + 0.5;
                object[i].marker_pos[1] = object[i].marker_pos[1] + 0.5;
                object[i].marker_grad = object[i].marker_grad + 0.5;
                int xwert = (int) object[i].marker_pos[0];
                int ywert = (int) object[i].marker_pos[1];
                int orientierung = (int) object[i].marker_grad;
                //printf("%d %d %d %d\n", id, xwert, ywert, orientierung);
                char buffer [33];
//                itoa(id,buffer,10);
                sprintf(buffer,"%d",id);
                //printf("%s", buffer);
                roboterkoord = roboterkoord + boost::lexical_cast<std::string>(id) + "," + boost::lexical_cast<std::string>(xwert)+ "," + boost::lexical_cast<std::string>(ywert) +"," +boost::lexical_cast<std::string>(orientierung) +";";

        }//checkt bis hier alle pattern
        std::cout << roboterkoord;
//          rsb::Informer<string>::DataPtr s(new string(roboterkoord));

    // Send the data.
//     informer->publish(s);

        /*swap the graphics buffers*/
    argSwapBuffers();
}

void process(IMG hCamera, cv::Mat &frame)
{
        // The image object to store the image from the camera.
        IplImage *image = NULL;

        // Height & Width of the camera image
        long IMGheight = ImageHeight(hCamera); 
        long IMGwidth = ImageWidth(hCamera); 

        // Getting the address of the camera source image
        void *lpBaseAddress = 0;
        //long lXInc;
        intptr_t lXInc;
        //long lYInc;
        intptr_t lYInc;
        cvbbool_t status = GetLinearAccess(hCamera, 0, &lpBaseAddress, &lXInc, &lYInc);
        if(!status) {
                std::cout << "Error GetLinearAccess";
                //throw ("Error accessing camera image");
        }

        if(image == NULL) {
                image = cvCreateImageHeader(cvSize(IMGwidth , IMGheight), 8, 3);
        }

        if(lpBaseAddress != 0) {
                image->imageData = (char*) lpBaseAddress;
        } else {
                std::cout << "BaseAddress is ZERO";
                //throw "baseAddress is ZERO";
        }

        IplImage dst_img = frame;
        cvCvtColor( image, &dst_img, CV_BGR2RGBA);
}
