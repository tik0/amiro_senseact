//for running system comands
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <mongoose.h>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <signal.h>

// For checking character pressed in the console
#include <kbhit.hpp>

#ifdef _WIN32
#define snprintf _snprintf
#endif


#ifdef _WIN32
#include <io.h>
#ifdef EXTERNAL_POLL
#define poll WSAPoll
#endif
#else
#include <syslog.h>
#include <sys/time.h>
#include <unistd.h>
#endif

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// Print nice messages
#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

//for Interartion speed
int static currentTimer,lastTimerCam=0;     //currentTimer holds the actual Time in microseconds
struct timeval tv1, tv2;

// Holds the compressed image
std::vector<uchar> buf;

static int send_html(struct mg_connection *conn) {
	// NOTE: If u want to send a html-file it must be named index.html in the document-root or 
	// the name index.html must be changed in mongoose.c (Line 1233 Date 01.07.2014)

	// if connection is a websocket connection -> all received signals are processed here
	if (conn->is_websocket) {
		
	  // Process the received signal
		int signalReceived = atoi((char*) conn->content);


		//Getting connection closed Signal by Client
		//INFO_MSG( "conn->wsbits: " << conn->wsbits )
		if(conn->wsbits == 136){
			INFO_MSG( "socket closed" << "/" << conn->wsbits )
		}
		
		return conn->content_len == 4 && !memcmp(conn->content, "exit", 4) ?
				MG_FALSE : MG_TRUE;
	} else {
		//load everything that belongs to the html site automatically
		return MG_FALSE;
	}
}


static int ev_handler(struct mg_connection *conn, enum mg_event ev) {

//  std::cout << "test " << ev << std::endl;
	//if a new CLient connects count up	
	if(ev== MG_WS_HANDSHAKE){
	}
	//if a Client leaves decrement
	if(ev == !MG_REPLY){
	}
	
	if (ev == MG_REQUEST) {
		return send_html(conn);
	} else if (ev == MG_AUTH) { // If not included you have to ahtenticate to load the homepage
		return MG_TRUE;
	}
	
	if (ev == MG_POLL && conn->is_websocket) {
	  mg_websocket_write(conn, 2, (const char*) &buf[0], buf.size());
	  return MG_FALSE;
	}
}

int main(int argc, char **argv) {

  // Open the camera device
  cv::VideoCapture cam;
  // Holds the image from the camera
  cv::Mat data;
  // Define compression parameters
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  // that's percent, so 100 == no compression, 1 == full
  compression_params.push_back(80);

  if (cam.open(0) == 0) {
    exit(1);
  }



	struct mg_server *server = mg_create_server(NULL, ev_handler);
  mg_set_option(server, "listening_port", "8080");
	mg_set_option(server, "document_root", "./ServerMon");
	printf("Started on port %s\n", mg_get_option(server, "listening_port"));
	// Begin of Interation loop
	int counter = 0;
	for (;;) {
    if( kbhit() ) {
      break;
    }
    // Get the new frame
    cam >> data;
    // Compress picture
    if(!data.empty()){
      imshow("test", data);
      cvWaitKey(10);

      imencode(".jpeg", data, buf, compression_params);

      mg_poll_server(server, 100);
      gettimeofday(&tv1,0); // tv.tv_usec tv.tv_sec

      //currentTimer holds the actual Time in milliseconds
      currentTimer = tv1.tv_sec * 1000 + tv1.tv_usec / 1000;
      if(currentTimer - lastTimerCam >= 1000 /*ms*/){
        std::cout << "FPS: " << counter  << " Frames / " << currentTimer - lastTimerCam << " ms" << std::endl;
        lastTimerCam = currentTimer;
        counter = 0;
      }
      ++counter;
    }
	}

	cam.release();
	mg_destroy_server(&server);
	return 0;
}

