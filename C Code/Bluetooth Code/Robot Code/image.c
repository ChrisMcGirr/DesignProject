#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <cv.h>
#include <highgui.h>
#include <time.h>

//	hci0	00:15:83:C2:E4:02
//	hci1	00:02:76:2A:81:A4
//          00:02:76:2A:81:A4
int main(int argc, char **argv)
{
	

	//Bluetooth Code
    struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
    char buf[1024] = { 0 };
    int s, client,status;
    int bytes =0;
    socklen_t opt = sizeof(rem_addr);

    // allocate socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // bind socket to port 1 of the first available 
    // local bluetooth adapter
    loc_addr.rc_family = AF_BLUETOOTH;
    loc_addr.rc_bdaddr = *BDADDR_ANY;
    loc_addr.rc_channel = (uint8_t) 1;
    bind(s, (struct sockaddr *)&loc_addr, sizeof(loc_addr));
	// put socket into listening mode
	listen(s, 1);
	// accept one connection
	client = accept(s, (struct sockaddr *)&rem_addr, &opt);
	ba2str( &rem_addr.rc_bdaddr, buf );
	fprintf(stderr, "accepted connection from %s\n", buf);
	memset(buf, 0, sizeof(buf));
   
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH,640);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT,480);
	IplImage* img = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U,1);
	IplImage* frame = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U,3);
   	
	int step = img->widthStep;
	int size = img->imageSize;	
	status = write(client, &size, sizeof(int));
	status = write(client, &step, sizeof(int));
	while(1){	
		frame = cvQueryFrame(capture);
		cvCvtColor(frame,img,CV_RGB2GRAY);
		for(int i=0; i<size; i+=status){	
			if((status = write(client, img->imageData+i, size-i))==-1){
				fprintf(stderr, "Failed to Send");
			}
		}	
		//fprintf(stderr, "Sent");
	}

    // close connection
	close(client);
    close(s);
    return 0;
}
