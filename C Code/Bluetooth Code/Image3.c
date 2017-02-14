#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <cv.h>
#include <highgui.h>
#include <time.h>

int main(int argc, char **argv)
{
	clock_t begin, end;
	double time_spent;
    struct sockaddr_rc addr = { 0 };
    int s, status, bytes_read;
    char dest[18] = "00:02:72:1E:DE:65";
	char buf[1024] = { 0 };
	
    // allocate a socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = 1;
    str2ba( dest, &addr.rc_bdaddr );

    // connect to server
    status = connect(s, (struct sockaddr *)&addr, sizeof(addr));
	printf("Connected \n");
	
	int imgSize = sizeof(uint8_t)*76800;
	uchar sockData[imgSize];
	int bytes = 0;
	bytes = read(s,&imgSize, sizeof(int));
	printf("Img Size %i \n", imgSize);
	int step;
	bytes = read(s,&step, sizeof(int));
	printf("Img step %i \n", step);
	IplImage* img = cvCreateImage(cvSize(640,120), IPL_DEPTH_8U,1);
	cvNamedWindow("Image",CV_WINDOW_AUTOSIZE);
	
	time(&begin);
	int counter =0;
	while(1){
		
		for(int i=0; i<imgSize; i+=bytes){
			if((bytes = read(s, sockData+i, imgSize-i))==-1){
				printf("Recv Failed");
			}
		}
		time(&end);
		++counter;
		cvSetData(img,(void*)sockData, step);				
		cvShowImage("Image", img);
		cvWaitKey(1);
		time_spent = counter/difftime(end, begin);
		printf("FPS %f \n", time_spent);	
			
	}
	cvDestroyAllWindows();
    close(s);
    return 0;
}
