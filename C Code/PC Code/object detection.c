#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cv.h>
#include <highgui.h>
#include <time.h>

void differenceFilter(IplImage* frame1, IplImage* frame2, IplImage* difference);
uint8_t isValidPixelCoord(uint16_t x, uint16_t y, IplImage* frame);
static uint8_t changeError = 30;

typedef struct Blob {
   uint16_t width;
   uint16_t height;
   uint16_t x;
   uint16_t y;
} Blob;

void getMotionBlobs(IplImage* frame, Blob* blobs);
uint8_t pointInBlob(uint16_t x, uint16_t y, Blob *in);
void blobCopy(Blob *org,Blob *copy);
uint8_t blobSizeCompare(Blob *one, Blob *two);
uint8_t getDirection(Blob* current, Blob* previous);
static uint16_t camWidth = 320;
static uint16_t camHeight = 240;

int main(int argc, char **argv)
{

	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH,camWidth);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT,camHeight);
	
	IplImage* img = cvQueryFrame(capture);
	IplImage* frame1 = cvCreateImage(cvSize(camWidth,camHeight), IPL_DEPTH_8U,1);
	IplImage* frame2 = cvCreateImage(cvSize(camWidth,camHeight), IPL_DEPTH_8U,1);
	cvCvtColor(img,frame1,CV_RGB2GRAY);
	
	cvNamedWindow("Image",CV_WINDOW_AUTOSIZE);
	
	img = cvQueryFrame(capture);
	cvCvtColor(img,frame2,CV_RGB2GRAY);
	
	IplImage* difference = cvCreateImage(cvSize(camWidth,camHeight), IPL_DEPTH_8U,1);
	differenceFilter(frame1,frame2,difference);
	Blob blobs[2] ={0};
	Blob previous ={0};
	uint8_t direction=0;
	
	clock_t start, end;
	double cpu_time_used;
	
	while(1){
		start = clock();
		cvCopy(frame2, frame1, NULL);
		img = cvQueryFrame(capture);
		cvCvtColor(img,frame2,CV_RGB2GRAY);
		differenceFilter(frame1,frame2,difference);
		getMotionBlobs(difference, blobs);
		cvRectangle(difference, cvPoint(blobs[1].x, blobs[1].y), cvPoint(blobs[1].x+blobs[1].width, blobs[1].y+blobs[1].height),cvScalar(135,135,135),5,8,0);
		direction = getDirection(&blobs[1],&previous);
		blobCopy(&blobs[1], &previous);
		/*
		printf("Blob Info\n");
		printf("X: %u\n", blobs[1].x);
		printf("y: %u\n", blobs[1].y);
		printf("Width: %u\n", blobs[1].width);
		printf("Height: %u\n", blobs[1].height);
		*/
		end = clock();
		cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
		printf("Computation Time %fs\n",cpu_time_used);
		memset(blobs,0,2*sizeof(Blob));
		cvShowImage("Image",difference);
		cvWaitKey(1);
	}
	
	return 0;
}
void differenceFilter(IplImage* frame1, IplImage* frame2, IplImage* difference){
	
	for(uint16_t i =0; i<frame1->width; i++){
		for(uint16_t j =0; j<frame1->height; j++){
			if(abs(cvGetReal2D(frame1, j,i)-cvGetReal2D(frame2,j,i))< changeError){
				cvSetReal2D(difference, j, i, 0);
			}
			else{
				cvSetReal2D(difference, j, i, 255);
			}
		}
	}

}

void getMotionBlobs(IplImage* frame, Blob* blobs){
	uint8_t ind =0;
	int size=0;
	
	for(int x=0; x<frame->width;x++){
		for(int y = 0; y<frame->height;y++){
			if(cvGetReal2D(frame,y,x)==255){
				ind = 0;
				blobs[ind].x =x;
				blobs[ind].y =y;
				blobs[ind].width =0;
				blobs[ind].height =0;
				
				uint8_t done =0;
				uint16_t oldWidth=0;
				uint16_t oldHeight =0;
				if(pointInBlob(x,y, &blobs[ind+1])){
					done=1;
				}
				else{
					done=0;
				}
				while(!done){
					//Check White Pixel On Top
					for(int i=-1; i<blobs[ind].width+1;i++){
						if(isValidPixelCoord(blobs[ind].x+i, blobs[ind].y-1, frame)){
							if(cvGetReal2D(frame, blobs[ind].y-1,blobs[ind].x+i)==255){
								i = blobs[ind].width +1;
								--blobs[ind].y;
								++blobs[ind].height;
							}
						}
					}
					//Check White Pixel on Right
					for(int i=-1; i<blobs[ind].height+1;i++){
						if(isValidPixelCoord(blobs[ind].width+1+blobs[ind].x,blobs[ind].y+i, frame)){
							if(cvGetReal2D(frame,blobs[ind].y+i,blobs[ind].x+blobs[ind].width+1)==255){
								i = blobs[ind].height +1;
								++blobs[ind].width;
							}
						}
					}
					//Check White Pixel on Left
					for(int i=-1; i<blobs[ind].height+1;i++){
						if(isValidPixelCoord(blobs[ind].x-1,blobs[ind].y+i, frame)){
							if(cvGetReal2D(frame,blobs[ind].y+i,blobs[ind].x-1)==255){
								i = blobs[ind].height +1;
								--blobs[ind].x;
								++blobs[ind].width;
							}
						}
					}
					//Check White Pixel on Bottom
					for(int i=-1; i<blobs[ind].width+1;i++){
						if(isValidPixelCoord(blobs[ind].x+i,blobs[ind].y+1+blobs[ind].height, frame)){
							if(cvGetReal2D(frame,blobs[ind].y+1+blobs[ind].height,blobs[ind].x+i)==255){
								i = blobs[ind].width+1;
								++blobs[ind].height;
							}
						}
					}
					if((blobs[ind].width ==oldWidth)&(blobs[ind].height==oldHeight)){
						done=1;
						if(blobSizeCompare(&blobs[ind],&blobs[ind+1])){
							blobs[ind+1].width = blobs[ind].width;
							blobs[ind+1].height = blobs[ind].height;
							blobs[ind+1].x = blobs[ind].x;
							blobs[ind+1].y = blobs[ind].y;
						}
						
					}
					else{
						oldWidth = blobs[ind].width;
						oldHeight = blobs[ind].height;
					}						
				}
			}
		}
	}
}
uint8_t isValidPixelCoord(uint16_t x, uint16_t y, IplImage* frame){
	
	if(x<0){
		return 0;
	}
	if(x>=frame->width){
		return 0;
	}
	if(y<0){
		return 0;
	}
	if(y >= frame->height){
		return 0;
	}
	else{
		return 1;
	}

}
uint8_t pointInBlob(uint16_t x, uint16_t y, Blob *in){
	
	if((x>=in->x)&(x<=(in->x+in->width))){
		if((y>=in->y)&(y<=(in->y+in->height))){
			return 1;
		}
	}
	else{
		return 0;
	}
}
uint8_t detectMotion(){

}
void blobCopy(Blob *org,Blob *copy){
	copy->x = org->x;
	copy->y = org->y;
	copy->width = org->width;
	copy->height = org->height;
}
uint8_t blobSizeCompare(Blob *one, Blob *two){
	if((one->width>two->width)&(one->height>two->height)){
		return 1;
	}
	else{
		return 0;
	}
}
uint8_t getDirection(Blob* current, Blob* previous){
	//printf("Previous %u and Current %u\n", previous->x,current->x);
	if(current->x > previous->x){
		printf("Right\n");
		return 1;
	}
	if(current->x < previous->x){
		printf("Left\n");
		return 2;
	}
	else{
		return 0;
	}
}

