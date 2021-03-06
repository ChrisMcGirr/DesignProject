/*************************************
This code is to be run on the PC while the code, objectdectectionRobot is running on the robot. This is used to debug what is happening with the robot. Displays the motion the robot sees and where that motion is occurring from the robot's perspective. Most of the functions are documented in objectdetectionRobot.c.
**************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <cv.h>
#include <highgui.h>
#include <time.h>

void differenceFilter(IplImage* frame1, IplImage* frame2,  uint8_t *difference);
void printDifference(IplImage* frame, uint8_t *difference);
uint8_t validQuadrant(uint8_t x, uint8_t y);
static uint8_t changeError = 30;

static uint16_t camWidth = 640;
static uint16_t camHeight = 480;

typedef struct Motion{
	uint8_t x[20];
	uint8_t y[20];
	uint8_t width[20];
	uint8_t height[20];
}Motion;
uint8_t motionSize=20;
void motionQuadrant(uint8_t *difference, Motion* quadrant);
uint8_t quadrantCompare(Motion* quadrants, uint8_t ind);
uint8_t pointInQuadrant(Motion* quadrants, uint8_t x, uint8_t y);
void filterQuadrants(Motion* quadrant);
void combineQuadrants(Motion* quadrant);
void copyQuadrant(Motion* current, Motion* previous);
void getDifference(int s, uint8_t* difference);


typedef struct Direction{
	int8_t directions[3];
	uint8_t index;
}Direction;
uint8_t FilterSize = 3;
int8_t detectDirection(Motion* previous, Motion* current, Direction* array);
int8_t detectMotionRegion(Motion* current);

int main(int argc, char **argv)
{


	
	cvNamedWindow("Image",CV_WINDOW_AUTOSIZE);
	uint8_t difference[1200]={0};
	IplImage* diffimg = cvCreateImage(cvSize(camWidth,camHeight), IPL_DEPTH_8U,3);
	int8_t temp=0;
	

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
	Motion quadrantsCurrent = {0};
	char go[3]="Go";
	if(write(s, go, sizeof(buf))==-1){
		printf("Failed to send Command\n");
	}

	while(1){
		if((bytes_read = read(s, buf, sizeof(buf)))==-1){
			printf("Recv Failed");
		}
		if(strcmp(buf,"Image")==0){
			getDifference(s,difference);
			motionQuadrant(difference,&quadrantsCurrent);
			combineQuadrants(&quadrantsCurrent);
			temp= detectMotionRegion(&quadrantsCurrent);
			printDifference(diffimg, difference);
			for(int i=0; i<motionSize;i++){
				if((quadrantsCurrent.width[i]>0)&(quadrantsCurrent.height[i]>0)){
					uint16_t horz = quadrantsCurrent.x[i]*16;
					uint16_t vert = quadrantsCurrent.y[i]*16;
					uint16_t horz2 = horz + (quadrantsCurrent.width[i])*16;
					uint16_t vert2 =vert +(quadrantsCurrent.height[i])*16;
					cvRectangle(diffimg, cvPoint(horz,vert),cvPoint(horz2,vert2),cvScalar(255,0,0), 2,8,0);
				}
			}
			memset(difference,0,1200*sizeof(uint8_t));
			memset(&quadrantsCurrent,0,sizeof(Motion));
			cvShowImage("Image",diffimg);
			cvWaitKey(1);
		}
		if(strcmp(buf,"Left")==0){
			printf("%s\n", buf);
		}
		if(strcmp(buf,"Right")==0){
			printf("%s\n", buf);
		}
		if(strcmp(buf,"Center")==0){
			printf("%s\n", buf);
		}
		if(strcmp(buf,"No Movement")==0){
			printf("%s\n", buf);
		}
	}
	
	return 0;
}
/**Reads Difference array from Bluetooth Socket
*Start to read Difference array from socket. Completes once we have read the whole array. 
*\param int s, the File Descriptor of the Bluetooth socket.
*\param uint8_t* difference pointer to local difference array
*/
void getDifference(int s, uint8_t* difference){
	int status=0;
	uint32_t size = 1200*sizeof(uint8_t);
	uint32_t bytes=0;
	
	for(int i=0; i<size; i+=bytes){
		if((bytes = read(s, difference+i, size-i))==-1){
			printf("Recv Failed");
		}
	}
}
void motionQuadrant(uint8_t *difference, Motion* quadrant){
	uint8_t done =0;
	uint8_t ind=0;
	uint8_t oldWidth=0;
	uint8_t oldHeight=0;
	
	for(int y=0; y<30; y++){
		for(int x = 0; x<40; x++){
			if(difference[y+x*30]>0){
				
				quadrant->x[ind] =x;
				quadrant->y[ind]=y;
				quadrant->width[ind]=1;
				quadrant->height[ind]=1;
				done =0;
				oldWidth=0;
				oldHeight =0;
				/*
				if(pointInQuadrant(quadrant,x,y)){
					done=1;
				}
				else{
					done=0;
				}
				*/
				
				while(!done){
					//Check White Pixel On Top
					for(int i=-1; i<quadrant->width[ind]+1;i++){
						if(validQuadrant(quadrant->x[ind]+i,quadrant->y[ind]-1)){
							if(difference[(quadrant->y[ind]-1)+(quadrant->x[ind]+i)*30]>0){
								i = quadrant->width[ind]+1;
								--quadrant->y[ind];
								++quadrant->height[ind];
							}
						}
					}
					//Check White Pixel on Right
					for(int i=-1; i<quadrant->height[ind]+1;i++){
						if(validQuadrant(quadrant->x[ind]+quadrant->width[ind]+1,quadrant->y[ind]+i)){
							if(difference[(quadrant->y[ind]+i)+(quadrant->x[ind]+quadrant->width[ind]+1)*30]>0){
								i = quadrant->height[ind] +1;
								++quadrant->width[ind];
							}
						}
					}
					//Check White Pixel on Left
					for(int i=-1; i<quadrant->height[ind]+1;i++){
						if(validQuadrant(quadrant->x[ind]-1,quadrant->y[ind]+i)){
							if(difference[(quadrant->y[ind]+i)+(quadrant->x[ind]-1)*30]>0){
								i = quadrant->height[ind]+1;
								--quadrant->x[ind];
								++quadrant->width[ind];
							}
						}
					}
					//Check White Pixel on Bottom
					for(int i=-1; i<quadrant->width[ind]+1;i++){
						if(validQuadrant(quadrant->x[ind]+i,quadrant->y[ind]+1+quadrant->height[ind])){
							if(difference[(quadrant->y[ind]+1+quadrant->height[ind])+(quadrant->x[ind]+i)*30]>0){
								i = quadrant->width[ind]+1;
								++quadrant->height[ind];
							}
						}
					}
					if((quadrant->width[ind] ==oldWidth)&(quadrant->height[ind]==oldHeight)){
						done=1;
						if(quadrantCompare(quadrant,ind)){
							quadrant->x[ind+1] = quadrant->x[ind];
							quadrant->y[ind+1] = quadrant->y[ind];
							quadrant->width[ind+1] = quadrant->width[ind];
							quadrant->height[ind+1] = quadrant->height[ind];
						}
						++ind;
						if(ind==motionSize){
							ind=0;
						}
					}
					else{
						oldWidth = quadrant->width[ind];
						oldHeight = quadrant->height[ind];
					}
				}
			}
		}
	}
}
int8_t detectDirection(Motion* previous, Motion* current, Direction* array){
	int8_t direction=0;
	if(previous->x[0]>current->x[0]){
		direction=1;
	}
	if(previous->x[0]<current->x[0]){
		direction=-1;
	}
	for(int i=0; i<array->index; i++){
		direction+=array->directions[i];
	}
	array->directions[array->index]=direction;
	++array->index;
	if(array->index>FilterSize){
		array->index = 0;
	}
	return direction;
}
int8_t detectMotionRegion(Motion* current){
	uint8_t position=0;
	position = current->x[0]+(current->width[0]>>2);
	if((position<=14)&(position>0)){
		return 1;
	}
	if((position>14)&(position<26)){
		return 0;
	}
	if(position>=26){
		return -1;
	}
	return -42;
}
void combineQuadrants(Motion* quadrant){
	uint8_t upperX=quadrant->x[0];
	uint8_t upperY=quadrant->y[0];
	uint8_t lowerX=quadrant->x[0]+quadrant->width[0];
	uint8_t lowerY=quadrant->y[0]+quadrant->height[0];
	
	for(int i=1; i<motionSize; i++){
		if((quadrant->width[i]>0)&(quadrant->height[i]>0)){
			if((quadrant->x[i]<upperX)){
				upperX = quadrant->x[i];
			}
			if(quadrant->y[i]<upperY){
				upperY = quadrant->y[i];
			}
			if(((quadrant->x[i]+quadrant->width[i])>lowerX)){
				lowerX=quadrant->x[i]+quadrant->width[i];
			}
			if((quadrant->y[i]+quadrant->height[i])>lowerY){
				lowerY=quadrant->y[i]+quadrant->height[i];
			}
			quadrant->x[i]=0;
			quadrant->y[i]=0;
			quadrant->width[i]=0;
			quadrant->height[i]=0;
		}
	}
	quadrant->x[0]=upperX;
	quadrant->y[0]=upperY;
	quadrant->width[0]=lowerX-upperX;
	quadrant->height[0]=lowerY-upperY;
	
}
void filterQuadrants(Motion* quadrant){
	for(int i=0; i<motionSize; i++){
		for(int j=i+1; j<motionSize; j++){
			if((quadrant->x[j]>=quadrant->x[i])&(quadrant->x[j]<=(quadrant->x[i]+quadrant->width[i]))){
				if((quadrant->y[j]>=quadrant->y[i])&(quadrant->y[j]<=(quadrant->y[i]+quadrant->height[i]))){
					quadrant->width[i]=quadrant->x[j]+quadrant->width[j]-quadrant->x[i];
					quadrant->height[i]=quadrant->y[j]+quadrant->height[j]-quadrant->y[i];
					
					quadrant->x[j]=0;
					quadrant->y[j]=0;
					quadrant->width[j]=0;
					quadrant->height[j]=0;
				}
			}
		}
		for(int j=0; j<i; j++){
			if((quadrant->x[j]>=quadrant->x[i])&(quadrant->x[j]<=(quadrant->x[i]+quadrant->width[i]))){
				if((quadrant->y[j]>=quadrant->y[i])&(quadrant->y[j]<=(quadrant->y[i]+quadrant->height[i]))){
					quadrant->width[i]=quadrant->x[j]+quadrant->width[j]-quadrant->x[i];
					quadrant->height[i]=quadrant->y[j]+quadrant->height[j]-quadrant->y[i];
					
					quadrant->x[j]=0;
					quadrant->y[j]=0;
					quadrant->width[j]=0;
					quadrant->height[j]=0;
				}
			}
		}
	}
}
void copyQuadrant(Motion* current, Motion* previous){
	for(int i =0; i<motionSize; i++){
		previous->x[i] = current->x[i];
		previous->y[i] = current->y[i];
		previous->width[i] = current->width[i];
		previous->height[i] = current->height[i];
	}
}
uint8_t pointInQuadrant(Motion* quadrants, uint8_t x, uint8_t y){
	uint8_t check=0;
	for(int i=0; i<motionSize;i++){
		if((x>=quadrants->x[i])&(x<=(quadrants->x[i]+quadrants->width[i]))){
			if((y>=quadrants->y[i])&(y<=(quadrants->y[i]+quadrants->height[i]))){
				check=1;
			}
		}
	}
	return check;
}
uint8_t quadrantCompare(Motion* quadrants, uint8_t ind){
	if((quadrants->width[ind]>quadrants->width[ind+1])&(quadrants->height[ind]>= quadrants->height[ind+1])){
		return 1;
	}
	else{
		return 0;
	}
}
uint8_t validQuadrant(uint8_t x, uint8_t y){
	if(y>=0 & x>=0){
		if(y<30 & x<40 ){
		return 1;
		}
		else{
		return 0;
		}
	}
	else{
		return 0;
	}
}
void differenceFilter(IplImage* frame1, IplImage* frame2, uint8_t *difference){
	uint16_t check=0;
	uint16_t ind=0;
	for(uint8_t j =0; j<40; j++){
		for(uint8_t i =0; i<30; i++){
			for(uint8_t y=0; y<16;y++){
				for(uint8_t x=0; x<16; x++){
					if(abs(cvGetReal2D(frame1, y+i*16,j*16+x)-cvGetReal2D(frame2,i*16+y,j*16+x))> changeError){
						++check;
					}
				}
			}
			if(check>90){
				difference[ind]=1;
			}
			ind++;
			check=0;
		}
	}
}
/**Draws the difference array onto an Image
*Simply draws the difference array of size 40*30 onto an image of size 640*480
*\param IplImage* frame, pointer to the Image you want to draw the difference array
*\param uint8_t* difference, pointer to difference array
*/
void printDifference(IplImage* frame, uint8_t *difference){
	uint16_t horz=0;
	uint16_t vert=0;
	
	for(int y=0; y<30; y++){
		for(int x=0; x<40; x++){
			horz = x*16;
			vert = y*16;
			if(difference[y+x*30]>0){
				cvRectangle(frame, cvPoint(horz,vert),cvPoint(horz+16,vert+16),cvScalar(255,255,255), CV_FILLED,8,0);
			}
			else{
				cvRectangle(frame, cvPoint(horz,vert),cvPoint(horz+16,vert+16),cvScalar(0,0,0), CV_FILLED,8,0);
			}
		}
	}
}
