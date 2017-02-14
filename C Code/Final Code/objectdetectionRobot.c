/***********************************************
Name: objectdetectionRobot.c
Date: 9/04/2014
Description: 
Object Detection Algorithm that utilizes the a USB Webcam
connected to the PhidgetSBC1072 board to detection regions of motion for moving objects.
The regions of movement are processed and analysed to see where they occur so that the robot
can avoid these regions of motion.
The algorithm uses OpenCV to take the Webcam image. OpenCV simply calls the Video for Linux 2 (V4L2) library, its a low
level camera handler for Linux machines. Improvements could be made by simply accessing this library rather than using the bulky
OpenCV library. The images are stored as IplImage structures. Makes it convenient to handle. 
However, the difference of the image is stored as an array of 1200 elements or 40*30 image. Handling is done to convert accesses
in the (X,Y) format to the single dimensional arrangement in memory. 
I hope this code is of any use and makes some sense. I have commented what I think is necessary. However, some of the Motor control code and
the Phidget Interface code was taken directly from previous Design Project teams who used the robot before.
Note the Motor Drivers were written by previous DP Teams and are not complete. The drivers should be written with
software PWM or hardware which would be better, but for the time being the motors are simply turned on and off when needed for a certain delay to mimic turning
right of left. 

*Note to Run this program you will have to run the debugRobot.c file on a computer and connect to the robot over bluetooth. The robot will wait for a bluetooth connection and an acknowledgement from a PC over bluetooth before starting the motion detection algorithm. 

To compile this file on the robot run:

g++ -o objectdetection objectdetection.c -lbluetooth -lphdiget21 `pkg-config --cflags opencv --libs opencv`

Author: Christopher McGirr - christopher.mcgirr@mail.mcgill.ca

*************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cv.h>
#include <highgui.h>
#include <time.h>
#include <phidget21.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

#define FORWARD 1 //defines the value to move the motor forwards
#define BACKWARD 0 //defines the value to move the motor backwards
#define MINDIST 13 //minimum distance in inches that stops robot movement

//Conversion parameters for converting raw values into inches(Defined through testing).

//Front three sensors
#define SLOPE 0.13
#define OFFSET 0.4225

//Radar
#define RSLOPE 0.1063
#define ROFFSET -2.404

//global vars
int frontStop = 1;
int backStop = 1;
int counterLR = 0;
int counterFB = 0;

char sensorValues[12] = { 9 }; 

//functions
int CCONV AttachHandler(CPhidgetHandle IFK, void *userptr);
int CCONV DetachHandler(CPhidgetHandle IFK, void *userptr);
int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown);
int CCONV InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State);
int CCONV OutputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State);
int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int index, int val1);
int stop(CPhidgetInterfaceKitHandle ifKit);
int pin(int h, CPhidgetInterfaceKitHandle ifKit, int state);
void turnRight(CPhidgetInterfaceKitHandle ifKit);
void turnLeft(CPhidgetInterfaceKitHandle ifKit);
void moveBothMotor(CPhidgetInterfaceKitHandle ifKit, int dir);
int display_properties(CPhidgetInterfaceKitHandle phid);
int interfacekit_simple(CPhidgetInterfaceKitHandle ifKit);
void delay(int milliseconds);

void differenceFilter(IplImage* frame1, IplImage* frame2,  uint8_t *difference);
void printDifference(IplImage* frame, uint8_t *difference);
uint8_t validQuadrant(uint8_t x, uint8_t y);
static uint8_t changeError = 30;

static uint16_t camWidth = 640;
static uint16_t camHeight = 480;
//! Motion Struct
/*!
	Structure Containing Motion Quadrants returned by the MotionQuadrant() function. Contains up to 20 Motion Quadrants with origin at X,Y of width and height. Note to clean up the code a bit we could make it an array of structs rather than a struct contain arrays. Each index corresponds to one particular Motion Quadrant that was found. 
*/
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
void sendDifference(int client, uint8_t* difference);

//! Direction Buffer
/*!
	Small Struct that contains the last three directions values.
	It was going to be used to implement a small moving average buffer
	to filter out the high filter noise of the direction value output by
	the function detectDirection()
*/
typedef struct Direction{
	int8_t directions[3];
	uint8_t index;
}Direction;
uint8_t FilterSize = 3;
int8_t detectDirection(Motion* previous, Motion* current, Direction* array);
int8_t detectMotionRegion(Motion* current);

char imgcommand[6]="Image"; /*!< Bluetooth Command to send to PC indicating we're send an image*/
char left[5]="Left";/*!< String used to indicate motion Left*/
char right[6]="Right";/*!< String used to indicate motion Right*/
char center[7]="Center";/*!< String used to indicate motion Center*/
char nomovement[12] ="No Movement";/*!< String used to indicate no movement*/

int main(int argc, char **argv)
{
	//****Start of Bluetooth Code***************************//
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
	fprintf(stderr, "Waiting for Bluetooth Connection\n");
	listen(s, 1);
	// accept one connection
	client = accept(s, (struct sockaddr *)&rem_addr, &opt);
	ba2str( &rem_addr.rc_bdaddr, buf );
	fprintf(stderr, "accepted connection from %s\n", buf);
	memset(buf, 0, sizeof(buf));
	//*****End of Bluetooth Code************************//
	
	//! Initialize first Webcam connected to Computer
	/*!
		Finds first webcam connect to the computer and starts it.		
	*/
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	//Set Image Size
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH,camWidth);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT,camHeight);
	
	//Capture first Image and create image holders
	IplImage* img = cvQueryFrame(capture);
	IplImage* frame1 = cvCreateImage(cvSize(camWidth,camHeight), IPL_DEPTH_8U,1);
	IplImage* frame2 = cvCreateImage(cvSize(camWidth,camHeight), IPL_DEPTH_8U,1);
	
	//!Initialize variables that we are going to need
	uint8_t difference[1200]={0}; //difference of two images
	Motion quadrantsCurrent; //stores the current motion quadrants
	memset(&quadrantsCurrent,0,sizeof(Motion)); /*!< Make sure the array is set to zero*/
	Direction directions = {0}; //direction buffer used for filter
	int8_t temp=-42; /*!< default state for temp is -42 meaning no movement. Temp is used to store motion direction*/
	
	//*Motor Settings*\\
	//Declare an InterfaceKit handle
	CPhidgetInterfaceKitHandle ifKit = 0;
	//create the InterfaceKit object
	CPhidgetInterfaceKit_create(&ifKit);
	interfacekit_simple(ifKit);
	stop(ifKit);
	
	//Flag Variables for our State Machine(Motion Detection Algorithm)
	uint8_t reset=1;
	uint8_t count=0;
	uint8_t ready=0;
	//make sure buffer is zero before reading
	memset(buf,0, sizeof(buf));
	while(strcmp(buf, "Go")>0){
		bytes = read(client, buf, sizeof(buf));
	}
	
	//This should really be run in a thread after everything is initialized
	while(1){
		//! Main Object Detection Loop
		/*!
			The algorithm loops infinitely. If reset == 0 we copy the previous frame to previous frame. Then take a picture and convert it to grayscale. Then apply our difference Filter and find the motion quadrants.
			Combine these quadrants into one large quadrant and detect in which region this motion lies. Either Left,Center or Right of the Robot. Then the robot checks where the motion occurs and acts accordingly. 
		*/
		if(reset==0){
			cvCopy(frame2, frame1, NULL); //save previous frame
			img = cvQueryFrame(capture); //take current frame
			cvCvtColor(img,frame2,CV_RGB2GRAY); //convert current frame to grayscale
			differenceFilter(frame1,frame2,difference); //take the difference of the two frames
			motionQuadrant(difference,&quadrantsCurrent); //find the motion quadrants
			combineQuadrants(&quadrantsCurrent); //combine these quadrants
			temp = detectMotionRegion(&quadrantsCurrent); /*!< Temp holds the motion region that is outputted*/
			
		}
		if(temp==1){
			//printf("Left\n");
			if(write(client,left, sizeof(buf))==-1){
				printf("Send Failed\n");
			}
			turnRight(ifKit);
			reset=1;
		}
		if(temp==-1){
			//printf("Right\n");
			if(write(client,right, sizeof(buf))==-1){
				printf("Send Failed\n");
			}
			//turnLeft(ifKit);
			reset=1;
		}
		if(temp==0){
			//printf("Center\n");
			if(write(client,center, sizeof(buf))==-1){
				printf("Send Failed\n");
			}
			moveBothMotor(ifKit,BACKWARD);
			reset=1;
		}
		if(temp<-1){
			//printf("No Movement\n");
			if(write(client,nomovement, sizeof(buf))==-1){
				printf("Send Failed\n");
			}
			temp=-42;
			reset=0;
		}
		//! If the robot moves we have to reset the camera and our algorithm
		/*!
			First the camera's auto-balance will have to be readjusted due to new lighting conditions when the robot moves. Then we makes sure we are taking the correct image to compare and finally reset all our flags to default
		*/
		if(reset==1){
			//printf("Letting Auto Balance Reset\n");
			if(write(client,imgcommand, sizeof(buf))==-1){
				printf("Send Failed\n");
			}
			sendDifference(client,difference);
			while(count<10){
				img = cvQueryFrame(capture);
				++count;
			}
			img = cvQueryFrame(capture);
			cvCvtColor(img,frame2,CV_RGB2GRAY);
			reset = 0;
			count = 0;
			temp=-42;
		}
		memset(difference,0,1200*sizeof(uint8_t));/*!< Reset Difference Array*/
		memset(&quadrantsCurrent,0,sizeof(Motion));/*!< Reset Quadrants Struct*/
	}
	
	return 0;
}
/**Send Difference Array over Bluetooth
*
*Must give the Bluetooth Socket FD and have bluetooth connect or it will throw an
*error that will not be catched. 
*\param client an integer argument is the File Descriptor of the Bluetooth socket.
*\param uint8_t* difference is a pointer to the difference array.
*/
void sendDifference(int client, uint8_t* difference){
	uint32_t size = 1200*sizeof(uint8_t);
	int status=0;
	for(int i=0; i<size; i+=status){	
			if((status = write(client, difference+i, size-i))==-1){
				fprintf(stderr, "Failed to Send");
			}
		}
}
/**Finds Motion Quadrants in a Difference Image
*
*Main algorithm that uses a path finding method to encapsulate all the areas of motion in a difference image, i.e. the difference between two images. Returns to the Motion type the areas which motion occurs.
*\param uint8_t* difference pointer to difference array, must be of size 1200.
*\param Motion* quadrant pointer to typedef that contains motion quadrants
*/
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
/** Detects which direction motion is moving
*
*Takes the previous Motion typedef that contains the motion quadrant and compares
the current Motion list to see which direction the motion is moving. Only checks one dimensional case of direction, i.e. either right or left. The code is not quite right. It only checks the first element of the Motion types previous and current. Attempts are made to filter the signal using the type Directions which contains the previous direction values so that we can better guess the direction. 
*\param Motion* previous motion list
*\param Motion* current motion list
*\param Direction* array buffer contain previous motion directions
*\return int8_t 1 for Right and -1 for Left
*/
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
/**Detect in which of the Three Regions Motion Occurs
*Simply returns which region the motion is occuring. Only checks first element of 
Motion Type. We assume you have called combineQuadrants() before calling this function.
*\param Motion* current pointer
*\return int8_t 1 for Left, 0 for Center and -1 for Right
*/
int8_t detectMotionRegion(Motion* current){
	uint8_t position=0;
	position = current->x[0]+(current->width[0]>>2);
	if((position<=10)&(position>0)){
		return 1;
	}
	if((position>10)&(position<30)){
		return 0;
	}
	if(position>=30){
		return -1;
	}
	return -42;
}
/**Combines all Motion Quadrants
*Searches through the Motion list and finds the minimum X,Y vertex of motion and the maximum X,Y vertex. Clears the reference Motion list and sets the combined motion quadrant to first element of Motion list.
*\param Motion* quadrant pointer to Motion Type
*/
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
/**Filter the largest Quadrants in the Motion structure
*Note this function is not working quite right there is a bug in the logic, but since it was not crucial to the robot's
operation it was left alone. 
*\param Motion* quadrant which is a pointer to the Motion type you want to filter. 
*/
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
/**Simply Copies Contents of type Motion
*\param Motion* Current
*\param Motion* Previous
*/
void copyQuadrant(Motion* current, Motion* previous){
	for(int i =0; i<motionSize; i++){
		previous->x[i] = current->x[i];
		previous->y[i] = current->y[i];
		previous->width[i] = current->width[i];
		previous->height[i] = current->height[i];
	}
}
/**Checks to see if Point is already in a Motion Quadrant
*\param Motion* quadrants, pointer to quadrant list
*\param uint8_t x horizontal index value of point to be checked
*\param uint8_t y vertical index value of point to be checked
*/
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
/**Compares two Motion Quadrants in a Motion Type
*If the next motion quadrant is larger than the previous we return 1(True)
*\param Motion* quadrants. pointer to Motion type containing motion quadrants
*\param uint8_t ind, index of the Motion list
*\return 1 if current quadrant is greater than new quadrant. 0 if less.
*/
uint8_t quadrantCompare(Motion* quadrants, uint8_t ind){
	if((quadrants->width[ind]>quadrants->width[ind+1])&(quadrants->height[ind]>= quadrants->height[ind+1])){
		return 1;
	}
	else{
		return 0;
	}
}
/**Array Bound Check
*Checks to see if the point you accessing is actually a valid point in the difference array.
*\param uint8_t x value
*\param uint8_t y value
*/
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
/**Takes the Difference of Two Images
*Searches two images and comapres the pixel value. If the difference is less then a certain threshold we do nothing. If it is greater we set the corresponding index in the difference array to 1, indicating a difference in pixel intensity. Note we compress the difference to 40*30 list rather than the full image. THe image is searched in 16*16 quadrants. If more than half that quadrant has changed from frame to frame we set that quadrant high, i.e 1, else 0. 
*\param IplImage* frame1 pointer to first image
*\param IplImage* frame2 pointer to second image
*\param uint8_t* difference pointer to output array that contains the difference of the two images.
*/
void differenceFilter(IplImage* frame1, IplImage* frame2, uint8_t *difference){
	uint16_t check=0;
	uint16_t ind=0;
	for(uint8_t j =0; j<40; j++){
		for(uint8_t i =0; i<30; i++){
			for(uint8_t y=0; y<16;y++){
				for(uint8_t x=0; x<4; x++){
					if(abs(cvGetReal2D(frame1, y+i*16,j*16+4*x)-cvGetReal2D(frame2,i*16+y,j*16+4*x))> changeError){
						++check;
					}
				}
			}
			if(check>32){
				difference[ind]=1;
			}
			ind++;
			check=0;
		}
	}
}


// - InterfaceKit simple -
// This simple example simply creates an InterfaceKit handle, hooks the event handlers and opens it.  It then waits
// for an InterfaceKit to be attached and waits for events to be fired. We progress through three steps, 1. Normal settings, 
// 2. Setting analog sensor sensitivity to 100, 3. Toggling Ratiometric, waiting for user input to proceed to next step to allow 
// data to be read.
//
// Copyright 2008 Phidgets Inc.  All rights reserved.
// This work is licensed under the Creative Commons Attribution 2.5 Canada License. 
// view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/

//Alerts system that new device gets attached during execution
int CCONV AttachHandler(CPhidgetHandle IFK, void *userptr)
{
	int serialNo;
	const char *name;
    
	CPhidget_getDeviceName(IFK, &name);
	CPhidget_getSerialNumber(IFK, &serialNo);
    
	printf("%s %10d attached!\n", name, serialNo);
    
	return 0;
}

//Alerts system that a device has been detached during execution
int CCONV DetachHandler(CPhidgetHandle IFK, void *userptr)
{
	int serialNo;
	const char *name;
    
	CPhidget_getDeviceName (IFK, &name);
	CPhidget_getSerialNumber(IFK, &serialNo);
    
	printf("%s %10d detached!\n", name, serialNo);
    
	return 0;
}

//Error handling section
int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
	printf("Error handled. %d - %s", ErrorCode, unknown);
	return 0;
}

//callback that will run if an input changes.
//Index - Index of the input that generated the event, State - boolean (0 or 1) representing the input state (on or off)
int CCONV InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
{
	printf("Digital Input: %d > State: %d\n", Index, State);
	return 0;
}

//callback that will run if an output changes.
//Index - Index of the output that generated the event, State - boolean (0 or 1) representing the output state (on or off)
int CCONV OutputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
{
	//printf("Digital Output: %d > State: %d\n", Index, State);
	return 0;
}


//callback that will run if the sensor value changes by more than the OnSensorChange trigger.
//Index - Index of the sensor that generated the event, Value - the sensor read value
//This can also be used to monitor any device attached to the phidgets.
int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int index, int val1)
{
    int rawVal = 0;
	int dist;

    CPhidgetInterfaceKit_getSensorRawValue(IFK, index, &rawVal); 

	
	if(index == 0)
	{
		// add your code for sensor event here
		;
	}
	else if(index == 1)
	{
		// add your code for sensor event here
		;
	}
	else if(index == 2)
	{
		// add your code for sensor event here
		;
	}
	else if(index == 3)
	{
		// add your code for sensor event here
		;
	}	
	else if(index == 4) // Right Sensor
	{
		dist = SLOPE * rawVal + OFFSET; //convert raw value read by sensor to distance in inches
		//sensorValues[4] = (char)(dist/10 + 48);
		//sensorValues[5] = (char)((dist % 10) + 48);
		
		sensorValues[6] = (char)(dist/100 + 48);
		sensorValues[7] = (char)((dist % 100)/10 + 48);
		sensorValues[8] = (char)(((dist % 100) % 10) + 48);
		//printf("Right Sensor: %d inches\n", dist);

	}
	else if(index == 5) //Center Sensor
	{
		dist = SLOPE * rawVal + OFFSET; //convert raw value read by sensor to distance in inches
		//sensorValues[2] = (char)(dist/10 + 48);
		//sensorValues[3] = (char)((dist % 10) + 48);
		
		sensorValues[3] = (char)(dist/100 + 48);
		sensorValues[4] = (char)((dist % 100)/10 + 48);
		sensorValues[5] = (char)(((dist % 100) % 10) + 48);
		//printf("Center Sensor: %d inches\n", dist);

		if(dist <= MINDIST)
			frontStop = 1;
	}
	else if(index == 6) //Left Sensor
	{
		dist = SLOPE * rawVal + OFFSET; //convert raw value read by sensor to distance in inches
		//sensorValues[0] = (char)(dist/10 + 48);
		//sensorValues[1] = (char)((dist % 10) + 48);
		sensorValues[0] = (char)(dist/100 + 48);
		sensorValues[1] = (char)((dist % 100)/10 + 48);
		sensorValues[2] = (char)(((dist % 100) % 10) + 48);
		//printf("Left Sensor: %d inches\n", dist);
	}
	//else if(index == 7)
	//{
		//dist = RSLOPE * rawVal + ROFFSET;
		//sensorValues[9] = (char)(dist/100 + 48);
		//sensorValues[10] = (char)((dist % 100)/10 + 48);
		//sensorValues[11] = (char)(((dist % 100) % 10) + 48);
		//printf("Radar is reading: %d\n", dist);
		
		//if(dist <= MINDIST)
			//backStop = 1;
	//}


	return 0;
}

/** Stops All Motors
*/
int stop(CPhidgetInterfaceKitHandle ifKit)
{
    CPhidgetInterfaceKit_setOutputState(ifKit,0,PFALSE);
    CPhidgetInterfaceKit_setOutputState(ifKit,1,PFALSE);
    CPhidgetInterfaceKit_setOutputState(ifKit,2,PFALSE);
    CPhidgetInterfaceKit_setOutputState(ifKit,3,PFALSE); 
}

int pin(int h, CPhidgetInterfaceKitHandle ifKit, int state)
{
    CPhidgetInterfaceKit_setOutputState(ifKit, h, state);
    return 0;
}


//Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
//Will also display the number of inputs, outputs, and analog inputs on the interface kit as well as the state of the ratiometric flag
//and the current analog sensor sensitivity.
int display_properties(CPhidgetInterfaceKitHandle phid)
{
	int serialNo, version, numInputs, numOutputs, numSensors, triggerVal, ratiometric, i;
	const char* ptr;
    
	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);
    
	CPhidgetInterfaceKit_getInputCount(phid, &numInputs);
	CPhidgetInterfaceKit_getOutputCount(phid, &numOutputs);
	CPhidgetInterfaceKit_getSensorCount(phid, &numSensors);
	CPhidgetInterfaceKit_getRatiometric(phid, &ratiometric);
    
	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("# Digital Inputs: %d\n# Digital Outputs: %d\n", numInputs, numOutputs);
	printf("# Sensors: %d\n", numSensors);
	printf("Ratiometric: %d\n", ratiometric);
    
	for(i = 0; i < numSensors; i++)
	{
		CPhidgetInterfaceKit_getSensorChangeTrigger (phid, i, &triggerVal);
        
		printf("Sensor#: %d > Sensitivity Trigger: %d\n", i, triggerVal);
	}
    
	return 0;
}

int interfacekit_simple(CPhidgetInterfaceKitHandle ifKit)
{
	int result, numSensors, i;
	const char *err;
    
	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)ifKit, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)ifKit, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);
    
	//Registers a callback that will run if an input changes.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetInterfaceKit_set_OnInputChange_Handler (ifKit, InputChangeHandler, NULL);
    
	//Registers a callback that will run if the sensor value changes by more than the OnSensorChange trig-ger.
	//Requires the handle for the IntefaceKit, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetInterfaceKit_set_OnSensorChange_Handler (ifKit, SensorChangeHandler, NULL);
    
	//Registers a callback that will run if an output changes.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetInterfaceKit_set_OnOutputChange_Handler (ifKit, OutputChangeHandler, NULL);
	
	
    
	//open the interfacekit for device connections
	CPhidget_open((CPhidgetHandle)ifKit, -1);
    
	//get the program to wait for an interface kit device to be attached
	printf("Waiting for interface kit to be attached....");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)ifKit, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}
    
	//read interface kit event data
	printf("Reading.....\n");
    
	printf("Modifying sensor sensitivity triggers....\n");
    
	//get the number of sensors available
	CPhidgetInterfaceKit_getSensorCount(ifKit, &numSensors);
    
	//Change the sensitivity trigger of the sensors
	for(i = 0; i < numSensors; i++)
	{
		CPhidgetInterfaceKit_setSensorChangeTrigger(ifKit, i, 1);  //inch precision
	}
    
    //Display the properties of the attached interface kit device
	display_properties(ifKit);
    
	//read interface kit event data
	printf("Reading.....\n");
    
	//all done, exit
	return 0;
}
/**Turns Robot Right
*Uses delays to achieve this. We should really be using PWM signals.
*/
void turnRight(CPhidgetInterfaceKitHandle ifKit){
	stop(ifKit);
	//left motor off
	CPhidgetInterfaceKit_setOutputState(ifKit,0,PFALSE);
	CPhidgetInterfaceKit_setOutputState(ifKit,1,PTRUE);
	//right motor forwards
	CPhidgetInterfaceKit_setOutputState(ifKit,2,PFALSE) ;
	CPhidgetInterfaceKit_setOutputState(ifKit,3,PTRUE) ;
	counterLR++;
	delay(650);
	stop(ifKit);
}
/**Turns Robot Left
*Uses delays to achieve this. We should really be using PWM signals.
*/
void turnLeft(CPhidgetInterfaceKitHandle ifKit){
	stop(ifKit);	
	//left motor backwards
	CPhidgetInterfaceKit_setOutputState(ifKit,0,PTRUE);
	CPhidgetInterfaceKit_setOutputState(ifKit,1,PFALSE);
	//right motor forwards
	CPhidgetInterfaceKit_setOutputState(ifKit,2,PTRUE) ;
	CPhidgetInterfaceKit_setOutputState(ifKit,3,PFALSE) ;
	counterLR++;
	delay(650);
	stop(ifKit);
}
/**Moves Robot Forward
*Uses delays to achieve this. We should really be using PWM signals.
*/
void moveBothMotor(CPhidgetInterfaceKitHandle ifKit, int dir){
	if(dir==FORWARD){
		CPhidgetInterfaceKit_setOutputState(ifKit,0,PFALSE);
		CPhidgetInterfaceKit_setOutputState(ifKit,1,PTRUE);
		CPhidgetInterfaceKit_setOutputState(ifKit,2,PTRUE);
		CPhidgetInterfaceKit_setOutputState(ifKit,3,PFALSE);
		counterFB++;
	}
	if(dir==BACKWARD){
		CPhidgetInterfaceKit_setOutputState(ifKit,0,PTRUE);
		CPhidgetInterfaceKit_setOutputState(ifKit,1,PFALSE);
		CPhidgetInterfaceKit_setOutputState(ifKit,2,PFALSE);
		CPhidgetInterfaceKit_setOutputState(ifKit,3,PTRUE);
		counterFB++;
	}
	delay(250);
	stop(ifKit);
}
/**Software Delay Function
*Simply counts until the input value is reached. Uses Timer given by OS in time.h
*\param int milliseconds the desired delay
*/
void delay(int milliseconds)
{
    long pause;
    clock_t now,then;

    pause = milliseconds*(CLOCKS_PER_SEC/1000);
    now = then = clock();
    while( (now-then) < pause )
        now = clock();
}
