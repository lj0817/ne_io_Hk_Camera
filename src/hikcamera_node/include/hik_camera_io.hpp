#ifndef HIKCAMERAIO
#define HIKCAMERAIO

#include <opencv2/opencv.hpp>

#include <string>
#include <cmath>
#include <chrono>

#include "MvCameraControl.h"



class HkCam
{
public:

	HkCam();
	~HkCam();
	void start();
	void shutdown();
	//void grab_rgb_image();
    //bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
	unsigned char *pData;
	MVCC_INTVALUE stParam;
	MV_FRAME_OUT stImageInfo ;
private:
	int WidthValue;
    int HeightValue;

    float ExposureTimeValue;
    float GainValue;
	float newexposuretime;

	void *handle;
	//unsigned char *pData;
	
	int nRet;
	
	
	MVCC_INTVALUE stIntVal;
	MVCC_FLOATVALUE stFloatVal;
	MV_CC_DEVICE_INFO_LIST stDeviceList;

	
};

#endif