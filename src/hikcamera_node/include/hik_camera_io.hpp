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
	void get_img();
	//void grab_rgb_image();
    //bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
	unsigned char *pData;
	MVCC_INTVALUE stParam;
	MV_FRAME_OUT stImageInfo;
	void *handle;
private:
	int WidthValue;
    int HeightValue;

    float ExposureTimeValue;
    float GainValue;
	float newexposuretime;

	//unsigned char *pData;
	
	int nRet;
	
	
	MVCC_INTVALUE stIntVal;
	MVCC_FLOATVALUE stFloatVal;
	MV_CC_DEVICE_INFO_LIST stDeviceList;

	
};

#endif