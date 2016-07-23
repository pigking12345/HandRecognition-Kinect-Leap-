#pragma once
#ifndef KHANDDETECTOR_H
#define KHANDDETECTOR_H

#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "KAPI.h"
#include <ctime>

enum ParameterType{
	HMAX, 
	HMIN, 
	SMAX, 
	SMIN, 
	VMAX, 
	VMIN, 
	SIZETHRESH, 
	PARACOUNT
};

struct TriPtIndices{
	int mid; 
	int left; // larger than mid
	int right; // smaller than mid
};

class KHandDetector {
public: 
	KHandDetector(Kinect*, BYTE**, BYTE**, NUI_SKELETON_FRAME**, LONG**); 
	~KHandDetector(); 

	static void createControlWnd(); 
	bool runOnce(int drawOrNot=0); 
	
	void drawLeapMotionPoint(const std::vector<cv::Vec3d>&, const std::vector<std::vector<cv::Vec3d>>&); 

	const std::vector<cv::Vec3d>& getPalms(); 
	const std::vector<std::vector<cv::Vec3d>>& getFingerTips(); 
private:
	
	static void initParameters(); 
	static void onTrackbar(int, void*); 
	static void onMouse(int event, int x, int y, int, void*); 

	bool transColor2Mat(); 
	bool calibrateZ(); 
	bool maskColorZ(); 
	bool maskColorHSV(int drawOrNot=0); 


private:
	// Static Variables for parameter window
	static HANDLE						s_paraMutex; 
	static int							s_paraArray[PARACOUNT]; 
	static int							s_pParaArray[PARACOUNT]; // parameters protected by mutex 
	static bool							s_finishSetting; 

	// Variables about Kinect
	Kinect*								m_pKinect; 
	BYTE**								m_ppColorBuffer;
	BYTE**								m_ppDepthBuffer; 
	NUI_SKELETON_FRAME**				m_ppSkeletonFrame;
	LONG**								m_ppColorCoordinates;

	// Variables for hand detection
	time_t								m_startSec; 
	time_t								m_prevSec; 
	std::vector<float>					m_calibrationData; 
	int									m_calibrationTimes; 
	float								m_ZThreshold; 

	cv::Mat								m_colorMat; 
	cv::Mat								m_color2DepthMat; 

	cv::Mat								m_hsvMat; 
	cv::Mat								m_skinColorMask; 
	std::vector<std::vector<cv::Point>>	m_contours;	

	std::vector<std::vector<cv::Point>>	m_filteredContours;
	std::vector<std::vector<cv::Point>> m_hulls; 
	std::vector<std::vector<int>>		m_hullIndices;
	std::vector<std::vector<cv::Vec4i>>	m_defects; 
	std::vector<std::vector<cv::Point>>	m_palmPoly; 
	std::vector<cv::Vec3d>				m_palmPosition3D; 
	std::vector<std::vector<cv::Point>> m_fingerTips2D;
	std::vector<std::vector<cv::Vec3d>> m_FingerTips3D; 

};
#endif