#pragma once
#ifndef LPROCESSOR_H
#define LPROCESSOR_H
#include "LListener.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <ctime>

class LProcessor {
public:
	LProcessor(MyListener* pListener); 
	~LProcessor(); 
	bool storeHandData(); 
	bool matching2KinectData(const std::vector<cv::Vec3d>&, const std::vector<std::vector<cv::Vec3d>>&); 

	const std::vector<cv::Vec3d>& getTransPalmPos(); 
	const std::vector<std::vector<cv::Vec3d>>& getTransFingers(); 



private:
	MyListener*							m_pListener; 
	std::vector<cv::Vec3d>				m_palmPositions; 
	std::vector<std::vector<cv::Vec3d>> m_fingerTips; 
	
	time_t								m_startSec; 
	time_t								m_prevSec; 
	cv::Mat								m_R; 
	cv::Mat								m_T; 
	std::vector<cv::Vec3d>				m_LSamples; 
	std::vector<cv::Vec3d>				m_KSamples; 
	
	std::vector<cv::Vec3d>				m_transPalmPositions; 
	std::vector<std::vector<cv::Vec3d>> m_transFingerTips; 
};

#endif