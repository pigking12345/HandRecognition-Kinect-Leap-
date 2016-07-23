#include "LProcessor.h"
#include <iostream>
#include "LeapToKinect.h"
using namespace std;
using namespace cv;

#define KLCALIBRATION_TIME 5

LProcessor::LProcessor(MyListener* pListener):
	m_pListener(pListener){
	m_startSec = 0; 
}

LProcessor::~LProcessor(){}

static bool compare_fingerVec(const Vec3d& first, const Vec3d& second){
	return first[0]<second[0]; 
}

bool LProcessor::storeHandData(){
	if(m_pListener->m_hFrameMutex==INVALID_HANDLE_VALUE)
		return false; 
	WaitForSingleObject(m_pListener->m_hFrameMutex, INFINITE); 
	Leap::Frame& frame = m_pListener->getLastFrame(); 

	if(!frame.isValid()){
		ReleaseMutex(m_pListener->m_hFrameMutex); 
		return false; 
	}


	Leap::HandList& handList = frame.hands(); 

	m_palmPositions.clear(); 
	m_fingerTips.clear(); 

	if(!handList.isEmpty()){
		Leap::Hand& leftHand = handList.leftmost();
		int leftHandId;
		if(leftHand.isValid()){
			leftHandId = leftHand.id();
			Leap::Vector transformedPoint;
			transformedPoint = m_pListener->scalePoint(leftHand.palmPosition());

			Vec3d leftPalm; 
			leftPalm[0] = transformedPoint.x;
			leftPalm[1] = transformedPoint.y;
			leftPalm[2] = transformedPoint.z; 
			m_palmPositions.push_back(leftPalm); 

			Leap::FingerList& fingers = leftHand.fingers();
			vector<Vec3d> tempFingers; 
			//m_leapFingerTips[0].resize(fingers.count()); 
			for (int i=0, e=(int)fingers.count(); i<e; i++){
				Vec3d tempFinger; 
				transformedPoint = m_pListener->scalePoint(fingers[i].tipPosition());
				// Store the palm position
				tempFinger[0] = transformedPoint.x; 
				tempFinger[1] = transformedPoint.y; 
				tempFinger[2] = transformedPoint.z; 
				tempFingers.push_back(tempFinger); 
			}
			// Even theres no finger, it should still push the vector
			m_fingerTips.push_back(tempFingers); 
		}

		Leap::Hand& rightHand = handList.rightmost(); 
		if(rightHand.isValid()&&rightHand.id()!=leftHandId){
			Leap::Vector transformedPoint;
			transformedPoint = m_pListener->scalePoint(rightHand.palmPosition());
			Vec3d rightPalm; 
			rightPalm[0] = transformedPoint.x;
			rightPalm[1] = transformedPoint.y;
			rightPalm[2] = transformedPoint.z; 
			m_palmPositions.push_back(rightPalm); 

			Leap::FingerList& fingers = rightHand.fingers();
			//m_leapFingerTips[1].resize(fingers.count()); 
			vector<Vec3d> tempFingers; 
			for (size_t i=0, e=fingers.count(); i<e; i++){
				Vec3d tempFinger;  
				transformedPoint = m_pListener->scalePoint(fingers[i].tipPosition());
				tempFinger[0] = transformedPoint.x; 
				tempFinger[1] = transformedPoint.y; 
				tempFinger[2] = transformedPoint.z; 
				tempFingers.push_back(tempFinger); 
			}
			m_fingerTips.push_back(tempFingers); 
		}

	}
	ReleaseMutex(m_pListener->m_hFrameMutex);
	for(int i=0; i<m_fingerTips.size(); i++){
		sort(m_fingerTips[i].begin(), m_fingerTips[i].end(), compare_fingerVec); 
	}
	return true; 
}

bool LProcessor::matching2KinectData(const vector<Vec3d>& Kpalms, const vector<vector<cv::Vec3d>>& KFingers){
	if(m_startSec==0){
		time(&m_startSec); 
		m_prevSec = m_startSec; 
		cout<<"Collecting data for matching Leap Motion to Kinect"<<endl; 
		cout<<KLCALIBRATION_TIME<<" sec left. "<<endl; 
	}
	
	time_t currentTime; 
	time(&currentTime); 

	if(currentTime-m_startSec<KLCALIBRATION_TIME){
		if(m_prevSec!=currentTime){
			cout<<KLCALIBRATION_TIME-(currentTime-m_startSec)<<" sec left. "<<endl; 
			m_prevSec = currentTime; 
		}
		int leapHandCount = (int)m_palmPositions.size(); 
		int kinectHandCount = (int)Kpalms.size(); 
		if(leapHandCount!=kinectHandCount||leapHandCount==0||kinectHandCount==0){
			return false; 
		}

		for(int i=0; i<leapHandCount; i++){
			if(m_fingerTips[i].size()!=5 || KFingers[i].size()!=5){
				return false; 
			}
		}

		for(int i=0; i<leapHandCount; i++){
			for(int j=0; j<5; j++){
				m_LSamples.push_back(Vec3d(m_fingerTips[i][j]));
				m_KSamples.push_back(Vec3d(KFingers[i][j]));
			}
		}

		return false; 
	}

	if(m_prevSec!=currentTime&&m_prevSec!=1){
		cout<<"Calibration finished. "<<endl; 
		if(m_LSamples.size()<3){
			cout<<"The collected data is sufficient. Re calibrating. "<<endl; 
			m_startSec = 0; 
			m_prevSec = 0; 
			return false; 
		}
		leapToKinectRT(m_LSamples, m_KSamples, m_R, m_T); 
		m_prevSec=1;  
	}

	m_transPalmPositions.clear(); 
	m_transFingerTips.clear(); 

	for(vector<Vec3d>::iterator it=m_palmPositions.begin(); it!=m_palmPositions.end(); it++){
		m_transPalmPositions.push_back(Vec3d()); 
		transformLeap(*it, m_transPalmPositions.back(), m_R, m_T); 
	}

	for(vector<vector<Vec3d>>::iterator it=m_fingerTips.begin(); it!=m_fingerTips.end(); it++){
		m_transFingerTips.push_back(vector<Vec3d>()); 
		transformLeap(*it, m_transFingerTips.back(), m_R, m_T); 
	}
	
	return true; 
}

const vector<Vec3d>& LProcessor::getTransPalmPos(){
	return m_transPalmPositions; 
}

const vector<vector<Vec3d>>& LProcessor::getTransFingers(){
	return m_transFingerTips; 
}