#include "KHandDetector.h"
#include "DBSCAN.h"

using namespace cv; 
using namespace std;
//------------------------------------------------------
//--------- Initialize static Variables ----------------
//------------------------------------------------------
HANDLE	KHandDetector::s_paraMutex = INVALID_HANDLE_VALUE;  
int KHandDetector::s_paraArray[PARACOUNT] = {0};  
int KHandDetector::s_pParaArray[PARACOUNT] = {0}; 
bool KHandDetector::s_finishSetting = false; 


//------------------------------------------------------
//------------- Predefine Values -----------------------
//------------------------------------------------------
#define DEFAULT_Z_THRESHOLD 0.73f
#define CALIBRATION_OFFSET	0.10f
#define CALIBRATE_MAX_RETRY 3
#define FINGERTIP_ANGLE_THRESHOLD 60
#define PI 3.14159265
#define FINGERTIP_MIN_DISTANCE 5
#define CONTOUR_STEP 2.0f
#define DEFECT_POINT_DEPTH_THRESHOLD 2000
#define DEFECT_POINT_COSINE_MINIMUM -0.5f
#define FINGERTIP_LEFT_RIGHT_AREA 3


//------------------------------------------------------
//-------------- Parameter Window ----------------------
//------------------------------------------------------
const char* PAINTWINDOW = "PAINT_WINDOW_NAME";
const char* BARWINDOW = "BAR_WINDOW_NAME"; 
const char* H_MIN_BAR = "H_MIN"; 
const char* H_MAX_BAR = "H_MAX"; 
const char* S_MIN_BAR = "S_MIN"; 
const char* S_MAX_BAR = "S_MAX"; 
const char* V_MIN_BAR = "V_MIN"; 
const char* V_MAX_BAR = "V_MAX"; 
const char* SIZE_THRESH_BAR = "SIZE_THRESH"; 

void KHandDetector::initParameters(){
	s_paraArray[HMIN] = 70; 
	s_paraArray[HMAX] = 130;
	s_paraArray[SMIN] = 40; 
	s_paraArray[SMAX] = 230;
	s_paraArray[VMIN] = 10; 
	s_paraArray[VMAX] = 200;
	s_paraArray[SIZETHRESH] = 3000;

	for(int i=0; i<PARACOUNT; i++){
		s_pParaArray[i] = s_paraArray[i]; 
	}
}

void KHandDetector::createControlWnd(){
	initParameters(); 
	namedWindow(BARWINDOW, CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED); 
	createTrackbar(H_MIN_BAR,		BARWINDOW, &s_paraArray[HMIN],		 180,					  onTrackbar); 
	createTrackbar(H_MAX_BAR,		BARWINDOW, &s_paraArray[HMAX],		 180,					  onTrackbar); 
	createTrackbar(S_MIN_BAR,		BARWINDOW, &s_paraArray[SMIN],		 255,					  onTrackbar); 
	createTrackbar(S_MAX_BAR,		BARWINDOW, &s_paraArray[SMAX],		 255,					  onTrackbar); 
	createTrackbar(V_MIN_BAR,		BARWINDOW, &s_paraArray[VMIN],		 255,					  onTrackbar); 
	createTrackbar(V_MAX_BAR,		BARWINDOW, &s_paraArray[HMAX],		 255,					  onTrackbar); 
	createTrackbar(SIZE_THRESH_BAR, BARWINDOW, &s_paraArray[SIZETHRESH], 5000,					  onTrackbar); 
	s_paraMutex = CreateMutex(NULL, false, NULL); 
	if(s_paraMutex == NULL){
		s_paraMutex = INVALID_HANDLE_VALUE; 
		cerr<<"Create parameter mutex fail. "<<endl; 
	}
	moveWindow(BARWINDOW, 1000, 0); 
	onTrackbar(0,NULL);

}

void KHandDetector::onTrackbar(int, void*){
	if(s_paraArray[HMAX]<1)
		s_paraArray[HMAX]=1; 
	if(s_paraArray[HMIN]>s_paraArray[HMAX]-1)
		s_paraArray[HMIN]=s_paraArray[HMAX]-1; 
	if(s_paraArray[SMAX]<1)
		s_paraArray[SMAX]=1; 
	if(s_paraArray[SMIN]>s_paraArray[SMAX]-1)
		s_paraArray[SMIN]=s_paraArray[SMAX]-1; 
	if(s_paraArray[VMAX]<1)
		s_paraArray[VMAX]=1; 
	if(s_paraArray[VMIN]>s_paraArray[VMAX]-1)
		s_paraArray[VMIN]=s_paraArray[VMAX]-1; 
	//createTrackbar(H_MIN_BAR,		BARWINDOW, &s_paraArray[HMIN],		 s_paraArray[HMAX],		  onTrackbar); 
	//createTrackbar(S_MIN_BAR,		BARWINDOW, &s_paraArray[SMIN],		 s_paraArray[SMAX],		  onTrackbar); 
	//createTrackbar(V_MIN_BAR,		BARWINDOW, &s_paraArray[VMIN],		 s_paraArray[VMAX],		  onTrackbar); 
	WaitForSingleObject(s_paraMutex, INFINITE);
	// Assign the parameter values to variables accessible by objects
	for(int i=0; i<PARACOUNT; i++){
		s_pParaArray[i] = s_paraArray[i]; 
	}
	ReleaseMutex(s_paraMutex);
}

void KHandDetector::onMouse(int event, int x, int y, int, void*){
	if(event==EVENT_LBUTTONDBLCLK){
		s_finishSetting = true; 
	}
}


//--------------------------------------------------------------
//---------------- Constructor & Destructor --------------------
//--------------------------------------------------------------
KHandDetector::KHandDetector(Kinect* pK, BYTE** ppColor, BYTE** ppDepth, NUI_SKELETON_FRAME** ppSkeleton, LONG** ppCoordinate){
	m_pKinect = pK; 
	m_ppColorBuffer = ppColor; 
	m_ppDepthBuffer = ppDepth; 
	m_ppSkeletonFrame = ppSkeleton; 
	m_ppColorCoordinates = ppCoordinate; 

	// Initialize Z threshold
	m_ZThreshold = DEFAULT_Z_THRESHOLD; 
	m_startSec = 0; 
	m_calibrationData.clear(); 
	m_calibrationTimes = 0; 

	// Initialize Matrices
	m_colorMat.create(m_pKinect->getColorHeight(), m_pKinect->getColorWidth(), CV_8UC4); 
	m_color2DepthMat.create(m_pKinect->getColorHeight(), m_pKinect->getColorWidth(), CV_8UC3); 
	m_hsvMat.create(m_colorMat.rows, m_colorMat.cols, CV_8UC3); 
	m_skinColorMask.create(m_hsvMat.rows, m_hsvMat.cols, CV_8U); 

}

KHandDetector::~KHandDetector(){
	if(s_paraMutex!=NULL&&s_paraMutex!=INVALID_HANDLE_VALUE){
		CloseHandle(s_paraMutex); 
		s_paraMutex = INVALID_HANDLE_VALUE; 
	}

}

//------------------------------------------------------------
//----------- Algorithms used for pre detection --------------
//------------------------------------------------------------
bool KHandDetector::transColor2Mat(){
	if(!m_pKinect||!m_ppColorBuffer||!m_ppDepthBuffer||!m_ppSkeletonFrame||!m_ppColorCoordinates)
		return false;

	INT colorBufferPitch = m_pKinect->getColorPitch();

	if (colorBufferPitch == 0)
	{
		return false;
	}

	for(UINT y=0, height = m_pKinect->getColorHeight(); y < height; ++y){
		Vec4b* pColorRow = m_colorMat.ptr<Vec4b>(y);

		for (UINT x = 0, width = m_pKinect->getColorWidth(); x < width; ++x)
		{

			pColorRow[x] = Vec4b((*m_ppColorBuffer)[y * colorBufferPitch + x * 4 + 0],
				(*m_ppColorBuffer)[y * colorBufferPitch + x * 4 + 1],
				(*m_ppColorBuffer)[y * colorBufferPitch + x * 4 + 2],
				(*m_ppColorBuffer)[y * colorBufferPitch + x * 4 + 3]);
		}
	}

	return true; 
}

bool KHandDetector::calibrateZ(){
	if(m_startSec==0){
		time(&m_startSec); 
		m_prevSec = 0; 
	}
	time_t currentTime; 
	time(&currentTime); 
	if(currentTime-m_startSec<5 && currentTime!=m_prevSec){
		cout<<"Calibration starts after "<<currentTime-m_startSec<<" seconds. "<<endl; 
		m_prevSec = currentTime; 
		return false; 
	}
	if(currentTime-m_startSec<10){
		if(currentTime!=m_prevSec){
			cout<<"Calibration ends in "<<currentTime-m_startSec-5<<" seconds. "<<endl; 
			m_prevSec = currentTime; 
		}

		bool getDataOrNot = false; 
		for (int i = 0; i<NUI_SKELETON_COUNT; i++)
		{
			NUI_SKELETON_DATA& skeletonData = (*m_ppSkeletonFrame)->SkeletonData[i];
			NUI_SKELETON_TRACKING_STATE trackingState = skeletonData.eTrackingState;

			if (NUI_SKELETON_TRACKED == trackingState)
			{
				Vector4 leftHandPos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT]; 
				Vector4 rightHandPos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT];

				if(skeletonData.eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_HAND_LEFT] == NUI_SKELETON_POSITION_TRACKED){
					m_calibrationData.push_back(leftHandPos.z); 
					getDataOrNot = true;
				}
				if(skeletonData.eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_HAND_RIGHT] == NUI_SKELETON_POSITION_TRACKED){
					m_calibrationData.push_back(rightHandPos.z); 
					getDataOrNot = true; 
				}
				if(getDataOrNot)
					break; 
			}
		}

		return false; 
	}

	if(currentTime-m_startSec==10){
		cout<<"Collected "<<m_calibrationData.size()<<" data. "<<endl; 
		if(m_calibrationData.size()==0){
			if(m_calibrationTimes<CALIBRATE_MAX_RETRY){
				cout<<"The number of collected data is not sufficient. Re-calibrate. "<<endl; 
				m_startSec = 0;
				m_prevSec = 0; 
				m_calibrationTimes++; 
				return false; 
			}
			else{
				cout<<"The calibration do not work properly. Use default threshold. "<<endl; 
				m_startSec = 1; 
				m_prevSec = 1; 
				return true; 
			}
		}
		DBSCAN dbscan(m_calibrationData, 0.005, 10);
		vector<float> output_data; 
		dbscan.clustering(output_data); 
		m_ZThreshold = *max_element(output_data.begin(), output_data.end())+CALIBRATION_OFFSET;
		cout<<"Z threshold is "<<m_ZThreshold<<endl; 
		cout<<"Calibration finished. "<<endl; 
		m_startSec = 1; 
		m_prevSec = 1; 
		namedWindow(PAINTWINDOW); 
		setMouseCallback(PAINTWINDOW, onMouse, 0); 
		return true; 
	}

	return true; 
} 

bool KHandDetector::maskColorZ(){
	HRESULT hr; 

	INT width = m_pKinect->getColorWidth();
	INT height = m_pKinect->getColorHeight(); 

	USHORT* depthBuffer = (USHORT*) (*m_ppDepthBuffer);

	INuiSensor* sensor = m_pKinect->getSensor();
	hr = sensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
		NUI_IMAGE_RESOLUTION_640x480,
		NUI_IMAGE_RESOLUTION_640x480,
		width*height,
		depthBuffer,
		width*height*2,
		*m_ppColorCoordinates
		);
	if(FAILED(hr)){
		cout<<"Fail to map color to depth. "<<endl;
		return false;  
	}

	for(int y=0; y<height; y++){
		for(int x=0; x<width; x++){
			//USHORT depthValue = NuiDepthPixelToDepth(depthBuffer[y*width+x]);
			Vector4 vec =  NuiTransformDepthImageToSkeleton(x,y,depthBuffer[y*width+x],NUI_IMAGE_RESOLUTION_640x480);
			if(vec.z<m_ZThreshold){
				LONG colorX = (*m_ppColorCoordinates)[(y*width+x)*2];
				LONG colorY = (*m_ppColorCoordinates)[(y*width+x)*2 + 1]; 
				if(colorX>=0&&colorX<width&&colorY>=0&&colorY<height){
					LONG colorIndex = colorX + colorY*width;
					LONG colorValue = ((LONG*)(*m_ppColorBuffer))[colorIndex];
					m_color2DepthMat.at<Vec3b>(y,x) = Vec3b((colorValue & 0x000000ffUL),
						(colorValue & 0x0000ff00UL)>>8,
						(colorValue & 0x00ff0000UL)>>16); 
				}
				else{
					m_color2DepthMat.at<Vec3b>(y,x) = Vec3b(0,0,0); 
				}
			}
			else{
				m_color2DepthMat.at<Vec3b>(y,x) = Vec3b(0,0,0);
			}
		}
	}
	return true; 
}

//------------------------------------------------------------
//----------- Algorithms used for hand detection -------------
//------------------------------------------------------------
static void findInscribedCircle(vector<Point>& contour, const vector<int>& hullIndices, Point& center, double& radius){
	if(hullIndices.size()<2||contour.size()<2)
		return; 

	int max_x=0, min_x=10000, max_y=0, min_y=10000; 

	for(int i=0, tempSize=hullIndices.size(); i<tempSize; i++){
		Point hullPoint = contour[hullIndices[i]];
		if(hullPoint.x<min_x)
			min_x = hullPoint.x;
		if(hullPoint.x>max_x)
			max_x = hullPoint.x;
		if(hullPoint.y>max_y)
			max_y = hullPoint.y;
		if(hullPoint.y<min_y)
			min_y = hullPoint.y; 
	}

	double dist, maxdist = -1;
	for(int j = min_x;j< max_x;j+=5){
		for(int k = min_y;k< max_y;k+=5){
			dist = pointPolygonTest(contour, cv::Point(j,k),true);
			if(dist > maxdist)
			{
				maxdist = dist;
				center = cv::Point(j,k);
			}
		}
	}
	if(maxdist>0){
		radius = maxdist; 
		return; 
	}

}

static bool compare_ptIndex(const TriPtIndices& first, const TriPtIndices& second){
	return first.mid<second.mid; 
}

static bool compare_fingerVec(const Vec3d& first, const Vec3d& second){
	return first[0]<second[0]; 
}

static double threePointCosine(const Point& left, const Point& mid, const Point& right){
	Point leftVec = left-mid; 
	Point rightVec = right-mid; 
	double consine = leftVec.ddot(rightVec)/(norm(leftVec)*norm(rightVec)); 
	return consine; 
}

static bool testInCircle(const Point& testPoint, const Point& center, const double& radius){
	Point diff = testPoint - center; 
	if(diff.ddot(diff)>radius*radius)
		return false; 
	return true; 
}

static void findFingerTipsDraw(const vector<Point>& contour, const vector<Vec4i>& defects, vector<Point>& fingerVector, const int depthThreshold, const Point& palmCenter, const double& radius, Mat& drawing){
	if(contour.size()<2||defects.size()<1)
		return; 
	
	// Initialize fingerTIpsList
	list<TriPtIndices> fingerTipsList; 
	fingerTipsList.clear(); 
	int previousIndex = 0; 
	int currentIndex = 0; 
	for(int i=0, tempSize=defects.size(); i<tempSize; i++){
		Matx<int,4,1> defectVector = defects[i];

		if((defectVector.val[3])<=depthThreshold){continue; }
		if(threePointCosine(contour[defectVector.val[0]],contour[defectVector.val[2]],contour[defectVector.val[1]])<DEFECT_POINT_COSINE_MINIMUM)
			continue; 
		if(testInCircle(contour[defectVector.val[0]],palmCenter, radius*1.5)||
			testInCircle(contour[defectVector.val[1]],palmCenter, radius*1.5))
			continue; 


		TriPtIndices ptInd1, ptInd2; 
		ptInd1.mid = defectVector.val[0]; 
		ptInd1.right = defectVector.val[2]; 
		ptInd1.left = -1; 
		ptInd2.mid = defectVector.val[1]; 
		ptInd2.left = defectVector.val[2]; 
		ptInd2.right = -1; 
		fingerTipsList.push_back(ptInd1);
		fingerTipsList.push_back(ptInd2);
	}

	fingerTipsList.sort(compare_ptIndex); 

	for(list<TriPtIndices>::iterator it=fingerTipsList.begin(), previt = fingerTipsList.begin(); it!=fingerTipsList.end();){
		currentIndex = it->mid; 
		if(it!=fingerTipsList.begin()){ // skip the first point
			if(currentIndex-previousIndex<FINGERTIP_MIN_DISTANCE){
				previt->mid = (currentIndex+previousIndex)/2; 
				if(previt->left==-1&&it->left!=-1)
					previt->left = it->left;
				else if(previt->right==-1&&it->right!=-1)
					previt->right = it->right; 
				currentIndex = previt->mid; 
				fingerTipsList.erase(it);
				it = previt; 
			}
		}
		previousIndex = currentIndex; 
		previt = it; 
		it++; 
	}

	// compare the head and the tail
	if(fingerTipsList.size()>1){
		list<TriPtIndices>::iterator cBegin = fingerTipsList.begin();
		list<TriPtIndices>::iterator  cEnd = fingerTipsList.end();
		cEnd--; 

		int conSize = contour.size(); 

		if(cBegin->mid+conSize-cEnd->mid<FINGERTIP_MIN_DISTANCE){
			cBegin->mid = ((cBegin->mid+conSize+cEnd->mid)/2)%conSize; 
			if(cBegin->left==-1&&cEnd->left!=-1)
				cBegin->left = cEnd->left; 
			else if(cBegin->right==-1&&cEnd->right!=-1)
				cBegin->right = cEnd->right; 
			fingerTipsList.erase(cEnd); 
		}
	}

	for(list<TriPtIndices>::iterator it=fingerTipsList.begin(); it!=fingerTipsList.end(); it++){
		if(it->left==-1){
			if(it->mid>it->right && it->mid-it->right<fingerTipsList.size()/2)
				it->left = (it->mid+FINGERTIP_LEFT_RIGHT_AREA)%contour.size();
			else
				it->left = (it->mid-FINGERTIP_LEFT_RIGHT_AREA+contour.size())%contour.size(); 
		}
		if(it->right==-1)
			if(it->mid>it->left&&it->mid-it->left<contour.size()/2)
				it->right = (it->mid+FINGERTIP_LEFT_RIGHT_AREA)%contour.size();
			else
				it->right = (it->mid-FINGERTIP_LEFT_RIGHT_AREA+contour.size())%contour.size(); 
	}

	// Calculate angle
	for(list<TriPtIndices>::iterator it=fingerTipsList.begin(); it!=fingerTipsList.end(); it++){
		// Determine if it is fingertip by angle
		Point midPoint = contour[it->mid]; 
		Point leftPoint = contour[it->left];
		Point rightPoint = contour[it->right];
		double cosine = threePointCosine(leftPoint, midPoint, rightPoint); 
		if(cosine>cos((double)FINGERTIP_ANGLE_THRESHOLD*PI/180.0)){
			fingerVector.push_back(midPoint); 
			circle(drawing, midPoint, 4,Scalar(0,255,0),-1); 
		}
	}
}

bool KHandDetector::maskColorHSV(int drawOrNot){
	if(!s_finishSetting)
		WaitForSingleObject(s_paraMutex, INFINITE); 
	cvtColor(m_color2DepthMat, m_hsvMat, CV_RGB2HSV); 
	Scalar hMin(s_pParaArray[HMIN], s_pParaArray[SMIN] , s_pParaArray[VMIN]);
	Scalar hMax(s_pParaArray[HMAX], s_pParaArray[SMAX], s_pParaArray[VMAX]);
	double sizeThreshold = (double)s_pParaArray[SIZETHRESH]; 
	if(!s_finishSetting)
		ReleaseMutex(s_paraMutex); 

	inRange(m_hsvMat, hMin, hMax, m_skinColorMask); 

	// Find contours
	vector<cv::Vec4i> hierarchy; 
	findContours( m_skinColorMask, m_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// Ignore contours with small size
	m_filteredContours.clear(); 
	for(int i=0, contourCount= m_contours.size(); i<contourCount; i++){
		// Calculate size
		double contourSize = contourArea(m_contours[i]);
		if(contourSize>sizeThreshold){
			vector<Point> tempContour; 
			approxPolyDP(m_contours[i], tempContour, CONTOUR_STEP, true); 
			m_filteredContours.push_back(tempContour); 
			drawContours( m_color2DepthMat, m_filteredContours, m_filteredContours.size()-1, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point() );
		}
	}

	// m_filteredContours.size should be the same as the number of hands detected. 
	m_hullIndices.clear(); 
	m_hullIndices.resize(m_filteredContours.size());
	m_defects.clear();
	m_defects.resize(m_filteredContours.size());
	m_palmPoly.clear(); 
	m_palmPoly.resize(m_filteredContours.size());
	m_fingerTips2D.clear();
	m_fingerTips2D.resize(m_filteredContours.size()); 
	m_palmPosition3D.clear(); 
	m_palmPosition3D.resize(m_filteredContours.size()); 
	m_FingerTips3D.clear();
	m_FingerTips3D.resize(m_filteredContours.size()); 

	for(int i=0; i<m_filteredContours.size(); i++){
		// Process for each hand
		convexHull(Mat(m_filteredContours[i]), m_hullIndices[i], true);
		convexityDefects(m_filteredContours[i], m_hullIndices[i], m_defects[i]); 

		// Find the palm position
		Point palmCenter; 
		double palmRadius; 
		findInscribedCircle(m_filteredContours[i], m_hullIndices[i], palmCenter, palmRadius); 
		circle(m_color2DepthMat, palmCenter, palmRadius, cv::Scalar(220,75,20));
		circle(m_color2DepthMat, palmCenter, palmRadius*1.5, cv::Scalar(255,255,255));
		circle(m_color2DepthMat, palmCenter, 5, cv::Scalar(0,0,255), -1);

		// Transform 2d palm position into 3D
		INT width = m_pKinect->getColorWidth();
		INT height = m_pKinect->getColorHeight(); 
		USHORT* depthBuffer = (USHORT*) (*m_ppDepthBuffer);
		Vector4 palm3d =  NuiTransformDepthImageToSkeleton(palmCenter.x,palmCenter.y,depthBuffer[palmCenter.y*width+palmCenter.x],NUI_IMAGE_RESOLUTION_640x480);
		m_palmPosition3D[i][0] = palm3d.x; 
		m_palmPosition3D[i][1] = palm3d.y;
		m_palmPosition3D[i][2] = palm3d.z; 

		// Find finger tips 2d position 
		findFingerTipsDraw(m_filteredContours[i], m_defects[i], m_fingerTips2D[i], DEFECT_POINT_DEPTH_THRESHOLD, palmCenter, palmRadius, m_color2DepthMat); 
		// Transform into 3d
		m_FingerTips3D[i].resize(m_fingerTips2D[i].size()); 
		for(int j=0; j<m_fingerTips2D[i].size(); j++){
			Point ft = m_fingerTips2D[i][j]; 
			Vector4 vec =  NuiTransformDepthImageToSkeleton(ft.x,ft.y,depthBuffer[ft.y*width+ft.x],NUI_IMAGE_RESOLUTION_640x480);
			m_FingerTips3D[i][j][0] = vec.x; 
			m_FingerTips3D[i][j][1] = vec.y; 
			m_FingerTips3D[i][j][2] = vec.z; 
		}
		sort(m_FingerTips3D[i].begin(), m_FingerTips3D[i].end(), compare_fingerVec); 
	}

	for(int i=0; i<m_palmPosition3D.size(); i++){
		for(int j=i; j<m_palmPosition3D.size(); j++){
			if(m_palmPosition3D[i][0]>m_palmPosition3D[j][0]){
				swap(m_palmPosition3D[i], m_palmPosition3D[j]); 
				swap(m_FingerTips3D[i], m_FingerTips3D[j]); 
			}
		}
	}

	if(drawOrNot==0)
		imshow(PAINTWINDOW, m_color2DepthMat); 

	if(!s_finishSetting){
		return false; 
	}

	return true; 
}

void KHandDetector::drawLeapMotionPoint(const vector<Vec3d>& palms, const vector<vector<Vec3d>>& fingers){
	int handCount = (int)palms.size(); 
	if(fingers.size()!=handCount){
		cerr<<"Palms size conflicts with fingers size. "<<endl; 
		return;  
	}

	for (int i=0; i<handCount; i++){
		Vector4 skeletonPoint;
		// Draw palm
		{
			const Vec3d& palmPoint = palms[i]; 
			skeletonPoint.x = palmPoint[0];
			skeletonPoint.y	= palmPoint[1];
			skeletonPoint.z	= palmPoint[2]; 
			LONG depthX, depthY; 
			USHORT depthVal; 
			NuiTransformSkeletonToDepthImage(skeletonPoint, &depthX, &depthY, &depthVal, NUI_IMAGE_RESOLUTION_640x480);
			circle(m_color2DepthMat, Point(depthX, depthY), 5, cv::Scalar(255,0,0),-1);

		}
		for(vector<Vec3d>::const_iterator it=fingers[i].cbegin(); it!=fingers[i].cend(); it++){
			skeletonPoint.x = (*it)[0]; 
			skeletonPoint.y = (*it)[1]; 
			skeletonPoint.z = (*it)[2]; 
			LONG depthX, depthY; 
			USHORT depthVal; 
			NuiTransformSkeletonToDepthImage(skeletonPoint, &depthX, &depthY, &depthVal, NUI_IMAGE_RESOLUTION_640x480);
			circle(m_color2DepthMat, Point(depthX, depthY), 5, cv::Scalar(255,0,0),-1);
		}

	}

	imshow(PAINTWINDOW, m_color2DepthMat); 
}

//------------------------------------------------------------
//----------------- Other public functions -------------------
//------------------------------------------------------------
bool KHandDetector::runOnce(int drawOrNot){
	if(!transColor2Mat())
		return false; 
	if(!calibrateZ())
		return false; 
	if(!maskColorZ())
		return false;
	return maskColorHSV(drawOrNot); 
}

const vector<Vec3d>& KHandDetector::getPalms(){
	return m_palmPosition3D; 
}

const vector<vector<Vec3d>>& KHandDetector::getFingerTips(){
	return m_FingerTips3D; 
}