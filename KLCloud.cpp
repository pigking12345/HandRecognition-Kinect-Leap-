#include "KAPI.h"

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

#include <string>
#include "KLCloud.h"

using namespace std; 
using namespace cv;

//------------------------------------------------------
//-------------- Pre-define Values ---------------------
//------------------------------------------------------
#define CLOUD_MAX_DEPTH 2.0f
#define CLOUD_NAME		"Kinect Cloud"
#define CLOUD_POINT_RADIUS 0.01f
#define CLOUD_LINE_STICKNESS 0.5f

//------------------------------------------------------
//----------- Initialize Static Variables --------------
//------------------------------------------------------
const float KLCloud::badFloat = std::numeric_limits<float>::quiet_NaN();

//------------------------------------------------------
//----------- Constructor & Destructor -----------------
//------------------------------------------------------
KLCloud::KLCloud(BYTE** ppColor, BYTE** ppDepth, LONG** ppCoordinates, int width, int height):
	m_pViewer(new pcl::visualization::PCLVisualizer("KL Viewer")),
	m_pCloud(new pcl::PointCloud<PointType>)
{
	m_ppColorBuffer = ppColor; 
	m_ppDepthBuffer = ppDepth; 
	m_ppColorCoordinates = ppCoordinates; 

	m_maxZ = CLOUD_MAX_DEPTH; 

	if(!m_ppColorBuffer||!m_ppDepthBuffer||!m_ppColorCoordinates)
		exit(-1); 

	m_width = width; 
	m_height= height; 

	// Initialize point cloud
	m_pCloud->width = width*height; 
	m_pCloud->height = 1; 
	m_pCloud->is_dense = false; 
	m_pCloud->points.resize(width*height); 

	// Initialize viewer
	m_pViewer->setBackgroundColor(0,0,0); 
	pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(m_pCloud);
	m_pViewer->addPointCloud<PointType>(m_pCloud, rgb, CLOUD_NAME);
	m_pViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, CLOUD_NAME);
	m_pViewer->initCameraParameters(); 
}

KLCloud::~KLCloud(){
	m_pViewer->removeAllPointClouds(); 
	m_pCloud->clear(); 
}

//------------------------------------------------------
//--------------- Draw Point CLoud ---------------------
//------------------------------------------------------
bool KLCloud::clearPrevDrawing(){
	PointType badPoint(0,0,0); 
	badPoint.x = badPoint.y = badPoint.z = badFloat; 
	m_pCloud->points.assign(m_pCloud->points.size(), badPoint); 
	m_pViewer->removeAllShapes(); 
	return true; 
}

bool KLCloud::drawKCLoud(){
	int pointIndex = 0, depthDataIndex = 0; 

	USHORT* depthBuffer = (USHORT*) (*m_ppDepthBuffer);

	for(DWORD y=0; y<m_height; y++){
		for(DWORD x=0; x<m_width; x++){
			// Process depth
			PointType& point = m_pCloud->points[pointIndex];
			Vector4 vec;

			// Transform 2D point into 3D
			vec = NuiTransformDepthImageToSkeleton(x,y,depthBuffer[depthDataIndex],NUI_IMAGE_RESOLUTION_640x480);
			if(vec.z<m_maxZ&&vec.z>0.4f){
				point.x = -vec.x;
				point.y = vec.y;
				point.z = vec.z;
				// Process color
				LONG colorX = (*m_ppColorCoordinates)[depthDataIndex*2];
				LONG colorY = (*m_ppColorCoordinates)[depthDataIndex*2 + 1]; 
				if(colorX>=0&&colorX<m_width&&colorY>=0&&colorY<m_height){
					LONG colorIndex = colorX + colorY*m_width;
					LONG colorValue = ((LONG*)(*m_ppColorBuffer))[colorIndex];
					point.r = (colorValue & 0x00ff0000UL)>>16;
					point.g = (colorValue & 0x0000ff00UL)>>8;
					point.b = (colorValue & 0x000000ffUL);
				}

			}
			pointIndex++; 
			depthDataIndex++;
		}
	}

	return true; 
}

static string strIdProducer(const string& prefixStr, int number){
	string result(prefixStr); 
	result+="_"; 
	result+=to_string(static_cast<long long>(number)); 
	return result; 
}

static string strIdProducer(const string& prefixStr, int number1, int number2){
	string result(prefixStr); 
	result+="_"; 
	result+=to_string(static_cast<long long>(number1)); 
	result+="_";
	result+=to_string(static_cast<long long>(number2)); 
	return result; 
}

bool KLCloud::drawLHand(const vector<Vec3d>& palms, const vector<vector<Vec3d>>& fingers){
	// Check if palms number matches hands number
	int handCount = (int)palms.size(); 
	if(fingers.size()!=handCount){
		cerr<<"Palms size conflicts with fingers size. "<<endl; 
		return false; 
	}

	for(int i=0; i<handCount; i++){
		PointType palmPoint; 
		palmPoint.x = -palms[i][0]; 
		palmPoint.y = palms[i][1]; 
		palmPoint.z = palms[i][2]; 
		m_pViewer->addSphere(palmPoint, CLOUD_POINT_RADIUS, 0.0, 0.0, 1.0, strIdProducer("Palm", i)); 
		
		int fingerCount = 0;
		for(vector<Vec3d>::const_iterator it=fingers[i].cbegin(); it!=fingers[i].cend(); it++){
			PointType fingerPoint; 
			fingerPoint.x = -(*it)[0]; 
			fingerPoint.y = (*it)[1]; 
			fingerPoint.z = (*it)[2]; 
			m_pViewer->addSphere(fingerPoint, CLOUD_POINT_RADIUS, 0.0, 0.0, 1.0, strIdProducer("FingerTips",i, fingerCount)); 
			m_pViewer->addLine(palmPoint, fingerPoint, 0.0, 0.0, 0.5, strIdProducer("Finger", i, fingerCount++)); 
		}
	}
	return true; 
}

bool KLCloud::updatePointCloud(const std::vector<cv::Vec3d>& palms, const std::vector<std::vector<cv::Vec3d>>& fingers){
	if(m_pViewer->wasStopped())
		return false; 
	if(!clearPrevDrawing())
		return false; 
	if(!drawKCLoud())
		return false; 
	if(!drawLHand(palms, fingers))
		return false; 
	m_pViewer->updatePointCloud(m_pCloud, CLOUD_NAME); 
	m_pViewer->spinOnce(); 
	return true; 
}
