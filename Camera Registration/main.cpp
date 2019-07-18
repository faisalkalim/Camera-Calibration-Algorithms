#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>


#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>

#include <iostream>
#include<string>



#include "Maxer_NDI_Tracker.h"

//using namespace std;
using namespace cv;


	Eigen::MatrixXd m_pointerPositions;
	Eigen::MatrixXd m_ObjectPositions;
	std::vector<cv::Point3f> objectPoints;


	Eigen::MatrixXd WorldToChessboard;   //OTS To Chessboard
	Eigen::MatrixXd chessboardToLapCam;
	Eigen::MatrixXd trackerToLapCam;
	Eigen::MatrixXd worldToLapCamTracker;  // OTS to Laparoscopic Camera Tracker


	unsigned int ndiPortPointerTip = 0;
	unsigned int ndiPortChessboard = 0;
	unsigned int ndiPortTelescope = 0;

	cv::VideoCapture lapCam;





	cv::Mat cameraMatrix, distCoeffs;  // Camera Calibration parameters read from file
	cv::Size boardSize;				 
	int boardWidth,boardHeight,squareSize;
	int numChessboardCorners = 35;

	ATLASMaxerOTS::NDITracker ndiTracker = ATLASMaxerOTS::NDITracker();
				 


void addPointerPosition();
void readCameraParameters();
void addLapCameraPose();
void calculateTtoChessboard();
Eigen::Matrix4d readTransformFromNDI(int port, bool& markerPresent);


int main()
{
	std::cout<<"OTSToChessboard Inverse"<<std::endl<<std::endl;
	
	std::cout<<"OTSToChessboard Inverse"<<std::endl<<std::endl;
	if(!ndiTracker.FindDevices())	
	{
		//error = true;
        	std::cout<<"Starting NDI Search Incomplete"<<std::endl;			
	}
	else
    	{	
		const char* pointerFileName = "/home/faisal/Camera Registration/TrackingArrays/pointer.rom";
		const char* chessboardFileName = "/home/faisal/Camera Registration/TrackingArrays/RefCalibGridCharucoboard.rom";
		ndiTracker.Init();
		//ndiPortTelescope = ndiTracker.AddTool(fileName);    	// Add Pointer Tool Definition File
		ndiPortPointerTip = ndiTracker.AddTool(pointerFileName); 	// Add Chessboard Tool Definition File
		ndiPortChessboard = ndiTracker.AddTool(chessboardFileName);
		ndiTracker.StartTracking();
	}
	std::cout<<"OTSToChessboard Inverse"<<std::endl<<std::endl;
	lapCam = VideoCapture(0);

	if(!lapCam.isOpened())
		std::cout<<"Failed to Open Camera  "<<std::endl;

	readCameraParameters();
	for( int i = 0; i < boardSize.height; ++i )
	    for( int j = 0; j < boardSize.width; ++j )
	      {
		  objectPoints.push_back(cv::Point3f(j*squareSize, i*squareSize, 0));
	      }


	bool shouldQuit = false;
	
	cv::Mat currentFrame;

	lapCam.set(cv::CAP_PROP_FRAME_WIDTH,1920);
	lapCam.set(cv::CAP_PROP_FRAME_HEIGHT,1080);
 
	lapCam>>currentFrame;
	lapCam>>currentFrame;
	lapCam>>currentFrame;
	lapCam>>currentFrame;
	uchar key = 1;
	
	while(!shouldQuit)
	{
			lapCam>>currentFrame;
			imshow("Image ", currentFrame);
			key  = cv::waitKey(10); 
			if(key == 'p')
				addPointerPosition();
			else if(key == 't')
				calculateTtoChessboard();
			else if (key == 'q')
				shouldQuit = true; 
	}
	lapCam.release();
	ndiTracker.StopTracking();
return 0;
}


Eigen::Matrix4d readTransformFromNDI(int port, bool& markerPresent)	{
	Eigen::Matrix4d result;	
	double buffer[8];	
	int status = ndiTracker.GetTrackingData(port, buffer);
	markerPresent = markerPresent && (status == 0); //the logical and ensures I can use it again and again
	//retrieve the information from the buffer
	//First 4 values are the rotation in quaternion, second 3 values are transformation, last value is debug info
	//as the tracker already delivers a quaternion, this initialization should be ok
	Eigen::Quaternion<double> rot = Eigen::Quaternion<double>(buffer[0], buffer[1], buffer[2], buffer[3]);
	Eigen::Affine3d trans(Eigen::Translation3d(Eigen::Vector3d(buffer[4], buffer[5], buffer[6])));
	//std::cout<<"Tracking Quality:: "<<buffer[4]<<"  "<<buffer[5]<<"  "<<buffer[6]<<"  " <<std::endl;
	result = trans.matrix()*Eigen::Affine3d (rot.normalized().toRotationMatrix()).matrix() ;
	
	return result;
}


void addPointerPosition()
{
	
				Eigen::Matrix4d pointerTransform;
				bool markerPresent = true;
				pointerTransform = readTransformFromNDI(ndiPortPointerTip, markerPresent);
				Eigen::Matrix4d toolTipTransform; 
				Eigen::Matrix4d offsetTransform =Eigen::MatrixXd::Identity(4,4)	;


				offsetTransform(0,3) = -19.39;
				offsetTransform(1,3) =  0.29;
				offsetTransform(2,3) = -157.82;
				toolTipTransform = pointerTransform*offsetTransform;
				if(!markerPresent)
				{	
					std::cout << "Pointer not visible "<<ndiPortPointerTip << std::endl; 
					return;
				}
		
				if(m_pointerPositions.cols() == 0)
				{
					std::cout << "Init the matrix" << std::endl; 
				  	m_pointerPositions.resize(3 , 1);

				}
				else				
				{
					m_pointerPositions.conservativeResize(Eigen::NoChange , m_pointerPositions.cols() + 1);
				}
				std::cout <<"Number of Points :  "<< m_pointerPositions.cols() << std::endl;
				//auto devices  = m_liveTrackingStream->devices();
				//std::cout << " Number of devices" << devices.size() << std::endl; 
				m_pointerPositions.block<3,1>(0,m_pointerPositions.cols() -1 ) = toolTipTransform.block<3,1>(0,3);
				std::cout << "Pointer Position : " << std::endl << toolTipTransform.block<3,1>(0,3) << std::endl<<std::endl; 
}

void addLapCameraPose()
{


				bool markerPresent = true;
				Eigen::Matrix4d LapCamTrackerToWorld = readTransformFromNDI(ndiPortTelescope, markerPresent);

//				 mat4 LapCamTrackerToWorld  = devices.at(2)->matrix;  // to be replaced by our tracker
				 worldToLapCamTracker =LapCamTrackerToWorld.inverse() ;   

				 
				
				 

				 cv::Mat img,imgUndistorted;
				 std::vector<Point2f> imagePoints; 
				 if(lapCam.isOpened())
				 {
				   
				   
				   
				   for(int i = 0;i<5;i++)
				   lapCam >>img;
				   
				   std::cout << "Image saved:: /home/faisal/Downloads/image.jpg\n" ;
				   cv::imwrite("/home/faisal/Downloads/image.jpg",img);
				 }
				 else
				 {
				   std::cout << "Opening Laparoscopy Camera Failed \n";
				   return;
				 }

				 cv::undistort(img,imgUndistorted,cameraMatrix,distCoeffs);
				 cv::Mat rvec(3,1,cv::DataType<double>::type);
				 cv::Mat tvec(3,1,cv::DataType<double>::type);

				 bool found = findChessboardCorners( img, boardSize, imagePoints,0);
				 if(found)
				 {
				   
				   std::cout<<"Found Points"<<std::endl;
				   std::cout<<objectPoints.size()<<std::endl;
				   std::cout<<imagePoints.size()<<std::endl;
				   cv::solvePnP (objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec,  tvec, false, cv::SOLVEPNP_ITERATIVE);
				   //std::cout<<rvec<<std::endl<<tvec<<std::endl;
				   
				   cv::Mat rotation, viewMatrix(4, 4, CV_64F);
				   cv::Rodrigues(rvec, rotation);

				   for(unsigned int row=0; row<3; ++row)
				   {
				     for(unsigned int col=0; col<3; ++col)
				     {
					 viewMatrix.at<double>(row, col) = rotation.at<double>(row, col);
				     }
				     viewMatrix.at<double>(row, 3) = tvec.at<double>(row, 0);
				   }
				   viewMatrix.at<double>(3, 3) = 1.0f;
				   viewMatrix.at<double>(3, 0) = 0.0f;
				   viewMatrix.at<double>(3, 1) = 0.0f;
				   viewMatrix.at<double>(3, 2) = 0.0f;
				   //cv2eigen(viewMatrix,chessboardToLapCam);
				   std::cout<<"View Matrix"<<std::endl<<std::endl;
				   std::cout<<viewMatrix<<std::endl<<std::endl;
				 }
				 else
				 {
				   std::cout<<"not found"<<std::endl;
				 }				
}


void readCameraParameters()
{
				 
				 cv::FileStorage fs2("/home/faisal/Camera Registration/Calibration Parameters/out_camera_data.xml", cv::FileStorage::READ);  	
				 if (!fs2.isOpened())
				  {
				      std::cout << "Could not open the configuration file: \n" << std::endl;
				      return;
				  }
				 
				 
				 fs2["camera_matrix"] >> cameraMatrix;
				 fs2["distortion_coefficients"] >> distCoeffs;
				 
				 

				 fs2["board_width"] >> boardWidth;
				 fs2["board_height"] >> boardHeight;
				 fs2["square_size"] >> squareSize;
				 fs2.release();
				 boardSize = cv::Size(boardWidth,boardHeight);
				 
				 std::cout << "camera_matrix: " << cameraMatrix << std::endl
				 << "distortion coeffs: " << distCoeffs << std::endl
				 << "board_size: " << boardSize << std::endl
				 << "square_size: " << squareSize << std::endl;	
}


void calculateTtoChessboard()
{
			std::cout<<"here"<<std::endl;
			m_ObjectPositions.resize(3 , numChessboardCorners);
				
			std::cout<<m_ObjectPositions.cols() << "   " << m_pointerPositions.cols()<<std::endl;
			std::cout<<std::endl<< objectPoints <<std::endl;
			for(int i = 0; i<numChessboardCorners; i++)
				{
				   m_ObjectPositions(0,i)= objectPoints.at(i).x;
				   m_ObjectPositions(1,i)= objectPoints.at(i).y;
				   m_ObjectPositions(2,i)= objectPoints.at(i).z;
				}
			std::cout<<"here 3"<<std::endl;
			// cv2eigen(objectPointsCVMat,m_ObjectPositions);*/


			bool markerPresent = false;
			Eigen::Matrix4d OTSToChessboard  = Eigen::umeyama( m_pointerPositions,m_ObjectPositions);
			Eigen::Matrix4d TGridToWorld  = readTransformFromNDI(ndiPortChessboard, markerPresent);
			std::cout<<"TGridToWorld"<<std::endl<<std::endl;
			std::cout <<TGridToWorld<<std::endl<<std::endl;
   
   
			std::cout<<"here 4"<<std::endl;   
   
   			std::cout<<"OTSToChessboard Inverse"<<std::endl<<std::endl;
   			std::cout <<OTSToChessboard.inverse()<<std::endl<<std::endl;
			std::cout<<"here 5"<<std::endl;
   
   			//mat4 GridtoT = TGridToWorld.inverse()*OTSToChessboard.inverse();   /// determination of Grid to T using pointer method;
   			Eigen::Matrix4d TtoGrid = OTSToChessboard * TGridToWorld;
   
   			std::cout<<"TtoGrid"<<std::endl<<std::endl;
   			std::cout<<TtoGrid<<std::endl<<std::endl;
				  
}
