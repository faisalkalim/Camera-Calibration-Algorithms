// A simple program that connects to an NDI tracker
#include <Maxer_NDI_Tracker.h>
#include <cstring>
#include <stdio.h>                      // for fprintf, printf, sprintf, etc
#include <string.h>                     // for strncmp, strlen, strncpy
#include <iostream>
#include <fstream>

#include <iomanip>
#include <ctime>
#include <chrono>

#include <mutex>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>



bool shouldQuit = false;
ATLASMaxerOTS::NDITracker  ndiTracker = ATLASMaxerOTS::NDITracker();
cv::VideoCapture cap;

int ndiPortLapCameraTracker  = -1;
int ndiPortTelescopeTracker  = -1;
int ndiPortChessboardTracker = -1;






cv::Mat currentImg;
cv::Mat cameraMatrix, distCoeffs,TtoChessboard;
	int boardWidth,boardHeight,squareSize;

std::string lapCameraTrackerFileName;
std::string telescopeTrackerFileName;
std::string chessboardTrackerFileName;

// cv::Mat lapCameraPose (4, 4, CV_64F);
cv::Mat telescopePose (4, 4, CV_64F);
cv::Mat chessboardPose(4, 4, CV_64F);
cv::Mat chessboardToLapCam(4, 4, CV_64F);



int recvNDIPose(unsigned int portnumber, double buffer[8])	{
  
		  memset(buffer, 0, 8 * sizeof(double));
		  return ndiTracker.GetTrackingData(portnumber,buffer);

	
}
cv::Mat getPoseMatrix(double buffer[8])
{
		  Eigen::Quaternion<double> rot = Eigen::Quaternion<double>(buffer[0], buffer[1], buffer[2], buffer[3]);
		  Eigen::Affine3d trans(Eigen::Translation3d(Eigen::Vector3d(buffer[4], buffer[5], buffer[6])));
		  
		  Eigen::Matrix4d result = trans.matrix()*Eigen::Affine3d (rot.normalized().toRotationMatrix()).matrix() ;
	
		  auto ndiPose = cv::Mat(4, 4, CV_64F, result.data());
		  cv::transpose(ndiPose,ndiPose);
		  //std::cout<<lapCameraPose<<std::endl;
		  return ndiPose.clone();
  
}

//using namespace cv;

using namespace ATLASMaxerOTS;
int main ()

{
  
	cap.open(1);
	if(!cap.isOpened())
	{
	  std::cout<<"Camera Not Opened: Quitting"<<std::endl;
	  return -1;
	}
	cap.set(cv::CAP_PROP_FRAME_WIDTH,1920);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT,1080);
	std::cout<<"Opened Video Camera"<<std::endl;
	cv::FileStorage configFile("../out_camera_data.xml", cv::FileStorage::READ);
	//cv::FileStorage posesFile("../poses_data.xml", cv::FileStorage::WRITE);
	
	std::ofstream posesFile;
	posesFile.open("../poses_data_text.txt");
				 if (!configFile.isOpened())
				  {
				      std::cout << "Could not open the configuration file: \n" << std::endl;
				      return -1;
				  }
	configFile["camera_matrix"] >> cameraMatrix;
	configFile["distortion_coefficients"] >> distCoeffs;	
	configFile["board_width"] >> boardWidth;
	configFile["board_height"] >> boardHeight;
	configFile["square_size"] >> squareSize;
	std::cout<<"cameraMatrix\t"<<cameraMatrix <<std::endl;
	std::cout<<"distCoeffs\t"<<distCoeffs <<std::endl;
	std::cout<<"squareSize\t"<< squareSize<<std::endl;
	
	
	configFile["TtoChessboard"] >> TtoChessboard;
	
	
	configFile["lapCameraTracker"] >> lapCameraTrackerFileName;
	configFile["telescopeTracker"] >> telescopeTrackerFileName;
	configFile["chessboardTracker"] >> chessboardTrackerFileName;
	configFile.release();
			
	
	
	bool okay= ndiTracker.FindDevices();
	if(!okay)
	{
	  std::cout<<"NDI Tracker could not find devices"<<std::endl;
	  return -1;
	}
	okay = ndiTracker.Init();
	if(!okay)
	{
	  std::cout<<"NDI Tracker initialization failed"<<std::endl;
	  return -1;
	}
	if(okay)
	{
	  ndiPortLapCameraTracker = ndiTracker.AddTool(const_cast<char*>(lapCameraTrackerFileName.c_str()));  
	  ndiPortTelescopeTracker = ndiTracker.AddTool(const_cast<char*>(telescopeTrackerFileName.c_str()));
	  ndiPortChessboardTracker = ndiTracker.AddTool(const_cast<char*>(chessboardTrackerFileName.c_str()));
	}
	ndiTracker.StartTracking();
	
	int status = 0;
	double poseBuffer[8];
	int quitKey = -1;
	bool allVisible = true;
	int captureCount = 1;
	while(true)
	{
	  cap>>currentImg;
	  imshow("Live Video",currentImg);
	  allVisible = true;
	  
	  
	  status = recvNDIPose(ndiPortTelescopeTracker,poseBuffer);
	  if(status==0)
	  {
	    //std::cout<<"Telescope Visible"<<std::endl;
	    telescopePose = getPoseMatrix(poseBuffer);
	  }
	  else
	  {
	    allVisible = false;
	  }
	  
	  
	  status = recvNDIPose(ndiPortChessboardTracker,poseBuffer);
	  if(status==0)
	  {
	    
	   //std::cout<<"Chessboard Visible"<<std::endl;
	   chessboardPose  = getPoseMatrix(poseBuffer); 
	   chessboardPose = chessboardPose*TtoChessboard.inv();
	   
	   
	  }
	  else
	  {
	    allVisible = false;
	  }
	  
	  
	  quitKey =cv::waitKey(5);
	  if(quitKey>0)
	  {
	    if(quitKey =='q')
	      break;
	    if(!allVisible)
	    {
	      std::cout<<"All markerst not visible"<<std::endl;
	      continue;
	    }
	    std::ostringstream stm ;
	    stm << captureCount ;
	    cv::imwrite("../Image"+ stm.str()+".jpg",currentImg);
	    
	    
	    
	    //////////////////////////////////////////////////////////////////////// Determine chessboardPose in laparoscopic camera coordinate system
	    cv::Size boardSize = cv::Size(boardWidth,boardHeight);
	    cv::Mat imgUndistorted;
	    std::vector<cv::Point2f> imagePoints; 

	    std::vector<cv::Point3f> objectPoints;
	    for( int i = 0; i < boardSize.height; ++i )
	    for( int j = 0; j < boardSize.width; ++j )
	    {
		  objectPoints.push_back(cv::Point3f(j*squareSize, i*squareSize, 0));
	    }
	    
	    
				 cv::undistort(currentImg,imgUndistorted,cameraMatrix,distCoeffs);
				 imshow("Undistorted", imgUndistorted);
				 bool found = findChessboardCorners( currentImg, boardSize, imagePoints,cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
				 if(found)
				 {
   				   cv::Mat rotation;
				   cv::Mat rvec(3,1,CV_64F);
				   cv::Mat tvec(3,1,CV_64F);
				   cv::solvePnP (objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec,  tvec, false, cv::SOLVEPNP_ITERATIVE);
				   cv::Rodrigues(rvec, rotation);
				      for(unsigned int row=0; row<3; ++row)
				      {
					for(unsigned int col=0; col<3; ++col)
					{
					   chessboardToLapCam.at<double>(row, col) = rotation.at<double>(row, col);
					}
					chessboardToLapCam.at<double>(row, 3) = tvec.at<double>(row, 0);
				      }
				   chessboardToLapCam.at<double>(3, 3) = 1.0f;
				   chessboardToLapCam.at<double>(3, 0) = 0.0f;
				   chessboardToLapCam.at<double>(3, 1) = 0.0f;
				   chessboardToLapCam.at<double>(3, 2) = 0.0f;
				   //std::cout<<"Printing for debugging:: tvec"<<std::endl<<tvec<<std::endl;
				   
				 }
				 else
				 {
				   std::cout<<"Patern could not be detected"<<std::endl;
				   continue;
				 }
	    
	    ////////////////////////////////////////////////////////////////////////
	    std::cout<<"Writing to Text File"<<std::endl;
	    //posesFile<<"lapCameraPose_"+ stm.str()+ " =  "<< lapCameraPose <<"\n\n";
	    //posesFile<<"telescopePose_"+ stm.str()+" =  "<< telescopePose<<"\n\n";
	    //posesFile<<"chessboardPose_"+ stm.str()+" =  "<< chessboardPose<<"\n\n";
	    //posesFile<<"chessboardToLapCam_"+ stm.str()+ " =  "<< chessboardToLapCam<<"\n\n";
// 	    if(captureCount==1)
// 	    {
// 	      posesFile<<"lapCameraPose_"+ stm.str()+ " =  "<< lapCameraPose <<"\n\n";
// 	    }
	    posesFile<<"TrackerToLapCam_"+ stm.str()+ " =  "<< chessboardToLapCam*chessboardPose.inv()*telescopePose<<";\n\n";
	    
	    
	    captureCount++;
	    
// 	    std::cout<<"Printing Lap Camera Pose"<<std::endl;
// 	    std::cout<<lapCameraPose<<std::endl;
// 	    
// 	    std::cout<<"Printing Telescope Pose"<<std::endl;
// 	    std::cout<<telescopePose<<std::endl;
// 	    
// 	    
// 	    std::cout<<"Printing chessboard Pose"<<std::endl;
// 	    std::cout<<chessboardPose<<std::endl;
// 	    
	    std::cout<<"Printing chessboardToLapCam"<<std::endl;
	    std::cout<<chessboardToLapCam<<std::endl;
 	    cv::waitKey(500);
	    
	  }
	   
	  
	  
	  
	}
	
	//posesFile.release();
	posesFile.close();
	ndiTracker.StopTracking();
	cap.release(); 
  


  return 0;
}
