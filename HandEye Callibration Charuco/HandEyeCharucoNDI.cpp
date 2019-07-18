#include <Maxer_NDI_Tracker.h>
#include <cstring>
#include <stdio.h>                      // for fprintf, printf, sprintf, etc
#include <string.h>                     // for strncmp, strlen, strncpy
#include <iostream>
#include <fstream>
#include <vector>




#include <opencv2/aruco/charuco.hpp>
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
cv::Mat camMatrix, distCoeffs,TtoChessboard;


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






using namespace std;
using namespace cv;
using namespace ATLASMaxerOTS;

namespace {
const char* about = "Pose estimation using a ChArUco board";
const char* keys  =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{sl       |       | Square side length (in meters) }"
        "{ml       |       | Marker side length (in meters) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{c        |       | Output file with calibrated camera parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       |       | Apply refind strategy }"
        "{r        |       | show rejected candidates too }";
}

/**
 */
static bool readCameraParameters(string filename) {
    	FileStorage fs(filename, FileStorage::READ);
    	if(!fs.isOpened())
        	return false;
    	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	fs["TtoChessboard"] >> TtoChessboard;
	fs["lapCameraTracker"] >> lapCameraTrackerFileName;
	fs["telescopeTracker"] >> telescopeTrackerFileName;
	fs["chessboardTracker"] >> chessboardTrackerFileName;
    	return true;
}


/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}


/**
 */
int main(int argc, char *argv[]) {

    int squaresX = 6;
    int squaresY = 8;
    float squareLength = 10;
    float markerLength = 8;
    int dictionaryId = 1;
    bool showRejected = false;
    bool refindStrategy = true;
    int camId = 0;

 

        bool readOk = readCameraParameters("/home/faisal/HandEye Callibration Charuco/out_camera_data.xml");
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
	std::cout<<camMatrix<<std::endl<<std::endl<<distCoeffs<<std::endl;
    	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    
        readOk = readDetectorParameters("/home/faisal/HandEye Callibration Charuco/detector_params.yml", detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    


    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    VideoCapture inputVideo;
    int waitTime=10;
  
        inputVideo.open(camId);
        waitTime = 10;
	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH,1920);
	inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT,1080);
    float axisLength = 0.5f * ((float)min(squaresX, squaresY) * (squareLength));

    // create charuco board object
    Ptr<aruco::CharucoBoard> charucoboard =
        aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

    double totalTime = 0;
    int totalIterations = 0;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
	  std::cout<<lapCameraTrackerFileName<<std::endl;
	}
	ndiTracker.StartTracking();
	
	int status = 0;
	double poseBuffer[8];
	int quitKey = -1;
	bool allVisible = true;
	int captureCount = 1;
	std::ofstream posesFile;
	posesFile.open("../poses_data_text.txt");
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    while(inputVideo.grab()) 
	{
        	Mat image, imageCopy;
        	inputVideo.retrieve(image);

        	double tick = (double)getTickCount();

        	vector< int > markerIds, charucoIds;
        	vector< vector< Point2f > > markerCorners, rejectedMarkers;
        	vector< Point2f > charucoCorners;
        	Vec3d rvec, tvec;

        	// detect markers
        	aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams,
                             rejectedMarkers);

        	// refind strategy to detect more markers
        	if(refindStrategy)
            	aruco::refineDetectedMarkers(image, board, markerCorners, markerIds, rejectedMarkers,
                                         camMatrix, distCoeffs);

        	// interpolate charuco corners
        	int interpolatedCorners = 0;
        	if(markerIds.size() > 0)
            	interpolatedCorners =
                		aruco::interpolateCornersCharuco(markerCorners, markerIds, image, charucoboard,
                                                 charucoCorners, charucoIds, camMatrix, distCoeffs);

        	// estimate charuco board pose
        	bool validPose = false;
        	if(camMatrix.total() != 0)
            		{
				validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard,
                                                        camMatrix, distCoeffs, rvec, tvec);
			}


        	double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        	totalTime += currentTime;
        	totalIterations++;
        	if(totalIterations % 30 == 0) 
			{
//            			cout << "Detection Time = " << currentTime * 1000 << " ms "
//                 		<< "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
       			}



		// draw results
        	image.copyTo(imageCopy);
        	if(markerIds.size() > 0) 
			{
            			aruco::drawDetectedMarkers(imageCopy, markerCorners);
        		}

        	if(showRejected && rejectedMarkers.size() > 0)
            			aruco::drawDetectedMarkers(imageCopy, rejectedMarkers, noArray(), Scalar(100, 0, 255));

        	if(interpolatedCorners > 0) 
			{
            			Scalar color;
            			color = Scalar(255, 0, 0);
            			aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
        		}

        	if(validPose)
            			aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);
		//else
				//std::cout<<"in valid pose: " <<std::endl;

        	imshow("out", imageCopy);
        	char key = (char)waitKey(waitTime);
        	if(key == 27) break;
		if(key == 'c')
			{
				allVisible = true;
	  
	  
	  			status = recvNDIPose(ndiPortTelescopeTracker,poseBuffer);
	  			if(status==0)
	  				{
	    					telescopePose = getPoseMatrix(poseBuffer);
	  				}
	  			else
	  				{
	    					allVisible = false;
	  				}
	  
	  
			status = recvNDIPose(ndiPortChessboardTracker,poseBuffer);
	  		if(status==0)
	  			{
	   				chessboardPose  = getPoseMatrix(poseBuffer); 
	   				chessboardPose = chessboardPose*TtoChessboard.inv();
				}
	  		else
	  			{
	    				allVisible = false;
	  			}

			if(validPose && allVisible)
				{
					cv::Mat rotationM;
					cv::Rodrigues(rvec, rotationM);
					for(unsigned int row=0; row<3; ++row)
						{
							for(unsigned int col=0; col<3; ++col)
								{
									chessboardToLapCam.at<double>(row, col) = rotationM.at<double>(row, col);
								}
							chessboardToLapCam.at<double>(row, 3) = tvec(row);
						}
					chessboardToLapCam.at<double>(3, 3) = 1.0f;
					chessboardToLapCam.at<double>(3, 0) = 0.0f;
					chessboardToLapCam.at<double>(3, 1) = 0.0f;
					chessboardToLapCam.at<double>(3, 2) = 0.0f;
	    		
					std::ostringstream stm ;
	    				stm << captureCount ;
					posesFile<<"TrackerToLapCam_"+ stm.str()+ " =  "<< chessboardToLapCam*chessboardPose.inv()*telescopePose<<";\n\n";
	    				captureCount++;
					std::cout<<chessboardToLapCam<<std::endl;

				}

		

			}
    	}
	posesFile.close();
	ndiTracker.StopTracking();
	cap.release(); 
    	return 0;
}
