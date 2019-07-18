#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <ctime>


#include <Maxer_NDI_Tracker.h>


#include <Eigen/Geometry>
#include <Eigen/Dense>



using namespace std;
using namespace cv;
using namespace ATLASMaxerOTS;


ATLASMaxerOTS::NDITracker  ndiTracker = ATLASMaxerOTS::NDITracker();


struct calibParams 
{
	int squaresX; 
	int squaresY;	
	float squareLength;
	float markerLength;
	int dictionaryId;
	
	string outputFile;
	string detectorParamsFile;
	string imageCapturePath;

	string lapCameraTrackerFileName;
	string telescopeTrackerFileName;
	string charucoBoardTrackerFileName;

	string zeroDegreePoseFile;
	string TtoCharucoBoardFile;
	string calibratedParametersFile;
	string handEyePosesFile;

	int numImages;
	int numAngles;
	int camID;

	bool refindStrategy;
	bool zeroRotationCalibration;
	bool useIntrinsicGuess;
	bool fixFocalLength;
	bool zeroTangentialDistortions;

};

int ndiPortLapCameraTracker  = -1;
int ndiPortTelescopeTracker  = -1;
int ndiPortCharucoBoardTracker = -1;

cv::Mat lapCameraPose (4, 4, CV_64F);
cv::Mat telescopePose (4, 4, CV_64F);
cv::Mat chessboardToLapCam(4, 4, CV_64F);
cv::Mat chessboardPose(4, 4, CV_64F);


cv::Mat cameraMatrixZeroDegRot;
cv::Mat distortion_coefficientsZeroDegRot; 
cv::Mat zeroDegreeRelPose;
cv::Mat TtoCharucoBoard;


int recvNDIPose(unsigned int portnumber, double buffer[8]);
cv::Mat getPoseMatrix(double buffer[8]);
void saveZeroRotationCameraTrackerPose(calibParams &params);


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

static bool readCalibrationParameters(string filename, calibParams &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        {
		
		return false;
	}
    int rand;
    fs["BoardSize_Width"] >> params.squaresX;
    fs["BoardSize_Height"] >> params.squaresY;
    fs["Square_Size"] >> params.squareLength;
    fs["Marker_Size"] >> params.markerLength;
    fs["DictionaryID"] >> params.dictionaryId;

    fs["outputFile"] >>params.outputFile;
    //std::cout<<rand<<std::endl;
    fs["detectorParameters"] >> params.detectorParamsFile;
    fs["imageCapturePath"] >> params.imageCapturePath;

    fs["lapCameraTracker"] >> params.lapCameraTrackerFileName;
    fs["telescopeTracker"] >> params.telescopeTrackerFileName;
    fs["charucoboarTracker"] >> params.charucoBoardTrackerFileName;

    fs["Calibrate_NrOfFramesToUse"] >> params.numImages;
    fs["Calibrate_NrOfAnglesToUse"] >> params.numAngles;
    fs["CameraID"] >> params.camID;

    fs["Refind_Strategy"] >> params.refindStrategy;
    fs["Calibrate_Zero_Degree_Position"] >> params.zeroRotationCalibration;
    fs["Use_intrinsic_guess"] >> params.useIntrinsicGuess;
    fs["Fix_Focal_Length"] >> params.fixFocalLength; 
    fs["Calibrate_AssumeZeroTangentialDistortion"] >> params.zeroTangentialDistortions; 


    //fs["perspectiveRemoveIgnoredMarginPerCell"] >> params.cameraMatrixZeroDegRot;
    //fs["maxErroneousBitsInBorderRate"] >> params.distortion_coefficientsZeroDegRot;  
    //fs["maxErroneousBitsInBorderRate"] >> params.TtoCharucoBoard; 

    fs["zeroDegreePose"] >> params.zeroDegreePoseFile;
    fs["TtoCharucoBoard"] >> params.TtoCharucoBoardFile;
    fs["calibrationParams"] >> params.calibratedParametersFile;
    fs["hand_eye_calibrations"] >> params.handEyePosesFile;
    

    return true;
}



/**
 */
static bool saveCameraParams(const string &filename, Size imageSize, float aspectRatio, int flags,
                             const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr) {
    FileStorage fs(filename, FileStorage::WRITE);
    if(!fs.isOpened())
        return false;

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    if(flags & CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

    if(flags != 0) {
        sprintf(buf, "flags: %s%s%s%s",
                flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}



/**
 */


int main(int argc, char *argv[]) {



		    
		    Mat cameraMatrixCalibrated, distCoeffsCalibrated;    
		    calibParams cParams = calibParams();
		    readCalibrationParameters("/home/faisal/Calibration Parameters/default.xml", cParams);



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
	  				ndiPortLapCameraTracker = ndiTracker.AddTool(const_cast<char*>(cParams.lapCameraTrackerFileName.c_str()));  
					ndiPortTelescopeTracker = ndiTracker.AddTool(const_cast<char*>(cParams.telescopeTrackerFileName.c_str()));
	  				ndiPortCharucoBoardTracker = ndiTracker.AddTool(const_cast<char*>(cParams.charucoBoardTrackerFileName.c_str()));
	  				std::cout<<cParams.lapCameraTrackerFileName<<std::endl;
				}
			ndiTracker.StartTracking();


		    bool showChessboardCorners = true;
		    //std::cout<<cParams.squaresX<<std::endl;
		    int calibrationFlags = 0;
		    float aspectRatio = 1;
		    int waitTime = 10;


    		    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
		    bool readOk = readDetectorParameters(cParams.detectorParamsFile, detectorParams);
		    if(!readOk) 
			{
		        	cerr << "Invalid detector parameters file" << endl;
		        	return 0;
		        }
		    VideoCapture inputVideo;
		    std::cout<<"Camera ID:  "<<cParams.camID<<std::endl;
		    inputVideo.open(cParams.camID);
		    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH,1920);
		    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT,1080);

		    Ptr<aruco::Dictionary> dictionary =
			        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cParams.dictionaryId));

		    // create charuco board object
		    Ptr<aruco::CharucoBoard> charucoboard =
		    		aruco::CharucoBoard::create(cParams.squaresX, cParams.squaresY, cParams.squareLength, cParams.markerLength, dictionary);
		    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

		if(cParams.zeroRotationCalibration)
			{
		    	
				saveZeroRotationCameraTrackerPose(cParams);
				if(cParams.zeroTangentialDistortions) calibrationFlags |= CALIB_ZERO_TANGENT_DIST;
		    		vector< vector< vector< Point2f > > > allCorners;
		    		vector< vector< int > > allIds;
		    		vector< Mat > allImgs;
		    		Size imgSize;

		    		while(inputVideo.grab()) 
					{
		        			Mat image, imageCopy;
		        			inputVideo.retrieve(image);
						vector< int > ids;
		        			vector< vector< Point2f > > corners, rejected;

		        			// detect markers		
		        			aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
	
		        			// refind strategy to detect more markers
		        			if(cParams.refindStrategy) aruco::refineDetectedMarkers(image, board, corners, ids, rejected);
	
		        			// interpolate charuco corners
		        			Mat currentCharucoCorners, currentCharucoIds;
		        			if(ids.size() > 0)
		            				aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners,
		                                        		     currentCharucoIds);

		        			// draw results
		        			image.copyTo(imageCopy);
		        			if(ids.size() > 0) aruco::drawDetectedMarkers(imageCopy, corners);

		        			if(currentCharucoCorners.total() > 0)
		            				aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);

		        			putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
		                		Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

		        			imshow("out", imageCopy);
		        			char key = (char)waitKey(waitTime);
		        			if(key == 27) break;
		        			if(key == 'c' && ids.size() > 0) 
							{
		            					cout << "Frame captured" << endl;
		            					allCorners.push_back(corners);
		            					allIds.push_back(ids);
		            					allImgs.push_back(image);
		            					imgSize = image.size();
		        				}
		    			}

		    		if(allIds.size() < 1) 
					{
					        cerr << "Not enough captures for calibration" << endl;
					        return 0;
		    			}
	
		    		Mat cameraMatrix, distCoeffs;
		    		vector< Mat > rvecs, tvecs;
		    		double repError;
		
		
		    		// prepare data for calibration
		    		vector< vector< Point2f > > allCornersConcatenated;
		    		vector< int > allIdsConcatenated;
		    		vector< int > markerCounterPerFrame;
		    		markerCounterPerFrame.reserve(allCorners.size());
		    		for(unsigned int i = 0; i < allCorners.size(); i++) 
					{
		        			markerCounterPerFrame.push_back((int)allCorners[i].size());
					        for(unsigned int j = 0; j < allCorners[i].size(); j++) 
							{
						            allCornersConcatenated.push_back(allCorners[i][j]);
						            allIdsConcatenated.push_back(allIds[i][j]);
						        }
					 }

		    		// calibrate camera using aruco markers
		    		double arucoRepErr;
		    		arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
		                                              markerCounterPerFrame, board, imgSize, cameraMatrix,
		                                              distCoeffs, noArray(), noArray(), calibrationFlags);

		    		// prepare data for charuco calibration
		    		int nFrames = (int)allCorners.size();
		    		vector< Mat > allCharucoCorners;
		    		vector< Mat > allCharucoIds;
		    		vector< Mat > filteredImages;
		    		allCharucoCorners.reserve(nFrames);
		    		allCharucoIds.reserve(nFrames);
		    		for(int i = 0; i < nFrames; i++) 
					{
	        				// interpolate using camera parameters
					        Mat currentCharucoCorners, currentCharucoIds;
					        aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard,
					                                         currentCharucoCorners, currentCharucoIds, cameraMatrix,
					                                         distCoeffs);

					        allCharucoCorners.push_back(currentCharucoCorners);
					        allCharucoIds.push_back(currentCharucoIds);
					        filteredImages.push_back(allImgs[i]);
					}
	
			    	if(allCharucoCorners.size() < 4) 
					{
		        			cerr << "Not enough corners for calibration" << endl;
					        return 0;
					}

		    		// calibrate camera using charuco
		    		repError =  aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize,
		                            		    	      cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

		    		bool saveOk =  saveCameraParams(cParams.outputFile, imgSize, aspectRatio, calibrationFlags,
		                				                    cameraMatrix, distCoeffs, repError);
		    		if(!saveOk) 
					{
					        cerr << "Cannot save output file" << endl;
		        			return 0;
		    			}
	
		    		cout << "Rep Error: " << repError << endl;
				cout << "Rep Error Aruco: " << arucoRepErr << endl;
		    		cout << "Calibration saved to " << cParams.outputFile << endl;
				cameraMatrixCalibrated = cameraMatrix.clone();
				distCoeffsCalibrated = distCoeffs.clone();
		    		// show interpolated charuco corners for debugging
		    		if(showChessboardCorners) 
					{
		        			for(unsigned int frame = 0; frame < filteredImages.size(); frame++) 
							{
		        		    			Mat imageCopy = filteredImages[frame].clone();
				            			if(allIds[frame].size() > 0) 
									{
					        	        		if(allCharucoCorners[frame].total() > 0) 
											{
							                    			aruco::drawDetectedCornersCharuco( imageCopy, allCharucoCorners[frame],
									                        	                               allCharucoIds[frame]);
		                							}
		            						}
		
		            					imshow("out", imageCopy);
		            					char key = (char)waitKey(0);
		            					if(key == 27) break;
		        				}
		    			}

			}


	if(true) // do hand eye calibration
			
		{


	
			int status = 0;
			double poseBuffer[8];
			int quitKey = -1;
			bool allVisible = true;
			int captureCount = 1;
			std::ofstream posesFile;
			posesFile.open(cParams.handEyePosesFile);
			cv::Mat cameraMatrix = cameraMatrixCalibrated.clone();
			cv::Mat distCoeffs =distCoeffsCalibrated.clone() ;
			FileStorage fs(cParams.TtoCharucoBoardFile, FileStorage::READ);
    			if(!fs.isOpened())
        				{
						return false;
					}
    					fs["TtoCharucoBoard"] >> TtoCharucoBoard;
					fs.release();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
        				if(cParams.refindStrategy)
            					aruco::refineDetectedMarkers(image, board, markerCorners, markerIds, rejectedMarkers,
                                         			cameraMatrix, distCoeffs);

        				// interpolate charuco corners
        				int interpolatedCorners = 0;
        				if(markerIds.size() > 0)
            					interpolatedCorners = aruco::interpolateCornersCharuco(markerCorners, markerIds, image, charucoboard,
                                                 			charucoCorners, charucoIds, cameraMatrix, distCoeffs);

        				// estimate charuco board pose
        				bool validPose = false;
        				if(cameraMatrix.total() != 0)
            					{
							validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard,
                                                        	cameraMatrix, distCoeffs, rvec, tvec);
						}



					// draw results
        				image.copyTo(imageCopy);
        				if(markerIds.size() > 0) 
						{
            						aruco::drawDetectedMarkers(imageCopy, markerCorners);
        					}

        				
        				if(interpolatedCorners > 0) 
						{
            						Scalar color;
            						color = Scalar(255, 0, 0);
            						aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
        					}

        				if(validPose)
            						aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 20);
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
									std::cout<<"Telescope Marker Not Visible"<<std::endl;
	  							}
	  
	  
							status = recvNDIPose(ndiPortCharucoBoardTracker,poseBuffer);
	  						if(status==0)
	  							{
	   								chessboardPose  = getPoseMatrix(poseBuffer); 
	   								chessboardPose = chessboardPose*TtoCharucoBoard.inv();
								}
	  						else
	  							{
	    								allVisible = false;
									std::cout<<"Telescope Marker Not Visible"<<std::endl;
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

			inputVideo.release();
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		}

    ndiTracker.StopTracking();
    return 0;
}
void calibrateHandeye(calibParams &params, const Mat &cameraMatrix, const Mat &distCoeffs)
{

}

int recvNDIPose(unsigned int portnumber, double buffer[8])	
{  
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

void saveZeroRotationCameraTrackerPose(calibParams &params)
{
	
	int status = 0;
	double poseBuffer[8];
	int quitKey = -1;
	
	int captureCount = 1;
	while(true)
	{
	  bool allVisible = true;
	  status = recvNDIPose(ndiPortLapCameraTracker,poseBuffer);
	  if(status==0)
	  {
	    
	    //std::cout<<"Camera Visible"<<std::endl;
	    lapCameraPose = getPoseMatrix(poseBuffer);
	  }
	  else
	  {
	    allVisible = false;
	  }
	  
	  
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
	  if(allVisible)
	  {
	    FileStorage fs(params.zeroDegreePoseFile, FileStorage::WRITE); 
	    fs<<"ZeroDegreeCameraPose"<<telescopePose.inv()*lapCameraPose;
	    fs.release();
	    break;
	  }
	  else
	  {
	    std::cout<<"The Markers are not visible"<<std::endl;
	  }
	}
}

double  findAngleOfRotationCameraTrackerPose(calibParams &params)
{
  double rotationAngle;
  cv::Mat zeroDegreeCameraPose;
  cv::Mat currentCameraPose;
  FileStorage fs("../ZeroDegreeCameraPose.xml", FileStorage::READ); // Read the settings
	    fs["ZeroDegreeCameraPose"]>>zeroDegreeCameraPose;
	    fs.release();
	   
  /*bool okay= ndiTracker.FindDevices();
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
	  ndiPortLapCameraTracker = ndiTracker.AddTool(const_cast<char*>(params.lapCameraTrackerFileName.c_str()));  
	  ndiPortTelescopeTracker = ndiTracker.AddTool(const_cast<char*>(params.telescopeTrackerFileName.c_str()));
	}
	ndiTracker.StartTracking();*/
	
	int status = 0;
	double poseBuffer[8];
	int quitKey = -1;
	int captureCount = 1;
	bool allVisible;
	while(true)
	{
	  allVisible = true; 
	  status = recvNDIPose(ndiPortLapCameraTracker,poseBuffer);
	  if(status==0)
	  {
	    
	    //std::cout<<"Camera Visible"<<std::endl;
	    lapCameraPose = getPoseMatrix(poseBuffer);
	  }
	  else
	  {
	    allVisible = false;
	  }
	  
	  
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
	  if(allVisible)
	  {
	    currentCameraPose  = telescopePose.inv()*lapCameraPose;
	 
	  Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor>> zeroDegreePose(zeroDegreeCameraPose.ptr<double>());
	  Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor>> currentPose(currentCameraPose.ptr<double>());
	  
	  
	  Eigen::Quaternion<double> q0(zeroDegreePose.block<3,3>(0,0));
	  Eigen::Quaternion<double> qc(currentPose.block<3,3>(0,0));
	
	  Eigen::Quaternion<double> qr=q0.inverse()*qc;  // relative pose of the telescope
	
	  rotationAngle = 2*acos(qr.w())*180/3.14;
	    break;
	  }
	  else
	  {
	    std::cout<<"The Markers are not visible"<<std::endl;
	  }
	}
	
	//ndiTracker.StopTracking();
	std::cout<<"Rotaion Angle"<<rotationAngle<<std::endl;
	return rotationAngle;
}

