
#include <cstring>
#include <stdio.h>                      // for fprintf, printf, sprintf, etc
#include <string.h>                     // for strncmp, strlen, strncpy
#include <iostream>
#include <fstream>


#include <mutex>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
// 



using namespace cv;

int main ()

{
  
  String fileName = "out_camera_data.xml";
  String DirectoryName = "/home/faisal/Projects/Calibration With NDI/";
  
  
  cv::Mat camMatrix, distCofs;
  double anglel;
  
  FileStorage intrinsicsFileCV;
  std::ofstream intrinsicsFileText;
  intrinsicsFileText.open("../intrinsicsFileText.txt");
  intrinsicsFileText << "ang\tcx\tcy\tfx\tfy\td1\t\td2\t\td3\n";
  for(int i = -7;i<9;i++)
  {
    std::ostringstream stm ;
 	  stm << i ;
	  
	  
	  std::cout<<stm.str()<<std::endl;
	  
    String completeFileName = DirectoryName + stm.str()+ " Degrees/"+fileName;
    intrinsicsFileCV.open(completeFileName, FileStorage::READ);
    
    intrinsicsFileCV["camera_matrix"]>>camMatrix;
    intrinsicsFileCV["distortion_coefficients"]>>distCofs;
    
    intrinsicsFileCV["RotationAngle"]>>anglel;
    std::cout<<camMatrix<<std::endl;
//     if(i<0)
//        anglel = -anglel;
    intrinsicsFileText<<anglel<<"\t"<<	camMatrix.at<double>(0,2)<<"\t"<<	camMatrix.at<double>(1,2)<<"\t"<<	camMatrix.at<double>(0,0)<<"\t"<<	camMatrix.at<double>(1,1)<<"\t";
    intrinsicsFileText<<distCofs.at<double>(0,0)<<"\t"<<	distCofs.at<double>(0,1)<<"\t"<<	distCofs.at<double>(0,4)<<"\n";
    
    intrinsicsFileCV.release();
    
  }
  intrinsicsFileText.close();
  


  return 0;
}