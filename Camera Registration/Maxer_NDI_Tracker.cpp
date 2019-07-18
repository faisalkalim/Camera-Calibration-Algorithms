#include "ndicapimaxer.h"
#include <cstring>
#include <stdio.h>                      // for fprintf, printf, sprintf, etc
#include <string.h>                     // for strncmp, strlen, strncpy
#include <iostream>
#include <sstream>
#include "Maxer_NDI_Tracker.h"
#if _MSC_VER >= 1700
  #include <vector>
  #include <algorithm>
  #include <future>
#endif


namespace ATLASMaxerOTS {

NDITracker::NDITracker()
{
  device =nullptr;
  name=nullptr;
}




bool NDITracker::FindDevices()
{
		bool checkDSR = false;
		#if _MSC_VER >= 1700
		    	ParallelProbe(device,0, checkDSR);
		#else
		    {
		      	const int MAX_SERIAL_PORTS = 20;
		      	for (int i = 0; i < MAX_SERIAL_PORTS; ++i)
		      	{
				name = ndiSerialDeviceName(i);
				int result = ndiSerialProbe(name,checkDSR);
				if (result == NDI_OKAY)
			{
			  	break;
			}
		      }
		    }
		#endif

		if (name != nullptr)
		  {
			std::cout<<name<<std::endl;
			return true;
		  }
		else
			return false;
}


#if _MSC_VER >= 1700
//----------------------------------------------------------------------------
bool NDITracker::ParallelProbe(ndicapi*& outDevice, bool checkDSR)
{
		  const int MAX_SERIAL_PORT_NUMBER = 20; // the serial port is almost surely less than this number
		  std::vector<bool> deviceExists(MAX_SERIAL_PORT_NUMBER);
		  std::fill(begin(deviceExists), end(deviceExists), false);
		  std::vector<std::future<void>> tasks;
		  std::mutex deviceNameMutex;
		  for (int i = 0; i < MAX_SERIAL_PORT_NUMBER; i++)
		  {
			    std::future<void> result = std::async([i, &deviceNameMutex, &deviceExists]()
			    {
				std::string devName;
				{
					std::lock_guard<std::mutex> guard(deviceNameMutex);
					devName = std::string(ndiSerialDeviceName(i));
				}
				int errnum = ndiSerialProbe(devName.c_str(),checkDSR);
				if (errnum == NDI_OKAY)
				{
					deviceExists[i] = true;
				}
			    });
			    tasks.push_back(std::move(result));
		  }
		  for (int i = 0; i < MAX_SERIAL_PORT_NUMBER; i++)
		  {
			tasks[i].wait();
		  }
		  for (int i = 0; i < MAX_SERIAL_PORT_NUMBER; i++)
		  {
			// use first device found
			if (deviceExists[i] == true)
			{
				//char* devicename = ndiSerialDeviceName(i);
				name = ndiSerialDeviceName(i);
				std::cout<<name<<std::endl;
		      		//outDevice = ndiOpenSerial(devicename);
		      		return true;
			}
		  }
		  std::cout<<"No NDI Device Found"<<std::endl;
		  return false;
}
#endif


bool NDITracker::Init()
{
		
		if (name != nullptr)
			{
			    device = ndiOpenSerial(name);
			}
		else
			return false; 
		if (device != nullptr)
  			{
    			    const char* reply = ndiCommand(device, "INIT:");
			    if (strncmp(reply, "ERROR", strlen(reply)) == 0 || ndiGetError(device) != NDI_OKAY)
			    {
			      	std::cerr << "Error when sending command: " << ndiErrorString(ndiGetError(device)) << std::endl;
			      	return false;
			    }

			    ndiCommand(device, "COMM:%d%03d%d", NDI_115200, NDI_8N1, NDI_NOHANDSHAKE);
			    
			    //ndiCommand(device, "BEEP:%d", 5);
			    ndiCommand(device, "APIREV:");
			    //ndiCommand(device,"TSTART:");
			    return true;
			}
		else
			return false;
}

unsigned int NDITracker::AddTool(const char* fileName)
{
  
  
		std::cout<<"Adding Tool:   "<<fileName<< "  "<< std::endl;
		char asciiEncodedHex[MAX_NDI_ROM_FILE_SIZE_IN_BYTES*2];
		int numOfFileBytes = ConvertBinaryFileToAsciiEncodedHex(asciiEncodedHex,fileName);
		int numOfChunks=numOfFileBytes/NDI_ROMFILE_CHUNK_SIZE;
		const char* reply = ndiPHRQ(device,"********","*","1","**","**");
		std::cout<<reply<<std::endl;
		
		unsigned int portNum;
		std::stringstream portNumStream(reply);
		portNumStream >> portNum;
		
		
 		for (int chunkIndex=0; chunkIndex<numOfChunks; chunkIndex++) 
 		{
 			    char chunk[129]; //64*2 +1 for the end of string
 			    strncpy(chunk,&(asciiEncodedHex[chunkIndex*NDI_ROMFILE_CHUNK_SIZE*2]),sizeof(chunk)-1);
 			    chunk[128] ='\0';
 			    int NDIAddress=chunkIndex*NDI_ROMFILE_CHUNK_SIZE; //the memory offset (in the NDI machine, not this PC)
 			    // where this chunk will start
 			    
 			    ndiPVWR(device,portNum,NDIAddress,chunk);
 			    //std::cout<<NDIAddress<<"   "<< chunk<<std::endl;
 		
 		}
 		reply=ndiPINIT(device,portNum);
		std::cout<<reply<<std::endl;
    
		reply=ndiPENA(device,portNum,'D');
		std::cout<<reply<<std::endl;
  
  return portNum;
}


int NDITracker::ConvertBinaryFileToAsciiEncodedHex(char *asciiEncodedHexStr, const char * fileName)
{
FILE* fptr=fopen(fileName,"rb");
	if (fptr==NULL) {
		std::cout<<"vrpn_Tracker_NDI_Polaris: can't open NDI .rom file %s\n"<<std::endl;
		return (-1);
	}
	
	// obtain file size:
	fseek(fptr , 0 , SEEK_END);
	long int fileSizeInBytes = ftell(fptr);
	rewind(fptr);
	
	if (fileSizeInBytes>MAX_NDI_ROM_FILE_SIZE_IN_BYTES) {
		fprintf(stderr,"vrpn_Tracker_NDI_Polaris: file is %ld bytes long - which is larger than expected NDI ROM file size of %d bytes.\n",
			fileSizeInBytes,MAX_NDI_ROM_FILE_SIZE_IN_BYTES);
		fclose(fptr);
		return (-1);
	}
	
	// allocate memory to contain the whole file:
        unsigned char* rawBytesFromRomFile;
        try { rawBytesFromRomFile = new unsigned char[fileSizeInBytes]; }
        catch (...) {
          std::cout<< "vrpn_Tracker_NDI_Polaris: Out of memory!\n"<<std::endl;
          fclose(fptr);
          return(-1);
        }
	
	// copy the file into the buffer:
	size_t result = fread (rawBytesFromRomFile,1,fileSizeInBytes,fptr);
	if (result != (unsigned int) fileSizeInBytes) {
		fprintf(stderr,"vrpn_Tracker_NDI_Polaris: error while reading .rom file!\n");
                try {
                  delete rawBytesFromRomFile;
                } catch (...) {
                  fprintf(stderr, "vrpn_Tracker_NDI_Polaris::convertBinaryFileToAsciiEncodedHex(): delete failed\n");
                  return -1;
                }
		fclose(fptr);
		return(-1);
	}
	fclose(fptr);

	// init array with "_" for debugging
	for (int i=0; i<MAX_NDI_ROM_FILE_SIZE_IN_BYTES; i++) {
		asciiEncodedHexStr[i]='_';
	}
	int byteIndex;
	for (byteIndex=0; byteIndex<fileSizeInBytes; byteIndex++) {
		sprintf(&(asciiEncodedHexStr[byteIndex*2]),"%02x ",rawBytesFromRomFile[byteIndex]);
	}
	
	// pad the length to make it a multiple of 64
	int numOfBytesToPadRemaining= NDI_ROMFILE_CHUNK_SIZE-(fileSizeInBytes% NDI_ROMFILE_CHUNK_SIZE);
	if (numOfBytesToPadRemaining== NDI_ROMFILE_CHUNK_SIZE) {numOfBytesToPadRemaining=0;}
	
	int paddedFileSizeInBytes=fileSizeInBytes+numOfBytesToPadRemaining;
	
	while (numOfBytesToPadRemaining>0) {
		asciiEncodedHexStr[byteIndex*2]='0';
		asciiEncodedHexStr[byteIndex*2+1]='0';
		byteIndex++;
		numOfBytesToPadRemaining--;
	}
	
	asciiEncodedHexStr[byteIndex*2]='\0'; //end of string marker, which is used just for debugging
	
	//std::cout<<asciiEncodedHexStr<<std::endl;
        try {
          delete rawBytesFromRomFile;
        } catch (...) {
          std::cout<<  "vrpn_Tracker_NDI_Polaris::convertBinaryFileToAsciiEncodedHex(): delete failed\n" <<std::endl;
          return -1;
        }
	
	return paddedFileSizeInBytes;

}


int NDITracker::GetTrackingData ( int portNum, double transform[8])
{
  
	      ndiTX(device,NDI_XFORMS_AND_STATUS);
	      int reply = ndiGetTXTransform(device, portNum,  transform);
	      return reply;
    	
}

void NDITracker::StartTracking()
{
		ndiTSTART(device);	
}

void NDITracker::StopTracking()
{
		ndiTSTOP(device);	
}



NDITracker::~NDITracker()
{
 ndiCloseSerial(device);
}








}
