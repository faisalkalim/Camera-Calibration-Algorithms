#include "ndicapimaxer.h"


namespace ATLASMaxerOTS
{

class NDITracker
{
public:
		NDITracker();
		~NDITracker();
		bool Init();
		bool FindDevices();
		int  GetTrackingData (int portHandle, double transform[8]);
		void StopTracking();
		void StartTracking();
		unsigned int AddTool(const char * fileName);


private:
		#if _MSC_VER >= 1700
		bool ParallelProbe(ndicapi*& outDevice, bool checkDSR);
		#endif
		int ConvertBinaryFileToAsciiEncodedHex(char *asciiEncodedHexStr, const char * fileName);
		
		



 		enum { NDI_ROMFILE_CHUNK_SIZE=64};
		enum { MAX_NDI_ROM_FILE_SIZE_IN_BYTES=1024};


		ndicapi* device;
  		const char* name;


};



}
