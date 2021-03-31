/**
* Load and display ORB-SLAM2 map in binary format.
*
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <Map.h>
#include <System.h>
#include <sys/stat.h>

using namespace std;
using namespace ORB_SLAM2;

int main(int argc, char **argv)
{
  if(argc != 3)
  {
    cerr << endl << "Usage: ./loadMap" << endl 
                 << "         filename" << endl
                 << "         display[ON/OFF]" << endl;
    return 1;
  }

  try {
    // Clone parameters from command line
    string mapFName = string(argv[1]);
    bool display = false;
    string displayS = string(argv[2]);
    if(displayS.compare("ON") == 0)
      display = true;

    ifstream mapStream(mapFName, ios_base::binary);
    if (!mapStream)
    {
        cerr << "Cannot Open Mapfile: " << mapFName << "!" << endl;
        return false;
    }
    cout << "Loading Mapfile: " << mapFName << flush << endl;
    boost::archive::binary_iarchive mapArchive(mapStream, boost::archive::no_header);

    // Instance of Map
    Map * loadedMap = new(Map);
    mapArchive >> loadedMap;

    auto mapPointsP = loadedMap->GetAllMapPoints();
    cv::Mat test = mapPointsP[0]->GetWorldPos();
    cout << test << endl;
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}