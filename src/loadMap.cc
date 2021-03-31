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
#include <MapDrawer.h>
#include <System.h>
#include <sys/stat.h>

using namespace std;
using namespace ORB_SLAM2;

int main(int argc, char **argv)
{
  if(argc != 4)
  {
    cerr << endl << "Usage: ./loadMap" << endl 
                 << "         map filename" << endl
                 << "         config filename" << endl
                 << "         display[ON/OFF]" << endl;
    return 1;
  }

  try {
    // Clone parameters from command line
    string mapFName = string(argv[1]);
    string cfgFName = string(argv[2]);
    bool display = false;
    string displayS = string(argv[3]);
    if(displayS.compare("ON") == 0)
      display = true;

    // Load additional settings from config file.
    // Other settings are taken when creating MapDrawer.
    cv::FileStorage fSettings(cfgFName, cv::FileStorage::READ);
    float mViewpointX = fSettings["Viewer.ViewpointX"];
    float mViewpointY = fSettings["Viewer.ViewpointY"];
    float mViewpointZ = fSettings["Viewer.ViewpointZ"];
    float mViewpointF = fSettings["Viewer.ViewpointF"];

    ifstream mapStream(mapFName, ios_base::binary);
    if (!mapStream)
    {
        cerr << "Cannot Open Mapfile: " << mapFName << "!" << endl;
        return false;
    }
    cout << "Loading Mapfile: " << mapFName << "..." << flush << endl;
    boost::archive::binary_iarchive mapArchive(mapStream, boost::archive::no_header);

    // Allocate a Map instance
    Map *loadedMap = new(Map);
    mapArchive >> loadedMap;
    MapDrawer *localDrawer = new MapDrawer(loadedMap, cfgFName);

    auto mapPointsP = loadedMap->GetAllMapPoints();
    cout << "..." << mapFName << " has been loaded with " << mapPointsP.size() << " points." << endl;

    if (display)
    {
      pangolin::CreateWindowAndBind("ORBSLAM2 - Map Viewer",1024,768);
      // 3D Mouse handler requires depth testing to be enabled
      glEnable(GL_DEPTH_TEST);

      // Issue specific OpenGl we might need
      glEnable (GL_BLEND);
      glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      // Define Camera Render Object (for view / scene browsing)
      pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

      // Add named OpenGL viewport to window and provide 3D Handler
      pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(0), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

      // Start sending pointcloud to Pangolin frames
      while(true)
      {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        localDrawer->DrawMapPoints();
        pangolin::FinishFrame();
      }
    }
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}