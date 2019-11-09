#include <iostream>
#include <cstdlib>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>

using namespace cv;
using namespace std;

void
process(const char* imsname1, const char* imsname2)
{
  cout<<"\n---------------------------------------------------\n"<<endl;
  cout<<"\n--------------- EXERCICE : PANORAMA  ---------------\n"<<endl;
  cout<<"\n---------------------------------------------------\n"<<endl;

  //Read imsname1 and imsname2 if they exist
  Mat ims1 = imread(imsname1,0);
  Mat ims2 = imread(imsname2,0);

  if(!ims1.data || !ims2.data){
    cerr<<"Image not found, exit"<<endl;
    exit(EXIT_FAILURE);
  }

 //-- Step 1: Detect the keypoints using SURF Detector
 int minHessian = 400;

 SurfFeatureDetector detector( minHessian );

 std::vector<KeyPoint> keypoints_1, keypoints_2;

 detector.detect( ims1, keypoints_1 );
 detector.detect( ims2, keypoints_2 );

 //-- Draw keypoints
 Mat img_keypoints_1; Mat img_keypoints_2;

 drawKeypoints( ims1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
 drawKeypoints( ims2, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

 //-- Show detected (drawn) keypoints
 imshow("Keypoints 1", img_keypoints_1 );
 imshow("Keypoints 2", img_keypoints_2 );

 waitKey(0);
}

void
usage (const char *s)
{
  std::cerr<<"Usage: "<<s<<" ims-name-1 ims-name-2"<<endl;
  exit(EXIT_FAILURE);
}

#define param 2
int
main( int argc, char* argv[] )
{
  if(argc != (param+1))
    usage(argv[0]);
  process(argv[1], argv[2]);
  return EXIT_SUCCESS;
}
