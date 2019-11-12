#include <iostream>
#include <cstdlib>
#include <stdio.h>


#include "opencv2/opencv_modules.hpp"
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

  //-- Draw keypoints --//
  Mat img_keypoints_1; Mat img_keypoints_2;

  drawKeypoints( ims1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  drawKeypoints( ims2, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

  //-- Show detected (drawn) keypoints
  imshow("Descripteur SURF image 1", img_keypoints_1 );
  imshow("Descripteur SURF image 2", img_keypoints_2 );

  //-- Save images with detected (drawn) keypoints
  imwrite("Descripteur_SURF_image_1.png", img_keypoints_1 );
  imwrite("Descripteur_SURF_image_2.png", img_keypoints_2 );

  //-- Step 2: Calculate descriptors (feature vectors) --//
  SurfDescriptorExtractor extractor;

  Mat descriptors_1, descriptors_2;

  extractor.compute( ims1, keypoints_1, descriptors_1 );
  extractor.compute( ims2, keypoints_2, descriptors_2 );

  //--  Matching descriptor vectors with a brute force matcher
  BFMatcher matcher(NORM_L2);
  vector< DMatch > matches;
  matcher.match( descriptors_1, descriptors_2, matches );

  //-- Draw matches
  Mat img_matches;
  drawMatches( ims1, keypoints_1, ims2, keypoints_2, matches, img_matches );

  //-- Show images with detected (drawn) matches
  imshow("Appariements Brute Force", img_matches );

  //-- Save images with detected (drawn) matches
  imwrite("Appariements_Brute_Force.png", img_matches );

  // -- Step 3: Determining the good matches --//
  FlannBasedMatcher t_matcher;
  vector< DMatch > t_matches;
  t_matcher.match( descriptors_1, descriptors_2, t_matches );
  double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_1.rows; i++ ){
    double dist = t_matches[i].distance;
    if( dist < min_dist ){
      min_dist = dist;
    }
  }

  cout << "-- Min dist : " << min_dist << endl;

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_1.rows; i++ )
  { if( t_matches[i].distance <= max(3*min_dist, 0.02) )
    { good_matches.push_back( t_matches[i]); }
  }

  //-- Draw "good" matches and keypoints
  Mat img_tmatches;
  drawMatches( ims1, keypoints_1, ims2, keypoints_2,
               good_matches, img_tmatches, Scalar::all(-1), Scalar::all(-1),
               vector<char>());

  //-- Show  images with good (drawn) matches
  imshow("Appariements seuillés",img_tmatches);

  //-- Save images with good (drawn) matches
  imwrite("appariements_seuilles.png", img_tmatches );

  // -- Step 4: Homography -- //

  //-- Localize the object
  std::vector<Point2f> scene_1;
  std::vector<Point2f> scene_2;

  for( int i = 0; i < (int)good_matches.size(); i++ ){
    //-- Get the keypoints from the good matches
    scene_1.push_back( keypoints_1[ good_matches[i].queryIdx ].pt );
    scene_2.push_back( keypoints_2[ good_matches[i].trainIdx ].pt );
  }

  Mat ransac_homography = findHomography( scene_2, scene_1, CV_RANSAC );

  //-- Get the corners from the image_1 ( the scene_1 to be "detected" )
  std::vector<Point2f> scene_1_corners(4);
  std::vector<Point2f> scene_2_corners(4);
  scene_2_corners[0] = cvPoint(0,0); scene_2_corners[1] = cvPoint( ims1.cols, 0 );
  scene_2_corners[2] = cvPoint( ims2.cols, ims2.rows ); scene_2_corners[3] = cvPoint( 0, ims2.rows );

  Mat imd_homography(ims2.size(), CV_8UC1);
  perspectiveTransform(scene_2_corners, scene_1_corners, ransac_homography);
  warpPerspective(ims2, imd_homography, ransac_homography, ims1.size()+ims2.size());

  cout << "imd_homography type : " << imd_homography.type() << " | channels : " << imd_homography.channels()  << endl;

  Mat imd_homography_color(ims2.size(), CV_8UC3);
  cout << "imd_homography_color type : " << imd_homography_color.type() << " | channels : " << imd_homography_color.channels() << endl;
  cvtColor(imd_homography, imd_homography_color, CV_GRAY2BGR);
  imshow("Image 2 déformée par H", imd_homography_color);
  imwrite("Homography.png", imd_homography_color);

  // Mat imd_panorama(ims2.size(), CV_8UC3);
  // hcontcat(ims1,imd_homography,imd_panorama);
  // imshow("panorama",imd_panorama);
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
