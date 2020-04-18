#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include <iostream>
#include <ctype.h>
#include <algorithm>
#include <iterator> 
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include<math.h>

using namespace cv;
using namespace std;

void adaptiveNonMaximalSuppresion( std::vector<cv::KeyPoint>& keypoints,
                                       const int numToKeep );

/*
* Function uses Optical flow to track the features into next frame 
*/
void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{ 

  vector<float> err;					
  Size winSize=Size(21,21);																								
  TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

  //tracks feature based on KLT algorithm
  calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

   // TO SEE THE RESULT OF FEATURE DETECTOR
        for(uint i = 0; i < points1.size(); i++)
        {
            // Select good points
            if(status[i] == 1) {
                // draw the tracks
                line(img_1,points1[i], points2[i], CV_RGB(255,0,0), 2);
                circle(img_1, points2[i], 5, CV_RGB(0,0,255), -1);
            }
        }
  imshow("Feature tracked", img_1 );
  
  // remove points whose value in status is 0 or the points lies outside of image
  int indexCorrection = 0;
  for( int i=0; i<status.size(); i++)
     {  Point2f pt = points2.at(i- indexCorrection);
     	if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0)) // if status is 0, point outside of image	
      {
     		  // if((pt.x<0)||(pt.y<0))	
         //  {
     		  // 	status.at(i) = 0;
     		  // }
          status.at(i) = 0;
     		  points1.erase (points1.begin() + (i - indexCorrection));
     		  points2.erase (points2.begin() + (i - indexCorrection));
     		  indexCorrection++;
     	}

     }
}

/*
* Function finds the feature points from the image 
*/
void featureDetection(Mat img_1, vector<Point2f>& points1)	{   //uses FAST as of now, modify parameters as necessary
  /* currently using FAST but try ORB too! to comapere the results */
  vector<KeyPoint> keypoints_1;
  int fast_threshold = 20;
  bool nonmaxSuppression = true;
  FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
  // adaptiveNonMaximalSuppresion( keypoints_1, 2000 );

   // TO SEE THE RESULT OF FEATURE DETECTOR
  Mat img_keypoints_1;
  drawKeypoints( img_1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  imshow("Feature Detection", img_keypoints_1 );

  // store key-points into vector
 KeyPoint::convert(keypoints_1, points1, vector<int>());
}

/*
* This will achieve an even distribution of those features which are locally "responsive".
* guided by the paper "Multi-Image Matching using Multi-Scale Oriented Patches" by Brown, Szeliski, and Winder.
*/
void adaptiveNonMaximalSuppresion( std::vector<cv::KeyPoint>& keypoints,
                                       const int numToKeep )
    {
      if( keypoints.size() < numToKeep ) { return; }

      //
      // Sort by response
      //
      std::sort( keypoints.begin(), keypoints.end(),
                 [&]( const cv::KeyPoint& lhs, const cv::KeyPoint& rhs )
                 {
                   return lhs.response > rhs.response;
                 } );

      std::vector<cv::KeyPoint> anmsPts;

      std::vector<double> radii;
      radii.resize( keypoints.size() );
      std::vector<double> radiiSorted;
      radiiSorted.resize( keypoints.size() );

      const float robustCoeff = 1.11; // see paper

      for( int i = 0; i < keypoints.size(); ++i )
      {
        const float response = keypoints[i].response * robustCoeff;
        double radius = std::numeric_limits<double>::max();
        for( int j = 0; j < i && keypoints[j].response > response; ++j )
        {
          radius = std::min( radius, cv::norm( keypoints[i].pt - keypoints[j].pt ) );
        }
        radii[i]       = radius;
        radiiSorted[i] = radius;
      }

      std::sort( radiiSorted.begin(), radiiSorted.end(),
                 [&]( const double& lhs, const double& rhs )
                 {
                   return lhs > rhs;
                 } );

      const double decisionRadius = radiiSorted[numToKeep];
      for( int i = 0; i < radii.size(); ++i )
      {
        if( radii[i] >= decisionRadius )
        {
          anmsPts.push_back( keypoints[i] );
        }
      }

      anmsPts.swap( keypoints );
    }
