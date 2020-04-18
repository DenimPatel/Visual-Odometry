#include "vo_features.h"

using namespace cv;
using namespace std;

// decide how many frames you want to precess 
#define MAX_FRAME 3000

// decide how many feature you want to extract from each frame
#define MIN_NUM_FEAT 1000 //2000

// EXPERIMENTAL VARIABLES 
static int waitUserKey;

/*
* Function that provides absolute scale between frame transfer
*/
double getScaleAndIdealTranslation(int frame_id, int sequence_id, double& real_x, double& real_y, double& real_z){
  
  string line;
  int i = 0;
  ifstream myfile ("/home/denim/SELF_LEARN/visualOdom/00.txt");
  double x = 0, y = 0, z = 0;
  double x_prev, y_prev, z_prev;

  // VARIABLES TO GET ORIENTATION
  float r33,r32,r31;
  if (myfile.is_open())
  {
  	// iterate through each line and update the z untill
  	// we reach to frame_id'th line
    while (( getline (myfile,line) ) && (i<=frame_id))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;
      std::istringstream in(line);

      for (int j=0; j<12; j++)  {
        in >> z ;
        if(j==7) y=z;
        if(j==3) x=z;
        if(j==3) x=z;
        if(j==8) r31=z;
        if(j==9) r32=z;
        if(j==10) r33=z;
        
      }
      i++;
    }
    myfile.close();
  }

  else {
    cout << "Unable to open file";
    return 0;
  }
  real_x = x;
  real_y = y;
  real_z = z;
  
  // L2 norm between two consucutive frame is scale value
  return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;
}

int main() {

  // declare variables to save two images
  Mat img_1, img_2;

  // declate matrix variables to save final rotation and translation
  Mat R_f, t_f; 

  // open file to save the results
  ofstream myfile;
  myfile.open ("results1_1.txt");

  // variable to store scale beteen transformation and initializa it with 1
  double scale = 1.00;

  // two variable to store name of two working image
  char filename1[200];
  char filename2[200];
  // initialize this two as first two images of dataset
  sprintf(filename1, "/home/denim/SELF_LEARN/visualOdom/00/image_2/%06d.png", 0);
  sprintf(filename2, "/home/denim/SELF_LEARN/visualOdom/00/image_2/%06d.png", 1);

  // variable for printing things on image
  char text[100];

  // define font 
  int fontFace = FONT_HERSHEY_PLAIN;
  
  // misc variables for CV to show things on screen - not that important
  double fontScale = 1;
  int thickness = 1;  
  cv::Point textOrg(10, 525); //10,30

  //read the first two frames from the dataset
  Mat img_1_c = imread(filename1);
  Mat img_2_c = imread(filename2);

  // check whether we got images or not, or send error message and quit 
  if ( !img_1_c.data || !img_2_c.data ) {
  	std::cout<<filename1<<endl; 
    std::cout<< " [INFO] Couldn't retrieve images from the source mentioned " << std::endl;
    return -1;
  }

  // we are not usig color information here, so conver image into grayscale
  cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
  cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

  // PART: feature detection and tracking _______________________________
 
 // vector to contain feature positions - co-ordinates
  vector<Point2f> points1, points2;  
  
  // detect feature from image_1 and store it into points_1
  featureDetection(img_1, points1);        //defined in header file

  // uchar vector to store status of tracking 
  vector<uchar> status;

  // track features in image_2
  featureTracking(img_1,img_2,points1,points2, status); 


//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// CAMERA CALIBRATION PARAMS ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters

  double focal = 718.8560;
  cv::Point2d pp(607.1928, 185.2157);
  //recovering the pose and the essential matrix

  // initialize matrices
  // E- essential matrix
  // R- Rotational matrix
  // t- translation matrix
  // mask- 
  Mat E, R, t, mask;

  // generate Essential matrix from point correspondence 
  E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
  
  // by de-composing Essential matrix, we can have Rotation and Translation matrix
  recoverPose(E, points2, points1, R, t, focal, pp, mask);

// now store these values to compare with next frame
  Mat prevImage = img_2;
  Mat currImage;
  vector<Point2f> prevFeatures = points2;
  vector<Point2f> currFeatures;

  char filename[100]; // to store file name of next frames

  R_f = R.clone(); // deep copy of Rotation matix
  t_f = t.clone(); // deep copy of translation matrix

  // to calculate the total time taken for this task
  clock_t begin = clock();

 // output window that shows the frames 
  namedWindow( "Road facing camera", WINDOW_AUTOSIZE );// Create a window for display.
 
 // output window that shows the trajectory
  namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.

// we store trajectory as matrix 
  Mat traj = Mat::zeros(600, 600, CV_8UC3);

// MAIN LOOP
// iterate untill maximum frames is reached
  for(int numFrame=2; numFrame < MAX_FRAME; numFrame++)	{
  	// get next frame
  	sprintf(filename, "/home/denim/SELF_LEARN/visualOdom/00/image_2/%06d.png", numFrame);
    // cout << numFrame << endl;
  	// cout << filename << endl;
  	Mat currImage_c = imread(filename);

  	// change to grayscale
  	cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);

  	// status vector for feature tracking
  	vector<uchar> status;
  	featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

  	// using current frame and previous frame, calculate Essential matrix
  	E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
  	
  	// decompose E to get rotation and translation
  	recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

	// make matrix of 2xfeatue size
  	// first row contains X co-ordinates
  	// second row contains Y co-ordinates
    Mat prevPts(2,prevFeatures.size(), CV_64F), currPts(2,currFeatures.size(), CV_64F);

    // iterate throgh all feature points
   for(int i=0;i<prevFeatures.size();i++)	{   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
  		prevPts.at<double>(0,i) = prevFeatures.at(i).x;
  		prevPts.at<double>(1,i) = prevFeatures.at(i).y;

  		currPts.at<double>(0,i) = currFeatures.at(i).x;
  		currPts.at<double>(1,i) = currFeatures.at(i).y;
    }

    if (prevFeatures.size() < MIN_NUM_FEAT){
      //cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
      //cout << "trigerring redection" << endl;
 		  featureDetection(prevImage, prevFeatures);
      featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);
      cout<<"Looking for more features to track"<<endl;
 	  }

/////////////////////////////////////////////////////////// scale change issue
/*
    // get scale value from KITTI- GT
  	scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));

    //cout << "Scale is " << scale << endl;
  	cout<<"car height before scale:"<<t_f.at<double>(1)<<"translation@ curr frame"<< t.at<double>(1)<<endl;

  	// check conditions of scaling such as should be legitimate, ha;dnjfndanjfnaj
    if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
    	// use scale to scale roation and translation matrix
      t_f = t_f + scale*(R_f*t);
      R_f = R*R_f;
    } 	
*/
///////////////////////////////////////////////////////////////////////////////////
    double real_x, real_y, real_z; 
    scale= getScaleAndIdealTranslation(numFrame, 0,real_x, real_y, real_z ); 
	  
    if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
    	// use scale to scale roation and translation matrix
      t_f = t_f + scale*(R_f*t);
      R_f = R*R_f;
    }
    else{
    	cout<<"Very SMALL MOTION"<<endl;
    	cout<<"scale"<<scale<<endl;
    }

    // TO PRINT CURRENT YAW ANGLE OF CAR CALCULATED BY VO
	// cout<<"YAW(vo):";
	// cout<<atan2(R_f.at<double>(2,0),sqrt( R_f.at<double>(2,1)* R_f.at<double>(2,1)  + R_f.at<double>(2,2)* R_f.at<double>(2,2)  ) );
	// cout<<endl;

    prevImage = currImage.clone();
    prevFeatures = currFeatures;

    // X and Z co-ordinate is printed into the image 
    int x = int(t_f.at<double>(0)) + 300;   	//basic offset
    int y = 500 - int(t_f.at<double>(2)) ;	//basic offset

    int GT_x = int(real_x) + 300;   	//basic offset
    int GT_y = 500 - int(real_z) ;	//basic offset

    circle(traj, Point(x, y) ,1, CV_RGB(0,255,0), 2);
    circle(traj, Point(GT_x, GT_y) ,1, CV_RGB(0,0,255), 2);

    // provides blue line as a text background
    rectangle( traj, Point(0, 500), Point(600, 530), CV_RGB(0,0,255), FILLED);
    sprintf(text, "Courrent position: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
    putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

    imshow( "Front camera", currImage_c );
    imshow( "Trajectory", traj );

    waitKey(1);

  }

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Total time taken: " << elapsed_secs << "s" << endl;

  return 0;
}
