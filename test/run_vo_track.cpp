// -------------- test the visual odometry -------------
#include<iostream>
#include<algorithm>
#include<chrono>
#include<iomanip>

#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp> 
#include "config.h"
#include "vo_track.h"

#include<opencv2/core/core.hpp>

using namespace std;


void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_settings" << endl;
        return 1;
    }

    slam::Config::setParameterFile ( argv[1] );
    slam::Trackvo::Ptr vo ( new slam::Trackvo );
    vo->vo_state_ = vo->INITIALIZING;

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string dataset_dir = slam::Config::get<string> ( "dataset_dir" );
    LoadImages(dataset_dir, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    std::cout << endl << "-------" << endl;
    std::cout << "Start processing sequence ..." << endl;
    std::cout << "Images in the sequence: " << nImages << endl << endl;

    slam::Camera::Ptr camera ( new slam::Camera );

    cv::Mat im;
    Mat traj = Mat(600, 600, CV_8UC3, cv::Scalar(255, 255, 255));

    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

        if ( im.data==nullptr)
            break;
        slam::Frame::Ptr pFrame = slam::Frame::createFrame();
        pFrame->cam_ = camera;
        pFrame->rgb_ = im;
        pFrame->time_stamp_ = tframe;

        std::cout<<"frame: "<<tframe<<std::endl;
        boost::timer timer;
        vo->addFrame( pFrame );
        std::cout<<"VO costs time: "<<timer.elapsed()<<std::endl<<std::endl;
        
        if ( vo->vo_state_ == slam::Trackvo::LOST )
            break;


        Mat t = vo->ref_frame_->t_.clone();

        char text[100];
        int fontFace = cv::FONT_HERSHEY_PLAIN;
        double fontScale = 1;
        int thickness = 1;  
        cv::Point textOrg(10, 50);

        cv::namedWindow( "Road facing camera", cv::WINDOW_AUTOSIZE );// Create a window for display.
        cv::namedWindow( "Trajectory", cv::WINDOW_AUTOSIZE );// Create a window for display.
        
        

        int x = int(t.at<double>(0)) + 300;
        int y = int(t.at<double>(2)) + 100;
        circle(traj, cv::Point(x, y) ,1, CV_RGB(255,0,0), 2);

        rectangle( traj, cv::Point(10, 30), cv::Point(550, 50), CV_RGB(255,255,255), CV_FILLED);
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t.at<double>(0), t.at<double>(1), t.at<double>(2));
        putText(traj, text, textOrg, fontFace, fontScale, cv::Scalar::all(0), thickness, 8);

        imshow( "Road facing camera", im );
        imshow( "Trajectory", traj );

        cv::waitKey(1);
        
    }
    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}