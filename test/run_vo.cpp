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
#include "vo.h"

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
    slam::Vo::Ptr vo ( new slam::Vo );
    vo->vo_state_ = vo->UNINITIALIZED;

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

    // visualization
    Mat traj;
    traj = Mat::zeros(600, 600, CV_32F);

    cv::Mat im;
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
        
        if ( vo->vo_state_ == slam::Vo::LOST )
            break;
        SE3 Tcw = pFrame->T_c_w_.inverse();
        
        // show the map and the camera pose 
        /*
        cv::Affine3d M(
            cv::Affine3d::Mat3( 
                Tcw.rotation_matrix()(0,0), Tcw.rotation_matrix()(0,1), Tcw.rotation_matrix()(0,2),
                Tcw.rotation_matrix()(1,0), Tcw.rotation_matrix()(1,1), Tcw.rotation_matrix()(1,2),
                Tcw.rotation_matrix()(2,0), Tcw.rotation_matrix()(2,1), Tcw.rotation_matrix()(2,2)
            ), 
            cv::Affine3d::Vec3(
                Tcw.translation()(0,0), Tcw.translation()(1,0), Tcw.translation()(2,0)
            )
        );
        
        cv::Point p(Tcw.translation()(0,0)+290, Tcw.translation()(1,0)+90);
        cv::circle(traj, p, 1, cv::Scalar(0, 255, 0));
        
        
        cv::imshow("traj", traj);
        */
        std::cout<<Tcw.translation()(0,0)<<" ,  "<<Tcw.translation()(1,0)<<std::endl;
        cv::imshow("image", im );
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