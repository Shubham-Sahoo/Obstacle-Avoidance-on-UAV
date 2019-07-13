#include "opencv2/opencv.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv/highgui.h>


using namespace cv;


 



int main(int argc, char** argv)
{

    ros::init(argc, argv, "image_converter");
    ros::NodeHandle n;
    
    //cv::Mat img; // << image MUST be contained here
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    std_msgs::Header header; // empty header
    header.seq = rand(); // user defined counter
    header.stamp = ros::Time::now(); // time
    ros::Publisher pub_img = n.advertise<sensor_msgs::Image>("capture", 5);

    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    namedWindow("edges",1);
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        //cvtColor(frame, edges, COLOR_BGR2GRAY);
        //GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        //Canny(edges, edges, 0, 30, 3);
        imshow("video", frame);
	img_bridge = cv_bridge::CvImage(header,sensor_msgs::image_encodings::RGB8, frame);
    	img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    	pub_img.publish(img_msg);
        if(waitKey(20) >= 0) break;
    }
    
    // the camera will be deinitialized automatically in VideoCapture destructor
    
    ros::spin();
    return 0;
}


