#include <iostream>
#include <vector>
#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//ort::ImageTransport it(nh);
//    image_transport::Publisher pub = it.advertise("video_avi", 1);
//    sensor_msgs:
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    Mat frame;
    VideoCapture cap("pool_vid.avi"); // open the avi file
    if(!cap.isOpened())  // check if we succeeded
        return -1;
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("raw_cam_image", 1);
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(40);
    ROS_INFO("Publishing video frames");

    while(nh.ok())
    {

        cap >> frame; // get a new frame from video
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = "video_stream";
        imshow("input image", frame);
        waitKey(30);
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
