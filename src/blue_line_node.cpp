#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>

#include <vector>
//#include "precomp.hpp"

#include "std_msgs/String.h"
#include <sstream>
#include "aux_functions.h"
ros::Publisher line_pub;


const int alpha = 0.9;
using namespace cv;
using namespace std;


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     Mat *img = ((Mat *)userdata);
     if  ( event == EVENT_LBUTTONDOWN )
     {
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
          cout << img->at<Vec3b>(y,x) << endl;


       //   cout << "color of chosen point " << color << " " << endl;
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

     }
}




void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    static geometry_msgs::Vector3Stamped line_msg;

    try
    {
        Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        Mat img_bin = image2binary(img);



        static int xmin = -1, ymin = -1, xmax = -1, ymax = -1;
        vector<double> com_row = find_com(img_bin, &ymin, &ymax);

        //cout << ymin << ", " << ymax << "\t" << com_row[ymin] << ", " << com_row[ymax] << endl;
        //------------------------

        int R = 500;
        int start = ymin;
        int N = 30;
        vector<double> l;
        vector<double> points;
        if (ymax - ymin < N)
        {
            cout << "image crop cols is smaller than 30" << endl;
            return;
        }
        while (R > 400)
        {
            l.clear();
            points.clear();
            if (start+N > ymax)
            {
                cout << "didn't find 30 points to build line" << endl;
                return;
            }
            vector<double> ys;
            for(int i=start;i<start+N-1;i++)
            {
                points.push_back(com_row[i]);
                ys.push_back(i);
            }

            l = polyfit(ys, points,1);
            int sum = 0;
            vector<double> ans = polyval(l, ys);
            int i = 0;
            for (vector<double>::iterator it = ans.begin(); it != ans.end(); it++, i++)
                sum += (*it - points[i]) * (*it - points[i]);
            R = sum;
            start = start + 5;
        }
        if (l.empty())
        {
            cout <<"didn't find 30 points to build line" << endl;
            return;
        }


        vector<double> com2x, com2y, com;


        for (int i = 0; i<= img_bin.rows;i++)
        {
            double val = (myPolyval(l,i) - com_row[i]);
            if (val * val < 1000)
            {
                com2x.push_back(com_row[i]);
                com2y.push_back(i);
            }
        }
        if (com2x.empty())
        {
            cout << "no points on line l that were found" << endl;
            return;
        }
        //for (int i = 0; i < com2x.size(); i++)
          //  cout << com2x.at(i) << ", " << com2y.at(i) << endl;
        vector<double> p = polyfit(com2y,com2x, 1);
        //cout << p.at(1) << " * x + " << p.at(0) << endl;
        for (int i = ymin; i < ymax; i++)
        {
            int j = myPolyval(p,i);
            img_bin.at<uchar>(i,j) = 100;
        }
        xmin = myPolyval(p,ymin);
        xmax = myPolyval(p,ymax);



        line( img, Point(xmin + PIXELS_TO_REMOVE,ymin), Point(xmax+ PIXELS_TO_REMOVE, ymax), Scalar(255,100,100), 2, 8, 0);
        line( img_bin, Point(xmin,ymin), Point(xmax, ymax), Scalar(100,100,100), 2, 8, 0);
        imshow("view",img);
        imshow("bin", img_bin);
        setMouseCallback("bin", CallBackFunc, &img);
        cv::waitKey(30);
        line_msg.header.stamp = ros::Time::now();
        line_msg.header.frame_id = "blue_line";
        line_msg.vector.x = (xmax - xmin) / ((ymax - ymin) * 1.0);
        line_msg.vector.y = myPolyval(p, 0) + PIXELS_TO_REMOVE;
        line_pub.publish(line_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    int *m, *n;
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::namedWindow("bin");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("raw_cam_image", 1, imageCallback);

    line_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/line", 1000);



    ros::spin();
    cv::destroyWindow("view");
    cv::destroyWindow("bin");
}

