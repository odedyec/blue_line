#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "polyfit.h"
#include <vector>
#include "std_msgs/String.h"
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


ros::Publisher line_pub_new;

#define THRESHOLD 50;
#define THRESHOLD_red 50;
#define THRESHOLD_green 50;
#define THRESHOLD_blue 50;
const int PIXELS_TO_REMOVE = 70;
const int alpha = 0.9;
using namespace cv;
using namespace std;


Mat lensFilter(const Mat& src)
{
    assert(src.type() == CV_8UC3);

    Mat redOnly;
    inRange(src, Scalar(0, 0, 0), Scalar(22, 40, 22), redOnly);

    return redOnly;
}


Mat dark_blue(const Mat& src)
{
    assert(src.type() == CV_8UC3);

    Mat redOnly;
    inRange(src, Scalar(10, 15, 10), Scalar(60, 60, 60), redOnly);
//    inRange(src, Scalar(15, 30, 35), Scalar(25, 45, 50), redOnly);

    return redOnly;
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;


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


double myPolyval(vector<double> p, int x)
{
    return p.at(1) * x + p.at(0);
}

Mat image2binary(Mat img)
{
    Mat gray_img;cvtColor( img, gray_img, CV_BGR2GRAY );
    Rect roi(PIXELS_TO_REMOVE, 0, gray_img.cols-PIXELS_TO_REMOVE , gray_img.rows);

    Mat img_crop = gray_img(roi);
    Mat img_bin = img_crop;
    double thresh=THRESHOLD;//threshold value
    double maxval=255;
    int type=1;//type of thresholding?
    threshold(img_crop, img_bin, thresh, maxval, type);
    return img_bin;
}

vector<double> find_com(Mat img_bin, int *ymin, int *ymax)
{
    vector<double> com_row(img_bin.rows, 0.0);// = new double[img_bin.rows];

    for (int j = 0; j < img_bin.rows; j++)
    {
        com_row[j] = 0;
        double sum = 0;
        for (int i = 0; i < img_bin.cols; i++)
            if (img_bin.at<uchar>(j, i) == 255)
            {
                sum++;
                com_row[j] += i;
            }

        if (sum>0)
        {
            if (*ymin == -1)
                *ymin = j;
            else
                *ymax = j;
        }
        else
        {
            if (*ymax != -1)
                break;
        }
        if(sum!=0)
        {
            com_row[j] = (com_row[j] / (sum));
        }
        else
            com_row[j] = 0;

    }
    return com_row;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    static geometry_msgs::Vector3Stamped line_msg_new;

    try
    {
        Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        Mat img_bin = image2binary(img);
        Mat without_lens = lensFilter(img);
        Mat dark_blue_filtered = dark_blue(img);




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
        imshow("one_new",without_lens);
        imshow("two_new",dark_blue_filtered);

        imshow("bin_new", img_bin);
        setMouseCallback("bin_new", CallBackFunc, NULL);
        cv::waitKey(1);
        line_msg_new.header.stamp = ros::Time::now();
        line_msg_new.header.frame_id = "blue_line_new";
        line_msg_new.vector.x = (xmax - xmin) / ((ymax - ymin) * 1.0);
        line_msg_new.vector.y = myPolyval(p, 0) + PIXELS_TO_REMOVE;
        line_pub_new.publish(line_msg_new);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "image_listener_new");
    ros::NodeHandle nh_new;
    cv::namedWindow("one_new");
    cv::namedWindow("two_new");
    cv::namedWindow("bin_new");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh_new);
    image_transport::Subscriber sub_new = it.subscribe("raw_cam_image", 1, imageCallback);

    line_pub_new = nh_new.advertise<geometry_msgs::Vector3Stamped>("/new_color", 1000);



    ros::spin();
    cv::destroyWindow("one_new");
    cv::destroyWindow("two_new");
    cv::destroyWindow("bin_new");



}

