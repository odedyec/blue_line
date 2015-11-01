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
#include <std_msgs/Bool.h>

ros::Publisher line_pub_new;


#define THRESHOLD 50;

#define THRESHOLD_red 50;
#define THRESHOLD_green 50;
#define THRESHOLD_blue 50;

const int PIXELS_TO_REMOVE = 0;
const int alpha = 0.9;
using namespace cv;
using namespace std;


Mat lensFilter(const Mat& src)
{
    assert(src.type() == CV_8UC3);

    Mat redOnly;
    inRange(src, Scalar(0, 0, 0), Scalar(22, 30, 22), redOnly);

    return redOnly;
}

Mat lensFilter2(const Mat& src)
{
    assert(src.type() == CV_8UC3);

    Mat redOnly;
    inRange(src, Scalar(15, 25, 12), Scalar(25, 42, 20), redOnly);

    return redOnly;
}

Mat lensFilter3(const Mat& src)
{
    assert(src.type() == CV_8UC3);

    Mat redOnly;
    inRange(src, Scalar(0, 0, 0), Scalar(50, 50, 25), redOnly);

    return redOnly;
}


Mat drain_filter(const Mat& src)
{
    assert(src.type() == CV_8UC3);

    Mat redOnly;
    inRange(src, Scalar(35,50,26), Scalar(53,80,40), redOnly);

    return redOnly;
}

Mat drain_filter2(const Mat& src)
{
    assert(src.type() == CV_8UC3);

    Mat redOnly;
    inRange(src, Scalar(150,180,90), Scalar(180,255,115), redOnly);

    return redOnly;
}

Mat dark_blue(const Mat& src)
{
    assert(src.type() == CV_8UC3);

    Mat redOnly;
    inRange(src, Scalar(32, 30, 15), Scalar(52,52,26), redOnly);
//    inRange(src, Scalar(15, 30, 35), Scalar(25, 45, 50), redOnly);

    return redOnly;
}



Mat flippers(const Mat& src)
{
    assert(src.type() == CV_8UC3);

    Mat redOnly;
    inRange(src, Scalar(64, 45, 20), Scalar(70,55,26), redOnly);
//    inRange(src, Scalar(15, 30, 35), Scalar(25, 45, 50), redOnly);

    return redOnly;
}
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     Mat *img = ((Mat *)userdata);
     if  ( event == EVENT_LBUTTONDOWN )
     {
//          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
//          cout << img->at<Vec3b>(y,x) << endl;


       //   cout << "color of chosen point " << color << " " << endl;
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
//          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
//          cout << img->at<Vec3b>(x,y) << endl;
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
//          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
//          cout << img->at<Vec3b>(y,x) << endl;
     }
}


double myPolyval(vector<double> p, int x)
{
    return p.at(1) * x + p.at(0);
}

Mat image2binary(Mat img)
{
    Mat gray_img;
    cvtColor( img, gray_img, CV_BGR2GRAY );
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

int xmin_LPF = -1, ymin_LPF = -1, xmax_LPF = -1, ymax_LPF = -1, debug = 0;

void resetLine(const std_msgs::Bool& msg)
{
    xmin_LPF = -1, ymin_LPF = -1, xmax_LPF = -1, ymax_LPF = -1;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    static geometry_msgs::Vector3Stamped line_msg_new;

    try
    {
        Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

        Mat without_lens = lensFilter(img);
        Mat without_lens2 = lensFilter2(img);
        Mat without_lens3 = lensFilter3(img);
        Mat img_bin = image2binary(img);//dark_blue(img);
        Mat filtered_drain = drain_filter(img);
        Mat filtered_drain2 = drain_filter2(img);
        Mat no_flippers = flippers(img);

        circle(img_bin, Point(390,215), 392,Scalar(0,0,0),17,0);
        circle(img, Point(390,215), 392,Scalar(0,0,0),17,0);

        bitwise_and(img_bin,0,img_bin,without_lens);
        bitwise_and(img_bin,0,img_bin,without_lens2);
        bitwise_and(img_bin,0,img_bin,filtered_drain);
        bitwise_and(img_bin,0,img_bin,filtered_drain2);
        bitwise_and(img_bin,0,img_bin,no_flippers);



        static int xmin = -1, ymin = -1, xmax = -1, ymax = -1;
        vector<double> com_row = find_com(img_bin, &ymin, &ymax);
        //int xmin_LPF, xmax_LPF, ymin_LPF, ymax_LPF;
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




        int THRESH = 100;
        double al = 0.1;
      //  int flag;
        int filt;
        ros::param::param("/Line/Filter", filt, 0);
        if (((xmin_LPF == -1))||((xmax_LPF) == -1)||((ymin_LPF) == -1)||((ymax_LPF) == -1) || !(filt))
        {

//            cout << xmin << "\t" << xmax << "\t" << ymin << "\t" << ymax << "NO" << endl;
            xmin_LPF = xmin;
            xmax_LPF = xmax;
            ymin_LPF = ymin;
            ymax_LPF = ymax;

        }
        else
        {

            //cout << xmin<<"\t"<<xmax<<"\t"<< xmin<<"\t"<<xmax<<" yes2" << endl;
//            cout << "before" << xmin_LPF<<"\t"<<xmax_LPF<<"\t"<< ymin_LPF<<"\t"<<ymax_LPF<<" yes" << endl;
            if  ((xmin < (xmin_LPF+THRESH)) && (xmin > (xmin_LPF-THRESH)))
                xmin_LPF = xmin_LPF * (1 - al) + al * xmin;

            if  ((ymin < (ymin_LPF+THRESH)) && (ymin > (ymin_LPF-THRESH)))
                ymin_LPF = ymin_LPF * (1 - al) + al * ymin;

            if  ((xmax < xmax_LPF+THRESH) && (xmax > (xmax_LPF-THRESH)))
                xmax_LPF = xmax_LPF * (1 - al) + al * xmax;

            if  ((ymax < (ymax_LPF+THRESH)) && (ymax > (ymax_LPF-THRESH)))
                ymax_LPF = ymax_LPF * (1 - al) + al * ymax;


//            cout << "after" << xmin_LPF<<"\t"<<xmax_LPF<<"\t"<< ymin_LPF<<"\t"<<ymax_LPF<< "\t" << xmin << " yes2" << endl;


        }





        //imshow("filtered_drain",filtered_drain);

        ros::param::param("/Line/Debug", debug, debug);
        if (debug)
        {
            cv::namedWindow("img_bin");
            cv::namedWindow("bin_new");
            line( img_bin, Point(xmin_LPF,ymin_LPF), Point(xmax_LPF, ymax_LPF), Scalar(99,28,242), 5, 8, 0);
            line( img, Point(xmin_LPF,ymin_LPF), Point(xmax_LPF, ymax_LPF), Scalar(99,28,242), 5, 8, 0);
          //  line( img, Point(xmin_LPF,ymin_LPF), Point(xmax_LPF, ymax_LPF), Scalar(99,28,242), 5, 8, 0);

            line( img, Point(xmin + PIXELS_TO_REMOVE,ymin), Point(xmax+ PIXELS_TO_REMOVE, ymax), Scalar(255,100,100), 2, 8, 0);
            line( img_bin, Point(xmin,ymin), Point(xmax, ymax), Scalar(100,100,100), 2, 8, 0);
            imshow("img_bin",img_bin);
      //  imshow("no_flippers",without_lens2);
            imshow("bin_new", img);

            setMouseCallback("img_bin", CallBackFunc, &img);
            cv::waitKey(30);
        }
        else
        {
            cv::destroyAllWindows();
            cv::waitKey(30);
        }
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
   // cv::namedWindow("filtered_drain");
  //  cv::namedWindow("no_flippers");

    //cv::startWindowThread();
    image_transport::ImageTransport it(nh_new);
    image_transport::Subscriber sub_new = it.subscribe("raw_cam_image", 1, imageCallback);
    ros::Subscriber sub_line_reset = nh_new.subscribe("line/reset", 1, resetLine);

    line_pub_new = nh_new.advertise<geometry_msgs::Vector3Stamped>("/line", 1000);



    ros::spin();
    cv::destroyAllWindows();




}

