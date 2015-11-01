#ifndef AUX_FUNCTIONS_H
#define AUX_FUNCTIONS_H

#include <iostream>
#include "polyfit.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

using namespace cv;
using namespace std;

#define THRESHOLD 50;
const int PIXELS_TO_REMOVE = 0;

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
#endif // AUX_FUNCTIONS_H
