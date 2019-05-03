#include "detecttrafficsign.h"
#include "detecttrafficsign.h"


DetectTrafficSign::DetectTrafficSign()
{
    createTrackbars();
};

void DetectTrafficSign::setTrafficSignInfo(int trafficSignInfo)
{
    PreTrafficSignInfo = TraficSignInfo;
    TraficSignInfo = trafficSignInfo;
}

int DetectTrafficSign::getTrafficSignInfor()
{
    return TraficSignInfo;
}

void DetectTrafficSign::setPreTrafficSignInfo(int preTrafficSignInfo)
{
    PreTrafficSignInfo = preTrafficSignInfo;
}

int DetectTrafficSign::getPreTrafficSignInfor()
{
    return PreTrafficSignInfo;
}

void DetectTrafficSign::on_trackbar(int, void *)
{ 

}

string intToString(int number)
{
    std::stringstream ss;
    ss << number;
    return ss.str();
}

void DetectTrafficSign::createTrackbars()
{
    
}

void DetectTrafficSign::morphOps(Mat &thresh)
{

    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle

    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

    dilate(thresh, thresh, dilateElement);
    dilate(thresh, thresh, dilateElement);
    dilate(thresh, thresh, dilateElement);

    erode(thresh, thresh, erodeElement);
    erode(thresh, thresh, erodeElement);
}

string DetectTrafficSign::type2str(int type)
{
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth)
    {
    case CV_8U:
        r = "8U";
        break;
    case CV_8S:
        r = "8S";
        break;
    case CV_16U:
        r = "16U";
        break;
    case CV_16S:
        r = "16S";
        break;
    case CV_32S:
        r = "32S";
        break;
    case CV_32F:
        r = "32F";
        break;
    case CV_64F:
        r = "64F";
        break;
    default:
        r = "User";
        break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}

int DetectTrafficSign::RegconizeTrafficSign(Mat cropImg)
{
    int turnwhat;
    long valLeft = 0;
    long valRight = 0;

    for (int i = cropImg.rows / 2; i < cropImg.rows; i++)
    {
       
        for (int j = cropImg.cols / 4; j < cropImg.cols / 2; j++)
        {
            valLeft += (int)cropImg.at<uchar>(i, j);
            valRight += (int)cropImg.at<uchar>(i, j + cropImg.cols / 4);
        }
    }
    valLeft > valRight ? turnwhat = 1 : turnwhat = -1;
   
    return turnwhat;
}

void DetectTrafficSign::trackFilteredTrafficSign(Mat threshold, Mat blur, Mat tfsMatDepth)
{
    Mat temp;
    threshold.copyTo(temp);
    imshow("tyhr", threshold);
    vector<vector<Point>> contours;
    Mat depth;
    // inRange(tfsMatDepth , Scalar(3000), Scalar(3900), depth);
    // bitwise_and(depth, temp, temp);
    // imshow("tem", temp);
    double refArea = 0;
    Rect boundRect;
    int indexMax = -1;
    vector<Point> contours_poly;

    findContours(temp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (int index = 0; index < contours.size(); index++)
    {
        Rect boundRect_temp;       
        boundRect_temp = boundingRect(contours[index]);  
        Mat Area = tfsMatDepth(boundRect_temp);
        if (Area.at<unsigned short>(1, 1) > 4000 && Area.at<unsigned short>(1, 1) < 2000) 
            continue;  
        double area = boundRect_temp.width * boundRect_temp.height;      
        if (area > MIN_SIGN_AREA && area < MAX_SIGN_AREA && area > refArea && boundRect_temp.width / boundRect_temp.height > 0.9 && boundRect_temp.width / boundRect_temp.height < 1.05)
        {
            indexMax = index;
            refArea = area;
        }
    }
    if (indexMax == -1)
    {
        setTrafficSignInfo(0);
        return;
    }
    // imshow("threshold1", threshold);
    
    if (refArea > MIN_SIGN_AREA_FOR_REG && refArea < MAX_SIGN_AREA_FOR_REG)
    {
        approxPolyDP(contours[indexMax], contours_poly, 3, true);
        boundRect = boundingRect(contours_poly);
        Mat cropImg;
        cropImg = blur(boundRect);
        imshow("trafficsign", cropImg);

        /* Set trafficsign infor*/ 
        setTrafficSignInfo(RegconizeTrafficSign(cropImg));
        
    }
    
    
}

void DetectTrafficSign::preProcess(Mat &Img, Mat &threshold)
{
    cvtColor(Img, Img, COLOR_BGR2HSV);
    inRange(Img, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
    GaussianBlur(threshold, threshold, Size(3, 3), 0, 0);
}

void DetectTrafficSign::detectTrafficSign(const Mat &IMG, Mat &tfsMatDepth)
{
    bool trackObjects = true;
    bool useMorphOps = true;
    Mat threshold;
    Point center;
    float radius = 0;
    Mat Img = IMG.clone();
    Mat blur;
    imshow("IMG,", Img);
    preProcess(Img, threshold);
    threshold.copyTo(blur);

    if (useMorphOps)
    {
        morphOps(threshold);
    }

    //pass in thresholded frame to our object tracking function
    //this function will return the x and y coordinates of the
    //filtered object
    if (trackObjects)
    {
        trackFilteredTrafficSign(threshold, blur, tfsMatDepth);
    }

    // center of traffic sign
    circle(Img, center, radius, Scalar(0, 0, 255), 1, 8, 0);
}