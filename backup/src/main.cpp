#include <ros/ros.h>
#include <string>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Float32.h>
#include "lane_detect.h"
#include "ros_handle.h"

#include "car_control.h"
#include "detecttrafficsign.h"

cv::VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),30, cv::Size(320, 240),true);
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "UET_ModelX");
  ni::openni2_init();
  Mat depth, color;
  

  ros_handle ros_obj;

  ros::Rate rate(30.);
  while(ros::ok())
  {
    ni::openni2_getmat(color, depth);
    ros::spinOnce();

    if(!ros_obj.has_obj)
    {
      DetectLane dtLane;
      DetectTrafficSign TFSign;
      vector<cv::Point> leftLane, rightLane;
      CarControl car;
      while (true)
      {
        if(ros_obj.has_obj)
        {
          ros_obj.speed = 0;
          ros_obj.steer = 0;
          ros_obj.camSteer = -30;
          ros_obj.pub_to_car();
          sleep(100);
          //cancel clock
          break;
        }
        imshow("color", color);
        imshow("depth", depth);

        // video writer


        dtLane.update(color, RIGHT);
        leftLane = dtLane.getLeftLane();
        rightLane = dtLane.getRightLane();



        car.set_Point (7);
        //car.setPID (0.9, 0, 0);
        car.setTurn (1);
        
        ros_obj.steer = car.driverCar(leftLane, rightLane);
        
        // ros_obj.speed = 0;
        ros_obj.camSteer = -30;
        
        ros_obj.pub_to_car();
        rate.sleep();
        waitKey(1);
      }
    }
    rate.sleep();
  }
  ni::openni2_destroy();
  cv::destroyAllWindows();
  return 0;
}

