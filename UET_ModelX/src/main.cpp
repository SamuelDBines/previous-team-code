#include <ros/ros.h>
#include <string>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>

#include <std_msgs/Float32.h>
#include "lane_detect.h"
#include "ros_handle.h"

#include "car_control.h"
#include "detecttrafficsign.h"
#include <unistd.h>
#include <ctime>
#include <cstdlib>
#define SPEED_CAMERA_CHANGE  0
#define SPEED_NORMAL 13

using namespace std::chrono;

int start_clock;

int time_for_turn_camera = 1000000;
int time_in_round_first = 18000000;

bool check(int time)
{
  int stop_s=clock();
  int duration = (stop_s-start_clock);
  if (duration < time) return  true;
  else return false;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "UET_ModelX");
  ni::openni2_init();
  Mat depth, color;
  
  vector<cv::Point> leftLane, rightLane;
  int carState;
  ros_handle ros_obj;
  ros_obj.has_obj = true;
  ros::Rate rate(30.);
  bool run = false;
  while(ros::ok())
  {
    ros::spinOnce();
    if(ros_obj.button_pressed == 2){
      run = true;
    }
    if(!ros_obj.has_obj)
    {
      DetectLane dtLane;
      DetectTrafficSign TFSign;
      CarControl car;
      Type laneType  = RIGHT;
      carState = 0;
        ni::openni2_getmat(color, depth);
              Mat tfsMatDepth = depth.clone();

      srand(time(NULL));
      int a = rand();
      if (a % 2 == 0) laneType = RIGHT;
      else laneType = LEFT;
      TFSign.detectTrafficSign(color, tfsMatDepth);

      //start clock here
      start_clock = clock();

      car.setSetAngle(ros_obj.mpu_angle);
      while(true)
      { 
        ros::spinOnce();
        ni::openni2_getmat(color, depth);
        if(ros_obj.has_obj)
        {
          ros_obj.speed = 0;
          ros_obj.steer = 0;
          ros_obj.camSteer = 0;
          ros_obj.pub_to_car();
          run = false;
          //sleep(100);

          //cancel clock
          break;
        }
        imshow("color", color);
        imwrite("color.png", color);
        // imshow("depth", depth);

        car.getAngle(ros_obj.mpu_angle);
        switch (carState)
        {
          // case -1:
          // if (check(time_for_turn_camera)) 
          // {
          //   Mat tfsMatDepth = depth.clone();

          //   TFSign.detectTrafficSign(color, tfsMatDepth);
          //   ros_obj.speed = 10;
          //   ros_obj.steer = 0;
          //   TFSign.detectTrafficSign(color, tfsMatDepth);
          //   int sign_direction = TFSign.getTrafficSignInfor();
          //     if(sign_direction == -1){
          //       laneType = RIGHT; 
                
          //     } else if(sign_direction == 1){
          //       laneType = LEFT;
          //       ros_obj.camSteer = 30;
          //     }
          // } else carState = 0;
          // break;
          case 0:  // camera dang quay
              
            if(check(time_for_turn_camera / 2) )
            {
              cout << "std" << endl;
              Mat tfsMatDepth = depth.clone();
      
              if(laneType == LEFT) ros_obj.camSteer = 30;
              else ros_obj.camSteer = -30;
              ros_obj.speed = SPEED_CAMERA_CHANGE;
              ros_obj.steer = 0;
            } 
              if(laneType == LEFT)
              {
                carState = 1;
              }
              else carState = 2;
            
            break;
          case 1:  // di theo lane trai
            if (true)
            {
              dtLane.update(color, laneType);
              leftLane = dtLane.getLeftLane();
              rightLane = dtLane.getRightLane();
              car.set_Point (7);
              // //car.setPID (0.9, 0, 0);
              car.setTurn (1);
              
              ros_obj.steer = car.driverCar(leftLane, rightLane);
              
              ros_obj.speed = SPEED_NORMAL;
              ros_obj.camSteer = 30;
            } else {
              laneType = RIGHT;
              carState = 0;
              car.setTurn(0);
              //start new clock here
            }
            break;
          default:
              dtLane.update(color, laneType);
              leftLane = dtLane.getLeftLane();
              rightLane = dtLane.getRightLane();
              car.set_Point (7);
              // //car.setPID (0.9, 0, 0);
              car.setTurn (-1);
              
              ros_obj.steer = car.driverCar(leftLane, rightLane);
              
              ros_obj.speed = SPEED_NORMAL;
              ros_obj.camSteer = -30;
            break;
        }
        ros_obj.pub_to_car();
        waitKey(1);
        rate.sleep();
      }
    }
    // ni::openni2_getmat(color, depth);

    // imshow("color", color);
    // imshow("depth", depth);

    // // video writer
    // video.write(color);

    // dtLane.update(color, RIGHT);
    // leftLane = dtLane.getLeftLane();
    // rightLane = dtLane.getRightLane();

    // TFSign.detectTrafficSign(color);
    // cout << TFSign.getTrafficSignInfor() << endl;


    // car.set_Point (3);
    // //car.setPID (0.9, 0, 0);
    // car.setTurn (1);
    
    // ros_obj.steer = car.driverCar(leftLane, rightLane);
    
    // // ros_obj.speed = 0;
    // // ros_obj.camSteer = -30;
    
    // ros_obj.pub_to_car();
 
    // waitKey(1);
    // rate.sleep();
  }
  ni::openni2_destroy();
  cv::destroyAllWindows();
  return 0;
}