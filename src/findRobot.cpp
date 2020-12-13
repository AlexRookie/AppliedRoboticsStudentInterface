#pragma once
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <atomic>
#include <unistd.h>

#include <sstream>
#include <experimental/filesystem>
#include "debugHeaders.hpp"

namespace student{
    bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
        cv::Mat hsv_img;
        cv::cvtColor(img_in, hsv_img,cv::COLOR_BGR2HSV); // convert the input image to hsv image

        cv::Mat blue_mask;    
        cv::inRange(hsv_img, cv::Scalar(100, 120, 150), cv::Scalar(135, 255, 255), blue_mask); //apply blue mask for the robot detection, given robot will be a blue triangle

        std::vector<std::vector<cv::Point>> contours;    
        cv::findContours(blue_mask, contours,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // find contours to detect the contours on the image

        #if FIND_ROBOT_DEBUG_PLOT 
        cv::imshow("findRobotHsv", hsv_img);
        cv::imshow("findRobotMask", blue_mask);
        cv::Mat contours_img;
        contours_img = img_in.clone();
        cv::drawContours(contours_img, contours, -1, cv::Scalar(0,0,0), 4, cv::LINE_AA); // debug plot
        std::cout << "N. contours: " << contours.size() << std::endl;
        #endif

        std::vector<cv::Point> approx_curve;
        std::vector<std::vector<cv::Point>> contours_approx;
        bool found=false;
        for(int i=0; i<contours.size(); ++i)
        { 
            cv::approxPolyDP(contours[i], approx_curve, 30, true); // approximate the contours to obtain a triangle

            if (approx_curve.size() != 3) continue; // check if the obtained curve is a triangle 

            double area = cv::contourArea(approx_curve);

            #if FIND_ROBOT_DEBUG_PLOT   
                std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
                std::cout << (i+1) << ") Aprox Contour size: " << approx_curve.size() << std::endl;
                std::cout << "Area: " << area << std::endl;
                contours_approx = {approx_curve};
                cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0,0,255), 3, cv::LINE_AA); //debug plot
            #endif
            found=true; // robot found
            break;
        }


  
        for (const auto& pt: approx_curve) {        
            triangle.emplace_back(pt.x/scale, pt.y/scale); // populate the triangle points
        }

        double cx = 0, cy = 0;

        for (auto vertex: triangle) 
        {
            cx += vertex.x; 
            cy += vertex.y;
        }
        // to get the center of triangle, this gives the position(x,y) of the robot  
        cx /= triangle.size(); 
        cy /= triangle.size();

        double dst = 0;
        Point top_vertex; 
        // to determine the third state, orientation of the robot
        for (auto& vertex: triangle)
        {
            const double dx = vertex.x-cx;      
            const double dy = vertex.y-cy;
            const double curr_d = dx*dx + dy*dy;
            if (curr_d > dst)
            { 
              dst = curr_d;
              top_vertex = vertex; // to find the top vertex of the robot
            }
        }


        x = cx;
        y = cy;

        const double dx = cx - top_vertex.x;
        const double dy = cy - top_vertex.y;
        theta = std::atan2(dy, dx); // atan2 of two points gives the orientation of the robot

    #if FIND_ROBOT_DEBUG_PLOT           
        cv::Point cv_baricenter(x*scale, y*scale); // convert back m to px
        cv::Point cv_vertex(top_vertex.x*scale, top_vertex.y*scale); // convert back m to px
        cv::line(contours_img, cv_baricenter, cv_vertex, cv::Scalar(0,255,0), 3);
        cv::circle(contours_img, cv_baricenter, 5, cv::Scalar(0,0,255), -1);
        cv::circle(contours_img, cv_vertex, 5, cv::Scalar(0,255,0), -1);      
        std::cout << "(x, y, theta) = " << x << ", " << y << ", " << theta*180/M_PI << std::endl;
    #endif  
        

      #if FIND_ROBOT_DEBUG_PLOT   // do this only if FIND_DEBUG_PLOT is defined 
        cv::imshow("findRobot", contours_img);
        cv::waitKey(1);
      #endif

        return found;
    }
}
