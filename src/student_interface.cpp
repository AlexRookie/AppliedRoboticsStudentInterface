#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
 
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <fstream>
 
#include <vector>
#include <atomic>
#include <unistd.h>
#include <algorithm>
#include <math.h>
 
#include <experimental/filesystem>
 
#include "dubins.h"
#include "clipper/cpp/clipper.hpp"
 
namespace student {

// - - - - - VARIABLES - - - - -

// Thresholds for resetting tree
const int max_nodes = 300;	//TODO: change
const int max_loops = 2000;
const int least_nodes = 20;

// RRT* switch
const bool RRT_STAR = true;

// Choose mission & params
const bool mission_2 = false;
const int bonus = 2;	// Bonus time for picking up victim (seconds)
const double speed = 0.2;	// Should be ~ 0.2 m/s (0.5m/2.5s)	


//TODO: Change path
//std::ofstream myfile("/home/shreyas/workspace/project_1/src/results.txt");
std::ofstream myfile("/home/robotics/workspace/group_8/src/results.txt");

// - - - - - - - - - - - - - - - 

void RRT(const float theta, Path& path, std::vector<Point>& rawPath, const Polygon& borders, int kmax, int npts, const std::vector<Polygon>& obstacle_list, std::vector<double> obs_radius, std::vector<Point> obs_center, double& length_path, const std::vector<double> gateInfo);

std::vector<int> Dijkstra(std::vector<std::vector<double>> costmap, const std::vector<std::pair<int,Polygon>>& victim_list);

bool sort_pair(const std::pair<int,Polygon>& a, const std::pair<int,Polygon>& b);

double compute_angle(Point a, Point b);


// - - - - - - - - - - - - - - - - - - - - - - - - -


void loadImage(cv::Mat& img_out, const std::string& config_folder){  
  static bool initialized = false;
  static std::vector<cv::String> img_list; // list of images to load
  static size_t idx = 0;  // idx of the current img
  static size_t function_call_counter = 0;  // idx of the current img
  const static size_t freeze_img_n_step = 30; // hold the current image for n iteration
  static cv::Mat current_img; // store the image for a period, avoid to load it from file every time
 
  if(!initialized){
    const bool recursive = false;
    // Load the list of jpg image contained in the config_folder/img_to_load/
    cv::glob(config_folder + "/img_to_load/*.jpg", img_list, recursive);
   
    if(img_list.size() > 0){
      initialized = true;
      idx = 0;
      current_img = cv::imread(img_list[idx]);
      function_call_counter = 0;
    }else{
      initialized = false;
    }
  }
 
  if(!initialized){
    throw std::logic_error( "Load Image can not find any jpg image in: " +  config_folder + "/img_to_load/");
    return;
  }
 
  img_out = current_img;
  function_call_counter++;  
 
  // If the function is called more than N times load increment image idx
  if(function_call_counter > freeze_img_n_step){
    function_call_counter = 0;
    idx = (idx + 1)%img_list.size();    
    current_img = cv::imread(img_list[idx]);
  }
 }
 
static int i;
static bool state = false;
 
 void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
   
    if (!state) {
        i = 0;
        state = true;
    }
 
    //CREATES ONE FOLDER IF DOESN'T EXISTS
    namespace fs = std::experimental::filesystem;
    std::stringstream src;
    src << config_folder << "saved_images/";
 
 
    if (!fs::is_directory(src.str()) || !fs::exists(src.str())) {
        fs::create_directory(src.str());
    }
 
    //SAVES IMAGE WHEN PRESS S ON THE KEYBOARD
 
    cv::imshow(topic, img_in);
    char k;
    k = cv::waitKey(30);
 
        std::stringstream image;
       
        switch (k) {       
        case 's':
               
            image << src.str() << std::setfill('0') << std::setw(4) << (i++) << ".jpg";
            std::cout << image.str() << std::endl;
            cv::imwrite(image.str(), img_in);
 
            std::cout << "The image" << image.str() << "was saved." << std::endl;
            break;
        default:
                break;
    }
 
  }
 
 
// Function to pick arena points - - - - - - -
 
  static cv::Mat bg_img;
  static std::vector<cv::Point2f> result;
  static std::string name;
  static std::atomic<bool> done;
  static int n;
  static double show_scale = 2.0;

/**
  *  @brief Function called after every mouse click
  *  @details Function obatined from Professor interface
  *  @param event mouse event occured 
  *  @param x x-position of mouse event
  *  @param y y-position of mouse event   
  */

  void mouseCallback(int event, int x, int y, int, void* p)
  {
    if (event != cv::EVENT_LBUTTONDOWN || done.load()) return;
   
    result.emplace_back(x*show_scale, y*show_scale);
    cv::circle(bg_img, cv::Point(x,y), 20/show_scale, cv::Scalar(0,0,255), -1);
    cv::imshow(name.c_str(), bg_img);
 
    if (result.size() >= n) {
      usleep(500*1000);
      done.store(true);
    }
  }

/**
  *  @brief Function to pick points from image
  *  @details Function obatined from Professor interface
  *  @param n0 number of points to be picked 
  *  @param img the image from which the points are to be chosen
  */
 
  std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat& img)
  {
    result.clear();
    cv::Size small_size(img.cols/show_scale, img.rows/show_scale);
    cv::resize(img, bg_img, small_size);
    //bg_img = img.clone();
    name = "Pick " + std::to_string(n0) + " points";
    cv::imshow(name.c_str(), bg_img);
    cv::namedWindow(name.c_str());
    n = n0;
 
    done.store(false);
 
    cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
    while (!done.load()) {
      cv::waitKey(500);
    }
 
    cv::destroyWindow(name.c_str());
    return result;
  }
 
// - - - - - - - - -
 
/**
  *  @brief Function for extrinsic calibration
  *  @details Extrinsic calibration to determine the Rotational and translational matrix. Four points will be chosen
  *  in the image plane and then these 4 points will be solved using the solvePnP interface from opencv to solve the 
  *  extrinsic problem.
  *  @param img_in input image 
  *  @param oject_points 4 points that are chosen in image
  *  @param camera_matrix The obtained camera matrix from intrinsic calibration  
  *  @param rvec Output rotational vector
  *  @param tvec Output translational vector
  *  @param config_folder Output folder (if file existing then function justs reads the file to get rvec and tvec)
  */
 
  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
 
    std::string extrinsic_path = config_folder + "/extrinsicCalib.csv";
    std::vector<cv::Point2f> imagePoints;
 
  if (!std::experimental::filesystem::exists(extrinsic_path)){
         
    std::experimental::filesystem::create_directories(config_folder);
    imagePoints = pickNPoints(4, img_in);
    std::ofstream output(extrinsic_path);
 
      if (!output.is_open()){
        throw std::runtime_error("Cannot write file: " + extrinsic_path);
      }
      for (const auto pt: imagePoints) {
        output << pt.x << " " << pt.y << std::endl;
      }
      output.close();
  }else{
      std::ifstream input_file(extrinsic_path);
 
      while (!input_file.eof()){
        double x, y;
        if (!(input_file >> x >> y)) {
          if (input_file.eof()) break;
          else {
            throw std::runtime_error("Malformed file: " + extrinsic_path);
          }
        }
        imagePoints.emplace_back(x, y);
      }
      input_file.close();
  }
 
    bool result = cv::solvePnP(object_points, imagePoints, camera_matrix, {}, rvec, tvec);
 
    return result;
 
  }

/**
  *  @brief Function undistort the given image
  *  @details Using the distortion coefficients obtained in the previous steps, remove the distorted 
  *  effect on the image. This is done using the opencv undistort function
  *  @param img_in input image
  *  @param img_out output image 
  *  @param camera_matrix The obtained camera matrix from intrinsic calibration  
  *  @param dist_coeffs distortion coefficients
  *  @param config_folder Output folder (if file existing then function justs reads the file to get rvec and tvec) 
  */
 
  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out,
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
 
    cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);
 
  }

/**
  *  @brief Perspective projection
  *  @details Now to have a bird’s eye view of the image, where we need to project the 3D objects in a image plane, 
  *  carry out Perspective Projection initially.  This is again carried out with the  opencv  interfaces,   
  *  projectPoints()  and getPerspectiveTransform().  
  *  @param cam_matrix camera_matrix
  *  @param rvec Output rotational vector
  *  @param tvec Output translational vector
  *  @param object_points_plane Object points   
  *  @param dest_image_points_plane destination image points plane
  *  @param config_folder Output folder (if file existing then function justs reads the file to get rvec and tvec) 
  */
 
  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, const std::vector<cv::Point2f>& dest_image_points_plane, cv::Mat& plane_transf, const std::string& config_folder){
    cv::Mat image_points;
    // projectPoint output is image_points
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);
    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
  }

/**
  *  @brief Image unwarping
  *  @details Image unwarping using opencv API warpPerspective()  
  *  @param img_in input image
  *  @param img_out output image
  *  @param transf tranformation matrix
  *  @param config_folder Output folder (if file existing then function justs reads the file to get rvec and tvec) 
  */
 
  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, const std::string& config_folder){
    cv::warpPerspective(img_in, img_out, transf, img_in.size());
  }
 

/**
    *  @brief Obstacle detection function
    *  @details Steps followed RGB→HSV→Apply red mask→Contours→approximate Polynomial→Return Obstacles 
    *  @param hsv_img HSV Image input
    *  @param scale Scaling factor
    *  @param obstacle_list Output obstacle list
    *  @param config_params configuration parameters realted to obstacle detection
    *  @return  true/false Execution successful
    */

  bool processObstacles(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& obstacle_list){
    
    // Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])
    cv::Mat red_mask_low, red_mask_high, red_mask;     
    cv::inRange(hsv_img, cv::Scalar(0, 102, 86), cv::Scalar(40, 255, 255), red_mask_low);
    cv::inRange(hsv_img, cv::Scalar(164, 102, 86), cv::Scalar(180, 255, 255), red_mask_high);
    cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); 

    // cv::Mat img_small;
    // cv::resize(red_mask, img_small, cv::Size(640, 512));

    
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    cv::Mat contours_img;

    // Process red mask
    //contours_img = img_in.clone();
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
    //std::cout << "N. contours: " << contours.size() << std::endl;
    for (int i=0; i<contours.size(); ++i)
    {
      //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
      approxPolyDP(contours[i], approx_curve, 3, true);

      Polygon scaled_contour;
      for (const auto& pt: approx_curve) {
        scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
      }
      obstacle_list.push_back(scaled_contour);
      //contours_approx = {approx_curve};
      //drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
      //std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
    }
    //std::cout << std::endl;
    // cv::imshow("Original", contours_img);
    // cv::waitKey(1);

    return true;
  }

/**
    *  @brief gate detection function
    *  @details Steps followed: RGB→HSV→Apply Green mask→Contours→approximate Polynomial→detect 4 points→Return 
    *  Gate
    *  @param hsv_img HSV Image input
    *  @param scale scaling factor
    *  @param gate Output gate polygon
    *  @param config_params configuration parameters realted to gate detection
    *  @return  true/false Execution successful
    */

  bool processGate(const cv::Mat& hsv_img, const double scale, Polygon& gate){
    
    // Find purple regions
    cv::Mat purple_mask;
    cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), purple_mask);    
    //cv::inRange(hsv_img, cv::Scalar(130, 10, 10), cv::Scalar(165, 255, 255), purple_mask);
    
    
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    //cv::Mat contours_img;

    // Process purple mask
    //contours_img = hsv_img.clone();
    cv::findContours(purple_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 4, cv::LINE_AA);
    // std::cout << "N. contours: " << contours.size() << std::endl;

    
    bool res = false;

    for( auto& contour : contours){
      const double area = cv::contourArea(contour);
      //std::cout << "AREA " << area << std::endl;
      //std::cout << "SIZE: " << contours.size() << std::endl;
      if (area > 500){
        approxPolyDP(contour, approx_curve, 30, true);

        if(approx_curve.size()!=4) continue;

        // contours_approx = {approx_curve};
        // drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);


        for (const auto& pt: approx_curve) {
          gate.emplace_back(pt.x/scale, pt.y/scale);
        }
        res = true;
        break;
      }      
    }


    // cv::imshow("Original", contours_img);
    // cv::waitKey(1);
    
    return res;
  }

 /**
    *  @brief Victim detection function along with digit recognition call
    *  @details 
    *  Steps followed: RGB→HSV→Apply Green mask→Contours→approximate Polynomial→detect more than 6 points(check 
    *  circle)→ get id of victim → return Victims  
    *  @param hsv_img HSV Image input
    *  @param scale scaling factor
    *  @param victim_list Output victim list pair
    *  @param config_params configuration parameters realted to gate detection
    *  @return  true/false Execution successful
    */

  bool processVictims(const cv::Mat& hsv_img, const double scale, std::vector<std::pair<int,Polygon>>& victim_list){
    
    // Find green regions
    cv::Mat green_mask;
     
    cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), green_mask);


    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    //cv::Mat contours_img;

    // Process red mask
    //contours_img = hsv_img.clone();
    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
    //std::cout << "N. contours: " << contours.size() << std::endl;
    int victim_id = 0;
    for (int i=0; i<contours.size(); ++i)
    {

      const double area = cv::contourArea(contours[i]);

      if(area < 500) continue;

      //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
      approxPolyDP(contours[i], approx_curve, 10, true);
      if(approx_curve.size() < 6) continue;

      Polygon scaled_contour;
      for (const auto& pt: approx_curve) {
        scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
      }
      victim_list.push_back({victim_id++, scaled_contour});
      //contours_approx = {approx_curve};
      //drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
      //std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
    }


    // cv::imshow("Original", contours_img);
    // cv::waitKey(1);
    
    return true;
  }

/**
    *  @brief Main function to call processObstacles, processGate and processVictims
    *  @param boundingRect The bounding rectangle of the victim location 
    *  @param img_in rgb image of the input
    *  @param scale scaling factor
    *  @param obstacle_list Output obstacle list 
    *  @param victim_list Output victim list pair
    *  @param gate Output gate polygon
    *  @param config_folder config folder 
    *  @return  true/false Execution successful
    */

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){

    // Convert color space from BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    
    const bool res1 = processObstacles(hsv_img, scale, obstacle_list);
    if(!res1) std::cout << "processObstacles return false" << std::endl;
    const bool res2 = processGate(hsv_img, scale, gate);
    if(!res2) std::cout << "processGate return false" << std::endl;
    const bool res3 = processVictims(hsv_img, scale, victim_list);
    if(!res3) std::cout << "processVictims return false" << std::endl;

    return res1 && res2 && res3;
  }
  bool processRobot(const cv::Mat& hsv_img, const double scale, Polygon& triangle, double& x, double& y, double& theta){

    cv::Mat blue_mask;    
     
    cv::inRange(hsv_img, cv::Scalar(90, 50, 50), cv::Scalar(140, 255, 255), blue_mask);

    // Process blue mask
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // cv::imshow("filterrrr", blue_mask);
    // cv::waitKey(1);

    // cv::Mat contours_img;
    // contours_img = hsv_img.clone();

    // drawContours(contours_img, contours, -1, cv::Scalar(0,0,0), 4, cv::LINE_AA);
    // std::cout << "N. contours: " << contours.size() << std::endl;

       
    bool found = false;
    for (int i=0; i<contours.size(); ++i)
    {
      //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
      
      cv::approxPolyDP(contours[i], approx_curve, 10, true);
      contours_approx = {approx_curve};

      // cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

      double area = cv::contourArea(approx_curve);
      
      if (approx_curve.size() != 3) continue;
      
      if (area < 300 || area>3000) continue;
      
      
      found = true;
      break;
    }

    if (found) 
    {
      for (const auto& pt: approx_curve) {
        triangle.emplace_back(pt.x/scale, pt.y/scale);
      }

      double cx = 0, cy = 0;
      for (auto item: triangle) 
      {
        cx += item.x;
        cy += item.y;
      }
      cx /= triangle.size();
      cy /= triangle.size();

      double dst = 0;
      Point vertex;
      for (auto& item: triangle)
      {
        double dx = item.x-cx;      
        double dy = item.y-cy;
        double curr_d = dx*dx + dy*dy;
        if (curr_d > dst)
        { 
          dst = curr_d;
          vertex = item;
        }
      }
      
      // cv::Moments m = cv::moments(approx_curve, false);
      // cv::Point center(m.m10/m.m00, m.m01/m.m00);
      // cv::Vec4f line;
      // cv::fitLine(approx_curve, line, cv::DIST_L2, 0, 0.01, 0.01);
      // cv::line(warpedFrame, cv::Point(line[2], line[3]), cv::Point(line[2]+line(0)*80, line(3)+line(1)*80), (0,255,0), 2);


     //cv::line(contours_img, center*scale, vertex*scale, (0,255,0), 2);
     //cv::circle(contours_img, center*scale, 20, cv::Scalar(0,0,0), -1);

      double dx = cx-vertex.x;
      double dy = cy-vertex.y;

      x = cx;
      y = cy;
      theta = std::atan2(dy, dx);


      //covariance = {};

      //std::cout << xc_m << " " << yc_m << " " << theta*180/M_PI << std::endl;
    }

    // cv::imshow("Original", contours_img);
    // cv::waitKey(1);

    return found;
  }


/**
    *  @brief find Robot function in student interface
    *  @details Directly utilized the function provided by the teaching assistant as I found that implementation was 
    *  already in the best shape.
    *  RGB→HSV→blue mask→Contours→Approximate polynomial→find 3 points of polynomial→Find center of 
    *  triangle→Find angle between top vertex and center(Orientation)→return state(x,y,ψ) 
    *  @param img_in Input Image
    *  @param scale scaling factor
    *  @param triangle The outpu triangle of robot
    *  @param x robot pose (center) x
    *  @param y robot pose (center) y
    *  @param theta robot pose (initial) theta
    *  @param config_folder config folder if any configuration params to be loaded
    *  @return  true/false robot found or not
    */
 
  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);
    // RGB to HSV to then detect blue of robot more easily
    return processRobot(hsv_img, scale, triangle, x, y, theta);    
  }

  /* Struct path_pos
  x, y, theta for POSE
  pathIndex, parentIndex to find the path/parent in the corresponding lists*/
  struct path_pos{
        double x;
        double y;
        double theta;
        int pathIndex;
        int parentIndex;
        double cost;
    };

  //Check if the point is in the polygon
  int insidePolygon(Polygon obstacle, Point pt){
    int counter = 0;
    double xinters;
    int N = obstacle.size();
    Point p1, p2;
 
    p1 = obstacle.at(0);
    // Iterate through obstacles
    for(int i = 1; i <= N; i++){
        p2 = obstacle.at(i % N);
        if(pt.y > std::min(p1.y, p2.y)){
            if(pt.y <= std::max(p1.y, p2.y)){
                if(pt.x <= std::max(p1.x, p2.x)){
                    if(p1.y != p2.y){
                        xinters = (pt.y - p1.y) *(p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                        if(p1.x == p2.x or pt.x <= xinters){
                        counter++;                     
                        }
                    }
                }          
            }
        }
        p1 = p2;
    }
 
    if(counter % 2 == 0){
        return 1;  
    }else{
        return 0;  
    }
 
}

/**
    * @brief Plan a safe and fast path in the arena
    * @param  borders        border of the arena [m]
    * @param obstacle_list  list of obstacle polygon [m]
    * @param victim_list    list of pair victim_id and polygon [m]
    * @param gate           polygon representing the gate [m]
    * @param x              x position of the robot in the arena reference system
    * @param y              y position of the robot in the arena reference system
    * @param theta          yaw of the robot in the arena reference system
    * @param config_folder  A custom string from config file.
    */
  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path, const std::string& config_folder){
   
    int kmax = 10;      // Max angle of curvature
    int npts = 50;  // Standard discretization unit of arcs
 
    // - - - GATE CENTER - - -
    double gateX = (gate[0].x + gate[1].x + gate[2].x + gate[3].x)/4;
    double gateY = (gate[0].y + gate[1].y + gate[2].y + gate[3].y)/4;
 
    // Compute gate orientation
    double gateTh;
    
    // Left
    if (fabs(gateX - borders[0].x) < fabs(gateX - borders[1].x)){
    	// Bottom-left
    	if (fabs(gateY - borders[0].y) < fabs(gateY - borders[3].y)){
    		// Horizontal
    		if (fabs(gateY - borders[0].y) < fabs(gateX - borders[0].x)){
    			gateTh = -M_PI/2;
    		// Vertical
    		} else {
    			gateTh = M_PI;
    		}
    	// Top-left
    	} else {
    		// Horizontal
    		if (fabs(gateY - borders[3].y) < fabs(gateX - borders[0].x)){
    			gateTh = M_PI/2;
    		// Vertical
    		} else {
    			gateTh = M_PI;
    		}
    	}
	// Right
    } else {
    	// Bottom-right
    	if (fabs(gateY - borders[0].y) < fabs(gateY - borders[3].y)){
    		// Horizontal
    		if (fabs(gateY - borders[0].y) < fabs(gateX - borders[1].x)){
    			gateTh = -M_PI/2;
    		// Vertical
    		} else {
    			gateTh = 0;
    		}
    	// Top-right
    	} else {
    		// Horizontal
    		if (fabs(gateY - borders[3].y) < fabs(gateX - borders[1].x)){
    			gateTh = M_PI/2;
    		// Vertical
    		} else {
    			gateTh = 0;
    		}
    	}    
    }

    std::vector<double> gateInfo = {gateX, gateY, gateTh};

    //  - - - VICTIM CENTER - - -
    std::vector<Point> victim_center;
    double victim_X;
    double victim_Y;
 
    for (int i = 0; i < victim_list.size(); i++){
        victim_X = 0;
        victim_Y = 0;
        Polygon currentPoly = std::get<1>(victim_list[i]);
        for (int pt = 0; pt < currentPoly.size(); pt++){
            victim_X += currentPoly[pt].x;
            victim_Y += currentPoly[pt].y;
        }
        victim_X /= currentPoly.size();
        victim_Y /= currentPoly.size();
        
        victim_center.emplace_back(victim_X, victim_Y);
    }
 
    //  - - - OBSTACLE WORLD - - -
    std::vector<double> obs_radius;
    std::vector<Point> obs_center;
   
    double obs_X;
    double obs_Y;
 
    //Center
    for (int i = 0; i < obstacle_list.size(); i++){
        obs_X = 0;
        obs_Y = 0;
        Polygon currentPoly = obstacle_list[i];
        for (int pt = 0; pt < currentPoly.size(); pt++){
            obs_X += currentPoly[pt].x;
            obs_Y += currentPoly[pt].y;
        }
        obs_X /= currentPoly.size();
        obs_Y /= currentPoly.size();
        obs_center.emplace_back(obs_X, obs_Y);
    }
    //Radius
    for (int i = 0; i < obstacle_list.size(); i++){
        double maxDist = 0.0;
        Polygon currentPoly = obstacle_list[i];
        for (int pt = 0; pt < currentPoly.size(); pt++){
            double dist = sqrt(pow((currentPoly[pt].x-currentPoly[(pt+1) % currentPoly.size()].x),2)+pow((currentPoly[pt].y-currentPoly[(pt+1) % currentPoly.size()].y),2));
            if(dist > maxDist){
                maxDist = dist;
            }  
        }
        obs_radius.emplace_back(maxDist / 2.0);
    }
 
    // - - - CHOOSE MISSION - - -

	std::vector<Point> rawPath; // Non-discretized path

	if (!mission_2){

		//Add victims as major points
		rawPath.push_back(Point(x,y));
		for (int i = 0; i < victim_center.size(); i++){
		    rawPath.push_back(victim_center[i]);
		}
		rawPath.push_back(Point(gateX, gateY));

		double length_path = 0;

		// Call planning algorithm
		RRT(theta, path, rawPath, borders, kmax, npts, obstacle_list, obs_radius, obs_center, length_path, gateInfo);


	} else {
	
		std::cout << "Mission 2" << std::endl;

		// Initialize costmap matrix
		std::vector<std::vector<double>> costmap(victim_center.size()+1, std::vector<double>(victim_center.size()+1));

		// Calculate connections - COSTMAP

		int cnt = 1;

		// Loop to compute costs in pair-wise combination
		for (int i = 0; i < victim_center.size()+1; i++){			
			for (int j = 0; j < victim_center.size()+1-i; j++){

				std::cout << "Combination: " << cnt << std::endl;
				cnt++;

				// Get 1st point
				if (i == 0){
					rawPath.push_back(Point(x, y));			
				} else {
					rawPath.push_back(victim_center[i-1]);
				}

				// Get 2nd point
				if (j == victim_center.size()-i){
					rawPath.push_back(Point(gateX, gateY));
				} else {
					rawPath.push_back(victim_center[i+j]);
				}

				double temp_theta = compute_angle(rawPath[0],rawPath[1]);
				double length_path = 0;			

				// Call planning algorithm
				RRT(temp_theta, path, rawPath, borders, kmax, npts, obstacle_list, obs_radius, obs_center, length_path, gateInfo);

				// Add time cost to costmap
				costmap[i][i+j] = length_path/speed;
				
				//Add time bonus, unless GOAL column
				if(i+j != costmap.size()-1){
					costmap[i][i+j] -= bonus;
				}
 
 				// Clear paths for next computation
				path = {};
				rawPath.clear();

			}	// End 2nd loop
		}	// End 1st loop

		// Print & write costmap
		myfile << "\t\t";
		
		for (int i = 0; i < costmap.size(); i++){
			if (i == costmap.size()-1){
				myfile << "Goal";
			} else {
				myfile << "\t" << std::get<0>(victim_list[i]) << "\t";
			}
		}
		myfile << std::endl;

		for (int i = 0; i < costmap.size(); i++){
			if (i == 0){
				myfile << "Start" << "\t";
			} else {
				myfile << "\t" << std::get<0>(victim_list[i-1]) << "\t";
			} 
			for (int j = 0; j < costmap.size(); j++){
				std::cout << costmap[i][j] << " ";
				if (costmap[i][j] == 0){
					myfile << "\t" << costmap[i][j] << "\t";
				} else {
					myfile << costmap[i][j] << "\t";
				}
			}
			std::cout << std::endl;
			myfile << std::endl;
		}
		
		// Find best cost path
		std::vector<int> best_conf = Dijkstra(costmap, victim_list);
		
		// Add major points to general rawPath
		rawPath.push_back(Point(x,y));
		for (int i = 0; i < best_conf.size(); i++){
		    rawPath.push_back(victim_center[best_conf[i]]);
		}
		rawPath.push_back(Point(gateX, gateY));
		
		double length_path = 0;
		
		// Compute the planning
		RRT(theta, path, rawPath, borders, kmax, npts, obstacle_list, obs_radius, obs_center, length_path, gateInfo);

  	}	// End mission 2
      
    myfile.close();
    
    std::cout << "\n\n - - - END OF PLANNING - - - \n\n\n";
       
}



// - - - - - - - - - - SECONDARY FUNCTIONS - - - - - - - - - - - -



  void RRT(const float theta, Path& path, std::vector<Point>& rawPath, const Polygon& borders, int kmax, int npts, const std::vector<Polygon>& obstacle_list, std::vector<double> obs_radius, std::vector<Point> obs_center, double& length_path, const std::vector<double> gateInfo){

	//List of all current nodes
	std::vector<path_pos> nodes_list;
	//List of all current paths
	std::vector<Path> paths_list;

	double s;

	srand(time(NULL));
	int MAX_X = (borders[1].x*100);
	int MIN_X = (borders[0].x*100);
	int MAX_Y = (borders[3].y*100);
	int MIN_Y = (borders[0].y*100);
	int samp_X = 0;
	int samp_Y = 0;
	double q_rand_x = 0;
	double q_rand_y = 0;
	int rand_count = 1;

	path_pos first_node = {}; //First node in the tree

	std::vector<Pose> temp_path; //Temporary path from point to point

	int goal = 1; //The first goal is the first number
	bool failed_to_reach_goal = false;
	bool trying_for_goal = false;
	
	int wrong_initial_angles = 0;

	while(goal < rawPath.size()){
	
		double best_goal_cost = 100000;	//TODO: remove
		path_pos best_goal_pos = {};	

		std::cout << "Current goal: " << goal << std::endl;
		nodes_list.clear();
		nodes_list.shrink_to_fit(); //For memory purposes
		paths_list.clear();
		paths_list.shrink_to_fit(); //For memory purposes

		//Initialize with robot position (x,y,theta)
		if(goal == 1){
		    first_node.x = rawPath[0].x;
		    first_node.y = rawPath[0].y;
		    first_node.theta = theta; // TODO: Change theta or 0;
		}
		//If not goal = 1, take the last position in Path
		else{
		    Pose p = path.points.back();
		    first_node.x = p.x;
		    first_node.y = p.y;
		    first_node.theta = p.theta;
		}

		//RRT Line 1 - List of nodes and paths
		Path p;
		nodes_list.push_back(first_node);
		paths_list.push_back(p); //Adding empty path for indexing purposes

		//RRT Line 2 - Reach each major point (rawPath)
		bool goalReached = false;
		int loop_cnt = 1;
		
		bool atLeastOneFound = false; //TODO: remove

		while(goalReached == false){
		   
		    //Reset if not found for too long
		    if(nodes_list.size() > max_nodes or loop_cnt == max_loops){
		    
		    	if (loop_cnt == max_loops){
		    		loop_cnt = 1;
		    		//std::cout << "Resetting: NOT CONVERGING" << std::endl;
		    		// TODO: remove to not exit code
		    		//throw std::runtime_error("Resetting: NOT CONVERGING");
		    		
		    		// TODO: remove to not look for other initial angles
		    		if (mission_2 and wrong_initial_angles < 2){
		    			if (wrong_initial_angles == 0){
		    				nodes_list.at(0).theta = theta + 0.523;
		    			} else {
		    				nodes_list.at(0).theta = theta - 0.523;
		    			}
		    			wrong_initial_angles += 1;
		    		} else {
		    			throw std::runtime_error("Resetting: NOT CONVERGING");
		    		}
		    		
		    	} else {
		    		std::cout << "Resetting: TOO MANY NODES" << std::endl;
		    	}
		    
				// Clear lists and keep only 1st element
		        path_pos l = nodes_list.at(0);
		        Path lp = paths_list.at(0);
		        nodes_list.clear();
		        nodes_list.shrink_to_fit();                                
		        paths_list.clear();
		        paths_list.shrink_to_fit();
		        nodes_list.push_back(l);
		        paths_list.push_back(lp);         
		    }
		    
		    // Count loops
		    if (loop_cnt % 100 == 0){
				std::cout << "Loop: " << loop_cnt << std::endl;
			}
			loop_cnt++;
			
			
		// - - - RRT or RRT* - - -
		
		if (!RRT_STAR){
		    
		    bool rand_clear = false;
		    int index;
		    double min_dist;

		    //RRT Line 3 & 4 - Compute next random point
		    while(!rand_clear){
		    	
		    	index = 0;
		        min_dist = 1000;

		        samp_X = rand()%(MAX_X-MIN_X+1)+MIN_X;
		        samp_Y = rand()%(MAX_Y-MIN_Y+1)+MIN_Y;

		        q_rand_x = samp_X/100.00;
		        q_rand_y = samp_Y/100.00;
		        rand_count += 1;

		        if (rand_count % 2 == 0){	// 50% of times next goal
		            q_rand_x =  rawPath[goal].x;
		            q_rand_y =  rawPath[goal].y;
		            trying_for_goal = true;
		        }

		        //RRT Line 5 - Find parent node (closest)
			    for(int i=0; i<nodes_list.size(); i++){
			        double dist_points = sqrt(pow((q_rand_x-nodes_list.at(i).x),2)+pow((q_rand_y-nodes_list.at(i).y),2));
			        if(dist_points < min_dist){
			            min_dist = dist_points;
			            index = i;
			        }
			    }

		        //RRT Line 4 - Check if small distance or reached goal
		        if((min_dist > 0.1 and min_dist < 0.4) or (rand_count % 2 == 0 and !failed_to_reach_goal) ){ //
		            rand_clear = true;
		        }
		    }
		   
		    //RRT Line 6 - Compute angle
		    double ANGLE = 0.0;
		    //ANGLE = compute_angle(Point(nodes_list.at(index).x,nodes_list.at(index).y), Point(q_rand_x,q_rand_y));
		    
		    // If going to next goal and not last goal
		    if (q_rand_x == rawPath[goal].x and q_rand_y == rawPath[goal].y and rawPath.size() != goal+1){
		    	ANGLE = compute_angle(Point(nodes_list.at(index).x,nodes_list.at(index).y), rawPath[goal+1]);
		    // If point is gate, arrive orthogonally
			} else if(q_rand_x == gateInfo[0] and q_rand_y == gateInfo[1]){
				ANGLE = gateInfo[2];
		    // If point before next goal or next goal is last
	    	} else {
		    	ANGLE = compute_angle(Point(nodes_list.at(index).x,nodes_list.at(index).y), rawPath[goal]);
		    }
		    
		    Path newPath;
		    dubinsCurve dubins = {};

			// Finding a path from one initial point to goal
		    dubins_shortest_path(dubins, nodes_list.at(index).x, nodes_list.at(index).y, nodes_list.at(index).theta, q_rand_x, q_rand_y, ANGLE, kmax); 
		   
		    // Dicretize the 3 arcs
		    discretize_arc(dubins.arc_1, s, npts, newPath); // Arc 1
		    discretize_arc(dubins.arc_2, s, npts, newPath); // Arc 2
		    discretize_arc(dubins.arc_3, s, npts, newPath); // Arc 3
		   
		    //RRT Line 7 - Collision check
		    // Find closests obstacle to point in curve
		    bool collision = false;

			// Check arena borders
		    for(int j=0; j<newPath.points.size(); j++){
		        if(newPath.points.at(j).x < (borders[0].x + 0.04) or newPath.points.at(j).x > (borders[1].x - 0.04) or newPath.points.at(j).y < (borders[0].y + 0.04)  or newPath.points.at(j).y > (borders[3].y - 0.04)){
		            collision = true;
		            if(trying_for_goal){failed_to_reach_goal = true; trying_for_goal = false;}        
		            break; 
		        }

				// Check obstacle collisions
		        for(int i=0; i<obstacle_list.size(); i++){
		            double dist_to_ob = sqrt(pow((newPath.points.at(j).x-obs_center.at(i).x),2)+pow((newPath.points.at(j).y-obs_center.at(i).y),2));
		            double result = insidePolygon(obstacle_list.at(i), Point(newPath.points.at(j).x,newPath.points.at(j).y)); // Possibly useless, since we check the radius of the obstacle anyways
		           
		            if(result != 1 or dist_to_ob < (obs_radius.at(i)+0.05)){
		                collision = true;
		                if(trying_for_goal){failed_to_reach_goal = true;trying_for_goal = false;}
		                break; 
		            }
		        }
		    }
		    
		    if(!collision){
		        
		        failed_to_reach_goal = false;
	
		        path_pos new_node = {};
		        new_node.x = newPath.points.back().x;
		        new_node.y = newPath.points.back().y;
		        
		        new_node.theta = newPath.points.back().theta;
		        new_node.pathIndex = nodes_list.size();

		        //RRT Line 8
		        new_node.parentIndex = index;
		        
		        nodes_list.push_back(new_node);
		        paths_list.push_back(newPath);

		        //RRT Line 9 - Check if goal reached
		        if(sqrt(pow((new_node.x - rawPath.at(goal).x),2)+pow((new_node.y - rawPath.at(goal).y),2)) < 0.1){
		            
		            goalReached = true;
		            std::cout << "Goal " << goal << " reached" << std::endl;
					goal += 1;
		        }
		    }

		    if(goalReached){

		        path_pos pos = nodes_list.back();
	   
		        while(pos.pathIndex != 0){
		            
		            temp_path.insert(temp_path.begin(),paths_list.at(pos.pathIndex).points.begin(),paths_list.at(pos.pathIndex).points.end());
		           
		        	// Go backwards to parent  
		        	pos = nodes_list.at(pos.parentIndex);

		        }
		    }
		    	       
		    
		    
		    
		    
		} else {	// - - - - - RRT_STAR - - - - -
		
		
		
		
		    
		    int index = 0;
		    double tmp_cost = 100000;
		    double best_x = 0;
		    double best_y = 0;
		    double best_theta = 0;
		    Path best_path = {};

		    //RRT Line 3 & 4 - Compute next random point

	        samp_X = rand()%(MAX_X-MIN_X+1)+MIN_X;
	        samp_Y = rand()%(MAX_Y-MIN_Y+1)+MIN_Y;

	        q_rand_x = samp_X/100.00;
	        q_rand_y = samp_Y/100.00;
	        rand_count += 1;

	        //if (rand_count % 2 == 0){	// 50% of times next goal
	        if (rand_count % 5 == 0){ // Pick goal every 5 iterations
	        //if (rand_count % 2 == 0 and !atLeastOneFound){ //TODO: remove
	            q_rand_x =  rawPath[goal].x;
	            q_rand_y =  rawPath[goal].y;
	            trying_for_goal = true;
	        }

	        //RRT Line 5 - Find parent node (lowest cost)
		    // Make sure able to reach parent
		    
		    for(int i=0; i<nodes_list.size(); i++){
		    
		        double dist_points = sqrt(pow((q_rand_x-nodes_list.at(i).x),2)+pow((q_rand_y-nodes_list.at(i).y),2));
		        
		       	if(dist_points > 0.1){
		       	//if(dist_points > 0.1 and dist_points < 0.4){	//TODO: remove max dist
			        
					double ANGLE = 0.0;

					// If going to next (not last) goal
					if (q_rand_x == rawPath[goal].x and q_rand_y == rawPath[goal].y and rawPath.size() != goal+1){
						ANGLE = compute_angle(Point(nodes_list.at(index).x,nodes_list.at(index).y), rawPath[goal+1]);
					// If point is gate, arrive orthogonally
					} else if(q_rand_x == gateInfo[0] and q_rand_y == gateInfo[1]){
						ANGLE = gateInfo[2];
					// If point before next goal or next goal is last
					} else {
						ANGLE = compute_angle(Point(nodes_list.at(index).x,nodes_list.at(index).y), rawPath[goal]);
					}
				
					// Loop to use different arrival angles 
					for (int ang = 0; ang < 3; ang++){
						
						ANGLE += (ang-1)*0.349; 	// +/- 20°
						
						/*if (!(q_rand_x == rawPath[goal].x and q_rand_y == rawPath[goal].y)){
							ANGLE += (ang-1)*0.349; 	// +/- 20°
						}*/	// TODO: remove
				
						Path newPath;
						dubinsCurve dubins = {};

						// Finding a path from one initial point to goal
						dubins_shortest_path(dubins, nodes_list.at(i).x, nodes_list.at(i).y, nodes_list.at(i).theta, q_rand_x, q_rand_y, ANGLE, kmax);

						// Dicretize the 3 arcs
						discretize_arc(dubins.arc_1, s, npts, newPath); // Arc 1
						discretize_arc(dubins.arc_2, s, npts, newPath); // Arc 2
						discretize_arc(dubins.arc_3, s, npts, newPath); // Arc 3

						bool collision = false;

						// Check arena borders
						for(int j=0; j<newPath.points.size(); j++){
							if(newPath.points.at(j).x < (borders[0].x + 0.02) or newPath.points.at(j).x > (borders[1].x - 0.02) or newPath.points.at(j).y < (borders[0].y + 0.02)  or newPath.points.at(j).y > (borders[3].y - 0.02)){
								collision = true;
								if(trying_for_goal){failed_to_reach_goal = true; trying_for_goal = false;}        
								break; 
							}

							// Check obstacle collisions
							for(int k=0; k<obstacle_list.size(); k++){
								double dist_to_ob = sqrt(pow((newPath.points.at(j).x-obs_center.at(k).x),2)+pow((newPath.points.at(j).y-obs_center.at(k).y),2));
								double result = insidePolygon(obstacle_list.at(k), Point(newPath.points.at(j).x,newPath.points.at(j).y)); // Possibly useless, since we check the radius of the obstacle anyways
							   
								if(result != 1 or dist_to_ob < (obs_radius.at(k)+0.05)){
									collision = true;
									if(trying_for_goal){failed_to_reach_goal = true;trying_for_goal = false;}
									break; 
								}
							}
						} // End collision check
					    
					    // Update if not collision and better cost
					    if (!collision and dubins.L + nodes_list.at(i).cost < tmp_cost){
				    	//if (!collision and dubins.L < 0.5 and dubins.L + nodes_list.at(i).cost < tmp_cost){ //TODO: remove max dist
						    index = i;        
						    tmp_cost = dubins.L + nodes_list.at(i).cost;
						    best_x = newPath.points.back().x;
						    best_y = newPath.points.back().y;
						    best_theta = newPath.points.back().theta;
						    best_path = newPath;
						}
						
					} // End angle variations
		            
		        } // End if condition (distance)
		        
			} // End loop parent search
		    
		    if (tmp_cost < 100000){	// If a parent was found basically
		    
		    	failed_to_reach_goal = false;
		    	
		    	std::cout << "New node" << std::endl;
	
				path_pos new_node = {};
				new_node.x = best_x;
				new_node.y = best_y;
			
				new_node.theta = best_theta;
				new_node.pathIndex = nodes_list.size();

				//RRT Line 8
				new_node.parentIndex = index;
				new_node.cost = tmp_cost;
			
				nodes_list.push_back(new_node);
				paths_list.push_back(best_path);

				//RRT Line 9 - Check if goal reached
			    if(sqrt(pow((new_node.x - rawPath.at(goal).x),2)+pow((new_node.y - rawPath.at(goal).y),2)) < 0.1){	//TODO: adjusted from 0.1
			        
			        //goalReached = true;
			        atLeastOneFound = true; // TODO: remove
			        std::cout << "Goal " << goal << " reached" << std::endl;
			        
			        if(tmp_cost < best_goal_cost){
			        	best_goal_cost = tmp_cost;
			        	best_goal_pos = nodes_list.back();
			        }
			        
			    }
			}

		    //if(goalReached){
		    if(atLeastOneFound and (nodes_list.size() > least_nodes or loop_cnt == 1000)){		// TODO: remove max nodes
		    //if(atLeastOneFound and (nodes_list.size() > 50 or loop_cnt == 3000)){	// Add to path if we reached the goal at least once

		        path_pos pos = best_goal_pos;
		        goalReached = true;
		        goal += 1;
	   
		        while(pos.pathIndex != 0){
		            
		            temp_path.insert(temp_path.begin(),paths_list.at(pos.pathIndex).points.begin(),paths_list.at(pos.pathIndex).points.end());
		           
		        	// Go backwards to parent  
		        	pos = nodes_list.at(pos.parentIndex);

		        }
		    }
		    
		    
   		} // END RRT / RRT_STAR  
		    
		} // Goal reached

		// Add points to final path
		path.points.insert(path.points.end(), temp_path.begin(), temp_path.end());
		temp_path.clear();
		temp_path.shrink_to_fit();

		for (int i = 0; i < path.points.size()-1; i++){
			length_path += sqrt(pow(path.points[i].x-path.points[i+1].x,2) + pow(path.points[i].y-path.points[i+1].y,2));
			
		}

	}
	
	std::cout << "\n - - RRT* OVER - - \n\n";

}

// Dijkstra's Algorithm - To solve best cost to goal
std::vector<int> Dijkstra(std::vector<std::vector<double>> costmap, const std::vector<std::pair<int,Polygon>>& victim_list){

	std::vector<double> best_cost(costmap.size());
	std::vector<std::vector<int>> combinations(costmap.size());
	bool no_changes = false;

	// Loop until it converges (no more changes)
	while (!no_changes){
		no_changes = true;
		for (int i = 0; i < costmap.size(); i++){ 			// Row
			for (int j = 0; j < costmap.size()-i; j++){		// Column
			
				// First time initialize values to first row
				if (i == 0 and best_cost[j] == 0){
					best_cost[j] = costmap[i][j];
					
				// If current node + current cost < next node
				} else if (costmap[i][j+i] + best_cost[i-1] < best_cost[j+i] and !(std::find(combinations[i-1].begin(), combinations[i-1].end(), j+i) != combinations[i-1].end())){
			
					// Change has occured
					no_changes = false;
				
					// Update best cost
					best_cost[j+i] = costmap[i][j+i] + best_cost[i-1];
		
					combinations[j+i] = {};
		
					// If START point has a combination, add it as well
					if (combinations[i-1].size() > 0){
						for (int k = 0; k < combinations[i-1].size(); k++){
							combinations[j+i].push_back(combinations[i-1][k]);
						}
					}
		
					// Then add link we just found
					combinations[j+i].push_back(i-1);
					
				// If next node + current cost < current node
				} else if (i > 0 and j+i != costmap.size()-1 and costmap[i][j+i] + best_cost[j+i] < best_cost[i-1] and !(std::find(combinations[j+i].begin(), combinations[j+i].end(), i-1) != combinations[j+i].end())){
		
					// Change has occured
					no_changes = false;
				
					// Update best cost
					best_cost[i-1] = costmap[i][j+i] + best_cost[j+i];
		
					combinations[i-1] = {};
		
					// If START point has a combination, add it as well
					if (combinations[j+i].size() > 0){
						for (int k = 0; k < combinations[j+i].size(); k++){
							combinations[i-1].push_back(combinations[j+i][k]);
						}
					}
		
					// Then add link we just found
					combinations[i-1].push_back(j+i);
			
				}	// END if/else
				
			}	// END column
		}	// END row	
	} // END while

	// Print result
	std::cout << std::endl << "Best time to goal: " << best_cost.back() << std::endl;
	myfile << std::endl << std::endl << "Best time to goal: " << best_cost.back() << std::endl;
	std::cout << "With combination:";
	myfile << "With combination:";
	for (int i = 0; i < combinations.back().size(); i++){
		if (i != 0){
			std::cout << ", " << std::get<0>(victim_list[combinations.back()[i]]);
			myfile << ", " << std::get<0>(victim_list[combinations.back()[i]]); 
		} else {
			std::cout << " " << std::get<0>(victim_list[combinations.back()[i]]);
			myfile << " " << std::get<0>(victim_list[combinations.back()[i]]);
		}
	}
	std::cout << std::endl << std::endl;
	myfile << std::endl << std::endl;

	for (int i = 0; i < combinations.back().size(); i++){
		std::cout << combinations.back()[i] << std::endl;
	}

	return combinations.back();

}

// To sort victim_list by 1st elem of pair (int)
bool sort_pair(const std::pair<int,Polygon>& a, const std::pair<int,Polygon>& b){
    return (a.first < b.first);
}

// Compute angle at next point
double compute_angle(Point a, Point b){
	
  	double angle = atan2(fabs(a.y - b.y), fabs(a.x - b.x));
  	
  	if (b.x > a.x and b.y < a.y){
  		angle = -angle;
    } else if (b.x < a.x and b.y > a.y){
    	angle = M_PI-angle;
    } else if (b.x < a.x and b.y < a.y){
    	angle = M_PI+angle;
    }
	
	return angle;
		    
}

}	// END SCRIPT









