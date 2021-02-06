#pragma once
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <atomic>
#include <unistd.h>

#include <sstream>
#include <experimental/filesystem>

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <config4cpp/Configuration.h>
using namespace config4cpp;
using namespace std;

/** @brief Class which reads and stores the configuration params related to processMap functions  */
class config_Params_ProcessMap{
    public:
        bool find_victims_debug_plot; /** variable for victim debug */
        bool find_gate_debug_plot;  /**variable for gate debug*/
        bool find_obstacles_debug_plot; /**variable for obstacle debug*/
        bool debug_victim_id; /**variable for victim id recognition debug*/
        bool use_ocr; /**OCR method of detection */
        bool use_flip; /**Flip the image input image*/
        string config_folder;
        bool light_green_gate;
        bool arena; /** Flag to check arena or simulator*/
        /**
        *  @brief read all params from config file and assign it to class variables
        *  @param config_folder config folder where the params are available
        */
        void read(const std::string& config_folder){
            std::string file_path = config_folder + "/config_params.cfg";
            const char *  configFile =file_path.c_str() ;
            Configuration *  cfg = Configuration::create();
            const char *     scope = "";
            try{
                cfg->parse(configFile);
                find_victims_debug_plot = cfg->lookupBoolean(scope, "find_victims_debug_plot");              
                find_gate_debug_plot = cfg->lookupBoolean(scope, "find_gate_debug_plot"); 
                find_obstacles_debug_plot = cfg->lookupBoolean(scope, "find_obstacles_debug_plot");
                debug_victim_id = cfg->lookupBoolean(scope, "debug_victim_id");
                use_ocr = cfg->lookupBoolean(scope, "use_ocr");
                use_flip = cfg->lookupBoolean(scope, "use_flip");

                light_green_gate = cfg->lookupBoolean(scope, "light_green_gate");
                arena = cfg->lookupBoolean(scope, "arena");
                this->config_folder = config_folder; 
            } catch(const ConfigurationException & ex) {
                cerr << ex.c_str() << endl;
                cfg->destroy();
                
            }
        }
        /**
        *  @brief print all params for debug
        */
        void print(){
            cout<<"Process Map Config and Debug Information"<<endl;
            cout<<"find_gate_debug_plot "<<find_gate_debug_plot<<endl;
            cout<<"find_obstacles_debug_plot "<<find_obstacles_debug_plot<<endl;
            cout<<"debug_victim_id "<<debug_victim_id<<endl;
            cout<<"find_victims_debug_plot "<<find_victims_debug_plot<<endl;
            cout<<"use_ocr "<<use_ocr<<endl;
            cout<<"use_flip "<<use_flip<<endl;
        }
};



namespace student{



    cv::Mat debug_image;
    bool sort_pair(const std::pair<int,Polygon>& a, const std::pair<int,Polygon>& b){
        return (a.first < b.first);
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

    bool processObstacles(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& obstacle_list,config_Params_ProcessMap config_params, bool arena){

        /**Steps involved in process Obstacles one by one*/
    
        cv::Mat red_mask_low, red_mask_high, red_mask; 

        if(arena){    
            cv::inRange(hsv_img, cv::Scalar(0, 50, 40), cv::Scalar(40, 255, 255), red_mask_low); /** First range of red region in HSV */
            cv::inRange(hsv_img, cv::Scalar(160, 50, 40), cv::Scalar(180, 255, 255), red_mask_high); /** Second range of red region in HSV */
        }
        else{
            cv::inRange(hsv_img, cv::Scalar(0, 50, 40), cv::Scalar(40, 255, 255), red_mask_low); /** First range of red region in HSV */
            cv::inRange(hsv_img, cv::Scalar(160, 50, 40), cv::Scalar(180, 255, 255), red_mask_high); /** Second range of red region in HSV */
        }
        cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); /** add the regions of red space in HSV format*/
        
        std::vector<std::vector<cv::Point>> contours, contours_approx;
        std::vector<cv::Point> approx_curve;

        cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); /** find all contours in the image */

        for (int i=0; i<contours.size(); ++i)
        {
            approxPolyDP(contours[i], approx_curve, 3, true); /** approximate the contours */

            Polygon scaled_contour;
            for (const auto& pt: approx_curve) {
                scaled_contour.emplace_back(pt.x/scale, pt.y/scale); 
            }
            obstacle_list.push_back(scaled_contour); /** create a obstacle list */
            if(config_params.find_obstacles_debug_plot){
                contours_approx .push_back(approx_curve);
                cv::drawContours(debug_image, contours_approx, -1, cv::Scalar(255,0,0), 3, cv::LINE_AA); /** debug plot in seperate debug image*/
            }
        }

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

    bool processGate(const cv::Mat& hsv_img, const double scale, Polygon& gate,config_Params_ProcessMap config_params, bool arena){
      
        cv::Mat green_mask;
        if(arena){
            if(config_params.light_green_gate)
                cv::inRange(hsv_img, cv::Scalar(20, 50, 62), cv::Scalar(51, 255, 162), green_mask);// get green mask to detect victim circles
            else
                cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), green_mask);// get green mask to detect victim circles
      
        }
        else
            cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), green_mask); // get green mask to detect victim circles
        
        std::vector<std::vector<cv::Point>> contours, contours_approx;
        std::vector<cv::Point> approx_curve;

        cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // find contours of green elements, but it will give victims as well
        bool res = false;

        for( auto& contour : contours){
            const double area = cv::contourArea(contour);
            if (area > 500){ // What is the reason of this?
            approxPolyDP(contour, approx_curve, 30, true); // approximate the contour 

            if(approx_curve.size()!=4) continue; // check if it is a rectangle with 4 points

            for (const auto& pt: approx_curve) {
                gate.emplace_back(pt.x/scale, pt.y/scale);
            }

            if(config_params.find_gate_debug_plot){
                contours_approx .push_back(approx_curve);
                cv::drawContours(debug_image, contours_approx, -1, cv::Scalar(255,255,255), 3, cv::LINE_AA);
            }

            res = true; // gate found
            break;
            }      
        }

        return res;
    }

    /**
    *  @brief Function to get victim ID
    *  @details This is mainly done to detect the number of Victims, so that the priority to save the victims can be 
    *  known to the planning algorithm. This involves template matching majorly and the templates were provided in 
    *  lecture. I have used majorly the opencv template matching methods. But I also implemented the tesseract-ocr 
    *  method,although the results were not satisfying. Template rotation was performed ,Each template will be 
    *  rotated by 
    *  5 degrees and then they are set to calculate the score and the maximum of all of that will be be finalized 
    *  as digits.
    *  Steps followed: Detect Victims→get bounding rect→mask and filter image→change background→read 
    *  templates→Resize and filter the ROI→change different orientation of ROI→match template→return Digit 
    *  @param boundingRect The bounding rectangle of the victim location 
    *  @param img hsv image of the input
    *  @param config_params configuration parameters realted to obstacle detection, includes template location
    *  @return  vicitm id
    */
    int get_victim_id(cv::Rect boundingRect, cv::Mat img,config_Params_ProcessMap config_params){


        if(config_params.use_ocr){// detection using tesseract ocr{
            cv::Mat debug_victim_img = img.clone();
            if(config_params.debug_victim_id){
                cv::cvtColor(img, debug_victim_img, cv::COLOR_HSV2BGR);
            }
            // Find green regions
            cv::Mat green_mask; 
            cv::inRange(img, cv::Scalar(40, 30, 40), cv::Scalar(85, 255, 180), green_mask); // green mask for victims


            // Apply some filtering
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1)); // Filter the green areas
            cv::dilate(green_mask, green_mask, kernel); // dialtion performed to do filtering
            cv::erode(green_mask, green_mask, kernel);// erosion performed to do filtering

            cv::Mat green_mask_inv, filtered(img.rows, img.cols, CV_8UC3, cv::Scalar(255,255,255)); 
            cv::bitwise_not(green_mask, green_mask_inv); // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask

            // Create Tesseract object
            tesseract::TessBaseAPI *ocr = new tesseract::TessBaseAPI();
            // Initialize tesseract to use English (eng) 
            ocr->Init(NULL, "eng");
            // Set Page segmentation mode to PSM_SINGLE_CHAR (10)
            ocr->SetPageSegMode(tesseract::PSM_SINGLE_CHAR);
            // Only digits are valid output characters
            ocr->SetVariable("tessedit_char_whitelist", "12345");
            
            img.copyTo(filtered, green_mask_inv);   // create copy of image without green shapes
           
            kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));

            cv::Mat processROI(filtered, boundingRect); // extract the ROI containing the digit
            
            if (processROI.empty()) return -1;

            cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI
            cv::threshold(processROI, processROI, 100, 255, 0);   // threshold and binarize the image, to suppress some noise

            // Apply some additional smoothing and filtering
            cv::erode(processROI, processROI, kernel);
            cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
            cv::erode(processROI, processROI, kernel);

            ocr->SetImage(processROI.data, processROI.cols, processROI.rows, 3, processROI.step);
	
            int id = std::stoi(ocr->GetUTF8Text());

            if(config_params.debug_victim_id){
                cv::Point point0 = cv::Point(boundingRect.x, boundingRect.y);
                cv::putText(debug_victim_img, ocr->GetUTF8Text(), point0, cv::FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv::LINE_AA);
                cv::imshow("debug_victim", debug_victim_img);
                cv::waitKey(30);
            }


            return id;
        }else{
            cv::Mat debug_victim_img = img.clone();
            if(config_params.debug_victim_id){
                
                cv::cvtColor(img, debug_victim_img, cv::COLOR_HSV2BGR);
            }
            
            // Find green regions
            cv::Mat green_mask;
            cv::inRange(img, cv::Scalar(40, 30, 40), cv::Scalar(85, 255, 180), green_mask); // green mask for victims

            // Apply some filtering
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
            cv::dilate(green_mask, green_mask, kernel);
            cv::erode(green_mask, green_mask, kernel);

            cv::Mat green_mask_inv, filtered(img.rows, img.cols, CV_8UC3, cv::Scalar(255,255,255));  // New matrix

            cv::bitwise_not(green_mask, green_mask_inv); // invert the mask background

            std::vector<cv::Mat> templROIs;

            // read all the templates from the template folder(generally config folder is /tmp) 
            for (int i = 1; i <= 5; ++i) {
                auto num_template = cv::imread(config_params.config_folder + "/template/" + std::to_string(i) + ".png"); 
                if(config_params.use_flip)// declared in config file
                    cv::flip(num_template, num_template, 1); // flip the template in case the actual image is flipped
                templROIs.emplace_back(num_template); // get all the template numbers
            }

            img.copyTo(filtered, green_mask_inv);   // create copy of image without green shapes

            kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));
            cv::Mat processROI(filtered, boundingRect); // extract the ROI containing the digit
            
            if (processROI.empty()) return -1;
            cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI
            cv::threshold(processROI, processROI, 100, 255, 0);   // threshold and binarize the image, to suppress some noise
            cv::erode(processROI, processROI, kernel); // apply some filtering
            cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
            cv::erode(processROI, processROI, kernel); 
	
            double maxScore = 0;
            int maxIdx = -1;
            int rotate_degrees =10;
            for (int j=0; j<templROIs.size(); ++j) {

              double score;
              // Reason for this loop is to check if the digit is rotated or not, so checking the digit by rotating 5 degrees in each iteration and finding the best possible match out off all the rotations
              for (int iter_rotate = 0; iter_rotate < 360/rotate_degrees; iter_rotate++) {
                  cv::Mat result;
                  cv::Point2f src_center(templROIs[j].cols / 2.0F, templROIs[j].rows / 2.0F);// rotating the template for every 5 degree and checking it is obtaining maximum score 
                  cv::Mat rot_mat = getRotationMatrix2D(src_center, iter_rotate * rotate_degrees, 1.0); // get the roation matrix of the rotated elements
                  cv::Mat dst;
                  cv::warpAffine(templROIs[j], dst, rot_mat, templROIs[j].size()); // Transorm the matrix with the rotation matrix and pass this to the template matching

                  // Match the ROI with the obtained rotated matrix element
                  cv::matchTemplate(processROI, dst, result, cv::TM_CCOEFF);
                  
                  cv::minMaxLoc(result, nullptr, &score);

                  // Compare the score with the others, if it is higher save this as the best match!
                  if (score > maxScore) {
                      maxScore = score;
                      maxIdx = j;
                  }
              }

            }

            if(config_params.debug_victim_id){
                cv::Point point0 = cv::Point(boundingRect.x, boundingRect.y);
                cv::putText(debug_victim_img, std::to_string(maxIdx + 1), point0, cv::FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv::LINE_AA);
                cv::imshow("debug_victim", debug_victim_img); // Debug victim if image
                cv::waitKey(1);
            }
            return maxIdx;
        }
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

    bool processVictims(const cv::Mat& hsv_img, const double scale, std::vector<std::pair<int,Polygon>>& victim_list,config_Params_ProcessMap config_params,bool arena){

        cv::Mat green_mask;
        if(arena)
            cv::inRange(hsv_img, cv::Scalar(40, 40, 50), cv::Scalar(75, 230, 160), green_mask);// get green mask to detect victim circles
        else
            cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), green_mask); // get green mask to detect victim circles

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
        // Dilate using the generated kernel
        cv::dilate(green_mask, green_mask, kernel);
        // Erode using the generated kernel
        cv::erode(green_mask,  green_mask, kernel);
        std::vector<std::vector<cv::Point>> contours, contours_approx;
        std::vector<cv::Point> approx_curve;

        cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // find contours in the imagge

        int victim_id = 0;

        for (int i=0; i<contours.size(); ++i)
        {

            const double area = cv::contourArea(contours[i]); 

            //if(area < 500) continue;

            approxPolyDP(contours[i], approx_curve, 10, true);// approximate the contours 

            if(approx_curve.size() < 6) continue; //if the approximated number of points are less than six,i.e. if it not a line,triangle,rectangle,pentagon ..then it should be a circle which is a victim

            Polygon scaled_contour;
            for (const auto& pt: approx_curve) {
                scaled_contour.emplace_back(pt.x/scale, pt.y/scale); 
            }
            victim_list.push_back({get_victim_id(cv::boundingRect(cv::Mat(approx_curve)), hsv_img,config_params), scaled_contour}); // call the get_victim_id function to get id and the victim pair 
            if(config_params.find_victims_debug_plot){
                contours_approx .push_back(approx_curve);
                cv::drawContours(debug_image, contours_approx, -1, cv::Scalar(0,0,0), 3, cv::LINE_AA);
            }
        }
	    sort(victim_list.begin(), victim_list.end(), sort_pair);
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

        config_Params_ProcessMap config_params;
        config_params.read(config_folder);

        // Convert color space from BGR to HSV
        cv::Mat hsv_img;
        cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);
        bool arena = config_params.arena;
        
        if(config_params.find_victims_debug_plot or config_params.find_gate_debug_plot or config_params.find_obstacles_debug_plot)
            debug_image = hsv_img.clone();

        const bool res1 = processObstacles(hsv_img, scale, obstacle_list, config_params,arena); // process the Obstacles in red
        if(!res1) std::cout << "processObstacles return false" << std::endl;
        const bool res2 = processGate(hsv_img, scale, gate,config_params,arena); // process the gate in green, but rectangle
        if(!res2) std::cout << "processGate return false" << std::endl;
        const bool res3 = processVictims(hsv_img, scale, victim_list,config_params,arena); // process the victims in green, but circles with digit recognition
        if(!res3) std::cout << "processVictims return false" << std::endl;

        if(config_params.find_victims_debug_plot or config_params.find_gate_debug_plot or config_params.find_obstacles_debug_plot){
            cv::imshow("processMap", debug_image); // debug the process map image
            cv::waitKey(0);            
        }


        return res1 && res2 && res3;
    }

}


