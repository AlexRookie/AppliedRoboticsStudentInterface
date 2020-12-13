#pragma once
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <atomic>
#include <unistd.h>

#include <sstream>
#include <experimental/filesystem>
namespace student{

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
}
