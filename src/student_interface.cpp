
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <atomic>
#include <unistd.h>

#include <sstream>
#include <experimental/filesystem>

namespace student {

/*
 void loadImage(cv::Mat& img_out, const std::string& config_folder){  
   throw std::logic_error( "STUDENT FUNCTION - LOAD IMAGE - NOT IMPLEMENTED" );
 }
*/
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

    static int image_number = 0; // image number to be saved in the destination folder
    static bool first_flag = true; // create a destination folder for the first time
    static std::string folder_path; //output folder_path

    if(first_flag){      

      std::stringstream ss;
      ss << config_folder << "/camera_image_captured/"; // Creating a folder name for storing the images Appending from the one in config file
      folder_path = ss.str();
      if(!std::experimental::filesystem::exists(folder_path)){ // If the folder doesnt exist in that location
          if(!std::experimental::filesystem::create_directories(folder_path))
              throw std::logic_error( "CANNOT CREATE DIRECTORY" );
      } // Error to check if the directory is created or nots
      first_flag = false; // open the folder only for the first time.s
    }

    cv::imshow( topic, img_in); // Show the image in the same topic name
    char c = (char)cv::waitKey(30);

    std::stringstream img_file; 
    if(c == 's'){ // if the pressed key is 's' -->save the image
        img_file << folder_path<< "raw_" <<(image_number++) << ".jpg"; // 
        cv::imwrite( img_file.str(), img_in ); // save the image in desired location
        std::cout << "Saved image " << img_file.str() << std::endl;
    }
  }

}

