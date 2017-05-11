///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


/***************************************************************************************************
 ** This sample demonstrates how to grab images and depth/disparity map with the ZED SDK          **
 ** Both images and depth/disparity map are displayed with OpenCV                                 **
 ** Most of the functions of the ZED SDK are linked with a key press event (using OpenCV)         **
 ***************************************************************************************************/

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sl/Core.hpp>
#include <sl/defines.hpp>
#include <boost/filesystem.hpp>


// user-defined parameters
bool toVisualize = true;
bool toSave = true;


using namespace sl;


int main(int argc, char **argv) {


    // Create a ZED camera object
    Camera zed;


    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD1080;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = sl::UNIT_METER;


    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) {
        return 1;
    }


    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;


    // Jetson only. Execute the calling thread on 2nd core
    Camera::sticktoCPUCore(2);


    // Create sl::Mat object
    sl::Mat image_zed(zed.getResolution(), MAT_TYPE_8U_C4);
    cv::Mat image_ocv(image_zed.getHeight(), image_zed.getWidth(), CV_8UC4, image_zed.getPtr<sl::uchar1>(sl::MEM_CPU));
    sl::Mat depth_zed(zed.getResolution(), MAT_TYPE_32F_C1);
    cv::Mat depth_ocv(depth_zed.getHeight(), depth_zed.getWidth(), CV_32FC1, depth_zed.getPtr<sl::uchar1>(sl::MEM_CPU));
    sl::Mat depth_image_zed(zed.getResolution(), MAT_TYPE_8U_C4);
    cv::Mat depth_image_ocv(depth_image_zed.getHeight(), depth_image_zed.getWidth(), CV_8UC4, depth_image_zed.getPtr<sl::uchar1>(sl::MEM_CPU));


    // Create OpenCV images to display (lower resolution to fit the screen)
    cv::Size displaySize(720, 404);
    cv::Mat image_ocv_display(displaySize, CV_8UC4);
    cv::Mat depth_image_ocv_display(displaySize, CV_8UC4);


    // Make folders for saving current image
    std::string folder_name_depth = "images/depth/";
    std::string folder_name_rgb = "images/rgb/";
    std::string folder_create_command;
    folder_create_command = "mkdir -p " + folder_name_depth;
    system(folder_create_command.c_str());
    folder_create_command = "mkdir -p " + folder_name_rgb;
    system(folder_create_command.c_str());


    // Loop until 'q' is pressed
    int image_counter = 1;
    char key = ' ';
    while (key != 'q') {

        // Grab and display image and depth
        if (zed.grab(runtime_parameters) == SUCCESS) {

            // Retrieve ZED images
            zed.retrieveImage(image_zed, VIEW_LEFT);
            zed.retrieveMeasure(depth_zed, MEASURE_DEPTH);
            std::cout << depth_ocv.at<float>(depth_ocv.rows/2, depth_ocv.cols/2) << std::endl;
            zed.retrieveImage(depth_image_zed, VIEW_DEPTH);


            // Show ZED images
            if (toVisualize) {
                cv::resize(image_ocv, image_ocv_display, displaySize);
                cv::resize(depth_image_ocv, depth_image_ocv_display, displaySize);
                cv::imshow("Image", image_ocv_display);
                cv::imshow("Depth", depth_image_ocv_display);
            }


            // Save ZED images
            if (toSave) {
                // convert for depth image save
                cv::Mat depth_ocv_save;
                depth_ocv_save = depth_ocv * 1000;
                depth_ocv_save.convertTo(depth_ocv_save, CV_16UC1);


                char image_index[255];
                sprintf(image_index, "%010d.png", image_counter);
                std::string image_file_name;

                image_file_name = folder_name_depth + image_index;
                cv::imwrite(image_file_name, depth_ocv_save);
                std::cout << image_file_name << std::endl;

                image_file_name = folder_name_rgb + image_index;
                cv::imwrite(image_file_name, image_ocv);
                std::cout << image_file_name << std::endl;
            }

            key = cv::waitKey(10);
            image_counter++;
        }
    }

    zed.close();
    return 0;
}



