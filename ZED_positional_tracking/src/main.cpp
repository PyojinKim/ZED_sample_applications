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


/******************************************************************************************
 ** This sample demonstrates a simple way to use the ZED as a positional tracker and     **
 **  show the result in a OpenGL window.                                                 **
 ** Even if the depth/images can be retrieved (grab is done in STANDARD mode),           **
 **  we save here the "printing" time to be more efficient.                              **
 ******************************************************************************************/


#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sl/Camera.hpp>
#include "TrackingViewer.hpp"


// Using std namespace
using namespace std;


// Create ZED object (camera, callback, pose)
std::thread zed_callback;
sl::Camera zed;
sl::Pose camera_pose;

// States
bool exit_ = false;

// OpenGL Window to display the ZED in world space
TrackingViewer viewer;

// CSV log file to store Motion Tracking data (with timestamp)
std::string csvName;

// Sample functions
void startZED();
void run();
void close();
void transformPose(sl::Transform &pose, float tx);
void transformPoseOpenGL(sl::Transform &input, sl::Transform &output);
void quaternion2rotationMatrix(float qw, float qx, float qy, float qz, sl::Rotation &rotation);


int main(int argc, char **argv) {

	// Setup configuration parameters for the ZED
    sl::InitParameters initParameters;
    initParameters.camera_resolution = sl::RESOLUTION_VGA;
    initParameters.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;
    initParameters.coordinate_units = sl::UNIT_METER;
    initParameters.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
	initParameters.sdk_verbose = true;


	// Open the ZED
    sl::ERROR_CODE err = zed.open(initParameters);
    if (err != sl::SUCCESS) {
        cout << sl::errorCode2str(err) << endl;
        zed.close();
        return 1;
    }


	// Initialize motion tracking parameters
    sl::TrackingParameters trackingParameters;
    trackingParameters.initial_world_transform = sl::Transform::identity();
    trackingParameters.enable_spatial_memory = true;


	// Enable motion tracking
    zed.enableTracking(trackingParameters);


	// Initialize OpenGL viewer
    viewer.init();


	// Start ZED callback
	startZED();


	// Set GLUT callback
    glutCloseFunc(close);
    glutMainLoop();
    return 0;
}


/**
*  This functions start the ZED's thread that grab images and data.
**/
void startZED()
{
	exit_ = false;
	zed_callback = std::thread(run);
}



/**
*  This function loops to get images and data from the ZED. It can be considered as a callback.
*  You can add your own code here.
**/
void run() {

    // Variables
    unsigned long long previous_timestamp, current_timestamp = 0;
    float ox, oy, oz, ow = 0;
    float r00, r01, r02 = 0;
    float r10, r11, r12 = 0;
    float r20, r21, r22 = 0;
	float tx, ty, tz = 0;
	float rx, ry, rz = 0;


	// Get the translation from the left eye to the center of the camera
	float camera_left_to_center = zed.getCameraInformation().calibration_parameters.T.x * 0.5f;


	// Create text for GUI
	char text_rotation[256];
	char text_translation[256];


	// If asked, create the CSV motion tracking file
	ofstream outputFile;
	if (!csvName.empty()) {
		outputFile.open(csvName + ".csv");
		if (!outputFile.is_open())
			cout << "WARNING: Can't create CSV file. Launch the sample with Administrator rights" << endl;
		else
			outputFile << "Timestamp(ns);Rotation_X(rad);Rotation_Y(rad);Rotation_Z(rad);Position_X(m);Position_Y(m);Position_Z(m);" << endl;
	}


	// loop until exit_ flag has been set to true
	while (!exit_) {
		if (!zed.grab()) {

			// Get camera position in World frame
			sl::TRACKING_STATE tracking_state = zed.getPosition(camera_pose, sl::REFERENCE_FRAME_WORLD);


			// Get motion tracking confidence
			int tracking_confidence = camera_pose.pose_confidence;


			if (tracking_state == sl::TRACKING_STATE_OK) {

				// Extract 3x1 rotation from pose
				sl::Vector3<float> rotation = camera_pose.getRotationVector();
                rx = rotation.x;
                ry = rotation.y;
                rz = rotation.z;

                // Extract 4x1 orientation from pose
                ox = camera_pose.getOrientation().ox;
                oy = camera_pose.getOrientation().oy;
                oz = camera_pose.getOrientation().oz;
                ow = camera_pose.getOrientation().ow;

                // Extract 3x3 rotation matrix (R_gc)
                r00 = camera_pose.pose_data.r00;
                r01 = camera_pose.pose_data.r01;
                r02 = camera_pose.pose_data.r02;

                r10 = camera_pose.pose_data.r10;
                r11 = camera_pose.pose_data.r11;
                r12 = camera_pose.pose_data.r12;

                r20 = camera_pose.pose_data.r20;
                r21 = camera_pose.pose_data.r21;
                r22 = camera_pose.pose_data.r22;

                // Extract translation from pose (p_gc)
                sl::Vector3<float> translation = camera_pose.getTranslation();
                tx = translation.tx;
                ty = translation.ty;
                tz = translation.tz;

                // Extract previous and current timestamp
                previous_timestamp = current_timestamp;
                current_timestamp = camera_pose.timestamp;
                double dt = (double) (current_timestamp - previous_timestamp) * 0.000000001;



                // Display the translation & orientation & timestamp
                printf("Translation: Tx: %.3f, Ty: %.3f, Tz: %.3f, dt: %.3lf\n", tx, ty, tz, dt);
                printf("Orientation: Ox: %.3f, Oy: %.3f, Oz: %.3f, Ow: %.3f\n\n", ox, oy, oz, ow);


				// Fill text
				sprintf(text_translation, "%3.2f; %3.2f; %3.2f", tx, ty, tz);
				sprintf(text_rotation, "%3.2f; %3.2f; %3.2f", rx, ry, rz);


				// Save the pose data in the csv file
				if (outputFile.is_open()) {
				    outputFile << zed.getCameraTimestamp() << "; " << text_translation << "; " << text_rotation << ";" << endl;
				}


                // Transform and send all the information to the OpenGL viewer
                transformPose(camera_pose.pose_data, camera_left_to_center);
                transformPoseOpenGL(camera_pose.pose_data, camera_pose.pose_data);
                viewer.updateZEDPosition(camera_pose.pose_data);
			}

			// Even if tracking state is not OK, send the text (state, confidence, values) to the viewer
			viewer.updateText(string(text_translation), string(text_rotation), tracking_state);
		}
		else sl::sleep_ms(1);
	}
}



/**
*  This function frees and close the ZED, its callback(thread) and the viewer
**/
void close()
{
    exit_ = true;
    zed_callback.join();
    zed.disableTracking();
    zed.close();
    viewer.exit();
}



/**
*  This function separates the camera frame and the motion tracking frame.
*  In this sample, we move the motion tracking frame to the center of the ZED ( baseline/2 for the X translation)
*  By default, the camera frame and the motion tracking frame are at the same place: the left sensor of the ZED.
**/
void transformPose(sl::Transform &pose, float tx)
{
    sl::Transform transform_;
    transform_.setIdentity(); // Create the transformation matrix to separate camera frame and motion tracking frame
    transform_.tx = tx; // Move the tracking frame at the center of the ZED (between ZED's eyes)
    pose = sl::Transform::inverse(transform_) * pose * transform_; // apply the transformation
}



void transformPoseOpenGL(sl::Transform &input, sl::Transform &output)
{
    sl::Transform transform_OpenGL;
    transform_OpenGL.setIdentity();
    double phi = 180 * (3.141592/180);

    transform_OpenGL.r00 = 1;
    transform_OpenGL.r01 = 0;
    transform_OpenGL.r02 = 0;

    transform_OpenGL.r10 = 0;
    transform_OpenGL.r11 = cos(phi);
    transform_OpenGL.r12 = -sin(phi);

    transform_OpenGL.r20 = 0;
    transform_OpenGL.r21 = sin(phi);
    transform_OpenGL.r22 = cos(phi);

    output = transform_OpenGL * input;
}



void quaternion2rotationMatrix(float qw, float qx, float qy, float qz, sl::Rotation &rotation)
{
    float a = qw;
    float b = qx;
    float c = qy;
    float d = qz;

    // quaternion to dcm
    sl::Rotation rotation_;
    rotation_.setIdentity();
    rotation_.r00 = a*a+b*b-c*c-d*d;
    rotation_.r01 = 2*(b*c-a*d);
    rotation_.r02 = 2*(b*d+a*c);

    rotation_.r10 = 2*(b*c+a*d);
    rotation_.r11 = a*a-b*b+c*c-d*d;
    rotation_.r12 = 2*(c*d-a*b);

    rotation_.r20 = 2*(b*d-a*c);
    rotation_.r21 = 2*(c*d+a*b);
    rotation_.r22 = a*a-b*b-c*c+d*d;

    rotation = rotation_;

    /* sl::Rotation test_rotation_matrix;
    quaternion2rotationMatrix(ow, ox, oy, oz, test_rotation_matrix);
    printf("r00: %.3f / %.3f \n\n", r00, test_rotation_matrix.r00);
    printf("r01: %.3f / %.3f \n\n", r01, test_rotation_matrix.r01);
    printf("r02: %.3f / %.3f \n\n", r02, test_rotation_matrix.r02);

    printf("r10: %.3f / %.3f \n\n", r10, test_rotation_matrix.r10);
    printf("r11: %.3f / %.3f \n\n", r11, test_rotation_matrix.r11);
    printf("r12: %.3f / %.3f \n\n", r12, test_rotation_matrix.r12);

    printf("r20: %.3f / %.3f \n\n", r20, test_rotation_matrix.r20);
    printf("r21: %.3f / %.3f \n\n", r21, test_rotation_matrix.r21);
    printf("r22: %.3f / %.3f \n\n", r22, test_rotation_matrix.r22); */
}



