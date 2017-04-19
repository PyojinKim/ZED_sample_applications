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


// Standard includes
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>

// ZED includes
#include <sl/Camera.hpp>


int main(int argc, char **argv)
{
    // Create a ZED camera object
    sl::Camera zed;


    // Set configuration parameters
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION_VGA;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;
    init_params.coordinate_units = sl::UNIT_METER;
    init_params.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
    init_params.sdk_verbose = true;


    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::SUCCESS) {
        exit(-1);
    }


    // Enable positional tracking with default parameters
    sl::TrackingParameters tracking_parameters;
    tracking_parameters.initial_world_transform = sl::Transform::identity();
    tracking_parameters.enable_spatial_memory = true;


    // Enable motion tracking
    zed.enableTracking(tracking_parameters);


    // Track the camera position during 1000 frames
    int i = 0;
    sl::Pose zed_pose;
    float tx, ty, tz = 0;
    float ox, oy, oz, ow = 0;
    float rx, ry, rz = 0;
    
    while (i < 1000) {
        if (!zed.grab()) {
            // Get camera position in World frame
            sl::TRACKING_STATE tracking_state = zed.getPosition(zed_pose, sl::REFERENCE_FRAME_WORLD);

            // Get motion tracking confidence
            int tracking_confidence = zed_pose.pose_confidence;

            if (tracking_state == sl::TRACKING_STATE_OK) {
                // Extract 3x1 rotation from pose
		//sl::Vector3<float> rotation = zed_pose.getRotationVector();
		//rx = rotation.x;
		//ry = rotation.y;
		//rz = rotation.z;

                // Extract 4x1 orientation from pose
		ox = zed_pose.getOrientation().ox;
		oy = zed_pose.getOrientation().oy;
		oz = zed_pose.getOrientation().oz;
		ow = zed_pose.getOrientation().ow;

                // Extract translation from pose
		sl::Vector3<float> translation = zed_pose.getTranslation();
		tx = translation.tx;
		ty = translation.ty;
		tz = translation.tz;


                // Display the translation & orientation & timestamp
                printf("Translation: Tx: %.3f, Ty: %.3f, Tz: %.3f, Timestamp: %llu\n", tx, ty, tz, zed_pose.timestamp);
                printf("Orientation: Ox: %.3f, Oy: %.3f, Oz: %.3f, Ow: %.3f\n\n", ox, oy, oz, ow);
                i++;
            }
        }
    }


    // Disable positional tracking and close the camera
    zed.disableTracking();
    zed.close();
    return 0;
}
