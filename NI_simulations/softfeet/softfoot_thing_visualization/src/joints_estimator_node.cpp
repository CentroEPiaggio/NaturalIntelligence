/* JOINT ESTIMATOR NODE - Creates estimators for specified feet and publish joint sates for them.
Author: Mathew Jose Pollayil
Email:  mathewjose.pollayil@phd.unipi.it  */

#include "softfoot_thing_visualization/joints_estimator.h"

#define     DEBUG_JEN      0       // Prints out debug info

int main(int argc, char** argv) {
    
    ros::init (argc, argv, "softfoot_thing_visualization_joints_estimator");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(4);

    // Get needed softfeet global params
    bool online_calib = false;
    if (!nh.getParam("softfoot_viz/calibrate_online", online_calib)) {
         ROS_INFO_STREAM("SoftFoot Joint Estimator : No specific request for calibration.");
    };
    std::string connected_feet_name;
    if (!nh.getParam("softfoot_viz/connected_feet_name", connected_feet_name)) {
         ROS_FATAL_STREAM("SoftFoot Joint Estimator : Could not find the feet name.");
         return 1;
    };
    std::vector<int> connected_feet_ids;
    if (!nh.getParam("softfoot_viz/connected_feet_ids", connected_feet_ids)) {
         ROS_FATAL_STREAM("SoftFoot Joint Estimator : Could not find the feet ids.");
         return 1;
    };

    // Determine the number of feet
    int n_feet = connected_feet_ids.size();
    ROS_INFO_STREAM("SoftFoot Joint Estimator : Currently no. of connected feet is " << n_feet << ".");

    // Array of pointers to joint estimator objects
    std::vector<std::shared_ptr<softfoot_thing_visualization::JointsEstimator>> joint_estimators;
    for (int i = 0; i < n_feet; i++) {
        joint_estimators.push_back(std::make_shared
            <softfoot_thing_visualization::JointsEstimator>(nh, connected_feet_ids[i], connected_feet_name));
    }

    ROS_INFO_STREAM("SoftFoot Joint Estimator : starting...");

    // If needed, calibrating after waiting for some time
    if (online_calib) {
        ROS_INFO_STREAM("SoftFoot Joint Estimator : online calibration is set.");
        ROS_INFO_STREAM("SoftFoot Joint Estimator : Please assure that the feet are on a flat" 
            << " surface and DON'T MOVE them until calibration is finished!" 
            << " This might take some time...");
        sleep(5);
        ROS_INFO_STREAM("SoftFoot Joint Estimator : starting to calibrate the sensing...");
        for (auto it : joint_estimators) {
            it->calibrate();
        }
        
        ROS_INFO_STREAM("SoftFoot Joint Estimator : calibration finished.");
    }

    // Check if calibrated before spinning
    ROS_INFO_STREAM("SoftFoot Joint Estimator : checking for correct calibration...");
    bool all_calibrated = true;
    for (auto it : joint_estimators) {
        if (!it->check_calibration()) all_calibrated = false;
    }
    if (!all_calibrated) {
        ROS_FATAL_STREAM("SoftFoot Joint Estimator : not all feet are calibrated, this is bad!");
        return 1;
    }

    // Spin estimators as fast as possible until node is shut down
    ROS_INFO_STREAM("SoftFoot Joint Estimator : starting to spin and estimate!");
    spinner.start();
    while (ros::ok()) {

        // Estimate and publish joint states
        for (auto it : joint_estimators) {
            it->estimate();
        }

    }
    spinner.stop();

    // Shutting down when finished
    ros::shutdown();
    return 0;
}