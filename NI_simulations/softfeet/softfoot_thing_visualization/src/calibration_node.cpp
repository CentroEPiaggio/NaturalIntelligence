/* CALIBRATION NODE - Creates estimator and calibrates specified foot and saves its config.
Author: Mathew Jose Pollayil
Email:  mathewjose.pollayil@phd.unipi.it  */

#include "softfoot_thing_visualization/joints_estimator.h"

#define     DEBUG_CAL      0       // Prints out debug info

int main(int argc, char** argv) {
    
    ros::init (argc, argv, "softfoot_thing_visualization_joints_estimator");
    ros::NodeHandle nh;

    std::cout << "/*************************************/" << std::endl;
    std::cout << "          SOFT FOOT CALIBRATION        " << std::endl;
    std::cout << "/*************************************/" << std::endl;

    // Requesting the name of the foot to calibrate
    int foot_id;
    std::string foot_name;

    std::cout << "What is the base name of the feet in the robot model? Please type and press ENTER." << std::endl;
    std::cin >> foot_name;
    std::cout << "What is the ID of this specific foot? Please type an integer and press ENTER." << std::endl;
    std::cin >> foot_id;

    // Calibrator object for the foot
    softfoot_thing_visualization::JointsEstimator calibrator(nh, foot_id, foot_name);

    ROS_INFO_STREAM("SoftFoot Calibrator : started. Please ensure that " << 
        foot_name + std::to_string(foot_id) << " is in neutral position on a flat"
            << " and DON'T MOVE it until calibration is finished!" 
            << " This might take some time...");
 
    // Calibrating after waiting for some time
    sleep(5);

    // The name of the saved calibration file will be the same as the foot

    ROS_INFO_STREAM("SoftFoot Calibrator : starting to calibrate the sensing for " << 
        foot_name + "_" + std::to_string(foot_id) << ".");

    calibrator.calibrate_and_save(foot_name + "_" + std::to_string(foot_id));

    // Finished message
    ROS_INFO_STREAM("SoftFoot Calibrator : calibration finished. Saved configuration to file.");

    // Shutting down when finished
    ros::shutdown();
    return 0;
    
}