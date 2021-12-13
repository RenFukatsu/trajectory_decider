#ifndef TARGET_DECIDER_H_
#define TARGET_DECIDER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "color_detector_params/hsv.h"
#include "trajectory_generator/PathArray.h"
#include "kalman_filter/TargetArray.h"

class TrajectoryDecider {
 public:
    struct EstimateTarget {
       kalman_filter::Target target;
       std::vector<geometry_msgs::Point> estimate_move;
    };
    TrajectoryDecider();
    void process();
    bool check_all_updates_are_true();
    void set_all_updates_to_false();
    void target_callback(const kalman_filter::TargetArrayConstPtr &input_targets);
    void roomba_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input_pose_, int my_number);
    void roomba_pose_callback();
    void move_targets(const kalman_filter::TargetArrayConstPtr& input_targets);
    void decide_trajectory();
    nav_msgs::Path pick_one_trajectry();
    double calc_score(const nav_msgs::Path& path);
    double calc_distance_cost(const nav_msgs::Path& path);
    double calc_head_cost();
    double calc_speed_cost();
    double calc_exist_robot_cost();
    void path_array_callback(const trajectory_generator::PathArrayConstPtr& input_paths, int my_number);

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::vector<ros::Publisher> roomba_ctrl_pubs_;
    ros::Publisher path_pub_;
    std::vector<ros::Subscriber> roomba_pose_subs_;
    std::vector<ros::Subscriber> path_array_subs_;
    ros::Subscriber target_sub_;
    double HZ;
    double PREDICT_TIME;
    double TIME_DEFFERENCE;
    std::vector<std::string> colors_;
    std::vector<EstimateTarget> targets_;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> roomba_poses_;
    std::vector<trajectory_generator::PathArray> roombas_paths_;
    std::vector<int> roomba_poses_updated_;
    std::vector<int> path_array_updated_;
    bool targets_updated_;
};

#endif  // TARGET_DECIDER_H_
