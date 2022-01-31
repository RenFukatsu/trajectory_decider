#ifndef TARGET_DECIDER_H_
#define TARGET_DECIDER_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "color_detector_params/hsv.h"
#include "kalman_filter/TargetArray.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "trajectory_generator/PathArray.h"

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
    void read_tracker_ids();
    void target_callback(const kalman_filter::TargetArrayConstPtr& input_targets);
    void roomba_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& input_pose_, int id);
    void move_targets(const kalman_filter::TargetArrayConstPtr& input_targets);
    void decide_trajectory();
    trajectory_generator::Path pick_one_trajectry(const std::vector<trajectory_generator::Path>& decided_paths);
    double calc_score(const trajectory_generator::Path& path,
                      const std::vector<trajectory_generator::Path>& decided_paths);
    double calc_distance_cost(const trajectory_generator::Path& path, const EstimateTarget& target);
    double calc_head_cost(const trajectory_generator::Path& path, const EstimateTarget& target);
    double calc_speed_cost(const trajectory_generator::Path& path, const EstimateTarget& target);
    double calc_exist_robot_cost(const trajectory_generator::Path& path,
                                 const std::vector<trajectory_generator::Path>& decided_paths);
    void path_array_callback(const trajectory_generator::PathArrayConstPtr& input_paths, int id);
    double norm(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b);
    double norm(const geometry_msgs::Point& a, const geometry_msgs::Point& b);
    double norm(double x1, double y1, double x2, double y2);
    void visualize_trajectory(const trajectory_generator::Path& path);

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::vector<ros::Publisher> roomba_ctrl_pubs_;
    ros::Publisher path_pub_;
    ros::Publisher visualize_trajectory_pub_;
    std::vector<ros::Subscriber> roomba_pose_subs_;
    std::vector<ros::Subscriber> path_array_subs_;
    ros::Subscriber target_sub_;
    double HZ;
    double PREDICT_TIME;
    double TIME_DIFFERENCE;
    double DISTANCE_GAIN;
    double HEAD_GAIN;
    double SPEED_GAIN;
    double EXIST_ROBOT_GAIN;
    std::vector<EstimateTarget> targets_;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> roomba_poses_;
    std::vector<trajectory_generator::PathArray> roombas_paths_;
    std::vector<int> roomba_poses_updated_;
    std::vector<int> path_array_updated_;
    bool targets_updated_;
    std::vector<int> tracker_robot_ids_;
};

#endif  // TARGET_DECIDER_H_
