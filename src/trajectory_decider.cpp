#include "trajectory_decider/trajectory_decider.h"

TrajectoryDecider::TrajectoryDecider() : private_nh_("~") {
    private_nh_.param("HZ", HZ, 10.0);
    private_nh_.param("PREDICT_TIME", PREDICT_TIME, 5.0);
    TIME_DEFFERENCE = 1.0 / HZ;

    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1);

    std::vector<color_detector_params_hsv::ThresholdHSV> _;
    color_detector_params_hsv::init(colors_, _);
    roomba_poses_updated_.resize(colors_.size());
    path_array_updated_.resize(colors_.size());
    set_all_updates_to_false();
    roomba_pose_subs_.resize(colors_.size());
    path_array_subs_.resize(colors_.size());
    target_sub_ = nh_.subscribe("target", 1, &TrajectoryDecider::target_callback, this);
    for (size_t i = 0; i < colors_.size(); i++) {
        std::string roomba = "roomba" + std::to_string(i + 1);
        roomba_pose_subs_[i] = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            roomba + "/target/position", 1, boost::bind(&TrajectoryDecider::roomba_pose_callback, this, _1, i + 1));
        path_array_subs_[i] = nh_.subscribe<trajectory_generator::PathArray>(
            roomba + "/trajectries", 1, boost::bind(&TrajectoryDecider::path_array_callback, this, _1, i + 1));
    }
}

void TrajectoryDecider::target_callback(const kalman_filter::TargetArrayConstPtr& input_targets) {
    move_targets(input_targets);
    targets_updated_ = true;
}

void TrajectoryDecider::roomba_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& input_pose,
                                             int my_number) {
    roomba_poses_[my_number - 1] = *input_pose;
    roomba_poses_updated_[my_number - 1] = 1;
}

void TrajectoryDecider::path_array_callback(const trajectory_generator::PathArrayConstPtr& input_paths, int my_number) {
    roombas_paths_[my_number - 1] = *input_paths;
    path_array_updated_[my_number - 1] = 1;
}

void TrajectoryDecider::move_targets(const kalman_filter::TargetArrayConstPtr& input_targets) {
    for (const auto& target : input_targets->targets) {
        EstimateTarget est;
        est.target = target;
        geometry_msgs::Point point = target.position;
        double x_vel = target.twist.linear.x;
        double y_vel = target.twist.linear.y;
        for (double time = 0; time <= PREDICT_TIME; time += TIME_DEFFERENCE) {
            point.x += x_vel * TIME_DEFFERENCE;
            point.y += y_vel * TIME_DEFFERENCE;
            est.estimate_move.push_back(point);
        }
        targets_.push_back(est);
    }
}

double TrajectoryDecider::calc_distance_cost(const nav_msgs::Path& path) {
    static auto norm = [](const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    };
    double min_dist = 10.0;
    for (const auto& target : targets_) {
        geometry_msgs::Point target_last_point = target.estimate_move.back();
        geometry_msgs::PoseStamped robot_last_pose = path.poses.back();
        double dist = norm(target_last_point, robot_last_pose.pose.position);
        min_dist = std::min(dist, min_dist);
    }
    return 1.0 / 1.0 + min_dist;
}

double TrajectoryDecider::calc_head_cost() { return 0; }

double TrajectoryDecider::calc_speed_cost() { return 0; }

double TrajectoryDecider::calc_exist_robot_cost() { return 0; }

double TrajectoryDecider::calc_score(const nav_msgs::Path& path) {
    double dist_cost = calc_distance_cost(path);
    // double head_cost = calc_head_cost();
    // double speed_cost = calc_speed_cost();
    // double exist_cost = calc_exist_robot_cost();
    return dist_cost;
}

nav_msgs::Path TrajectoryDecider::pick_one_trajectry() {
    double max_score = 0;
    int roomba_index = -1;
    int path_index = -1;
    nav_msgs::Path best_path;
    for (size_t i = 0; i < roombas_paths_.size(); i++) {
        for (size_t j = 0; j < roombas_paths_[i].paths.size(); j++) {
            double score = calc_score(roombas_paths_[i].paths[j]);
            if (max_score < score) {
                max_score = score;
                roomba_index = i;
                path_index = j;
            }
        }
    }
    ROS_ASSERT(roomba_index != -1);
    best_path = roombas_paths_[roomba_index].paths[path_index];
    roombas_paths_.erase(roombas_paths_.begin() + roomba_index);
    return best_path;
}

void TrajectoryDecider::decide_trajectory() {
    int num_roomba = colors_.size();
    int num_target = targets_.size();
    int num_path = roombas_paths_.size();
    int num_point = roombas_paths_.front().paths.size();
    ROS_INFO_STREAM_THROTTLE(10.0, "roomba :" << num_roomba);
    ROS_INFO_STREAM_THROTTLE(10.0, "target :" << num_target);
    ROS_INFO_STREAM_THROTTLE(10.0, "path / roomba :" << num_path);
    ROS_INFO_STREAM_THROTTLE(10.0, "point / path :" << num_point);
    ROS_INFO_STREAM_THROTTLE(10.0, "r * (r + 1) / 2 * r * pa * t :" << num_roomba * (num_roomba + 1) / 2 * num_roomba *
                                                                            num_path * num_target);
    ROS_DEBUG_STREAM_THROTTLE(
        10.0, "upper * point :" << num_roomba * (num_roomba + 1) / 2 * num_roomba * num_path * num_target * num_point);
    for (size_t i = 0; i < colors_.size(); i++) {
        nav_msgs::Path path = pick_one_trajectry();
        path_pub_.publish(path);
    }
}

bool TrajectoryDecider::check_all_updates_are_true() {
    bool res = true;
    if (!targets_updated_) {
        res = false;
        ROS_WARN_STREAM_THROTTLE(7.0, "kf target info has not been updated");
    }
    for (size_t i = 0; i < roomba_poses_updated_.size(); i++) {
        if (!roomba_poses_updated_[i]) {
            res = false;
            ROS_WARN_STREAM_THROTTLE(7.0, "roomba" << std::to_string(i + 1) << "'s pose has not been updated");
        }
    }
    for (size_t i = 0; i < path_array_updated_.size(); i++) {
        if (!path_array_updated_[i]) {
            res = false;
            ROS_WARN_STREAM_THROTTLE(7.0, "roomba" << std::to_string(i + 1) << "'s path has not been updated");
        }
    }
    return res;
}

void TrajectoryDecider::set_all_updates_to_false() {
    targets_updated_ = false;
    for (size_t i = 0; i < roomba_poses_updated_.size(); i++) {
        roomba_poses_updated_[i] = 0;
    }
    for (size_t i = 0; i < path_array_updated_.size(); i++) {
        path_array_updated_[i] = 0;
    }
}

void TrajectoryDecider::process() {
    ros::Rate loop_rate(HZ);

    while (ros::ok()) {
        double start_time = ros::Time::now().toSec();

        if (check_all_updates_are_true()) {
            decide_trajectory();
            set_all_updates_to_false();
        }

        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO_STREAM("loop time : " << ros::Time::now().toSec() - start_time << "[sec]");
    }
}
