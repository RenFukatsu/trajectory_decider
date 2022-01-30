#include "trajectory_decider/trajectory_decider.h"

TrajectoryDecider::TrajectoryDecider() : private_nh_("~") {
    private_nh_.param("HZ", HZ, 10.0);
    private_nh_.param("PREDICT_TIME", PREDICT_TIME, 5.0);
    private_nh_.param("TIME_DIFFERENCE", TIME_DIFFERENCE, 0.5);
    private_nh_.param("DISTANCE_GAIN", DISTANCE_GAIN, 1.0);
    private_nh_.param("HEAD_GAIN", HEAD_GAIN, 1.0);
    private_nh_.param("SPEED_GAIN", SPEED_GAIN, 1.0);
    private_nh_.param("EXIST_ROBOT_GAIN", EXIST_ROBOT_GAIN, 1.0);

    path_pub_ = nh_.advertise<trajectory_generator::Path>("path", 1);

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
        for (double time = 0; time <= PREDICT_TIME; time += TIME_DIFFERENCE) {
            point.x += x_vel * TIME_DIFFERENCE;
            point.y += y_vel * TIME_DIFFERENCE;
            est.estimate_move.push_back(point);
        }
        targets_.push_back(est);
    }
}

double TrajectoryDecider::norm(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b) {
    return norm(a.pose.position, b.pose.position);
}

double TrajectoryDecider::norm(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    return norm(a.x, a.y, b.x, b.y);
}

double TrajectoryDecider::norm(double x1, double y1, double x2, double y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

double TrajectoryDecider::calc_distance_cost(const trajectory_generator::Path& path, const EstimateTarget& target) {
    geometry_msgs::Point target_last_point = target.estimate_move.back();
    geometry_msgs::PoseStamped robot_last_point = path.poses.back();
    double dist = norm(target_last_point, robot_last_point.pose.position);
    return 1.0 / (1.0 + dist);
}

double TrajectoryDecider::calc_head_cost(const trajectory_generator::Path& path, const EstimateTarget& target) {
    double target_theta = std::atan2(target.target.twist.linear.y, target.target.twist.linear.x);
    geometry_msgs::PoseStamped robot_last_point = path.poses.back();
    tf2::Quaternion quat;
    tf2::convert(robot_last_point.pose.orientation, quat);
    double r, p, y;
    tf2::Matrix3x3(quat).getRPY(r, p, y);
    double robot_theta = y;
    return 1.0 / (1.0 + std::abs(target_theta - robot_theta));
}

double TrajectoryDecider::calc_speed_cost(const trajectory_generator::Path& path, const EstimateTarget& target) {
    double tx = target.target.twist.linear.x;
    double ty = target.target.twist.linear.y;
    double target_speed = std::sqrt(tx * tx + ty * ty);
    double robot_speed = path.velocity;
    return 1.0 / (1.0 + std::abs(target_speed - robot_speed));
}

double TrajectoryDecider::calc_exist_robot_cost(const trajectory_generator::Path& path,
                                                const std::vector<trajectory_generator::Path>& decided_paths) {
    double cost = 1.0;
    for (const auto& dpath : decided_paths) {
        double dist = norm(path.poses.back(), dpath.poses.back());
        cost *= 1.0 - 1.0 / (1.0 + dist);
    }
    return cost;
}

double TrajectoryDecider::calc_score(const trajectory_generator::Path& path,
                                     const std::vector<trajectory_generator::Path>& decided_paths) {
    double max_cost = 0.0;
    for (const auto& target : targets_) {
        double dist_cost = calc_distance_cost(path, target);
        double head_cost = calc_head_cost(path, target);
        double speed_cost = calc_speed_cost(path, target);
        double exist_cost = calc_exist_robot_cost(path, decided_paths);
        double cost = (DISTANCE_GAIN * dist_cost + HEAD_GAIN * head_cost + SPEED_GAIN * speed_cost) * EXIST_ROBOT_GAIN *
                      exist_cost;
        max_cost = std::max(cost, max_cost);
    }
    return max_cost;
}

trajectory_generator::Path TrajectoryDecider::pick_one_trajectry(
    const std::vector<trajectory_generator::Path>& decided_paths) {
    double max_score = 0;
    int roomba_index = -1;
    int path_index = -1;
    trajectory_generator::Path best_path;
    for (size_t i = 0; i < roombas_paths_.size(); i++) {
        for (size_t j = 0; j < roombas_paths_[i].paths.size(); j++) {
            double score = calc_score(roombas_paths_[i].paths[j], decided_paths);
            if (max_score < score) {
                max_score = score;
                roomba_index = i;
                path_index = j;
            }
        }
    }
    ROS_ASSERT(roomba_index != -1);
    best_path = roombas_paths_[roomba_index].paths[path_index];
    best_path.id = roomba_index;
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
    std::vector<trajectory_generator::Path> decided_paths;
    for (size_t i = 0; i < colors_.size(); i++) {
        trajectory_generator::Path path = pick_one_trajectry(decided_paths);
        path_pub_.publish(path);
        decided_paths.push_back(path);
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
