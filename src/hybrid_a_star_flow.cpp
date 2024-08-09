/*******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 Zhang Zhimeng
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "hybrid_a_star/hybrid_a_star_flow.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "hybrid_a_star/timer.h"

double vehicle_length_;
double vehicle_width_;
double vehicle_rear_dis_;
double wheel_base_;
double steering_angle_;

double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

HybridAStarFlow::HybridAStarFlow(ros::NodeHandle &nh) {
    double steering_angle = nh.param("planner/steering_angle", 30);
    int steering_angle_discrete_num = nh.param("planner/steering_angle_discrete_num", 2);
    double wheel_base = nh.param("planner/wheel_base", 0.8);         //轴距，即前后轮的距离
    double segment_length = nh.param("planner/segment_length", 1.0); //每一段采样的长度
    int segment_length_discrete_num = nh.param("planner/segment_length_discrete_num", 8);
    double steering_penalty = nh.param("planner/steering_penalty", 1.2);
    double steering_change_penalty = nh.param("planner/steering_change_penalty", 1.5);
    double reversing_penalty = nh.param("planner/reversing_penalty", 2.0);
    double shot_distance = nh.param("planner/shot_distance", 15.0);

    bool reverse_enable = nh.param("planner/reverse_enable", 0);

    vehicle_length_ = nh.param("planner/vehicle_length", 2.0);
    vehicle_width_ = nh.param("planner/vehicle_width", 1.0);
    vehicle_rear_dis_ = nh.param("planner/vehicle_rear_dis", 0.5);
    wheel_base_ = wheel_base;
    steering_angle_ = steering_angle;

    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
            steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base_,
            steering_penalty, reversing_penalty, steering_change_penalty, shot_distance, 72, reverse_enable
    );
    costmap_sub_ptr_ = std::make_shared<CostMapSubscriber>(nh, "/map", 1);
    init_pose_sub_ptr_ = std::make_shared<InitPoseSubscriber2D>(nh, "/initialpose", 1);
    goal_pose_sub_ptr_ = std::make_shared<GoalPoseSubscriber2D>(nh, "/move_base_simple/goal", 1);

    path_pub_ = nh.advertise<nav_msgs::Path>("searched_path", 1);
    spath_pub_ = nh.advertise<nav_msgs::Path>("searched_path_smoothed", 1);
    spathWithDirection_pub_ = nh.advertise<nav_msgs::Path>("searched_path_smoothed_with_d", 1);
    searched_tree_pub_ = nh.advertise<visualization_msgs::Marker>("searched_tree", 1);
    vehicle_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vehicle_path", 1);
    goal_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("goal_pose", 1);
    start_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("start_pose", 1);
    path_forward_pub_ = nh.advertise<visualization_msgs::MarkerArray>("path_forward", 1);
    path_backward_pub_ = nh.advertise<visualization_msgs::MarkerArray>("path_backward", 1);

    smoother.initNh(nh);
    has_map_ = false;
}

void HybridAStarFlow::Run() {
    ReadData();

    if (!has_map_) {
        if (costmap_deque_.empty()) {
            return;
        }

        current_costmap_ptr_ = costmap_deque_.front();
        costmap_deque_.pop_front();

        //current_costmap_ptr_->info.width，这里的width不是米制的，而是地图的格子数目的宽度
        const double map_resolution = 0.2;
        kinodynamic_astar_searcher_ptr_->Init(
                current_costmap_ptr_->info.origin.position.x,
                1.0 * current_costmap_ptr_->info.width * current_costmap_ptr_->info.resolution,
                current_costmap_ptr_->info.origin.position.y,
                1.0 * current_costmap_ptr_->info.height * current_costmap_ptr_->info.resolution,
                vehicle_length_, vehicle_width_, vehicle_rear_dis_,
                current_costmap_ptr_->info.resolution,
                map_resolution
        );
        //map_w，这里的map应该就不是米制的了，而是和状态分辨率有关
        unsigned int map_w = std::floor(current_costmap_ptr_->info.width / map_resolution * current_costmap_ptr_->info.resolution);
        unsigned int map_h = std::floor(current_costmap_ptr_->info.height / map_resolution * current_costmap_ptr_->info.resolution);
        for (unsigned int w = 0; w < map_w; ++w) {
            for (unsigned int h = 0; h < map_h; ++h) {
                //这里的x, y应该是米制下的
                auto x = static_cast<unsigned int> ((w + 0.5) * map_resolution
                                                    / current_costmap_ptr_->info.resolution);
                auto y = static_cast<unsigned int> ((h + 0.5) * map_resolution
                                                    / current_costmap_ptr_->info.resolution);

                if (current_costmap_ptr_->data[y * current_costmap_ptr_->info.width + x]) {
                    kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
                }
            }
        }
        //这里生成维诺图, 和地图一样的尺寸（width * height），不是米制
        bool** binMap;//二维数组，
        binMap = new bool*[current_costmap_ptr_->info.width];

        for (int x = 0; x < current_costmap_ptr_->info.width; x++) { binMap[x] = new bool[current_costmap_ptr_->info.width]; }//这里可简化为一次申请

        for (int x = 0; x < current_costmap_ptr_->info.width; ++x) {
            for (int y = 0; y < current_costmap_ptr_->info.width; ++y) {
            binMap[x][y] = current_costmap_ptr_->data[y * current_costmap_ptr_->info.width + x] ? true : false;
            }
        }//转化为二值地图


        voronoiDiagram.initializeMap(current_costmap_ptr_->info.width, current_costmap_ptr_->info.width,
                                     current_costmap_ptr_->info.resolution, binMap);//注意这里传入到DynamicVoronoi里并进行保存，所以没有delete
        voronoiDiagram.update();
        voronoiDiagram.visualize("");//将Voronoi Diagram初始化、更新并显示

        /// maximum possible curvature of the non-holonomic vehicle
        float kappaMax = 1.f / (wheel_base_ * 57.3 / steering_angle_ * 1.1);
        /// maximum distance to obstacles that is penalized
        float obsDMax = vehicle_width_ * 0.5 + 0.5;// = Constants::minRoadWidth;
        /// maximum distance for obstacles to influence the voronoi field
        float vorObsDMax = vehicle_width_ * 0.5 + 0.5;// = Constants::minRoadWidth;
        smoother.init(kappaMax, obsDMax, vorObsDMax);


        has_map_ = true;
    }
    costmap_deque_.clear();
    //原来是在这里一直循环
    while (HasStartPose() && HasGoalPose()) {
        InitPoseData();

        double start_yaw = tf::getYaw(current_init_pose_ptr_->pose.pose.orientation);
        double goal_yaw = tf::getYaw(current_goal_pose_ptr_->pose.orientation);
        Vec3d start_state = Vec3d(
                current_init_pose_ptr_->pose.pose.position.x,
                current_init_pose_ptr_->pose.pose.position.y,
                start_yaw
        );
        Vec3d goal_state = Vec3d(
                current_goal_pose_ptr_->pose.position.x,
                current_goal_pose_ptr_->pose.position.y,
                goal_yaw
        );

        if (kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state)) {
            auto path = kinodynamic_astar_searcher_ptr_->GetPath();
            //这里要对第一个点的前向和后向进行修正
            {
                double path_yaw = atan2((path[1] -path[0])(1), (path[1] -path[0])(0));
                if(abs(start_yaw - path_yaw) > 0.5 * M_PI)  path[0](3) = 0;
            }
            smoother.tracePath(path);


            // for(int i = 0; i < path.size(); ++i)
            // {
            //     std::cout << path[i].transpose() << std::endl;
            // }
            PublishPath(path);
            
            PublishVehiclePath(path, vehicle_length_, vehicle_width_, 5u);
            PublishSearchedTree(kinodynamic_astar_searcher_ptr_->GetSearchedTree());
            PublishCurrentStartAndGoal();
            Timer smooth_used_time;
            smoother.smoothPath(voronoiDiagram);
            std::cout << "Soomthen the path time(ms): " << smooth_used_time.End() << "\n" << std::endl;
            auto path_smoothed = smoother.getPath();
            PublishPathSmoothed(path_smoothed);
            // std::cout << "path size: " << path.size() << ", smoothed path size: " << path_smoothed.size() << std::endl;
            // nav_msgs::Path path_ros;
            // geometry_msgs::PoseStamped pose_stamped;
            // for (const auto &pose: path_smoothed) {
            //     pose_stamped.header.frame_id = "world";
            //     pose_stamped.pose.position.x = pose.x();
            //     pose_stamped.pose.position.y = pose.y();
            //     pose_stamped.pose.position.z = 0.0;

            //     pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.z());

            //     path_ros.poses.emplace_back(pose_stamped);
            // }

            // path_ros.header.frame_id = "world";
            // path_ros.header.stamp = ros::Time::now();
            // static tf::TransformBroadcaster transform_broadcaster;
            // for (const auto &pose: path_ros.poses) {
            //     tf::Transform transform;
            //     transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, 0.0));

            //     tf::Quaternion q;
            //     q.setX(pose.pose.orientation.x);
            //     q.setY(pose.pose.orientation.y);
            //     q.setZ(pose.pose.orientation.z);
            //     q.setW(pose.pose.orientation.w);
            //     transform.setRotation(q);

            //     transform_broadcaster.sendTransform(tf::StampedTransform(transform,
            //                                                              ros::Time::now(), "world",
            //                                                              "ground_link")
            //     );
            //     ros::Duration(0.01).sleep();
            // }
        }


        // debug
//        std::cout << "visited nodes: " << kinodynamic_astar_searcher_ptr_->GetVisitedNodesNumber() << std::endl;
        kinodynamic_astar_searcher_ptr_->Reset();
    }
}

//从每个subscriber里面读取一次最新？的数据
void HybridAStarFlow::ReadData() {
    costmap_sub_ptr_->ParseData(costmap_deque_);
    init_pose_sub_ptr_->ParseData(init_pose_deque_);
    goal_pose_sub_ptr_->ParseData(goal_pose_deque_);
}

void HybridAStarFlow::InitPoseData() {
    current_init_pose_ptr_ = init_pose_deque_.front();
    init_pose_deque_.pop_front();

    current_goal_pose_ptr_ = goal_pose_deque_.front();
    goal_pose_deque_.pop_front();
}

bool HybridAStarFlow::HasGoalPose() {
    return !goal_pose_deque_.empty();
}

bool HybridAStarFlow::HasStartPose() {
    return !init_pose_deque_.empty();
}

void HybridAStarFlow::PublishPath(const VectorVec4d &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = timestamp_;

    path_pub_.publish(nav_path);

    visualization_msgs::MarkerArray path_forward;
    visualization_msgs::MarkerArray path_backward;
    visualization_msgs::Marker path_point_delete;
    path_point_delete.action = visualization_msgs::Marker::DELETEALL;
    path_forward.markers.emplace_back(path_point_delete);
    path_backward.markers.emplace_back(path_point_delete);

    for(unsigned int i = 0; i < path.size(); i += 1)
    {
        visualization_msgs::Marker path_point;

        path_point.header.frame_id = "world";
        path_point.header.stamp = ros::Time::now();
        path_point.type = visualization_msgs::Marker::SPHERE;
        path_point.id = i;
        path_point.scale.x = 0.1;
        path_point.scale.y = 0.1;
        path_point.scale.z = 0.1;
        path_point.color.a = 1.0;
        path_point.pose.position.x = path[i].x();
        path_point.pose.position.y = path[i].y();
        path_point.pose.position.z = 0;
        path_point.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());

        if(path[i](3) > 0.1) //前向
        {
            path_point.color.r = 0.0;
            path_point.color.b = 0.0;
            path_point.color.g = 1.0;
            path_point.action = visualization_msgs::Marker::ADD;
            path_forward.markers.emplace_back(path_point);
        }
        else
        {
            path_point.color.r = 1.0;
            path_point.color.b = 0.0;
            path_point.color.g = 0.0;
            path_point.action = visualization_msgs::Marker::ADD;
            path_backward.markers.emplace_back(path_point);
        }   
    }
    path_forward_pub_.publish(path_forward);
    path_backward_pub_.publish(path_backward);
}

void HybridAStarFlow::PublishPathSmoothed(const VectorVec4d &spath) {
    nav_msgs::Path nav_path, nav_pathWithDirection;
    geometry_msgs::PoseStamped pose_stamped, pose_stampedWithDirection;
    for (const auto &pose: spath) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());
        nav_path.poses.emplace_back(pose_stamped);

        pose_stampedWithDirection = pose_stamped;
        pose_stampedWithDirection.pose.position.z = pose(3);
        nav_pathWithDirection.poses.emplace_back(pose_stampedWithDirection);
    }
    // std::cout << "------发布了smoothed？-------" << std::endl;
    nav_path.header.frame_id = "world";
    nav_path.header.stamp = timestamp_;
    spath_pub_.publish(nav_path);

    nav_pathWithDirection.header.frame_id = "world";
    nav_pathWithDirection.header.stamp = timestamp_;
    spathWithDirection_pub_.publish(nav_pathWithDirection);

}

void HybridAStarFlow::PublishVehiclePath(const VectorVec4d &path, double width,
                                         double length, unsigned int vehicle_interval = 5u) {
    visualization_msgs::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
        visualization_msgs::Marker vehicle;

        if (i == 0) {
            vehicle.action = 3;
        }

        vehicle.header.frame_id = "world";
        vehicle.header.stamp = ros::Time::now();
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.id = static_cast<int>(i / vehicle_interval);
        vehicle.scale.x = width;
        vehicle.scale.y = length;
        vehicle.scale.z = 0.01;
        vehicle.color.a = 0.1;

        vehicle.color.r = 1.0;
        vehicle.color.b = 0.0;
        vehicle.color.g = 0.0;

        vehicle.pose.position.x = path[i].x();
        vehicle.pose.position.y = path[i].y();
        vehicle.pose.position.z = 0.0;

        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
        vehicle_array.markers.emplace_back(vehicle);
    }

    vehicle_path_pub_.publish(vehicle_array);
}

void HybridAStarFlow::PublishSearchedTree(const VectorVec4d &searched_tree) {
    visualization_msgs::Marker tree_list;
    tree_list.header.frame_id = "world";
    tree_list.header.stamp = ros::Time::now();
    tree_list.type = visualization_msgs::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::Marker::ADD;
    tree_list.ns = "searched_tree";
    tree_list.scale.x = 0.02;

    tree_list.color.a = 1.0;
    tree_list.color.r = 0;
    tree_list.color.g = 0;
    tree_list.color.b = 0;

    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;

    geometry_msgs::Point point;
    for (const auto &i: searched_tree) {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.emplace_back(point);

        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.emplace_back(point);
    }

    searched_tree_pub_.publish(tree_list);
}

void HybridAStarFlow::PublishCurrentStartAndGoal()
{
    geometry_msgs::PoseStamped pose;
    pose = *current_goal_pose_ptr_;
    goal_pose_pub_.publish(pose);

    geometry_msgs::PoseStamped pose2;
    pose2.header = current_init_pose_ptr_->header;
    pose2.pose = current_init_pose_ptr_->pose.pose;
    start_pose_pub_.publish(pose2);

    std::cout << "The start is: " << pose2.pose.position.x << ", " << pose2.pose.position.y << ", " 
              << tf::getYaw(pose2.pose.orientation) << std::endl;;
    std::cout << "The goal is: " << pose.pose.position.x << ", " << pose.pose.position.y << ", " 
              << tf::getYaw(pose.pose.orientation) << std::endl;;
}
