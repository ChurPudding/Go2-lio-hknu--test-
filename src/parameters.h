// #ifndef PARAM_H
// #define PARAM_H
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <cstring>
#include <string>
#include "preprocess.h"

extern bool odom_only;
extern std::string odom_header_frame_id;
extern std::string odom_child_frame_id;

extern bool is_first_frame;
extern double lidar_end_time, first_lidar_time, time_con;
extern double last_timestamp_lidar, last_timestamp_imu;
extern int pcd_index;

extern std::string lid_topic, imu_topic;
extern bool zupt_en, zupt_vel_en, zupt_omg_en;
extern double zupt_cov_vel, zupt_cov_omg;
extern double zupt_timeout;   // /zupt_active 를 이보다 오래 못 받으면 false 로 봄 [s]
extern std::string zupt_flag_topic;
extern bool leg_en, leg_use_z;
extern double leg_cov, leg_scale;
extern std::string leg_odom_topic;
extern std::vector<double> leg_R_ib;   // base_link -> IMU(L1) 회전, row-major 9개
extern std::vector<double> leg_lever;   // base 원점 -> L1 원점 (base 프레임, m)
extern double leg_rate_hz;  extern bool leg_att_en;
extern bool leg_omg_en;
extern bool leg_vel_only;
extern double leg_delay;   // 다리 샘플 시각 보정 [s]
extern bool prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
extern bool use_imu_as_input, space_down_sample;
extern bool extrinsic_est_en, publish_odometry_without_downsample;
extern int init_map_size, con_frame_num;
extern double match_s, satu_acc, satu_gyro, cut_frame_time_interval;
extern float plane_thr;
extern double filter_size_surf_min, filter_size_map_min, fov_deg;
extern double cube_len;
extern float DET_RANGE;
extern bool imu_en, gravity_align, non_station_start;
extern double imu_time_inte;
extern double laser_point_cov, acc_norm;
extern double acc_cov_input, gyr_cov_input, vel_cov;
extern double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
extern double imu_meas_acc_cov, imu_meas_omg_cov;
extern int lidar_type, pcd_save_interval;
extern std::vector<double> gravity_init, gravity;
extern std::vector<double> extrinT;
extern std::vector<double> extrinR;
extern bool runtime_pos_log, pcd_save_en, path_en;
extern bool scan_pub_en, scan_body_pub_en;
extern shared_ptr<Preprocess> p_pre;
extern double time_lag_imu_to_lidar;

void readParameters(shared_ptr<rclcpp::Node> &nh);
