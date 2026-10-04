// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <vector>
#include "tracking_health.h"
#include "state/StateHelper.h"
#include "state/State.h"
#include "types/Landmark.h"

// A reset starts an independent local coordinate frame. Preserve raw state
// diagnostics, but export usable poses separately with explicit frame epochs.
template<class Manager> class SupervisedVio {
  ov_msckf::VioManagerOptions options_;
  bool aprilgrid_, tag_zupt_, enabled_;
  int corners_;
  std::unique_ptr<Manager> manager_;
  std::deque<ov_core::ImuData> history_;
  std::ofstream health_log_, valid_poses_, landmarks_, epochs_;
  std::filesystem::path output_;
  std::map<size_t,Eigen::Vector3d> map_;
  void make_manager() { manager_=std::make_unique<Manager>(options_,aprilgrid_,corners_,tag_zupt_); }
  void save_map() {
    if(map_.empty()) return;
    auto name=output_/("map-epoch-"+std::to_string(epoch)+".ply");
    std::ofstream f(name);f.exceptions(std::ios::failbit|std::ios::badbit);
    f<<"ply\nformat ascii 1.0\ncomment Local sparse landmarks; no loop closure or cross-epoch registration\n"
     <<"element vertex "<<map_.size()<<"\nproperty double x\nproperty double y\nproperty double z\nend_header\n"<<std::setprecision(17);
    for(const auto &item:map_) f<<item.second.x()<<' '<<item.second.y()<<' '<<item.second.z()<<'\n';
  }
public:
  TrackingHealth health;
  size_t epoch=0;
  SupervisedVio(ov_msckf::VioManagerOptions options,bool aprilgrid,int corners,bool tag_zupt,
                bool enabled,const std::filesystem::path &output)
      : options_(options),aprilgrid_(aprilgrid),tag_zupt_(tag_zupt),enabled_(enabled),corners_(corners),output_(output) {
    make_manager();
    health_log_.open(output/"tracking.csv");valid_poses_.open(output/"valid-poses.csv");
    landmarks_.open(output/"landmarks.csv");epochs_.open(output/"epochs.csv");
    for(auto *f:{&health_log_,&valid_poses_,&landmarks_,&epochs_}) {f->exceptions(std::ios::failbit|std::ios::badbit);*f<<std::setprecision(17);}
    health_log_<<"t_rel_s,reset_epoch,state,pose_valid,accepted_visual_features,last_visual_age_s,speed_m_s,position_std_m,reason\n";
    valid_poses_<<"t_rel_s,reset_epoch,px,py,pz,qx,qy,qz,qw,vx,vy,vz\n";
    landmarks_<<"t_rel_s,reset_epoch,feature_id,x,y,z\n";
    epochs_<<"reset_epoch,event,t_rel_s,reason\n";
    epochs_<<"0,start,0,initial\n";
  }
  Manager *operator->() {return manager_.get();}
  bool enabled() const {return enabled_;}
  void finish() {save_map();}
  std::vector<Eigen::Vector3d> map_points() const {
    std::vector<Eigen::Vector3d> points;points.reserve(map_.size());
    for(const auto &item:map_) points.push_back(item.second);
    return points;
  }
  void feed_measurement_imu(const ov_core::ImuData &m) {
    history_.push_back(m);
    while(!history_.empty() && history_.front().timestamp<m.timestamp-3) history_.pop_front();
    manager_->feed_measurement_imu(m);
  }
  void feed_measurement_camera(const ov_core::CameraData &c) {
    if(enabled_ && health.lost) {
      save_map();map_.clear();++epoch;
      epochs_<<epoch<<",start,"<<c.timestamp<<",reinitialization\n";
      make_manager();health=TrackingHealth{};
      for(const auto &m:history_) manager_->feed_measurement_imu(m);
    }
    manager_->feed_measurement_camera(c);
    const auto s=manager_->get_state();
    const bool current=manager_->initialized_time()>=0 && std::abs(s->_timestamp-c.timestamp)<1e-7;
    double speed=0,stddev=0;bool finite=true;
    if(current) {
      auto cov=ov_msckf::StateHelper::get_marginal_covariance(s,{s->_imu});
      finite=s->_imu->pos().allFinite() && s->_imu->vel().allFinite() && s->_imu->Rot().allFinite() && cov.allFinite();
      speed=s->_imu->vel().norm();
      if(cov.rows()!=15 || cov.diagonal().minCoeff()<0) finite=false;
      else stddev=std::sqrt(cov.diagonal().template segment<3>(3).maxCoeff());
    }
    health.update(c.timestamp,current,current && manager_->zero_velocity_update(),manager_->accepted_visual_features(),finite,speed,stddev);
    health_log_<<c.timestamp<<','<<epoch<<','<<health.state<<','<<health.valid<<','<<manager_->accepted_visual_features()<<',';
    if(health.last_visual>=0) health_log_<<c.timestamp-health.last_visual;
    health_log_<<','<<speed<<','<<stddev<<','<<health.reason<<'\n';
    if(health.lost) {
      if(enabled_) epochs_<<epoch<<",lost,"<<c.timestamp<<','<<health.reason<<'\n';
      return;
    }
    if(!health.valid) return;
    const auto p=s->_imu->pos(),v=s->_imu->vel();
    Eigen::Quaterniond q(s->_imu->Rot().transpose());q.normalize();
    valid_poses_<<c.timestamp<<','<<epoch<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<','<<v.x()<<','<<v.y()<<','<<v.z()<<'\n';
    // Archive each real filter landmark by stable ID, retaining its latest
    // valid estimate after marginalization. Never merge independent epochs.
    if(manager_->accepted_visual_features()==0) return;
    for(const auto &f:s->_features_SLAM) {
      if((int)f.first<=4*s->_options.max_aruco_features) continue;
      auto lm=f.second;Eigen::Vector3d point=lm->get_xyz(false);
      if(ov_type::LandmarkRepresentation::is_relative_representation(lm->_feat_representation)) {
        auto ext=s->_calib_IMUtoCAM.at(lm->_anchor_cam_id);
        auto anchor=s->_clones_IMU.at(lm->_anchor_clone_timestamp);
        point=anchor->Rot().transpose()*ext->Rot().transpose()*(point-ext->pos())+anchor->pos();
      }
      if(!point.allFinite()) continue;
      map_[f.first]=point;
      landmarks_<<c.timestamp<<','<<epoch<<','<<f.first<<','<<point.x()<<','<<point.y()<<','<<point.z()<<'\n';
    }
  }
};
