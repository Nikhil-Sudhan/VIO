// Read-only diagnostics on copied tracks; never modifies the estimator state.
#pragma once
#include "core/VioManager.h"
#include "aprilgrid_tracker.h"
#include "state/State.h"
#include "track/TrackBase.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "feat/FeatureInitializer.h"
#include <fstream>
#include <algorithm>

class DiagnosticVioManager : public PiVioManager {
public:
  using PiVioManager::PiVioManager;
  void audit(std::ostream &out, double t) {
    using Init=ov_core::FeatureInitializer;
    std::unordered_map<size_t,std::unordered_map<double,Init::ClonePose>> cameras;
    std::vector<double> times;
    for(const auto &pose:state->_clones_IMU) times.push_back(pose.first);
    for(const auto &calib:state->_calib_IMUtoCAM)
      for(const auto &pose:state->_clones_IMU) {
        Eigen::Matrix3d R=calib.second->Rot()*pose.second->Rot();
        Eigen::Vector3d p=pose.second->pos()-R.transpose()*calib.second->pos();
        cameras[calib.first].emplace(pose.first,Init::ClonePose(R,p));
      }
    auto settings=params.featinit_options;
    Init init(settings);
    auto database=trackFEATS->get_feature_database()->get_internal_data();
    size_t eligible=0,triangulated=0,refined=0;
    size_t ill_conditioned=0,behind_camera=0,too_near=0,too_far=0;
    std::vector<double> rejected_conditions,rejected_depths;
    double depth_sum=0;
    std::vector<double> recent_disparities;
    for(const auto &entry:database) {
      for(const auto &series:entry.second->timestamps) {
        const auto &stamps=series.second;
        if(stamps.empty() || std::abs(stamps.back()-t)>1e-7)continue;
        auto old=std::lower_bound(stamps.begin(),stamps.end(),t-.5);
        if(old==stamps.end() || t-*old<.3)continue;
        const auto &uvs=entry.second->uvs.at(series.first);
        recent_disparities.push_back((uvs.back()-uvs.at(std::distance(stamps.begin(),old))).norm());
      }
      auto copy=std::make_shared<ov_core::Feature>(*entry.second);
      copy->clean_old_measurements(times);
      size_t n=0;for(const auto &v:copy->timestamps)n+=v.second.size();
      if(n<3)continue;
      ++eligible;
      if(!init.single_triangulation(copy,cameras)) {
        // Reconstruct the upstream linear system on this COPY to expose why
        // triangulation failed. Counts may overlap; thresholds are unchanged.
        auto &anchor=cameras.at(copy->anchor_cam_id).at(copy->anchor_clone_timestamp);
        Eigen::Matrix3d A=Eigen::Matrix3d::Zero();
        Eigen::Vector3d b=Eigen::Vector3d::Zero();
        for(const auto &series:copy->timestamps) for(size_t i=0;i<series.second.size();++i) {
          auto &cam=cameras.at(series.first).at(series.second[i]);
          const auto &uv=copy->uvs_norm.at(series.first)[i];
          Eigen::Vector3d ray=anchor.Rot()*cam.Rot().transpose()*Eigen::Vector3d(uv(0),uv(1),1);
          ray.normalize();
          Eigen::Matrix3d Ai=Eigen::Matrix3d::Identity()-ray*ray.transpose();
          A+=Ai;b+=Ai*anchor.Rot()*(cam.pos()-anchor.pos());
        }
        Eigen::Vector3d p=A.colPivHouseholderQr().solve(b);
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(A);
        const auto singular=svd.singularValues();
        const double condition=singular(0)/singular(2);
        ill_conditioned+=!std::isfinite(condition) || std::abs(condition)>settings.max_cond_number;
        behind_camera+=p(2)<=0;too_near+=p(2)<settings.min_dist;too_far+=p(2)>settings.max_dist;
        if(std::isfinite(condition))rejected_conditions.push_back(condition);
        if(std::isfinite(p(2)))rejected_depths.push_back(p(2));
        continue;
      }
      ++triangulated;depth_sum+=copy->p_FinA(2);
      if(init.single_gaussnewton(copy,cameras))++refined;
    }
    std::sort(recent_disparities.begin(),recent_disparities.end());
    std::sort(rejected_conditions.begin(),rejected_conditions.end());
    std::sort(rejected_depths.begin(),rejected_depths.end());
    out<<t<<','<<database.size()<<','<<times.size()<<','<<eligible<<','<<triangulated<<','<<refined<<','
       <<(triangulated?depth_sum/triangulated:0)<<','<<state->_features_SLAM.size()<<','
       <<is_initialized_vio<<','<<did_zupt_update<<','<<recent_disparities.size()<<','
       <<(recent_disparities.empty()?0:recent_disparities.at(recent_disparities.size()/2))<<','
       <<state->_imu->bias_g().transpose().format(Eigen::IOFormat(Eigen::FullPrecision,0,",",","))<<','
       <<ill_conditioned<<','<<behind_camera<<','<<too_near<<','<<too_far<<','
       <<(rejected_conditions.empty()?0:rejected_conditions[rejected_conditions.size()/2])<<','
       <<(rejected_depths.empty()?0:rejected_depths[rejected_depths.size()/2])<<'\n';
  }
};
