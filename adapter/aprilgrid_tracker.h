// Optional single-AprilGrid feature frontend for OpenVINS camera/IMU fusion.
// Supplies 2D corner observations only: no PnP poses or known target geometry.
#pragma once
#include <cmath>
#include <stdexcept>
#include <set>
#include <opencv2/objdetect/aruco_detector.hpp>
#include "core/VioManager.h"
#include "state/State.h"
#include "track/TrackBase.h"
#include "feat/FeatureDatabase.h"
#include "update/UpdaterZeroVelocity.h"

class AprilGridTracker : public ov_core::TrackBase {
public:
  AprilGridTracker(std::unordered_map<size_t,std::shared_ptr<ov_core::CamBase>> cameras, int reserved, int corners=4, bool stationarity=false)
      : TrackBase(cameras,36*corners,reserved,false,HistogramMethod::NONE), reserved_(reserved), corners_(corners),
        detector_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), parameters()) {
    if(reserved<=36) throw std::runtime_error("AprilGrid needs at least 37 reserved tag IDs");
    if(corners!=1 && corners!=2 && corners!=4) throw std::runtime_error("AprilGrid corners per tag must be 1, 2 or 4");
    if(stationarity) stationarity_db_=std::make_shared<ov_core::FeatureDatabase>();
  }
  std::shared_ptr<ov_core::FeatureDatabase> stationarity_database() const { return stationarity_db_; }
  void feed_new_camera(const ov_core::CameraData &message) override {
    // SLAM consumes its database's observations after each update. Keep a
    // separate, bounded history so the motion veto still has both frames.
    if(stationarity_db_) stationarity_db_->cleanup_measurements(message.timestamp-.5);
    for(size_t k=0;k<message.images.size();++k) {
      const size_t cam=message.sensor_ids.at(k);
      std::lock_guard<std::mutex> feed_lock(mtx_feeds.at(cam));
      const auto &im=message.images.at(k); const auto &mask=message.masks.at(k);
      std::vector<std::vector<cv::Point2f>> corners;
      std::vector<int> tags;
      detector_.detectMarkers(im,corners,tags);
      std::set<int> seen,duplicates;
      for(int tag:tags) if(!seen.insert(tag).second) duplicates.insert(tag);
      std::vector<size_t> ids;
      std::vector<cv::KeyPoint> points;
      for(size_t i=0;i<tags.size();++i) {
        if(tags[i]<0 || tags[i]>=36 || duplicates.count(tags[i]) || corners[i].size()!=4) continue;
        for(size_t n=0;n<corners_;++n) {
          const auto &uv=corners[i][n];
          if(!std::isfinite(uv.x) || !std::isfinite(uv.y) || uv.x<0 || uv.y<0 || uv.x>=im.cols || uv.y>=im.rows) continue;
          if(mask.at<uint8_t>(int(uv.y),int(uv.x))>127) continue;
          const auto norm=camera_calib.at(cam)->undistort_cv(uv);
          const size_t id=size_t(tags[i])+n*reserved_;
          database->update_feature(id,message.timestamp,cam,uv.x,uv.y,norm.x,norm.y);
          if(stationarity_db_) stationarity_db_->update_feature(id,message.timestamp,cam,uv.x,uv.y,norm.x,norm.y);
          ids.push_back(id);points.emplace_back(uv,1.f);
        }
      }
      std::lock_guard<std::mutex> display_lock(mtx_last_vars);
      img_last[cam]=im;img_mask_last[cam]=mask;ids_last[cam]=ids;pts_last[cam]=points;
    }
  }
private:
  static cv::aruco::DetectorParameters parameters() {
    cv::aruco::DetectorParameters p;
    p.markerBorderBits=2;
    p.cornerRefinementMethod=cv::aruco::CORNER_REFINE_SUBPIX;
    return p;
  }
  size_t reserved_;
  size_t corners_;
  std::shared_ptr<ov_core::FeatureDatabase> stationarity_db_;
  cv::aruco::ArucoDetector detector_;
};

class PiVioManager : public ov_msckf::VioManager {
public:
  PiVioManager(ov_msckf::VioManagerOptions &options, bool aprilgrid=false, int corners=4, bool tag_zupt=false)
      : ov_msckf::VioManager(options) {
    if(aprilgrid) {
      if(options.state_options.num_cameras!=1 || options.use_aruco)
        throw std::runtime_error("AprilGrid frontend requires one camera and use_aruco=false");
      trackARUCO=std::make_shared<AprilGridTracker>(state->_cam_intrinsics_cameras,state->_options.max_aruco_features,corners,tag_zupt);
    }
    if(tag_zupt) {
      if(!aprilgrid || !params.try_zupt || params.zupt_visual_veto_px<=0 || params.zupt_max_disparity!=0)
        throw std::runtime_error("Tag ZUPT veto needs AprilGrid, inertial ZUPT and a positive veto threshold without visual override");
      // Only the source of image displacement changes. This still requires
      // the unchanged inertial consistency test; target poses are never used.
      updaterZUPT=std::make_shared<ov_msckf::UpdaterZeroVelocity>(params.zupt_options,params.imu_noises,
          std::static_pointer_cast<AprilGridTracker>(trackARUCO)->stationarity_database(),propagator,params.gravity_mag,params.zupt_max_velocity,
          params.zupt_noise_multiplier,params.zupt_max_disparity,params.zupt_visual_veto_px);
    }
  }
};
