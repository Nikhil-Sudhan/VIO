// SPDX-License-Identifier: GPL-3.0-or-later
// Ordered, ROS-free replay. Input times are integer nanoseconds in one clock.
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <opencv2/imgcodecs.hpp>
#include "core/VioManager.h"
#include "state/StateHelper.h"
#include "state/State.h"
#include "utils/print.h"
#include "utils/sensor_data.h"
#include "diagnostics.h"

using Clock = std::chrono::steady_clock;
struct Row { int64_t ns; std::vector<std::string> fields; };
std::vector<Row> read_csv(const std::string &name, size_t columns) {
  std::ifstream f(name);
  if (!f) throw std::runtime_error("Cannot open " + name);
  std::vector<Row> rows;
  std::string line;
  while (std::getline(f,line)) {
    if (line.empty() || line[0]=='#') continue;
    std::stringstream s(line); std::string cell; Row r;
    while (std::getline(s,cell,',')) r.fields.push_back(cell);
    if (r.fields.size()!=columns) throw std::runtime_error("Wrong CSV column count: " + name);
    size_t used=0; r.ns=std::stoll(r.fields[0],&used);
    if (used!=r.fields[0].size() || r.ns<0) throw std::runtime_error("Invalid timestamp");
    if (!rows.empty() && r.ns<=rows.back().ns) throw std::runtime_error("Nonincreasing input time: " + name);
    rows.push_back(std::move(r));
  }
  if (rows.size()<2) throw std::runtime_error("Insufficient input: " + name);
  return rows;
}
double finite_number(const std::string &s) {
  size_t used=0; double v=std::stod(s,&used);
  if (used!=s.size() || !std::isfinite(v)) throw std::runtime_error("Nonfinite or invalid IMU input");
  return v;
}
int main(int argc, char **argv) try {
  if (argc!=6) {
    std::cerr << "Usage: vio_replay config.yaml images.csv imu.csv output_dir max_frames(0=all)\n"
              << "images: #t_ns,path0[,path1]; imu: #t_ns,wx,wy,wz,ax,ay,az (SI, gravity retained)\n";
    return 2;
  }
  auto parser=std::make_shared<ov_core::YamlParser>(argv[1]);
  std::string verbosity="WARNING";
  parser->parse_config("verbosity",verbosity,false);
  ov_core::Printer::setPrintLevel(verbosity);
  ov_msckf::VioManagerOptions options;
  options.print_and_load(parser);
  bool aprilgrid=false;
  parser->parse_config("use_aprilgrid_features",aprilgrid,false);
  int aprilgrid_corners=4;bool tag_zupt=false;
  parser->parse_config("aprilgrid_corners_per_tag",aprilgrid_corners,false);
  parser->parse_config("aprilgrid_zupt_veto",tag_zupt,false);
  options.use_multi_threading_pubs=false;
  options.use_multi_threading_subs=false;
  if (!parser->successful()) throw std::runtime_error("Incomplete estimator configuration");
  const size_t cameras=options.state_options.num_cameras;
  if (cameras<1 || cameras>2) throw std::runtime_error("Adapter supports one or two cameras");
  auto frames=read_csv(argv[2],cameras+1), imu=read_csv(argv[3],7);
  const int64_t origin=std::min(frames.front().ns,imu.front().ns);
  auto seconds=[origin](int64_t ns) { return double(ns-origin)*1e-9; };
  const std::filesystem::path output(argv[4]);
  if (!std::filesystem::create_directory(output)) throw std::runtime_error("Output must be a NEW directory");
  std::ofstream manifest(output/"run.txt");
  manifest << "origin_ns=" << origin << "\nconfig=" << argv[1]
           << "\nimages=" << argv[2] << "\nimu=" << argv[3]
           << "\nposition=IMU origin in local OpenVINS gravity frame\nvelocity=local frame m/s"
           << "\nquaternion=Hamilton xyzw, IMU-to-local active rotation"
           << "\nstate_time=camera-clock seconds relative to origin; IMU time adds estimated dt"
           << "\ncovariance=OpenVINS 15D error state [orientation,pos,vel,gyro_bias,accel_bias]"
           << "\nmode=offline replay; processing time is not live sensor latency\n";
  std::ofstream poses(output/"poses.csv"), timing(output/"timing.csv"), covs(output/"covariance.csv");
  poses << "t_rel_s,px,py,pz,qx,qy,qz,qw,vx,vy,vz,dt_cam_to_imu_s,msckf_features\n";
  timing << "frame,t_rel_s,processing_ms,initialized,state_current\n";
  covs << "#t_rel_s,15x15 row-major OpenVINS error covariance\n";
  poses << std::setprecision(17); timing << std::setprecision(17); covs << std::setprecision(17);
  manifest << "aprilgrid_features=" << aprilgrid << '\n';
  manifest<<"aprilgrid_corners_per_tag="<<aprilgrid_corners<<"\naprilgrid_zupt_veto="<<tag_zupt<<'\n';
  DiagnosticVioManager estimator(options,aprilgrid,aprilgrid_corners,tag_zupt);
  const bool diagnostics=std::getenv("VIO_DIAGNOSTICS")!=nullptr;
  const char *cadence=std::getenv("VIO_DIAGNOSTICS_EVERY");
  const size_t audit_every=cadence?std::stoull(cadence):20;
  if(audit_every==0)throw std::runtime_error("Diagnostic cadence must be positive");
  std::ofstream audit;
  if(diagnostics) {
    audit.open(output/"track-audit.csv");
    audit<<"t_rel_s,database_features,clones,eligible_copied_tracks,triangulated,refined,mean_triangulated_depth_m,slam_landmarks,static_initialized,did_zupt,recent_tracks,median_halfsecond_disparity_px,bgx,bgy,bgz,rejected_ill_conditioned,rejected_behind_camera,rejected_too_near,rejected_too_far,rejected_median_condition,rejected_median_depth_m\n";
  }
  size_t index=0, states=0, processed=0;
  const size_t limit=std::stoull(argv[5]);
  const auto start=Clock::now();
  for (size_t k=0;k<frames.size() && (!limit || k<limit);++k) {
    auto begun=Clock::now(); const double t=seconds(frames[k].ns);
    // Feed through at least one sample beyond the current calibrated image time.
    const double dt=estimator.get_state()->_calib_dt_CAMtoIMU->value()(0);
    const double required=t+dt;
    if (seconds(imu.front().ns)>required) continue;
    if (seconds(imu.back().ns)<=required+0.02) break;
    while (index<imu.size() && seconds(imu[index].ns)<=required+0.02) {
      ov_core::ImuData m; m.timestamp=seconds(imu[index].ns);
      for(int j=0;j<3;++j) { m.wm(j)=finite_number(imu[index].fields[j+1]); m.am(j)=finite_number(imu[index].fields[j+4]); }
      estimator.feed_measurement_imu(m); ++index;
    }
    ov_core::CameraData camera; camera.timestamp=t;
    for(size_t c=0;c<cameras;++c) {
      cv::Mat input=cv::imread(frames[k].fields[c+1],cv::IMREAD_UNCHANGED), gray;
      if (input.type()==CV_16UC1) input.convertTo(gray,CV_8UC1,1.0/256.0);
      else if (input.type()==CV_8UC1) gray=input;
      else throw std::runtime_error("Image must be 8/16 bit grayscale: " + frames[k].fields[c+1]);
      auto model=options.camera_intrinsics.at(c);
      if (gray.cols!=model->w() || gray.rows!=model->h()) throw std::runtime_error("Image/calibration geometry mismatch");
      camera.sensor_ids.push_back(c); camera.images.push_back(gray);
      camera.masks.push_back(options.use_mask ? options.masks.at(c) : cv::Mat::zeros(gray.size(),CV_8UC1));
    }
    estimator.feed_measurement_camera(camera); ++processed;
    if(diagnostics && k%audit_every==0) estimator.audit(audit,t);
    auto state=estimator.get_state();
    const bool current=estimator.initialized() && std::abs(state->_timestamp-t)<1e-7;
    timing << k << ',' << t << ',' << std::chrono::duration<double,std::milli>(Clock::now()-begun).count()
           << ',' << estimator.initialized() << ',' << current << '\n';
    if (current) {
      Eigen::Quaterniond q(state->_imu->Rot().transpose()); q.normalize();
      auto p=state->_imu->pos(), v=state->_imu->vel();
      if (!p.allFinite() || !v.allFinite() || !q.coeffs().allFinite()) throw std::runtime_error("Nonfinite estimator state");
      poses << state->_timestamp << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
            << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
            << v.x() << ',' << v.y() << ',' << v.z() << ',' << state->_calib_dt_CAMtoIMU->value()(0)
            << ',' << estimator.get_good_features_MSCKF().size() << '\n';
      auto cov=ov_msckf::StateHelper::get_marginal_covariance(state,{state->_imu});
      if (cov.rows()!=15 || !cov.allFinite()) throw std::runtime_error("Invalid covariance");
      covs << state->_timestamp;
      for(int r=0;r<15;++r) for(int c=0;c<15;++c) covs << ',' << cov(r,c);
      covs << '\n'; ++states;
    }
    if (k%100==0) std::cerr << "frames=" << processed << " states=" << states << " t=" << t << '\n';
  }
  manifest << "processed_frames=" << processed << "\noutput_states=" << states
           << "\nwall_seconds=" << std::chrono::duration<double>(Clock::now()-start).count() << '\n';
  if (!states) throw std::runtime_error("No initialized states; replay did not validate estimator");
  return 0;
} catch (const std::exception &e) { std::cerr << "ERROR: " << e.what() << '\n'; return 1; }
