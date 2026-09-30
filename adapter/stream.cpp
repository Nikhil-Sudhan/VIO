// SPDX-License-Identifier: GPL-3.0-or-later
// Bounded, ordered monocular input transport. Protocol documented in docs/STREAM.md.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <time.h>
#include <unistd.h>
#include <vector>
#include <opencv2/imgcodecs.hpp>
#include "core/VioManager.h"
#include "aprilgrid_tracker.h"
#include "state/StateHelper.h"
#include "state/State.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using Clock=std::chrono::steady_clock;
bool live_mode=false;
uint64_t boot_ns() {
  timespec t{};
  if (clock_gettime(CLOCK_BOOTTIME,&t)) throw std::runtime_error("Cannot read BOOTTIME");
  return uint64_t(t.tv_sec)*1000000000ULL+t.tv_nsec;
}
void read_exact(void *dest,size_t n) {
  auto *p=static_cast<unsigned char *>(dest);
  while(n) {
    if (live_mode) {
      pollfd item{STDIN_FILENO,POLLIN,0};
      const int ready=poll(&item,1,1000);
      if (ready<=0) throw std::runtime_error("Live input stalled or poll interrupted; output is stale");
    }
    const ssize_t got=read(STDIN_FILENO,p,n);
    if (got<=0) throw std::runtime_error("Truncated/disconnected input; explicit END required");
    p+=got;n-=got;
  }
}
uint64_t u64(const unsigned char *p,size_t n=8) {
  uint64_t v=0;for(size_t i=0;i<n;++i) v|=uint64_t(p[i])<<(8*i);return v;
}
double number(const unsigned char *p) {
  const uint64_t bits=u64(p);double value;static_assert(sizeof(value)==sizeof(bits));
  std::memcpy(&value,&bits,sizeof(value));
  if(!std::isfinite(value)) throw std::runtime_error("Nonfinite IMU payload");
  return value;
}
struct Frame { uint64_t ns,receipt,sequence;cv::Mat gray; };

class DisplayVioManager : public PiVioManager {
public:
  using PiVioManager::PiVioManager;
  bool zero_velocity_update() const { return did_zupt_update; }
  cv::Mat current_tracks_image() {
    // Drawing stationary histories copies ever-longer feature vectors while
    // ZUPT intentionally bypasses normal track cleanup. Display current real
    // observations instead; this does not alter estimator measurements.
    cv::Mat image;
    const std::string overlay=did_zupt_update ? "stationary" : (!is_initialized_vio ? "init" : "");
    trackFEATS->display_active(image,255,255,0,255,255,255,overlay);
    if(trackARUCO) trackARUCO->display_active(image,0,255,255,255,255,255,overlay);
    return image;
  }
};

int main(int argc,char **argv) try {
  if(argc!=4 || (std::string(argv[3])!="reference" && std::string(argv[3])!="live")) {
    std::cerr<<"Usage: vio_stream config.yaml NEW_output_directory reference|live < binary_input\n";return 2;
  }
  live_mode=std::string(argv[3])=="live";
  auto parser=std::make_shared<ov_core::YamlParser>(argv[1]);
  std::string verbosity="WARNING";
  parser->parse_config("verbosity",verbosity,false);
  ov_core::Printer::setPrintLevel(verbosity);
  ov_msckf::VioManagerOptions options;options.print_and_load(parser);
  bool aprilgrid=false;
  parser->parse_config("use_aprilgrid_features",aprilgrid,false);
  int aprilgrid_corners=4;bool tag_zupt=false;
  parser->parse_config("aprilgrid_corners_per_tag",aprilgrid_corners,false);
  parser->parse_config("aprilgrid_zupt_veto",tag_zupt,false);
  options.use_multi_threading_pubs=false;options.use_multi_threading_subs=false;
  if(!parser->successful() || options.state_options.num_cameras!=1)
    throw std::runtime_error("Complete monocular calibration is required");
  const std::filesystem::path output(argv[2]);
  if(!std::filesystem::create_directory(output)) throw std::runtime_error("Output must be a NEW directory");
  std::ofstream manifest(output/"run.txt"),poses(output/"poses.csv"),timing(output/"timing.csv"),covs(output/"covariance.csv"),cycles(output/"cycles.csv"),intake(output/"intake.csv");
  for(auto *file : {&manifest,&poses,&timing,&covs,&cycles,&intake}) file->exceptions(std::ios::badbit|std::ios::failbit);
  manifest<<"config="<<argv[1]<<"\nmode="<<argv[3]<<"\nprotocol=PIVIO001"
          <<"\nposition=IMU origin in local OpenVINS gravity frame\nvelocity=local frame m/s"
          <<"\nquaternion=Hamilton xyzw, IMU-to-local active rotation"
          <<"\nstate_time=camera-clock seconds relative to origin; IMU time adds estimated dt"
          <<"\ncovariance=OpenVINS 15D error state [orientation,pos,vel,gyro_bias,accel_bias]"
          <<"\nreset_epoch=0; new process creates a new output directory\n";
  poses<<"t_rel_s,px,py,pz,qx,qy,qz,qw,vx,vy,vz,dt_cam_to_imu_s,msckf_features\n";
  timing<<"frame,t_rel_s,processing_ms,initialized,state_current,sensor_to_output_ms,receipt_to_output_ms,pending_frames\n";
  covs<<"#t_rel_s,15x15 row-major OpenVINS error covariance\n";
  cycles<<"frame,total_processing_ms\n";
  intake<<"kind,sequence,t_rel_s,receipt_age_ms,read_ms,feed_ms,camera_processing_ms\n";
  poses<<std::setprecision(17);timing<<std::setprecision(17);covs<<std::setprecision(17);
  manifest<<"aprilgrid_features="<<aprilgrid<<'\n';
  manifest<<"aprilgrid_corners_per_tag="<<aprilgrid_corners<<"\naprilgrid_zupt_veto="<<tag_zupt<<'\n';
  DisplayVioManager estimator(options,aprilgrid,aprilgrid_corners,tag_zupt);
  double last_visual_update=-1;
  // Optional native RViz bridge. These are actual OpenVINS tracks and landmarks,
  // never reconstructed from a plotted path or generated for presentation.
  const bool display=std::getenv("VIO_NATIVE_DISPLAY")!=nullptr;
  std::array<unsigned char,8> magic{};read_exact(magic.data(),magic.size());
  if(std::memcmp(magic.data(),"PIVIO001",8)) throw std::runtime_error("Wrong protocol magic/version");
  uint64_t origin=0,last_imu=0,last_frame=0,last_frame_seq=0,last_imu_seq=0,states=0,processed=0;
  bool have_origin=false,have_imu=false,have_frame=false;
  size_t max_pending=0;std::deque<Frame> frames;
  const auto begun=Clock::now();
  auto seconds=[&](uint64_t ns) {return double(int64_t(ns)-int64_t(origin))*1e-9;};
  auto process=[&]() {
    while(!frames.empty() && have_imu) {
      const Frame &f=frames.front();const double t=seconds(f.ns);
      const double dt=estimator.get_state()->_calib_dt_CAMtoIMU->value()(0);
      if(!std::isfinite(dt) || std::abs(dt)>.5) throw std::runtime_error("Time offset outside supported buffer range");
      if(seconds(last_imu)<t+dt+.01) break;
      if(live_mode && (f.ns>boot_ns()+1000000ULL || boot_ns()-f.ns>1000000000ULL))
        throw std::runtime_error("Camera measurement stale or outside host clock");
      const auto start=Clock::now();
      ov_core::CameraData c;c.timestamp=t;c.sensor_ids={0};c.images={f.gray};
      c.masks={options.use_mask ? options.masks.at(0) : cv::Mat::zeros(f.gray.size(),CV_8UC1)};
      estimator.feed_measurement_camera(c);++processed;
      auto state=estimator.get_state();
      // initialized() also requires the first normal visual update. Before
      // movement, a successful stationary initialization can remain in ZUPT.
      const bool current=estimator.initialized_time()>=0 && std::abs(state->_timestamp-t)<1e-7;
      const bool stationary_update=current && estimator.zero_velocity_update();
      if(current && !stationary_update && !estimator.get_good_features_MSCKF().empty())
        last_visual_update=t;
      const uint64_t emitted=boot_ns();
      timing<<f.sequence<<','<<t<<','<<std::chrono::duration<double,std::milli>(Clock::now()-start).count()
        <<','<<estimator.initialized()<<','<<current<<',';
      if(live_mode) timing<<double(int64_t(emitted)-int64_t(f.ns))/1e6<<','<<double(int64_t(emitted)-int64_t(f.receipt))/1e6;
      else timing<<"nan,nan";
      timing<<','<<frames.size()<<'\n';
      if(current) {
        Eigen::Quaterniond q(state->_imu->Rot().transpose());q.normalize();
        auto p=state->_imu->pos(),v=state->_imu->vel();
        auto cov=ov_msckf::StateHelper::get_marginal_covariance(state,{state->_imu});
        if(!p.allFinite() || !v.allFinite() || !q.coeffs().allFinite() || cov.rows()!=15 || !cov.allFinite())
          throw std::runtime_error("Nonfinite estimator state/covariance");
        poses<<t<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<','
          <<v.x()<<','<<v.y()<<','<<v.z()<<','<<state->_calib_dt_CAMtoIMU->value()(0)<<','
          <<estimator.get_good_features_MSCKF().size()<<'\n';
        covs<<t;for(int r=0;r<15;++r) for(int col=0;col<15;++col) covs<<','<<cov(r,col);covs<<'\n';++states;
      }
      if(display && processed%4==0) {
        cv::Scalar luma_mean,luma_std;
        cv::meanStdDev(f.gray,luma_mean,luma_std);
        cv::Mat tracks=estimator.current_tracks_image();
        if(!tracks.empty()) {
          if(!cv::imwrite((output/"tracks.tmp.jpg").string(),tracks,{cv::IMWRITE_JPEG_QUALITY,85}))
            throw std::runtime_error("Cannot write tracker image");
          std::filesystem::rename(output/"tracks.tmp.jpg",output/"tracks.jpg");
        }
        std::ofstream snap(output/"display.tmp.json");
        snap.exceptions(std::ios::badbit|std::ios::failbit);
        snap<<std::setprecision(17)<<"{\"sequence\":"<<f.sequence<<",\"t\":"<<t
            <<",\"emitted_boot_ns\":"<<boot_ns()<<",\"initialized\":"<<(current && estimator.initialized()?"true":"false")
            <<",\"stationary_ready\":"<<(current && !estimator.initialized()?"true":"false")
            <<",\"zero_velocity_update\":"<<(stationary_update?"true":"false")
            <<",\"image_mean_luma\":"<<luma_mean[0]<<",\"image_std_luma\":"<<luma_std[0]
            <<",\"last_msckf_update_age_s\":";
        if(last_visual_update>=0) snap<<t-last_visual_update; else snap<<"null";
        if(current) {
          Eigen::Quaterniond q(state->_imu->Rot().transpose());q.normalize();
          auto p=state->_imu->pos(),v=state->_imu->vel();
          snap<<",\"position\":["<<p.x()<<','<<p.y()<<','<<p.z()<<"],\"quaternion\":["
              <<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<"],\"velocity\":["<<v.x()<<','<<v.y()<<','<<v.z()<<']';
          auto points=[&](const char *name,const std::vector<Eigen::Vector3d> &values) {
            snap<<",\""<<name<<"\":[";bool first=true;
            for(const auto &pt:values) if(pt.allFinite()) {
              if(!first) snap<<',';
              first=false;snap<<'['<<pt.x()<<','<<pt.y()<<','<<pt.z()<<']';
            }
            snap<<']';
          };
          points("msckf",estimator.get_good_features_MSCKF());
          points("slam",estimator.get_features_SLAM());
          points("aruco",estimator.get_features_ARUCO());
        }
        snap<<"}\n";snap.close();
        std::filesystem::rename(output/"display.tmp.json",output/"display.json");
      }
      poses.flush();covs.flush();timing.flush();
      cycles<<f.sequence<<','<<std::chrono::duration<double,std::milli>(Clock::now()-start).count()<<'\n';
      cycles.flush();
      if(processed%100==0) std::cerr<<"frames="<<processed<<" states="<<states<<" t="<<t<<'\n';
      frames.pop_front();
    }
  };
  while(true) {
    const auto read_started=Clock::now();
    std::array<unsigned char,29> h{};read_exact(h.data(),h.size());
    const unsigned char kind=h[0];const uint64_t ns=u64(h.data()+1),receipt=u64(h.data()+9),seq=u64(h.data()+17);
    const uint64_t length=u64(h.data()+25,4);
    if(kind==3) {
      if(ns || receipt || seq || length) throw std::runtime_error("Invalid END record");
      process();if(!frames.empty()) throw std::runtime_error("Missing IMU tail for pending cameras");break;
    }
    if(kind!=1 && kind!=2) throw std::runtime_error("Unknown input message type");
    if(ns>uint64_t(INT64_MAX) || !ns || length>4000000) throw std::runtime_error("Invalid timestamp/oversized input");
    if(!have_origin) {origin=ns;have_origin=true;manifest<<"origin_ns="<<origin<<'\n';}
    if(live_mode) {
      const uint64_t now=boot_ns();
      if(receipt<ns || receipt>now || now-receipt>1000000000ULL)
        throw std::runtime_error("Invalid/stale receipt timestamp: kind="+std::to_string(kind)+
          " sequence="+std::to_string(seq)+" measurement_ns="+std::to_string(ns)+
          " receipt_ns="+std::to_string(receipt)+" now_ns="+std::to_string(now)+
          " receipt_before_measurement="+std::to_string(receipt<ns)+
          " receipt_in_future="+std::to_string(receipt>now)+
          " older_than_1s="+std::to_string(receipt<=now && now-receipt>1000000000ULL));
    }
    std::vector<unsigned char> payload(length);read_exact(payload.data(),length);
    const auto feed_started=Clock::now();
    if(kind==1) {
      if(length!=48 || (have_imu && (ns<=last_imu || seq!=last_imu_seq+1)))
        throw std::runtime_error("Invalid IMU payload/time/sequence");
      ov_core::ImuData m;m.timestamp=seconds(ns);
      for(int j=0;j<3;++j) {m.wm(j)=number(payload.data()+8*j);m.am(j)=number(payload.data()+24+8*j);}
      estimator.feed_measurement_imu(m);last_imu=ns;last_imu_seq=seq;have_imu=true;
    } else {
      if(length<8 || (have_frame && (ns<=last_frame || seq!=last_frame_seq+1)))
        throw std::runtime_error("Invalid camera time/sequence");
      const uint64_t w=u64(payload.data(),4),height=u64(payload.data()+4,4);
      auto model=options.camera_intrinsics.at(0);
      if(w!=uint64_t(model->w()) || height!=uint64_t(model->h()) || length!=8+w*height)
        throw std::runtime_error("Camera geometry/calibration mismatch");
      if(!have_imu || ns<origin) throw std::runtime_error("IMU pre-roll required before first camera");
      frames.push_back({ns,receipt,seq,cv::Mat(int(height),int(w),CV_8UC1,payload.data()+8).clone()});
      last_frame=ns;last_frame_seq=seq;have_frame=true;max_pending=std::max(max_pending,frames.size());
      if(frames.size()>8) throw std::runtime_error("Camera/IMU join queue exceeded eight frames");
    }
    const auto process_started=Clock::now();
    process();
    const double feed_ms=std::chrono::duration<double,std::milli>(process_started-feed_started).count();
    // Sample IMU intake to distinguish input/pipe delays from camera work.
    // This diagnostic never changes packet order, timestamps or stale guards.
    if(kind==2 || seq%20==0 || feed_ms>10) {
      intake<<int(kind)<<','<<seq<<','<<seconds(ns)<<',';
      if(live_mode) intake<<double(int64_t(boot_ns())-int64_t(receipt))/1e6;else intake<<"nan";
      intake<<','<<std::chrono::duration<double,std::milli>(feed_started-read_started).count()
            <<','<<feed_ms<<','<<std::chrono::duration<double,std::milli>(Clock::now()-process_started).count()<<'\n';
      if(kind==2 && seq%20==0) intake.flush();
    }
  }
  manifest<<"processed_frames="<<processed<<"\noutput_states="<<states<<"\nmax_pending_frames="<<max_pending
    <<"\nwall_seconds="<<std::chrono::duration<double>(Clock::now()-begun).count()<<"\ntransport_completed=true\n";
  manifest.flush();
  if(!states) throw std::runtime_error("No initialized states; transport completion is not a VIO pass");
  return 0;
} catch(const std::exception &e) {std::cerr<<"ERROR: "<<e.what()<<'\n';return 1;}
