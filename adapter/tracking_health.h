// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

// Application envelope for slow handheld mapping, not a filter accuracy test.
// Existing landmarks and filter initialization alone never establish tracking.
struct TrackingHealth {
  std::string state="initializing", reason="";
  bool valid=false, lost=false, locked=false;
  double last_visual=-1, last_constraint=-1, moving_time=0, previous=-1, lock_candidate=-1;
  void update(double t, bool current, bool zupt, size_t accepted, bool finite,
              double speed, double position_std) {
    if(lost) return;
    double dt=previous<0 ? 0 : t-previous;
    previous=t;valid=false;reason.clear();
    if(!current) { state="initializing";return; }
    auto fail=[&](const char *why) {state="lost";reason=why;lost=true;valid=false;};
    if(!finite || !std::isfinite(speed) || !std::isfinite(position_std)) {fail("nonfinite_state");return;}
    if(speed>2.0) {fail("handheld_speed_limit");return;}
    if(position_std>1.0) {fail("position_uncertainty_limit");return;}
    if(accepted>0) last_visual=t;
    if(accepted>=3) {
      if(lock_candidate>=0 && t-lock_candidate<=.3) locked=true;
      lock_candidate=t;
    }
    if(locked && (accepted>0 || zupt)) last_constraint=t;
    if(locked) {
      if(last_constraint<0 || t-last_constraint>1.0) {fail("visual_corrections_expired");return;}
      state=zupt?"stationary":"tracking";valid=true;return;
    }
    if(zupt) {state="ready";return;}
    moving_time+=std::max(0.0,std::min(dt,.2));
    if(moving_time>2.0) {fail("depth_initialization_timeout");return;}
    state="initializing_depth";
  }
};
