#include "../adapter/tracking_health.h"
#include <cassert>
#include <limits>
int main() {
  TrackingHealth h;
  h.update(0,false,false,0,true,0,0);assert(!h.valid && h.state=="initializing");
  for(int i=1;i<=100;++i) h.update(i*.05,true,true,0,true,0,.1);
  assert(!h.valid && h.state=="ready"); // ZUPT alone is not a 3D map.
  h.update(5.05,true,false,4,true,.1,.1);assert(!h.valid);
  h.update(5.1,true,false,4,true,.1,.1);assert(h.valid);
  for(int i=103;i<200;++i) h.update(i*.05,true,true,0,true,0,.1);
  h.update(10,true,false,0,true,.1,.1);assert(h.valid); // No false loss after a long rest.
  h.update(11.1,true,false,0,true,.1,.1);assert(h.lost && !h.valid);
  TrackingHealth depth;
  for(int i=0;i<60;++i) depth.update(i*.05,true,false,0,true,.1,.1);
  assert(depth.lost && depth.reason=="depth_initialization_timeout");
  TrackingHealth fast;fast.update(0,true,false,100,true,2.1,.1);assert(fast.lost);
  TrackingHealth uncertain;uncertain.update(0,true,false,100,true,0,1.1);assert(uncertain.lost);
  TrackingHealth bad;bad.update(0,true,true,100,true,std::numeric_limits<double>::quiet_NaN(),.1);assert(bad.lost);
}
