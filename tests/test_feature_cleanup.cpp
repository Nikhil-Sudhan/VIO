// Verify that linear compaction preserves the former erase-by-erase result.
#include "feat/Feature.h"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <iostream>
#include <random>

using ov_core::Feature;
template<class Drop> void original_cleanup(Feature &f, Drop drop) {
  for (auto &entry : f.timestamps) {
    auto &t=entry.second; auto &u=f.uvs.at(entry.first); auto &n=f.uvs_norm.at(entry.first);
    for(size_t i=0;i<t.size();) {
      if(drop(t[i])) {t.erase(t.begin()+i);u.erase(u.begin()+i);n.erase(n.begin()+i);}
      else ++i;
    }
  }
}
void equal(const Feature &a,const Feature &b) {
  assert(a.timestamps==b.timestamps);
  for(const auto &entry:a.timestamps) {
    const auto id=entry.first;
    assert(a.uvs.at(id).size()==b.uvs.at(id).size());
    assert(a.uvs_norm.at(id).size()==b.uvs_norm.at(id).size());
    for(size_t i=0;i<entry.second.size();++i) {
      assert((a.uvs.at(id)[i].array()==b.uvs.at(id)[i].array()).all());
      assert((a.uvs_norm.at(id)[i].array()==b.uvs_norm.at(id)[i].array()).all());
    }
  }
}
Feature sample(size_t count) {
  Feature f;
  for(size_t cam=0;cam<3;++cam) for(size_t i=0;i<count;++i) {
    f.timestamps[cam].push_back(i*.05);
    f.uvs[cam].push_back(Eigen::Vector2f(float(i+cam),float(i*3+cam)));
    f.uvs_norm[cam].push_back(Eigen::Vector2f(float(i)/100,float(i)/200));
  }
  return f;
}
int main() {
  std::mt19937 rng(17);
  for(size_t count : {0,1,2,11,100,5000}) {
    auto source=sample(count);
    for(int trial=0;trial<8;++trial) {
      std::vector<double> list;
      for(size_t i=0;i<count;++i) if(rng()%4==0) list.push_back(i*.05);
      for(int mode=0;mode<3;++mode) {
        auto actual=source,expected=source;
        const double cutoff=count*.025;
        auto drop=[&](double t) {bool found=std::find(list.begin(),list.end(),t)!=list.end();
          return mode==0 ? !found : mode==1 ? found : t<=cutoff;};
        original_cleanup(expected,drop);
        if(mode==0) actual.clean_old_measurements(list);
        else if(mode==1) actual.clean_invalid_measurements(list);
        else actual.clean_older_measurements(cutoff);
        equal(actual,expected);
      }
    }
  }
  auto original=sample(6000),optimized=original;
  std::vector<double> keep={0,299.5,299.55,299.6,299.65,299.7,299.75,299.8,299.85,299.9,299.95};
  auto a=std::chrono::steady_clock::now();
  original_cleanup(original,[&](double t){return std::find(keep.begin(),keep.end(),t)==keep.end();});
  auto b=std::chrono::steady_clock::now();
  optimized.clean_old_measurements(keep);
  auto c=std::chrono::steady_clock::now();equal(original,optimized);
  std::cout<<"PASS: 144 exact-result comparisons; 6000-observation history benchmark old_ms="
    <<std::chrono::duration<double,std::milli>(b-a).count()<<" new_ms="
    <<std::chrono::duration<double,std::milli>(c-b).count()<<'\n';
}
