// Native numerical check for the SuiteSparse 7 qrTol compatibility patch.
#include <aslam/calibration/algorithms/linalg.h>
#include <cholmod.h>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
  cholmod_common common;
  cholmod_l_start(&common);
  auto* a=cholmod_l_allocate_sparse(3,3,4,1,1,0,CHOLMOD_REAL,&common);
  if (!a) return 2;
  auto* p=static_cast<int64_t*>(a->p);
  auto* i=static_cast<int64_t*>(a->i);
  auto* x=static_cast<double*>(a->x);
  p[0]=0;p[1]=2;p[2]=2;p[3]=4; // middle column is empty
  i[0]=0;i[1]=2;i[2]=0;i[3]=1;
  x[0]=3;x[1]=4;x[2]=5;x[3]=12;
  const double eps=1e-12;
  double actual=aslam::calibration::qrTol(a,&common,eps);
  const double expected=20*6*eps*13;
  if (std::abs(actual/expected-1)>1e-12) throw std::runtime_error("Wrong maximum column norm");
  x[0]=3e200;x[1]=4e200;x[2]=5e-200;x[3]=12e-200;
  actual=aslam::calibration::qrTol(a,&common,eps);
  if (!std::isfinite(actual) || std::abs(actual/(20*6*eps*5e200)-1)>1e-12)
    throw std::runtime_error("Column norm overflow/underflow");
  a->packed=0;
  bool rejected=false;
  try { aslam::calibration::qrTol(a,&common,eps); } catch (const std::exception&) { rejected=true; }
  a->packed=1;
  if (!rejected) throw std::runtime_error("Unsupported representation was accepted");
  cholmod_l_free_sparse(&a,&common);
  cholmod_l_finish(&common);
  std::cout << "qrTol numerical checks passed: known norm, empty column, scaling, invalid layout\n";
}
