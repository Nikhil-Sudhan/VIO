// Check independent covariance-column tiles against Eigen's original update.
#include <Eigen/Dense>
#include <opencv2/core/utility.hpp>
#include <chrono>
#include <iostream>
#include <stdexcept>
using Matrix=Eigen::MatrixXd;
void tiled(Matrix &P,const Matrix &K,const Matrix &M) {
  const int n=P.rows(),tile=64,blocks=(n+tile-1)/tile;
  cv::parallel_for_(cv::Range(0,blocks),[&](const cv::Range &range) {
    for(int b=range.start;b<range.end;++b) {
      const int col=b*tile,width=std::min(tile,n-col);
      if(col)P.block(0,col,col,width).noalias()-=K.topRows(col)*M.middleRows(col,width).transpose();
      P.block(col,col,width,width).triangularView<Eigen::Upper>()-=K.middleRows(col,width)*M.middleRows(col,width).transpose();
    }
  });
  P=P.selfadjointView<Eigen::Upper>();
}
int main() {
  cv::setNumThreads(4);int cases=0;double worst=0;
  for(int n:{22,63,64,65,127,128,129,667})for(int m:{1,2,12,50}) {
    Matrix M=Matrix::Random(n,m),K=Matrix::Random(n,m),P=Matrix::Random(n,n);
    P=P.selfadjointView<Eigen::Upper>();
    Matrix expected=P,actual=P;
    expected.triangularView<Eigen::Upper>()-=K*M.transpose();expected=expected.selfadjointView<Eigen::Upper>();
    tiled(actual,K,M);
    double error=(expected-actual).cwiseAbs().maxCoeff()/std::max(1.,expected.cwiseAbs().maxCoeff());
    worst=std::max(error,worst);if(error>2e-13)throw std::runtime_error("Covariance mismatch");++cases;
    if(n==667 && m==50) {
      volatile double sink=0;
      for(int mode=0;mode<2;++mode) {
        auto start=std::chrono::steady_clock::now();
        for(int k=0;k<30;++k) {Matrix x=P;if(mode)tiled(x,K,M);else {x.triangularView<Eigen::Upper>()-=K*M.transpose();x=x.selfadjointView<Eigen::Upper>();}sink+=x(0,0);}
        std::cout<<"mode="<<mode<<" mean_ms="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/30<<'\n';
      }
    }
  }
  std::cout<<"PASS cases="<<cases<<" worst_scaled_error="<<worst<<'\n';
}
