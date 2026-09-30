// Algebra and cost check for the block-sparse EKF cross covariance P H^T.
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>
struct Block { int offset, size; };
using Matrix=Eigen::MatrixXd;
Matrix original(const Matrix &P,const Matrix &H,const std::vector<Block>&vars,const std::vector<Block>&order) {
  Matrix out=Matrix::Zero(P.rows(),H.rows());
  for(const auto &v:vars) {
    Matrix m=Matrix::Zero(v.size,H.rows());int col=0;
    for(const auto &h:order) {
      m.noalias()+=P.block(v.offset,h.offset,v.size,h.size)*H.middleCols(col,h.size).transpose();col+=h.size;
    }
    out.middleRows(v.offset,v.size)=m;
  }
  return out;
}
Matrix all_rows(const Matrix &P,const Matrix &H,const std::vector<Block>&order) {
  Matrix out=Matrix::Zero(P.rows(),H.rows());int col=0;
  for(const auto &h:order) {
    out.noalias()+=P.middleCols(h.offset,h.size)*H.middleCols(col,h.size).transpose();col+=h.size;
  }
  return out;
}
Matrix gathered(const Matrix &P,const Matrix &H,const std::vector<Block>&order) {
  Matrix small(P.rows(),H.cols());int col=0;
  for(const auto &h:order) {small.middleCols(col,h.size)=P.middleCols(h.offset,h.size);col+=h.size;}
  return small*H.transpose();
}
Matrix sparse_rows(const Matrix &P,const Matrix &H,const std::vector<Block>&order) {
  Matrix out=Matrix::Zero(P.rows(),H.rows());int col=0;
  for(const auto &h:order) {
    for(int row=0;row<H.rows();) {
      while(row<H.rows() && H.block(row,col,1,h.size).isZero(0.0))++row;
      const int first=row;
      while(row<H.rows() && !H.block(row,col,1,h.size).isZero(0.0))++row;
      if(row>first)out.middleCols(first,row-first).noalias()+=P.middleCols(h.offset,h.size)*H.block(first,col,row-first,h.size).transpose();
    }
    col+=h.size;
  }
  return out;
}
int main() {
  std::mt19937 rng(20260930);int cases=0;double worst=0;
  for(int landmarks:{0,20,190}) for(int measures:{2,12,50}) for(int trial=0;trial<4;++trial) {
    std::vector<Block> vars;int n=0;
    for(int size:{15,1,6,6,6,6,6,6,6,6,6,6,6}) {vars.push_back({n,size});n+=size;}
    for(int i=0;i<landmarks;++i) {vars.push_back({n,3});n+=3;}
    auto order=vars;std::shuffle(order.begin(),order.end(),rng);order.resize(std::min(size_t(30),order.size()));
    int cols=0;for(auto b:order)cols+=b.size;
    Matrix A=Matrix::Random(n,n),P=A*A.transpose()+Matrix::Identity(n,n),H=Matrix::Random(measures,cols);
    if(trial>=2) {
      int column=0;
      for(size_t k=0;k<order.size();++k) {
        if(k>5) for(int r=0;r<measures;++r) if(r/2!=int(k)%std::max(1,measures/2))H.block(r,column,1,order[k].size).setZero();
        column+=order[k].size;
      }
    }
    // Expand the sparse ordered Jacobian to validate arbitrary block order and gaps.
    Matrix full=Matrix::Zero(measures,n);int col=0;
    for(auto b:order){full.middleCols(b.offset,b.size)=H.middleCols(col,b.size);col+=b.size;}
    Matrix reference=P*full.transpose(),old=original(P,H,vars,order),now=all_rows(P,H,order),gemm=gathered(P,H,order),sparse=sparse_rows(P,H,order);
    for(const auto *m:{&old,&now,&gemm,&sparse}) {
      double error=(*m-reference).cwiseAbs().maxCoeff()/std::max(1.,reference.cwiseAbs().maxCoeff());worst=std::max(worst,error);
      if(error>2e-13)throw std::runtime_error("Cross-covariance differs from dense oracle");
    }
    ++cases;
    if(landmarks==190 && measures==50 && trial==3) {
      volatile double sink=0;const int repeats=50;
      for(int mode=0;mode<4;++mode) {
        auto start=std::chrono::steady_clock::now();
        for(int i=0;i<repeats;++i) {Matrix z=mode==0?original(P,H,vars,order):mode==1?all_rows(P,H,order):mode==2?gathered(P,H,order):sparse_rows(P,H,order);sink+=z(0,0);}
        std::cout<<"mode="<<mode<<" mean_ms="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/repeats<<'\n';
      }
    }
  }
  std::cout<<"PASS cases="<<cases<<" worst_scaled_error="<<worst<<'\n';
}
