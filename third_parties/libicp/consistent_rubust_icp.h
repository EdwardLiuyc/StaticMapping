#pragma once

#include "icp.h"

class ConsistentRubustIcp : public Icp {

public:

  ConsistentRubustIcp (double *M,const int32_t M_num,const int32_t dim) : Icp(M,M_num,dim) {}
  virtual ~ConsistentRubustIcp () {}

private:

  double fitStep (double *T,const int32_t T_num,Matrix &R,Matrix &t,const std::vector<int32_t> &active);
  std::vector<int32_t> getInliers (double *T,const int32_t T_num,const Matrix &R,const Matrix &t,const double indist);
  double getResidual(double *T,const int32_t T_num,const Matrix &R,const Matrix &t,const std::vector<int> &active);
};