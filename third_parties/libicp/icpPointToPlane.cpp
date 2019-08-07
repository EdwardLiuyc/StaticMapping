/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

Authors: Andreas Geiger

openMP support by Manolis Lourakis, Foundation for Research & Technology -
Hellas, Heraklion, Greece

libicp is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libicp is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libicp; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

#ifdef _OPENMP
#include <omp.h>
#endif

#include "icpPointToPlane.h"

using namespace std;

// Also see (3d part): "Linear Least-Squares Optimization for Point-to-Plane ICP
// Surface Registration" (Kok-Lim Low)
double IcpPointToPlane::fitStep(double *T, const int32_t T_num, Matrix &R,
                                Matrix &t, const std::vector<int32_t> &active) {
  int32_t i;
  int32_t nact = (int)active.size();

  // init matrix for point32_t correspondences
  Matrix p_m(nact, m_dim);  // model
  Matrix p_t(nact, m_dim);  // template

  // dimensionality 2
  if (m_dim == 2) {
    // extract matrix and translation vector
    double r00 = R.val[0][0];
    double r01 = R.val[0][1];
    double r10 = R.val[1][0];
    double r11 = R.val[1][1];
    double t0 = t.val[0][0];
    double t1 = t.val[1][0];

    // init A and b
    Matrix A(nact, 3);
    Matrix b(nact, 1);

    // establish correspondences
#pragma omp parallel for private(i) default(none)                   \
    shared(T, active, nact, p_m, p_t, A, b, r00, r01, r10, r11, t0, \
           t1)  // schedule (dynamic,2)
    for (i = 0; i < nact; i++) {
      // kd tree query + result
      std::vector<float> query(m_dim);
      kdtree::KDTreeResultVector result;

      // get index of active point
      int32_t idx = active[i];

      // transform point according to R|t
      query[0] = (float)(r00 * T[idx * 2 + 0] + r01 * T[idx * 2 + 1] + t0);
      query[1] = (float)(r10 * T[idx * 2 + 0] + r11 * T[idx * 2 + 1] + t1);

      // search nearest neighbor
      m_kd_tree->n_nearest(query, 1, result);

      // model point
      double dx = m_kd_tree->the_data[result[0].idx][0];
      double dy = m_kd_tree->the_data[result[0].idx][1];

      // model point normal
      double nx = M_normal[result[0].idx * 2 + 0];
      double ny = M_normal[result[0].idx * 2 + 1];

      // template point
      double sx = query[0];
      double sy = query[1];

      // setup least squares system
      A.val[i][0] = ny * sx - nx * sy;
      A.val[i][1] = nx;
      A.val[i][2] = ny;
      b.val[i][0] = nx * dx + ny * dy - nx * sx - ny * sy;
    }

    // linear least square matrices
    Matrix A_ = ~A * A;
    Matrix b_ = ~A * b;

    // solve linear system
    if (b_.solve(A_)) {
      // rotation matrix
      Matrix R_ = Matrix::eye(2);
      R_.val[0][1] = -b_.val[0][0];
      R_.val[1][0] = +b_.val[0][0];

      // orthonormalized rotation matrix
      Matrix U, W, V;
      R_.svd(U, W, V);
      R_ = U * ~V;

      // fix improper matrix problem
      if (R_.det() < 0) {
        Matrix B = Matrix::eye(m_dim);
        B.val[m_dim - 1][m_dim - 1] = R_.det();
        R_ = V * B * ~U;
      }

      // translation vector
      Matrix t_(2, 1);
      t_.val[0][0] = b_.val[1][0];
      t_.val[1][0] = b_.val[2][0];

      // compose: R|t = R_|t_ * R|t
      R = R_ * R;
      t = R_ * t + t_;

      return max((R_ - Matrix::eye(2)).l2norm(), t_.l2norm());
    }

    // dimensionality 3
  } else {
    // extract matrix and translation vector
    double r00 = R.val[0][0];
    double r01 = R.val[0][1];
    double r02 = R.val[0][2];
    double r10 = R.val[1][0];
    double r11 = R.val[1][1];
    double r12 = R.val[1][2];
    double r20 = R.val[2][0];
    double r21 = R.val[2][1];
    double r22 = R.val[2][2];
    double t0 = t.val[0][0];
    double t1 = t.val[1][0];
    double t2 = t.val[2][0];

    // init A and b
    Matrix A(nact, 6);
    Matrix b(nact, 1);

    // establish correspondences
#pragma omp parallel for private(i) default(none)                              \
    shared(T, active, nact, p_m, p_t, A, b, r00, r01, r02, r10, r11, r12, r20, \
           r21, r22, t0, t1, t2)  // schedule (dynamic,2)
    for (i = 0; i < nact; i++) {
      // kd tree query + result
      std::vector<float> query(m_dim);
      kdtree::KDTreeResultVector result;

      // get index of active point
      int32_t idx = active[i];

      // transform point according to R|t
      query[0] = (float)(r00 * T[idx * 3 + 0] + r01 * T[idx * 3 + 1] +
                         r02 * T[idx * 3 + 2] + t0);
      query[1] = (float)(r10 * T[idx * 3 + 0] + r11 * T[idx * 3 + 1] +
                         r12 * T[idx * 3 + 2] + t1);
      query[2] = (float)(r20 * T[idx * 3 + 0] + r21 * T[idx * 3 + 1] +
                         r22 * T[idx * 3 + 2] + t2);

      // search nearest neighbor
      m_kd_tree->n_nearest(query, 1, result);

      // model point
      double dx = m_kd_tree->the_data[result[0].idx][0];
      double dy = m_kd_tree->the_data[result[0].idx][1];
      double dz = m_kd_tree->the_data[result[0].idx][2];

      // model point normal
      double nx = M_normal[result[0].idx * 3 + 0];
      double ny = M_normal[result[0].idx * 3 + 1];
      double nz = M_normal[result[0].idx * 3 + 2];

      // template point
      double sx = query[0];
      double sy = query[1];
      double sz = query[2];

      // setup least squares system
      A.val[i][0] = nz * sy - ny * sz;
      A.val[i][1] = nx * sz - nz * sx;
      A.val[i][2] = ny * sx - nx * sy;
      A.val[i][3] = nx;
      A.val[i][4] = ny;
      A.val[i][5] = nz;
      b.val[i][0] = nx * dx + ny * dy + nz * dz - nx * sx - ny * sy - nz * sz;
    }

    // linear least square matrices
    Matrix A_ = ~A * A;
    Matrix b_ = ~A * b;

    // start_clock();
    // solve linear system
    if (b_.solve(A_)) {
      // rotation matrix
      Matrix R_ = Matrix::eye(3);
      R_.val[0][1] = -b_.val[2][0];
      R_.val[1][0] = +b_.val[2][0];
      R_.val[0][2] = +b_.val[1][0];
      R_.val[2][0] = -b_.val[1][0];
      R_.val[1][2] = -b_.val[0][0];
      R_.val[2][1] = +b_.val[0][0];

      // orthonormalized rotation matrix
      Matrix U, W, V;
      R_.svd(U, W, V);
      R_ = U * ~V;

      // fix improper matrix problem
      if (R_.det() < 0) {
        Matrix B = Matrix::eye(m_dim);
        B.val[m_dim - 1][m_dim - 1] = R_.det();
        R_ = V * B * ~U;
      }

      // translation vector
      Matrix t_(3, 1);
      t_.val[0][0] = b_.val[3][0];
      t_.val[1][0] = b_.val[4][0];
      t_.val[2][0] = b_.val[5][0];

      // compose: R|t = R_|t_ * R|t
      R = R_ * R;
      t = R_ * t + t_;

      // end_clock();
      return max((R_ - Matrix::eye(3)).l2norm(), t_.l2norm());
    }
  }

  // failure
  return 0;
}

std::vector<int32_t> IcpPointToPlane::getInliers(double *T, const int32_t T_num,
                                                 const Matrix &R,
                                                 const Matrix &t,
                                                 const double indist) {
  // init inlier vector + query point + query result
  vector<int32_t> inliers;
  std::vector<float> query(m_dim);
  kdtree::KDTreeResultVector neighbor;

  // dimensionality 2
  if (m_dim == 2) {
    // extract matrix and translation vector
    double r00 = R.val[0][0];
    double r01 = R.val[0][1];
    double r10 = R.val[1][0];
    double r11 = R.val[1][1];
    double t0 = t.val[0][0];
    double t1 = t.val[1][0];

    // check for all points if they are inliers
    for (int32_t i = 0; i < T_num; i++) {
      // transform point according to R|t
      double sx = r00 * T[i * 2 + 0] + r01 * T[i * 2 + 1] + t0;
      query[0] = (float)sx;
      double sy = r10 * T[i * 2 + 0] + r11 * T[i * 2 + 1] + t1;
      query[1] = (float)sy;

      // search nearest neighbor
      m_kd_tree->n_nearest(query, 1, neighbor);

      // model point
      double dx = m_kd_tree->the_data[neighbor[0].idx][0];
      double dy = m_kd_tree->the_data[neighbor[0].idx][1];

      // model point normal
      double nx = M_normal[neighbor[0].idx * 2 + 0];
      double ny = M_normal[neighbor[0].idx * 2 + 1];

      // check if it is an inlier
      if (/*neighbor[0].dis<3*indist && */ abs((sx - dx) * nx +
                                               (sy - dy) * ny) < indist)
        inliers.push_back(i);
    }

    // dimensionality 3
  } else {
    // extract matrix and translation vector
    double r00 = R.val[0][0];
    double r01 = R.val[0][1];
    double r02 = R.val[0][2];
    double r10 = R.val[1][0];
    double r11 = R.val[1][1];
    double r12 = R.val[1][2];
    double r20 = R.val[2][0];
    double r21 = R.val[2][1];
    double r22 = R.val[2][2];
    double t0 = t.val[0][0];
    double t1 = t.val[1][0];
    double t2 = t.val[2][0];

    // check for all points if they are inliers
    for (int32_t i = 0; i < T_num; i++) {
      // transform point according to R|t
      double sx =
          r00 * T[i * 3 + 0] + r01 * T[i * 3 + 1] + r02 * T[i * 3 + 2] + t0;
      query[0] = (float)sx;
      double sy =
          r10 * T[i * 3 + 0] + r11 * T[i * 3 + 1] + r12 * T[i * 3 + 2] + t1;
      query[1] = (float)sy;
      double sz =
          r20 * T[i * 3 + 0] + r21 * T[i * 3 + 1] + r22 * T[i * 3 + 2] + t2;
      query[2] = (float)sz;

      // search nearest neighbor
      m_kd_tree->n_nearest(query, 1, neighbor);

      // model point
      double dx = m_kd_tree->the_data[neighbor[0].idx][0];
      double dy = m_kd_tree->the_data[neighbor[0].idx][1];
      double dz = m_kd_tree->the_data[neighbor[0].idx][2];

      // model point normal
      double nx = M_normal[neighbor[0].idx * 3 + 0];
      double ny = M_normal[neighbor[0].idx * 3 + 1];
      double nz = M_normal[neighbor[0].idx * 3 + 2];

      // check if it is an inlier
      if (abs((sx - dx) * nx + (sy - dy) * ny + (sz - dz)) * nz < indist)
        inliers.push_back(i);
    }
  }

  // return vector with inliers
  return inliers;
}

void IcpPointToPlane::computeNormal(const kdtree::KDTreeResultVector &neighbors,
                                    double *M_normal, const double flatness) {
  // dimensionality 2
  if (m_dim == 2) {
    // extract neighbors
    Matrix P(neighbors.size(), 2);
    Matrix mu(1, 2);
    for (uint32_t i = 0; i < neighbors.size(); i++) {
      double x = m_kd_tree->the_data[neighbors[i].idx][0];
      double y = m_kd_tree->the_data[neighbors[i].idx][1];
      P.val[i][0] = x;
      P.val[i][1] = y;
      mu.val[0][0] += x;
      mu.val[0][1] += y;
    }
    // zero mean
    mu = mu / (double)neighbors.size();
    Matrix Q = P - Matrix::ones(neighbors.size(), 1) * mu;

    // principal component analysis
    Matrix H = ~Q * Q;
    Matrix U, W, V;
    H.svd(U, W, V);

    // normal
    M_normal[0] = U.val[0][1];
    M_normal[1] = U.val[1][1];

    // dimensionality 3
  } else {
    // extract neighbors
    Matrix P(neighbors.size(), 3);
    Matrix mu(1, 3);
    for (uint32_t i = 0; i < neighbors.size(); i++) {
      double x = m_kd_tree->the_data[neighbors[i].idx][0];
      double y = m_kd_tree->the_data[neighbors[i].idx][1];
      double z = m_kd_tree->the_data[neighbors[i].idx][2];
      P.val[i][0] = x;
      P.val[i][1] = y;
      P.val[i][2] = z;
      mu.val[0][0] += x;
      mu.val[0][1] += y;
      mu.val[0][2] += z;
    }

    // zero mean
    mu = mu / (double)neighbors.size();
    Matrix Q = P - Matrix::ones(neighbors.size(), 1) * mu;

    // principal component analysis
    Matrix H = ~Q * Q;
    Matrix U, W, V;
    H.svd(U, W, V);

    // normal
    M_normal[0] = U.val[0][2];
    M_normal[1] = U.val[1][2];
    M_normal[2] = U.val[2][2];
  }
}

double *IcpPointToPlane::computeNormals(const int32_t num_neighbors,
                                        const double flatness) {
  double *M_normal = (double *)malloc(m_kd_tree->N * m_dim * sizeof(double));
  kdtree::KDTreeResultVector neighbors;
  for (int32_t i = 0; i < m_kd_tree->N; i++) {
    m_kd_tree->n_nearest_around_point(i, 0, num_neighbors, neighbors);
    // m_kd_tree->r_nearest_around_point(i,0,0.05,neighbors);
    if (m_dim == 2)
      computeNormal(neighbors, M_normal + i * 2, flatness);
    else
      computeNormal(neighbors, M_normal + i * 3, flatness);
  }

  return M_normal;
}

double IcpPointToPlane::getResidual(double *T, const int32_t T_num,
                                    const Matrix &R, const Matrix &t,
                                    const std::vector<int> &active) {
  if (active.empty()) return 0;
  int nact = active.size();
  double residual = 0;

  std::vector<float> query(m_dim);
  kdtree::KDTreeResultVector result;

  if (m_dim == 2) {
    // extract matrix and translation vector
    double r00 = R.val[0][0];
    double r01 = R.val[0][1];
    double r10 = R.val[1][0];
    double r11 = R.val[1][1];
    double t0 = t.val[0][0];
    double t1 = t.val[1][0];
    for (int32_t i = 0; i < nact; i++) {
      // kd tree query + result
      int32_t idx = active[i];

      // transform point according to R|t
      double tx = r00 * T[idx * 2 + 0] + r01 * T[idx * 2 + 1] + t0;
      double ty = r10 * T[idx * 2 + 0] + r11 * T[idx * 2 + 1] + t1;
      query[0] = (float)tx;
      query[1] = (float)ty;

      // search nearest neighbor
      m_kd_tree->n_nearest(query, 1, result);
      // model point
      double mx = m_kd_tree->the_data[result[0].idx][0];
      double my = m_kd_tree->the_data[result[0].idx][1];
      // model point normal
      double nx = M_normal[result[0].idx * 2 + 0];
      double ny = M_normal[result[0].idx * 2 + 1];

      residual += abs(nx * (mx - tx) + ny * (my - ty));
    }
    // dimensionality 3
  } else {
    // extract matrix and translation vector
    double r00 = R.val[0][0];
    double r01 = R.val[0][1];
    double r02 = R.val[0][2];
    double r10 = R.val[1][0];
    double r11 = R.val[1][1];
    double r12 = R.val[1][2];
    double r20 = R.val[2][0];
    double r21 = R.val[2][1];
    double r22 = R.val[2][2];
    double t0 = t.val[0][0];
    double t1 = t.val[1][0];
    double t2 = t.val[2][0];
    for (int32_t i = 0; i < nact; i++) {
      // kd tree query + result
      std::vector<float> query(m_dim);
      kdtree::KDTreeResultVector result;
      // get index of active point
      int32_t idx = active[i];
      // transform point according to R|t
      double tx = r00 * T[idx * 3 + 0] + r01 * T[idx * 3 + 1] +
                  r02 * T[idx * 3 + 2] + t0;
      double ty = r10 * T[idx * 3 + 0] + r11 * T[idx * 3 + 1] +
                  r12 * T[idx * 3 + 2] + t1;
      double tz = r20 * T[idx * 3 + 0] + r21 * T[idx * 3 + 1] +
                  r22 * T[idx * 3 + 2] + t2;
      query[0] = (float)tx;
      query[1] = (float)ty;
      query[2] = (float)tz;
      // search nearest neighbor
      m_kd_tree->n_nearest(query, 1, result);
      //
      // model point
      double mx = m_kd_tree->the_data[result[0].idx][0];
      double my = m_kd_tree->the_data[result[0].idx][1];
      double mz = m_kd_tree->the_data[result[0].idx][2];

      // model point normal
      double nx = M_normal[result[0].idx * 3 + 0];
      double ny = M_normal[result[0].idx * 3 + 1];
      double nz = M_normal[result[0].idx * 3 + 2];

      // modified by liu.y.c
      // 用 l2-norm 代替,否则最后得到的误差有负数，会影响判断
      Matrix E(1, 3);
      E.val[0][0] = mx - tx;
      E.val[0][1] = my - ty;
      E.val[0][2] = mz - tz;
      Matrix N(3, 1);
      N.val[0][0] = nx;
      N.val[1][0] = ny;
      N.val[2][0] = nz;
      residual += (E * N).l2norm();
      //       residual += nx * (mx-tx) + ny * (my-ty) + nz * (mz-tz);
    }
  }
  residual /= nact;
  return residual;
}
