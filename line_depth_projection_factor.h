#pragma once

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "utility.h"

using namespace std;
using namespace ceres;
using namespace Eigen;

// \brief 直线残差的投影计算
//
// ceres::SizedCostFunction<2, 7, 7，7，2>说明：
// 2: 残差维度
// 7： i帧imu pose(P_wbi, R_wbi)
// 7:  j帧imu pose(P_wb,j R_wbj)
// 7:  外参
// 2： i帧直线在World坐标系中2个深度参数
class LineDepthProjectionFactor : public ceres::SizedCostFunction<2, 7, 7,7,2> {
public:
    LineDepthProjectionFactor(const Vector3d &_line_i_s, const Vector3d &_line_i_e,const Vector3d &_line_j_s,const Vector3d &_line_j_e);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    void check(double **parameters);



    Vector3d line_i_s,line_i_e,line_j_s,line_j_e;


    static Eigen::Matrix2d sqrt_info;

};
