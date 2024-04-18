#ifndef SRC_LINE_TRIANGULATE_H
#define SRC_LINE_TRIANGULATE_H

#include <iostream>
#include <bits/stdc++.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <cstdlib> 
#include <ctime>
#include <vector>

#include "utility.h"
#include "camera.h"
#include "global.h"

using namespace std;
using namespace ceres;
using namespace Eigen;

class LineCostFunction : public ceres::SizedCostFunction<2,2>
{
    public:
	    LineCostFunction(Eigen::Vector3d _line_i_s, Eigen::Vector3d _line_i_e,Eigen::Vector3d _line_j_s,Eigen::Vector3d _line_j_e,Eigen::Vector3d _t,Eigen::Matrix3d _R) : line_i_s(_line_i_s),line_i_e(_line_i_e),line_j_s(_line_j_s),line_j_e(_line_j_e),t(_t),R(_R) {}
        bool Evaluate(double const* const* parameters,
        double* residuals,
        double** jacobians) const override{
        double line_depth1=parameters[0][0];
	    double line_depth2=parameters[0][1];
            Eigen::Map<Eigen::Vector2d> residual(residuals);
            residual[0] = (line_j_s.cross(line_j_e).transpose().normalized()*R.transpose()*(line_depth2*(line_i_e)-line_depth1*(line_i_s)))(0,0);   //残差，也就是代价函数的输出
            residual[1] = (line_depth1*line_depth2*line_j_s.cross(line_j_e).transpose().normalized()*R.transpose()*(line_i_s*(line_depth2*(line_i_e)-line_depth1*(line_i_s)).transpose()-(line_depth2*(line_i_e)-line_depth1*(line_i_s)).transpose()*line_i_s*Matrix3d::Identity())*line_i_e
	                       -line_j_s.cross(line_j_e).transpose().normalized()*R.transpose()*(t*(line_depth2*(line_i_e)-line_depth1*(line_i_s)).transpose()-(line_depth2*line_i_e-line_depth1*line_i_s).transpose()*t*Matrix3d::Identity())*(line_depth2*line_i_e-line_depth1*line_i_s))(0,0);

            if(jacobians!=nullptr)
            if(jacobians[0] != NULL) {
            {
                Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J(jacobians[0]);
                J(0,0)=(-line_j_s.cross(line_j_e).transpose().normalized()*R.transpose()*(line_i_s))(0,0);
				J(0,1)=(line_j_s.cross(line_j_e).transpose().normalized()*R.transpose()*(line_i_e))(0,0);		
                J(1,0)=(line_j_s.cross(line_j_e).transpose().normalized()*R.transpose()*(line_depth2*line_depth2*line_i_e-2*line_depth1*line_depth2*line_i_s)*(line_i_s.cross(line_i_e)))(0,0)
				                +(line_j_s.cross(line_j_e).transpose().normalized()*R.transpose()*(-line_depth2*line_i_e*line_i_s.transpose()-line_depth2*line_i_s*line_i_e.transpose()+2*line_depth1*line_i_e*line_i_s.transpose()-(-line_depth2*line_i_e.transpose()*line_i_s-line_depth2*line_i_s.transpose()*line_i_e+2*line_depth1*line_i_s.transpose()*line_i_e)(0,0)*Matrix3d::Identity())*t)(0,0);
				J(1,1)=(line_j_s.cross(line_j_e).transpose().normalized()*R.transpose()*(line_depth2*line_depth2*line_i_e-2*line_depth1*line_depth2*line_i_s)*(line_i_s.cross(line_i_e)))(0,0)
				                +(line_j_s.cross(line_j_e).transpose().normalized()*R.transpose()*(2*line_depth2*line_i_e*line_i_e.transpose()-line_depth1*line_i_e*line_i_s.transpose()-line_depth2*line_i_s*line_i_e.transpose()-(2*line_depth2*line_i_e.transpose()*line_i_e-line_depth1*line_i_e.transpose()*line_i_s-line_depth2*line_i_s.transpose()*line_i_e)(0,0)*Matrix3d::Identity())*t)(0,0);
            }
}
            return true;
        }
	    Vector3d line_i_s,line_i_e,line_j_s,line_j_e;    // 平面上端点数据
        Vector3d t ;//R和t是从第i帧到第j帧的变换位姿
        Matrix3d R ;
};

class LinedirCostFunction : public ceres::SizedCostFunction<1,3>
{
    public:
	    LinedirCostFunction(Eigen::Vector3d _line_j_s,Eigen::Vector3d _line_j_e,Eigen::Matrix3d _R) : line_j_s(_line_j_s),line_j_e(_line_j_e),R(_R) {}
        bool Evaluate(double const* const* parameters,
        double* residuals,
        double** jacobians) const override{
            Vector3d line_direction=Vector3d(parameters[0][0],parameters[0][1],parameters[0][2]);
	       
            //Eigen::Map<Eigen::Vector1d> residual(residuals);
            residuals[0] = (line_j_s.cross(line_j_e).transpose().normalized()*R.transpose()*(line_direction))(0,0);   //残差，也就是代价函数的输出			
   
            if(jacobians!=nullptr)
			{
             if(jacobians[0] != NULL) 
            {
                Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J(jacobians[0]);
                J=line_j_s.cross(line_j_e).transpose().normalized()*R.transpose();	
            }
			
            }
            return true;
        }
	    Vector3d line_j_s,line_j_e;    // 平面上端点数据
        Matrix3d R ;
};
class LinedepthCostFunction : public ceres::SizedCostFunction<1,1>
{
    public:
	    LinedepthCostFunction(Eigen::Vector3d _line_i_s, Eigen::Vector3d _line_i_e,Eigen::Vector3d _line_j_s,Eigen::Vector3d _line_j_e,Eigen::Vector3d _t,Eigen::Matrix3d _R,double *_line_dir) : line_i_s(_line_i_s),line_i_e(_line_i_e),line_j_s(_line_j_s),line_j_e(_line_j_e),t(_t),R(_R),line_dir(_line_dir) {}
        bool Evaluate(double const* const* parameters,
        double* residuals,
        double** jacobians) const override{
            Vector3d line_direction=Vector3d(line_dir[0],line_dir[1],line_dir[2]).normalized();
	    double line_depth=parameters[0][0];
            //Eigen::Map<Eigen::Vector1d> residual(residuals); 
            Vector3d tmpb=Utility::skewSymmetric(line_direction)*R*line_j_s.cross(line_j_e).normalized();         
            residuals[0] = line_depth*(tmpb.transpose()*line_i_s.cross(line_i_e).normalized())(0,0)-(tmpb.transpose()*Utility::skewSymmetric(t)*line_direction)(0,0);      
            if(jacobians!=nullptr)
			{
             if(jacobians[0] != NULL) 
            {
                Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J(jacobians[0]);	
                J=(tmpb.transpose()*line_i_s.cross(line_i_e).normalized());
            }
			
}
            return true;
        }
	    Vector3d line_i_s,line_i_e,line_j_s,line_j_e;    // 平面上端点数据
        Vector3d t ;//R和t是从第i帧到第j帧的变换位姿
        Matrix3d R ;
        double *line_dir;
};



class LineTriangulate{
    public:
    LineTriangulate(camera _cam1,double *_para_line,double *_para_Pose):cam1(_cam1),para_line(_para_line),para_Pose(_para_Pose){
        line_para_depth=false;
        
    }
    camera cam1;
    double *para_line;
    double *para_Pose;
    bool line_para_depth;
    void leastsquare_depth();  
    void avg_plucker();
    void leastsquare_plucker();

};

#endif