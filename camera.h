#ifndef SRC_CAMERA_H
#define SRC_CAMERA_H

#include <iostream>
#include <bits/stdc++.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <cstdlib> 
#include <ctime>

#include "pose_local_parameterization.h"
#include "line_projection_factor.h"
#include "utility.h"
#include "global.h"

using namespace std;
using namespace ceres;
using namespace Eigen;

struct Point
{
    double x;
    double y;
    double z;
};
struct line
{
    Point point1;
    Point point2;
};

class camera
{
    public:
    vector<line> lines;
    Eigen::Matrix3d K;
    double width=752.0;
    double height=480.0;
    vector<vector<int>> windows;
    vector<vector<line>> camera_observation;
    vector<Eigen::Isometry3d> T;
    double *para_Pose;
    camera(double *_para_Pose):para_Pose(_para_Pose)
    {
        lines.clear();
        T.clear();
        K<<1115,0,367.2,0,1114,238.5,0,0,1;
        line new_line;
        new_line.point1.x=0;
        new_line.point1.y=0;
        new_line.point1.z=0;
        new_line.point2.x=0;
        new_line.point2.y=0;
        new_line.point2.z=0;
        vector<int> a(WINDOW_SIZE+1,0);
        vector<line> smallline(WINDOW_SIZE+1,new_line);
        windows.resize(LINE_NUM,a);
        camera_observation.resize(LINE_NUM,smallline);
    }
    void add_line(double x,double y,double z,double x1,double y1,double z1);
    void new_pose(Eigen::AngleAxisd V,Eigen::Vector3d translation);
    void set_observation(int i,int j);
   

};

#endif