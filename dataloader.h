#ifndef SRC_DATALOADER_H
#define SRC_DATALOADER_H

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <string>

#include "global.h"
#include "camera.h"

using namespace std;
using namespace Eigen;

class DataLoader{
    public:
    DataLoader();
    void wirteTransformer(const std::string& filename, const Eigen::AngleAxisd& angleAxis, const Eigen::Vector3d& translation);
    void writeLine(const std::string& filename,const double& x,const double& y,const double& z,const double& x1,const double& y1,const double& z1);
    void writeTriangulate(string filename,double (*para_depth_line)[2],camera cam1,double *para_pose);

};

#endif