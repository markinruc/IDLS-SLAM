#include "dataloader.h"

DataLoader::DataLoader(){

};

void DataLoader::writeTransformer(const std::string& filename, const Eigen::AngleAxisd& angleAxis, const Eigen::Vector3d& translation){
    std::ofstream outfile(filename, std::ios::app); // 以追加模式打开文件
    if (!outfile.is_open()) {
        std::cerr << "Unable to open file for writing!" << std::endl;
        return;
    }

    outfile << angleAxis.angle() << " " << angleAxis.axis().x() << " " << angleAxis.axis().y() << " " << angleAxis.axis().z() << " ";
    outfile << translation.x() << " " << translation.y() << " " << translation.z() << std::endl;

    outfile.close();
    std::cout << "Camera pose data saved to " << filename << std::endl;
   

};

void DataLoader::writeLine(const std::string& filename,const double& x,const double& y,const double& z,const double& x1,const double& y1,const double& z1){
     std::ofstream outfile(filename, std::ios::app); // 以追加模式打开文件
    if (!outfile.is_open()) {
        std::cerr << "Unable to open file for writing!" << std::endl;
        return;
    }

    outfile << x << " " << y << " " << z << " " << x1 << " " << y1 << " " << z1 << std::endl;

    outfile.close();
    std::cout << "Line data saved to " << filename << std::endl;

};

void DataLoader::writeTriangulate(const std::string& filename){

};