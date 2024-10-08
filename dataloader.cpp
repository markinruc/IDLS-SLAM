#include "dataloader.h"

DataLoader::DataLoader(){

};

double caldepth(Vector3d n,Vector3d d,Vector3d p)
{
    Vector3d b=p.cross(d);
    Matrix<double, 3, 1> A;
    A << b;
    double x;
    x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(n);
    return x;
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

void DataLoader::writenewLine(string filename,double (*para_depth_line)[2],camera cam1,double *para_pose){
        Eigen::Vector3d t0(para_pose[0], para_pose[1], para_pose[2]);
        Eigen::Quaterniond q0(para_pose[6], para_pose[3], para_pose[4], para_pose[5]);
        Eigen::Matrix3d R0=q0.toRotationMatrix();
        ofstream outFile(filename);
        if (!outFile.is_open()) {
            std::cerr << "Unable to open file for writing!" << std::endl;
            return;
        }
        for (int i=0;i<LINE_NUM;i++)//遍历滑窗内所有的空间线
       {
     
		    double line_depth[2]={1,1};
            Vector3d line_i_s,line_i_e;
		    line_depth[0]=1/para_depth_line[i][0];
            line_depth[1]=1/para_depth_line[i][1];		        
	        line_i_s<<cam1.camera_observation[i][0].point1.x,cam1.camera_observation[i][0].point1.y,cam1.camera_observation[i][0].point1.z;
	        line_i_e<<cam1.camera_observation[i][0].point2.x,cam1.camera_observation[i][0].point2.y,cam1.camera_observation[i][0].point2.z;
            line_i_s=line_i_s*line_depth[0];
            line_i_e=line_i_e*line_depth[1];
            Vector3d line_s,line_e;
            line_s=R0*line_i_s+t0;
            line_e=R0*line_i_e+t0;
            outFile << line_s.transpose() <<" "<< line_e.transpose() << endl;
       }
       outFile.close();

};

void DataLoader::writenewLine(string filename,double (*para_line)[5],camera cam1,double *para_pose){
        Eigen::Vector3d t0(para_pose[0], para_pose[1], para_pose[2]);
        Eigen::Quaterniond q0(para_pose[6], para_pose[3], para_pose[4], para_pose[5]);
        Eigen::Matrix3d R0=q0.toRotationMatrix();
        ofstream outFile(filename);
        if (!outFile.is_open()) {
            std::cerr << "Unable to open file for writing!" << std::endl;
            return;
        }
        for (int i=0;i<LINE_NUM;i++)//遍历滑窗内所有的空间线
       {
     		    
		    double orth[5];
            for (int j = 0; j < 5; ++j)
            {
                orth[j] = para_line[i][j];
            }
            Eigen::Vector3d Lci_n, Lci_d;
            Vector3d line_i_s,line_i_e;
	        Utility::cvtOrthonormalToPlucker(orth, Lci_n, Lci_d);
            line_i_s<<cam1.camera_observation[i][0].point1.x,cam1.camera_observation[i][0].point1.y,cam1.camera_observation[i][0].point1.z;
	        line_i_e<<cam1.camera_observation[i][0].point2.x,cam1.camera_observation[i][0].point2.y,cam1.camera_observation[i][0].point2.z;
            double ds=caldepth(Lci_n, Lci_d,line_i_s);
            double de=caldepth(Lci_n, Lci_d,line_i_e);
            line_i_s=line_i_s*ds;
            line_i_e=line_i_e*de;
            Vector3d line_s,line_e;
            line_s=R0*line_i_s+t0;
            line_e=R0*line_i_e+t0;
            outFile << line_s.transpose() <<" "<< line_e.transpose() << endl;
       }
       outFile.close();

};

void DataLoader::writenewPose(string filename,double (*para_Pose)[7]){
    ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "Unable to open file for writing!" << std::endl;
        return;
    }
    for(int i=0;i<WINDOW_SIZE+1;i++)
    {
        Eigen::Vector3d t(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        Eigen::Quaterniond q(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]);
        outFile << q.transpose() <<" "<< t.transpose() << endl;
    }

    outFile.close();

};