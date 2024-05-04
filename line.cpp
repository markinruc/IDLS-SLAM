/***
line.cpp author:Wanting Li
camera--Observation models for cameras, simulated observations
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
***/
#include <iostream>
#include<bits/stdc++.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <cstdlib> 
#include <ctime>
#include <string>

#include "pose_local_parameterization.h"
#include "line_projection_factor.h"
#include "line_depth_projection_factor.h"
#include "utility.h"
#include "line_triangulate.h"
#include "camera.h"
#include "global.h"
#include "dataloader.h"

using namespace std;
using namespace ceres;
using namespace Eigen;

double para_Pose[WINDOW_SIZE+1][7];
double para_Ex_Pose[NUM_OF_CAM][7];
double para_line_depth[LINE_NUM][2];
double para_line[LINE_NUM][5];


/*
struct LINE_FITTING_COST {
  LINE_FITTING_COST(Eigen::Vector3d _line_i_s, Eigen::Vector3d _line_i_e,Eigen::Vector3d _line_j_s,Eigen::Vector3d _line_j_e,Eigen::Vector3d _t,Eigen::Matrix3d _R) : line_i_s(_line_i_s),line_i_e(_line_i_e),line_j_s(_line_j_s),line_j_e(_line_j_e),t(_t),R(_R) {}

  // 残差的计算
  template<typename T>
  bool operator()(
    const T *const line_depth, // 模型参数，待优化的参数，有3维
    T *residual) const {
    residual[0] = T(line_j_s).cross(T(line_j_e)).transpose().normalized()*T(R).transpose()*(line_depth[1]*(T(line_i_e))-line_depth[0]*(T(line_i_s)));   //残差，也就是代价函数的输出
    residual[1] = line_depth[0]*line_depth[1]*T(line_j_s).cross(T(line_j_e)).transpose().normalized()*T(R).transpose()*(T(line_i_s)*(line_depth[1]*(T(line_i_e))-line_depth[0]*(T(line_i_s))).transpose()-(line_depth[1]*(T(line_i_e))-line_depth[0]*(T(line_i_s))).transpose()*T(line_i_s)*Matrix3d::Identity())*T(line_i_e)
	              -T(line_j_s).cross(T(line_j_e)).transpose().normalized()*T(R).transpose()*(T(t)*(line_depth[1]*(T(line_i_e))-line_depth[0]*(T(line_i_s))).transpose()-(line_depth[1]*(T(line_i_e))-line_depth[0]*(T(line_i_s))).transpose()*T(t)*Matrix3d::Identity())*(line_depth[1]*(T(line_i_e))-line_depth[0]*(T(line_i_s)));
	return true;
  }

  const Vector3d line_i_s,line_i_e,line_j_s,line_j_e;    // 平面上端点数据
  const Vector3d t ;//R和t是从第i帧到第j帧的变换位姿
  const Matrix3d R ;
};
*/



int main(int argc, char** argv) {
      fill(para_line[0],para_line[0]+LINE_NUM*5,1);
      fill(para_Ex_Pose[0],para_Ex_Pose[0]+NUM_OF_CAM*7,0);
      para_Ex_Pose[0][6]=1;
//    fill(para_Pose[0],para_Pose[0]+(WINDOW_SIZE+1)*7,1);
	//设置随机数
	unsigned seed; 
	seed = time(0);
    srand(seed);
    line_filename="line-world.txt";
    transform_filename="transformer.txt";
    triangulate_filename="triangulate.txt";
    //构建线观测
	camera cam1(para_Pose);
    DataLoader dataload;
	for (int i=0;i < WINDOW_SIZE + 1; i++)
	{
	    //Eigen::AngleAxisd V(3.1415926 / (rand() % 10 + 1), Eigen::Vector3d(rand() % 10, rand() % 10, rand() % 10).normalized());
        Eigen::AngleAxisd V(0, Eigen::Vector3d(0,0,0).normalized());
        Eigen::Vector3d translation(rand() % 20-10, rand() % 20-10, rand() % 2);
	    cam1.new_pose(V,translation);
        dataloader.wirteTransformer(transform_filename,V,translation);

	}
    
	for (int i=0;i<LINE_NUM;i++)
	{
        double x=rand() % 5+30;
        double y=rand() % 5+30;
        double z=rand() % 5+30;
        double x1=rand() % 5+30;
        double y1=rand() % 5+30;
        double z1=rand() % 5+30;
	    if(cam1.add_line(x,y,z,x1,y1,z1))
        {
            dataloader.writeLine(x,y,z,x1,y1,z1);
        }
        else
        {
            i=i-1;
        }
	}
        for (int i=0;i<LINE_NUM;i++)
	{
	    for(int j=0;j<WINDOW_SIZE + 1;j++)
		{
		    //if((rand() % 2)==1)
			//{
			    cam1.set_observation(i,j);
			//}
		}
	}
    
	//初始化
	LineTriangulate line_triangulate(cam1,para_line_depth,para_Pose);
	line_triangulate.leastsquare_depth();
    dataload.writeTriangulate("line_depth_"+triangulate_filename,para_line_depth,cam1,para_Pose[0]);

    /*
    LineTriangulate line_triangulate(cam1,para_line,para_Pose);
    line_triangulate.avg_plucker();
    line_triangulate.leastsquare_plucker();
    dataload.writeTriangulate("line_"+triangulate_filename,para_line,cam1,para_Pose[0]);
    */
	
    //构建寻优问题
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);//couchy核函数
	
	for (int i = 0; i < WINDOW_SIZE + 1; i++)//添加所有的优化变量
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();//这个参数是告诉求解器这个是个单元四元数
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);//Ps、Rs转变成para_Pose
    }
    
	//camera到IMU的外参也添加到估计
    for (int i = 0; i < NUM_OF_CAM; i++) {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
    
    }
	
	
	
    
    int line_m_cnt = 0;
	int line_feature_index = -1;
    int imu_i=0;
    int imu_j=0;
	for (int i=0;i<LINE_NUM;i++)//遍历滑窗内所有的空间线
    {

        ++line_feature_index;
        for (int j=0;j<WINDOW_SIZE ;j++){
		     if(cam1.windows[i][j]>0)
			 {
		            imu_i = j;
                    
			        Vector3d line_i_s,line_i_e;
		            line_i_s<<cam1.camera_observation[i][imu_i].point1.x,cam1.camera_observation[i][imu_i].point1.y,cam1.camera_observation[i][imu_i].point1.z;
		            line_i_e<<cam1.camera_observation[i][imu_i].point2.x,cam1.camera_observation[i][imu_i].point2.y,cam1.camera_observation[i][imu_i].point2.z;
                    for (int imu_j;imu_j<WINDOW_SIZE + 1;imu_j++)//遍历空间点在所有可观测的图像帧上的线
           {
		    if(cam1.windows[i][imu_j]>0)
			{ 
		
               Vector3d line_j_s,line_j_e;
		       line_j_s<<cam1.camera_observation[i][imu_j].point1.x,cam1.camera_observation[i][imu_j].point1.y,cam1.camera_observation[i][imu_j].point1.z;
		       line_j_e<<cam1.camera_observation[i][imu_j].point2.x,cam1.camera_observation[i][imu_j].point2.y,cam1.camera_observation[i][imu_j].point2.z;
               Eigen::Vector3d t0(para_Pose[imu_i][0], para_Pose[imu_i][1], para_Pose[imu_i][2]);
               Eigen::Quaterniond q0(para_Pose[imu_i][6], para_Pose[imu_i][3], para_Pose[imu_i][4], para_Pose[imu_i][5]);

               Eigen::Vector3d t1(para_Pose[imu_j][0], para_Pose[imu_j][1], para_Pose[imu_j][2]);
               Eigen::Quaterniond q1(para_Pose[imu_j][6], para_Pose[imu_j][3], para_Pose[imu_j][4], para_Pose[imu_j][5]);
               //line_triangulate(line_i_s,line_i_e,line_j_s,line_j_e,t0,t1,q0.toRotationMatrix(),q1.toRotationMatrix(),imu_i);
               LineDepthProjectionFactor *l = new LineDepthProjectionFactor(line_i_s,line_i_e,line_j_s,line_j_e);//这里面会计算残差以及残差对优化变量雅克比矩阵
               problem.AddResidualBlock(l, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_line_depth[line_feature_index]);//这里指定了相关的优化变量
               /*
               LineProjectionFactor *l = new LineProjectionFactor(line_i_s,line_i_e,line_j_s,line_j_e);//这里面会计算残差以及残差对优化变量雅克比矩阵
               problem.AddResidualBlock(l, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_line[line_feature_index]);//这里指定了相关的优化变量              
               */
               line_m_cnt++;					           
            }
		}
			 }
		}
        

               


        
        
    }
    cout<<line_m_cnt<<endl;
    //配置并运行求解器
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    options.minimizer_progress_to_stdout = true;//输出到cout
    
    Solver::Summary summary;//优化信息
    ceres::Solve(options, &problem, &summary);//求解!!!
    std::cout << summary.BriefReport() << endl;//输出优化的简要信息
    for (int i=0;i<WINDOW_SIZE + 1;i++)
    {
         Eigen::Quaterniond Qi(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]);
         Eigen::Vector3d Pi(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
         Eigen::Matrix3d Ri = Qi.toRotationMatrix();
         cout<<"Ri"<<Ri<<endl;
         cout<<"Pi"<<Pi<<endl;

    }
    for (int i=0;i<LINE_NUM;i++)
    {
         
         cout<<"inv1:"<<para_line_depth[i][0]<<endl;
         cout<<"inv2:"<<para_line_depth[i][1]<<endl;

    }   
     

     
    return 0;
}
