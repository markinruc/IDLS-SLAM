#include "line_triangulate.h"

/*
 三点确定一个平面 a(x-x0)+b(y-y0)+c(z-z0)=0  --> ax + by + cz + d = 0   d = -(ax0 + by0 + cz0)
 平面通过点（x0,y0,z0）以及垂直于平面的法线（a,b,c）来得到
 (a,b,c)^T = vector(AO) cross vector(BO)
 d = O.dot(cross(AO,BO))
 */
Vector4d pi_from_ppp(Vector3d x1, Vector3d x2, Vector3d x3) {
    Vector4d pi;
    pi << ( x1 - x3 ).cross( x2 - x3 ), - x3.dot( x1.cross( x2 ) ); // d = - x3.dot( (x1-x3).cross( x2-x3 ) ) = - x3.dot( x1.cross( x2 ) )

    return pi;
}

void LineTriangulate::avg_plucker()
{
    /*普吕克矩阵分解*/
    int imu_i=0;	
    for (int i=0;i<LINE_NUM;i++)//遍历滑窗内所有的空间线
    {
        Eigen::Vector3d line_i_s;Eigen::Vector3d line_i_e;
        line_i_s<<cam1.camera_observation[i][imu_i].point1.x,cam1.camera_observation[i][imu_i].point1.y,cam1.camera_observation[i][imu_i].point1.z;
	    line_i_e<<cam1.camera_observation[i][imu_i].point2.x,cam1.camera_observation[i][imu_i].point2.y,cam1.camera_observation[i][imu_i].point2.z;
        Eigen::Vector3d t0(para_Pose[imu_i][0], para_Pose[imu_i][1], para_Pose[imu_i][2]);
        Eigen::Quaterniond q0(para_Pose[imu_i][6], para_Pose[imu_i][3], para_Pose[imu_i][4], para_Pose[imu_i][5]);
        Eigen::Matrix3d R0=q0.toRotationMatrix();
        vector<double> d_ei;
        vector<Vector3d> v_ei;
        vector<Vector3d> n_ei;
        Eigen::Vector3d t_0;
        t_0<<0,0,0;
        Vector4d pi_0=pi_from_ppp(line_i_s,line_i_e,t_0);
        for (int imu_j = imu_i + 1;imu_j<WINDOW_SIZE + 1;imu_j++)//遍历空间点在所有可观测的图像帧上的线
        {
		    if(cam1.windows[i][imu_j]>0)
			{ 
		
                Vector3d line_j_s,line_j_e;
		        line_j_s<<cam1.camera_observation[i][imu_j].point1.x,cam1.camera_observation[i][imu_j].point1.y,cam1.camera_observation[i][imu_j].point1.z;
		        line_j_e<<cam1.camera_observation[i][imu_j].point2.x,cam1.camera_observation[i][imu_j].point2.y,cam1.camera_observation[i][imu_j].point2.z;
               			   
                Eigen::Vector3d t1(para_Pose[imu_j][0], para_Pose[imu_j][1], para_Pose[imu_j][2]);
                Eigen::Quaterniond q1(para_Pose[imu_j][6], para_Pose[imu_j][3], para_Pose[imu_j][4], para_Pose[imu_j][5]);
			    Eigen::Matrix3d R1=q1.toRotationMatrix();
			   
			    Eigen::Vector3d t = R0.transpose() * (t1 - t0);//R和t是从第i帧到第j帧的变换位姿
                Eigen::Matrix3d R = R0.transpose() * R1;
                

                //计算起始帧上直线构成的平面

                //计算结束帧上直线构成的平面
                line_j_s=R*line_j_s+t;
                line_j_e=R*line_j_e+t;
               

                Vector4d pi_1=pi_from_ppp(line_j_s,line_j_e,t);

                Matrix4d matrix_pu = pi_0*pi_1.transpose() - pi_1*pi_0.transpose();

                Vector3d pu_n, pu_v;
                pu_n = matrix_pu.block<3, 1>(0, 3);
                pu_v << -matrix_pu(1, 2), matrix_pu(0, 2), -matrix_pu(0, 1);
                if(line_para_depth)
                {
                    Eigen::Matrix<double, 3, 3> A;
	                A.block<3,1>(0,0)=line_i_e;
	                A.block<3,1>(0,1)=line_i_s;
	                A.block<3,1>(0,2)=pu_v;
                    JacobiSVD<Eigen::MatrixXd> svd(A, ComputeThinU | ComputeThinV );
                    Matrix3d V = svd.matrixV(), U = svd.matrixU();
                    Matrix3d S = U.inverse() * A * V.transpose().inverse();
                    double z1=V(0,2);
                    double z2=V(1,2);
                    if (z1*z2>0)
                    {
                        if(z1>0)
                        {
                            para_depth_line[i][0]=1/z1;
                            para_depth_line[i][1]=1/z2;
                        }
                        else
                        {
                           para_depth_line[i][0]=-1/z1;
                           para_depth_line[i][1]=-1/z2;
                        }
                    }

                }
                else
                {
                    double d=pu_n.norm()/pu_v.norm();
                    pu_n=pu_n.normalized();
                    pu_v=pu_v.normalized();
                    v_ei.push_back(pu_v);
                    n_ei.push_back(pu_n);
                    d_ei.push_back(d);

                }
                
	          
            }
        }
        if(line_para_depth)
        {
            return;
        }
        else
        {
            Vector3d sum_n(0,0,0);
            Vector3d sum_v(0,0,0);
            double sum_d=0;
            for(int k=0;k<d_ei.size();k++)
           {
               sum_d=sum_d+d_ei[k];
               sum_n=sum_n+n_ei[k];
               sum_v=sum_v+v_ei[k];
           }
           sum_d=sum_d/d_ei.size();
           sum_v=sum_v/v_ei.size();
           sum_n=sum_n/n_ei.size();
           Utility::cvtPluckerToOrthonormal(sum_d*sum_n,sum_v , para_line[i]);
        }

      
		
		
		/*		
		,pw_n,pw_d,pj_n,pj_d
        pw_n=R0*pu_n+Utility::skewSymmetric(t0)*R0*pu_d;
	    pw_d=R0*pu_d;
		
	pj_n=R1.transpose()*pw_n-R1.transpose()*Utility::skewSymmetric(t1)*pw_d;
	pj_d=R1.transpose()*pw_d;
        cout<<"di"<<pu_n.norm()/pu_d.norm()<<endl;
	//cout<<"di'"<<(ptsi.cross(ptei)).norm()/(ptei-ptsi).norm()<<endl;*/
    }
}



void LineTriangulate::leastsquare_depth()
{	
   
   int imu_i=0;		
   avg_plucker();
   for (int i=0;i<LINE_NUM;i++)//遍历滑窗内所有的空间线
    {
	
        ceres::Problem class_problem_line;
		double line_depth[2]={1,1};
        Vector3d line_i_s,line_i_e;

		line_depth[0]=1/para_depth_line[i][0];
        line_depth[1]=1/para_depth_line[i][1];		        
	    line_i_s<<cam1.camera_observation[i][imu_i].point1.x,cam1.camera_observation[i][imu_i].point1.y,cam1.camera_observation[i][imu_i].point1.z;
	    line_i_e<<cam1.camera_observation[i][imu_i].point2.x,cam1.camera_observation[i][imu_i].point2.y,cam1.camera_observation[i][imu_i].point2.z;
        Eigen::Vector3d t0(para_Pose[imu_i][0], para_Pose[imu_i][1], para_Pose[imu_i][2]);
        Eigen::Quaterniond q0(para_Pose[imu_i][6], para_Pose[imu_i][3], para_Pose[imu_i][4], para_Pose[imu_i][5]);
         Eigen::Matrix3d R0=q0.toRotationMatrix();
        for (int imu_j = imu_i + 1;imu_j<WINDOW_SIZE + 1;imu_j++)//遍历空间点在所有可观测的图像帧上的线
        {
		    if(cam1.windows[i][imu_j]>0)
			{ 
		
                Vector3d line_j_s,line_j_e;
		        line_j_s<<cam1.camera_observation[i][imu_j].point1.x,cam1.camera_observation[i][imu_j].point1.y,cam1.camera_observation[i][imu_j].point1.z;
		        line_j_e<<cam1.camera_observation[i][imu_j].point2.x,cam1.camera_observation[i][imu_j].point2.y,cam1.camera_observation[i][imu_j].point2.z;
               
			   
               Eigen::Vector3d t1(para_Pose[imu_j][0], para_Pose[imu_j][1], para_Pose[imu_j][2]);
               Eigen::Quaterniond q1(para_Pose[imu_j][6], para_Pose[imu_j][3], para_Pose[imu_j][4], para_Pose[imu_j][5]);
			   Eigen::Matrix3d R1=q1.toRotationMatrix();
			   
			    Eigen::Vector3d t = R0.transpose() * (t1 - t0);//R和t是从第i帧到第j帧的变换位姿
                Eigen::Matrix3d R = R0.transpose() * R1;
            


                ceres::CostFunction* cost_function=new LineCostFunction(line_i_s,line_i_e,line_j_s,line_j_e,t,R);
                class_problem_line.AddResidualBlock(cost_function,nullptr,line_depth);
			   /*
               class_problem_line.AddResidualBlock(     // 向问题中添加误差项
                             // 使用自动求导，将定义的代价函数结构体传入。模板参数：误差类型，输出维度即残差的维度，输入维度即优化参数的维度，维数要与前面struct中一致
                             new ceres::AutoDiffCostFunction<LINE_FITTING_COST, 2, 2>(
                             new LINE_FITTING_COST(line_i_s,line_i_e,line_j_s,line_j_e,t,R)
                              ),
                             nullptr,            // 核函数，这里不使用，为空
                             line_depth                 // 待估计参数
                             );
				*/
               				           
            }
		}
			
        

        ceres::Solver::Options options_line;     // 这里有很多配置项可以填
        options_line.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
        options_line.minimizer_progress_to_stdout = true;   // 输出到cout
        ceres::Solver::Summary summary_line;
        ceres::Solve(options_line, &class_problem_line, &summary_line);	
        cout << summary_line.BriefReport() << endl;   //输出优化的简要信息		
        //放入初始化矩阵
		if (line_depth[0]>1&&line_depth[1]>1)
		{
		    para_depth_line[i][0]=1/line_depth[0];
		    para_depth_line[i][1]=1/line_depth[1];
		}
        
        
    }    
               								
      

}
void LineTriangulate::leastsquare_plucker(){
    line_para_depth=false;
    avg_plucker();
    int imu_i=0;
		
    for (int i=0;i<LINE_NUM;i++)//遍历滑窗内所有的空间线
    {
	
        ceres::Problem class_problem_line_dir;
        ceres::Problem class_problem_line_depth;
        double line_direction[3]={1,1,1};
        double line_depth[1]={1};
        double orth[5];
        Vector3d n,v;
        for(int k=0;k<5;k++)
        {
            orth[k]=para_line[i][k];
        }
        Utility::cvtOrthonormalToPlucker(orth, n, v);
        line_depth[0]=n.norm()/v.norm();
        v=v.normalized();
        line_direction[0]=v.x();
        line_direction[1]=v.y();
        line_direction[2]=v.z();
        Vector3d line_i_s,line_i_e;

			        
	    line_i_s<<cam1.camera_observation[i][imu_i].point1.x,cam1.camera_observation[i][imu_i].point1.y,cam1.camera_observation[i][imu_i].point1.z;
	    line_i_e<<cam1.camera_observation[i][imu_i].point2.x,cam1.camera_observation[i][imu_i].point2.y,cam1.camera_observation[i][imu_i].point2.z;
        Eigen::Vector3d t0(para_Pose[imu_i][0], para_Pose[imu_i][1], para_Pose[imu_i][2]);
        Eigen::Quaterniond q0(para_Pose[imu_i][6], para_Pose[imu_i][3], para_Pose[imu_i][4], para_Pose[imu_i][5]);
         Eigen::Matrix3d R0=q0.toRotationMatrix();
        for (int imu_j = imu_i + 1;imu_j<WINDOW_SIZE + 1;imu_j++)//遍历空间点在所有可观测的图像帧上的线
        {
		    if(cam1.windows[i][imu_j]>0)
			{ 
		
                Vector3d line_j_s,line_j_e;
		        line_j_s<<cam1.camera_observation[i][imu_j].point1.x,cam1.camera_observation[i][imu_j].point1.y,cam1.camera_observation[i][imu_j].point1.z;
		        line_j_e<<cam1.camera_observation[i][imu_j].point2.x,cam1.camera_observation[i][imu_j].point2.y,cam1.camera_observation[i][imu_j].point2.z;
               
			   
               Eigen::Vector3d t1(para_Pose[imu_j][0], para_Pose[imu_j][1], para_Pose[imu_j][2]);
               Eigen::Quaterniond q1(para_Pose[imu_j][6], para_Pose[imu_j][3], para_Pose[imu_j][4], para_Pose[imu_j][5]);
			   Eigen::Matrix3d R1=q1.toRotationMatrix();
			   
			    Eigen::Vector3d t = R0.transpose() * (t1 - t0);//R和t是从第i帧到第j帧的变换位姿
                Eigen::Matrix3d R = R0.transpose() * R1;
                ceres::CostFunction* cost_function_dir=new LinedirCostFunction(line_j_s,line_j_e,R);
                class_problem_line_dir.AddResidualBlock(cost_function_dir,nullptr,line_direction);
                ceres::CostFunction* cost_function_depth=new LinedepthCostFunction(line_i_s,line_i_e,line_j_s,line_j_e,t,R,line_direction);                          
                class_problem_line_depth.AddResidualBlock(cost_function_depth,nullptr,line_depth);
            }    
        }
        ceres::Solver::Options options_line_dir;     // 这里有很多配置项可以填
        options_line_dir.linear_solver_type =ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
        //options_line_dir.minimizer_progress_to_stdout = true;   // 输出到cout
        ceres::Solver::Summary summary_line_dir;
        ceres::Solve(options_line_dir, &class_problem_line_dir, &summary_line_dir);	
        //cout << summary_line_dir.BriefReport() << endl;   //输出优化的简要信息
        ceres::Solver::Options options_line_depth;     // 这里有很多配置项可以填
        options_line_depth.linear_solver_type =ceres::DENSE_NORMAL_CHOLESKY;
        //options_line_depth.minimizer_progress_to_stdout = true;

        ceres::Solver::Summary summary_line_depth;
        ceres::Solve(options_line_depth, &class_problem_line_depth, &summary_line_depth);	
        //cout << summary_line_depth.BriefReport() << endl;   //输出优化的简要信息 		
        Vector3d pu_n=line_i_s.cross(line_i_e).normalized()*line_depth[0];
	    Vector3d pu_d=Vector3d(line_direction[0],line_direction[1],line_direction[2]).normalized();
        cout<<i<<"line-num"<<pu_n.norm()/pu_d.norm()<<endl;
        cout<<i<<"line-num"<<pu_d<<endl;
        Utility::cvtPluckerToOrthonormal(pu_n,pu_d , para_line[i]);
    }
    
}