#include "line_projection_factor.h"
//information matrix
Eigen::Matrix2d LineProjectionFactor::sqrt_info=Matrix2d::Identity();

LineProjectionFactor::LineProjectionFactor(const Vector3d &_line_i_s, const Vector3d &_line_i_e,const Vector3d &_line_j_s,const Vector3d &_line_j_e)
        : line_i_s(_line_i_s),line_i_e(_line_i_e),line_j_s(_line_j_s),line_j_e(_line_j_e)
{
};

bool LineProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
    Eigen::Matrix3d Ri = Qi.toRotationMatrix();
    Eigen::Matrix3d Rj = Qj.toRotationMatrix();
    Eigen::Matrix3d ric = qic.toRotationMatrix();
	double orth[5];
    for (int i = 0; i < 5; ++i)
    {
        orth[i] = parameters[3][i];
    }
    Eigen::Quaterniond qW(parameters[3][3], parameters[3][0], parameters[3][1], parameters[3][2]);
    double phi = parameters[3][4];

    //i时刻相机坐标系下的map line普吕克坐标
	Eigen::Vector3d Lci_n, Lci_d;
	Utility::cvtOrthonormalToPlucker(orth, Lci_n, Lci_d);
        //cout<<"n"<<Lci_n<<"d"<<Lci_d<<endl;
	//i时刻IMU坐标系下的map line普吕克坐标
	Eigen::Vector3d Lbi_n, Lbi_d;
	Lbi_n=ric*Lci_n+Utility::skewSymmetric(tic)*ric*Lci_d;
	Lbi_d=ric*Lci_d;
	//世界坐标系下的map line普吕克坐标
	Eigen::Vector3d Lw_n, Lw_d;
	Lw_n=Ri*Lbi_n+Utility::skewSymmetric(Pi)*Ri*Lbi_d;
	Lw_d=Ri*Lbi_d;
	//在j时刻imu坐标系下的map line普吕克坐标
	Eigen::Vector3d Lbj_n, Lbj_d;
	Lbj_n=Rj.transpose()*Lw_n-Rj.transpose()*Utility::skewSymmetric(Pj)*Lw_d;
	Lbj_d=Rj.transpose()*Lw_d;
	//在j时刻相机坐标系下的map line普吕克坐标
	Eigen::Vector3d Lcj_n, Lcj_d;
	Lcj_n=ric.transpose()*Lbj_n-ric.transpose()*Utility::skewSymmetric(tic)*Lbj_d;
	Lcj_d=ric.transpose()*Lbj_d;
	//计算残差边
        Eigen::Map<Eigen::Vector2d> residual(residuals);
         residual = Eigen::Vector2d(line_j_s.dot(Lcj_n), line_j_e.dot(Lcj_n))/Lcj_n.head(2).norm();
        //std::cout<<"residual"<<std::endl<<residual<<std::endl;        	       
	residual = sqrt_info*residual;
        //std::cout<<"residual"<<std::endl<<residual<<std::endl;
	//计算雅可比
	if (jacobians)
    {

        Eigen::Matrix<double, 2, 6> reduce;
        Eigen::Matrix<double, 2, 3> reduce_1;
        Eigen::Matrix<double, 3, 6> reduce_2;
        double l1 = Lcj_n(0);
        double l2 = Lcj_n(1);
        double l3 = Lcj_n(2);
        double u1 = line_j_s(0);
        double v1 = line_j_s(1);
        double u2 = line_j_e(0);
        double v2 = line_j_e(1);

        double l1l2_23 = Lcj_n.head(2).norm()*(l1*l1 + l2*l2);
        reduce_1 << (u1*l2*l2 - l1*l2*v1 - l1*l3)/l1l2_23, (v1*l1*l1 - l1*l2*u1 - l2*l3)/l1l2_23, 1/Lcj_n.head(2).norm(),
                (u2*l2*l2 - l1*l2*v2 - l1*l3)/l1l2_23, (v2*l1*l1 - l1*l1*v2 - l2*l3)/l1l2_23, 1/Lcj_n.head(2).norm();
        //std::cout<<"reduce_1"<<std::endl<<reduce_1<<std::endl;
        reduce_2.setZero();
        reduce_2.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        //std::cout<<"reduce_2"<<std::endl<<reduce_2<<std::endl;
        reduce = reduce_1*reduce_2;
        //std::cout<<"reduce"<<std::endl<<reduce<<std::endl;
        reduce = sqrt_info*reduce;
        //std::cout<<"reduce"<<std::endl<<reduce<<std::endl;
		
		if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Eigen::Matrix<double, 6, 6> jaco_i;
	        jaco_i.setZero();
            jaco_i.block<3,3>(0,0) = -ric.transpose() * Rj.transpose()*Utility::skewSymmetric(Lw_d);
            jaco_i.block<3,3>(0,3) = -ric.transpose() * Rj.transpose()*Ri*Utility::skewSymmetric(Lbi_n)-ric.transpose() * Rj.transpose()*Utility::skewSymmetric(Pi)*Ri*Utility::skewSymmetric(Lbi_d)+ric.transpose() * Rj.transpose()*Utility::skewSymmetric(Pj)*Ri*Utility::skewSymmetric(Lbi_d)+ric.transpose() * Utility::skewSymmetric(tic)*Rj.transpose()*Ri*Utility::skewSymmetric(Lbi_d);
			jaco_i.block<3,3>(3,3) = -ric.transpose() * Rj.transpose()*Ri*Utility::skewSymmetric(Lbi_d);
            //std::cout<<"jaco_i"<<std::endl<<jaco_i<<std::endl;
            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;         
            jacobian_pose_i.rightCols<1>().setZero();
            //std::cout<<"jacobian_pose_i"<<std::endl<<jacobian_pose_i<<std::endl;
        }
		if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

            Eigen::Matrix<double, 6, 6> jaco_j;
			jaco_j.setZero();
            jaco_j.block<3,3>(0,0) = ric.transpose() * Rj.transpose()*Utility::skewSymmetric(Lw_d);
            jaco_j.block<3,3>(0,3) = ric.transpose() * Utility::skewSymmetric(Lbj_n)-ric.transpose()*Utility::skewSymmetric(tic)*Utility::skewSymmetric(Lbj_d);
			jaco_j.block<3,3>(3,3) = ric.transpose() * Utility::skewSymmetric(Lbj_d);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }
		if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);
            Eigen::Matrix<double, 6, 6> jaco_ex;
            jaco_ex.setZero();
            jaco_ex.block<3,3>(0,0) = -ric.transpose() * Rj.transpose()*Ri*Utility::skewSymmetric(Lbi_d)+ric.transpose()*Utility::skewSymmetric(Lbj_d);
            jaco_ex.block<3,3>(0,3) = -ric.transpose() * Rj.transpose()*Ri*ric*Utility::skewSymmetric(Lci_n)+Utility::skewSymmetric(ric.transpose() * Rj.transpose()*Ri*ric*Lci_n)
			                          -ric.transpose() * Rj.transpose()*Ri*Utility::skewSymmetric(tic)*ric*Utility::skewSymmetric(Lci_d)+Utility::skewSymmetric(ric.transpose() * Rj.transpose()*Ri*Utility::skewSymmetric(tic)*ric*Lci_d)
									  -ric.transpose() * Rj.transpose()*Utility::skewSymmetric(Pi)*Ri*ric*Utility::skewSymmetric(Lci_d)+Utility::skewSymmetric(ric.transpose() * Rj.transpose()*Utility::skewSymmetric(Pi)*Ri*ric*Lci_d)
									  +ric.transpose() * Rj.transpose()*Utility::skewSymmetric(Pj)*Ri*ric*Utility::skewSymmetric(Lci_d)-Utility::skewSymmetric(ric.transpose() * Rj.transpose()*Utility::skewSymmetric(Pj)*Ri*ric*Lci_d)
									  +ric.transpose() *Utility::skewSymmetric(tic)* Rj.transpose()*Ri*ric*Utility::skewSymmetric(Lci_d)-Utility::skewSymmetric(ric.transpose() *Utility::skewSymmetric(tic)* Rj.transpose()*Ri*ric*Lci_d);
			jaco_ex.block<3,3>(3,3) = -ric.transpose() * Rj.transpose()*Ri *ric* Utility::skewSymmetric(Lci_d)+ Utility::skewSymmetric(Lcj_d);

            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[3])
        {
            //准备工作
            double w1 = std::cos(phi);
            double w2 = std::sin(phi);
            Eigen::Vector3d U1 = Lci_n.normalized();
            Eigen::Vector3d U2 = Lci_d.normalized();
            Eigen::Vector3d U3 = Lci_n.cross(Lci_d).normalized();


            Eigen::Map<Eigen::Matrix<double, 2, 5, Eigen::RowMajor>> jacobian_line(jacobians[3]);
            Eigen::Matrix<double, 6, 6> invTbc;
            invTbc << ric.transpose(), -ric.transpose()*Utility::skewSymmetric(tic),
                    Eigen::Matrix3d::Zero(),  ric.transpose();
            Eigen::Matrix<double, 6, 6> invTwbj;
            invTwbj << Rj.transpose(), -Rj.transpose()*Utility::skewSymmetric(Pj),
                    Eigen::Matrix3d::Zero(),  Rj.transpose();
            Eigen::Matrix<double, 6, 6> Twbi;
		    Twbi << Ri, Utility::skewSymmetric(Pj)*Ri,
                    Eigen::Matrix3d::Zero(),  Ri;
            Eigen::Matrix<double, 6, 6> Tbc;
	        Tbc << ric, Utility::skewSymmetric(tic)*ric.transpose(),
                    Eigen::Matrix3d::Zero(),  ric;
            Eigen::Matrix<double, 6, 5> jaco_l;
            jaco_l.setZero();
            jaco_l.block<3, 1>(3, 0) = w2*U3;
            jaco_l.block<3, 1>(0, 1) = -w1*U3;
            jaco_l.block<3, 1>(0, 2) = w1*U2;
            jaco_l.block<3, 1>(3, 2) = -w2*U1;
            jaco_l.block<3, 1>(0, 4) = -w2*U1;
            jaco_l.block<3, 1>(3, 4) = w1*U2;
            //std::cout<<"jaco_l"<<std::endl<<jaco_l<<std::endl;
            jacobian_line = reduce*invTbc*invTwbj*Twbi*Tbc*jaco_l;
        }				
	}
	return true;
};
void LineProjectionFactor::check(double **parameters)   
{
}
