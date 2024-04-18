#include "camera.h"


 void camera::add_line(double x,double y,double z,double x1,double y1,double z1)
    {
        line new_line;
        if(x==x1&&y==y1&&z==z1)
        {
            cout<<"input point is equal"<<endl;
            return; 
        }
        new_line.point1.x=x;
        new_line.point1.y=y;
        new_line.point1.z=z;
        new_line.point2.x=x1;
        new_line.point2.y=y1;
        new_line.point2.z=z1;
        lines.push_back(new_line);
    }
    void camera::new_pose(Eigen::AngleAxisd V,Eigen::Vector3d translation)
    {
        Eigen::Isometry3d new_t=Isometry3d::Identity();
        new_t.pretranslate(translation);
        new_t.rotate(V);
        T.push_back(new_t);
        Matrix3d Rs=new_t.rotation();
	    std::cout <<"+++"<<T.size()<<endl<< new_t.matrix()<<endl;
        para_Pose[T.size()-1][0] = translation.x();
        para_Pose[T.size()-1][1] = translation.y();
        para_Pose[T.size()-1][2] = translation.z();
        Quaterniond q(Rs);
        para_Pose[T.size()-1][3] = q.x();
        para_Pose[T.size()-1][4] = q.y();
        para_Pose[T.size()-1][5] = q.z();
        para_Pose[T.size()-1][6] = q.w();
    }

    void camera::set_observation(int i,int j)
    {
        if(i>=lines.size()||j>=T.size())
        {
            cout<<"输入越界"<<endl;
            return;
        }
        windows[i][j]=1;
        line line_now=lines[i];
        Eigen::Isometry3d T_now=T[j];
        line observation;
        Eigen::Vector3d observation1,observation2;
        observation1<<line_now.point1.x,line_now.point1.y,line_now.point1.z;
        observation2<<line_now.point2.x,line_now.point2.y,line_now.point2.z;
        observation1=T_now*observation1;
        observation2=T_now*observation2;
        if(j==0)
        {
            cout<<"line-num"<<i<<":"<<(observation1.cross(observation2).norm())/((observation1-observation2).norm())<<endl;
            cout<<"line-numd"<<i<<":"<<(observation1-observation2).normalized()<<endl;
        }
        observation1=(1/observation1[2])*observation1;    
        observation2=(1/observation2[2])*observation2;
        //cout<<line_now.point2.x<<" "<<line_now.point2.y<<" "<<line_now.point2.z<<endl;
        //std::cout <<"+++"<<endl<< T_now.matrix() << std::endl<<K.matrix()<<"+++"<<endl;

        /*if(observation1[0]>width||observation1[1]>height||observation2[0]>width||observation2[1]>height)
        {
            cout<<i<<" "<<j<<endl;
            cout<<"不能完整的检测所有点"<<endl;
            //cout<<"观测点1:   "<<observation1[0]<<"   "<<observation1[1]<<"  "<<observation1[2]<<endl;
            //cout<<"观测点2:   "<<observation2[0]<<"   "<<observation2[1]<<"  "<<observation2[2]<<endl;
            //return ;
        }*/
        line line1;
        line1.point1.x=observation1[0];
        line1.point1.y=observation1[1];
        line1.point1.z=observation1[2];
        line1.point2.x=observation2[0];
        line1.point2.y=observation2[1];
        line1.point2.z=observation2[2];
        camera_observation[i][j]=line1;
        //std::cout << T_now.matrix() << std::endl;
        /*cout<<"观测点1:   "<<observation1[0]<<"   "<<observation1[1]<<"  "<<observation1[2]<<endl;
        cout<<"观测点2:   "<<observation2[0]<<"   "<<observation2[1]<<"  "<<observation2[2]<<endl;
        cout<<"观测点1camera:   "<<camera_observation[i][j].point1.x<<"   "<<camera_observation[i][j].point1.y<<"  "<<camera_observation[i][j].point1.z<<endl;
        cout<<"观测点2camera:   "<<camera_observation[i][j].point2.x<<"   "<<camera_observation[i][j].point2.y<<"  "<<camera_observation[i][j].point2.z<<endl;*/

    }