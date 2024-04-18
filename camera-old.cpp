#include<iostream>
#include<bits/stdc++.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
using namespace std;
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
    camera()
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
        vector<int> a(10,0);
        vector<line> smallline(10,new_line);
        windows.resize(10,a);
        camera_observation.resize(10,smallline);
    }
    void add_line(double x,double y,double z,double x1,double y1,double z1)
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
    void new_pose(Eigen::AngleAxisd V,Eigen::Vector3d translation)
    {
        Eigen::Isometry3d new_t;
        new_t.pretranslate(translation);
        new_t.rotate(V);
        T.push_back(new_t);
    }

    void set_observation(int i,int j)
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
        observation1=(1/line_now.point1.z)*K*(T_now*observation1);
        observation2=(1/line_now.point2.z)*K*(T_now*observation2);
        if(observation1[0]>width||observation1[1]>height||observation2[0]>width||observation2[1]>height)
        {
            cout<<"不能完整的检测所有点"<<endl;
            return ;
        }
        line line1;
        line1.point1.x=observation1[0];
        line1.point1.y=observation1[1];
        line1.point1.z=observation1[2];
        line1.point2.x=observation2[0];
        line1.point2.y=observation2[1];
        line1.point2.z=observation2[2];
        camera_observation[i][j]=line1;
        cout<<"观测点1:   "<<observation1[0]<<"   "<<observation1[1]<<"  "<<observation1[2]<<endl;
        cout<<"观测点2:   "<<observation2[0]<<"   "<<observation2[1]<<"  "<<observation2[2]<<endl;
    }

};

int main()
{
    camera cam1;
    Eigen::AngleAxisd V1(3.1415926 / 4, Eigen::Vector3d(1, 0, 0).normalized());
    Eigen::Vector3d translation1(1, 0, 0);
    Eigen::AngleAxisd V2(0.0, Eigen::Vector3d(1, 0, 0).normalized());
    Eigen::Vector3d translation2(0, 0, 0);
    cam1.add_line(1,1,1,2,2,2);
    cam1.new_pose(V1,translation1);
    cam1.new_pose(V2,translation2);
    cam1.set_observation(0,0);
    cam1.set_observation(0,1);
    return 0;
}
