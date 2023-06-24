# include"eigentest.h"

int main()
{
    Eigen::Quaterniond q1(0.35,0.2,0.3,0.1),q2(-0.5,0.4,-0.1,0.2);//四元数姿态,相对于世界坐标系，由世界坐标系经过变换获得。
    q1.normalize(),q2.normalize();//所有的四元数，在变换前，或在构造变换矩阵前需要归一化，确保其旋转矩阵是正交矩阵，变换是正交变换。
    Eigen::Vector3d t1(0.3,0.1,0.1),t2(-0.1,0.5,0.3);//位移向量

    Eigen::Isometry3d TR1W(q1);//构造变换矩阵，由于q1和t1是在世界坐标系上，所以构造出来的是由世界坐标系到R1坐标系的变换矩阵，倒着念
    TR1W.pretranslate(t1);

    Eigen::Isometry3d TR2W(q2);//构造变换矩阵，由于q2和t2是在世界坐标系上，所以构造出来的是由世界坐标系到R2坐标系的变换矩阵
    TR1W.pretranslate(t2);

    Eigen::Vector3d p(0.5,0,0.2);

    Eigen::Vector3d pout=TR2W*TR1W.inverse()*p;// TR1W.inverse()是TR1W的逆，表示是由R1坐标系到世界坐标系的变换矩阵
    cout<<"p在R1坐标系上位置：p="<<p.transpose()<<endl;
    cout<<"p在R2坐标系上位置：pout="<<pout.transpose()<<endl;


}