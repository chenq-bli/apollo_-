# include"eigentest.h"

int main()
{
    Eigen::Matrix<float,3,3> matrix_33;
    matrix_33<<1,2,3,4,5,6,7,8,9;
    Eigen::Matrix3d matrix_33zero=Eigen::Matrix3d::Zero(3,3);
    Eigen::Matrix3d matrix_33random=Eigen::Matrix3d::Random(3,3);
    // for(int i=0; i<3; i++)
    // {
    //     for(int j=0;j<3;j++)
    //     {
    //         cout<<matrix_33(i,j)<<"\t";
    //     }
    //     cout<<endl;
    // }


    // //求解特征向量
        // for(int i=0; i<3; i++)
    // {
    //     for(int j=0;j<3;j++)
    //     {
    //         cout<<matrix_33random(i,j)<<"\t";
    //     }
    //     cout<<endl;
    // }
    //  cout<<endl;
    //  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>  eigenvalue_solve1(matrix_33random.transpose()*matrix_33random);
    //  cout<<eigenvalue_solve1.eigenvalues()<<endl;
    //  cout<<endl;
    //  cout<<eigenvalue_solve1.eigenvectors()<<endl;
    

    // //求解方程组
    // Eigen::Matrix<double,3,3> A=Eigen::MatrixXd::Random(3,3);//Matrix<double,3,3>中必须是都double类型，因为MatrixXd中默认定义了是double
    // for(int i=0; i<3; i++)
    // {
    //     for(int j=0;j<3;j++)
    //     {
    //         cout<<A(i,j)<<"\t";
    //     }
    //     cout<<endl;
    // }

    // Eigen::Matrix<double,3,1> B=Eigen::MatrixXd::Random(3,1);
    // Eigen::Matrix<double,3,1> x1=A.inverse()*B;
    // Eigen::Matrix<double,3,1> x2=A.colPivHouseholderQr().solve(B);
    // Eigen::Matrix<double,3,1> x3=A.ldlt().solve(B);
    // cout << x1.transpose()<<endl;
    // cout << x2.transpose()<<endl;
    // cout << x3.transpose()<<endl;
    

    //旋转矩阵变欧拉角
     Eigen::AngleAxisd Z_pig4(M_PI/2,Eigen::Vector3d(0,0,1));//创建旋转向量
     Eigen::Matrix3d roMatrix1=Z_pig4.toRotationMatrix();//旋转向量转化为旋转矩阵
     Eigen::Vector3d eulerangle=roMatrix1.eulerAngles(0,1,2);
     cout<<eulerangle.transpose()<<endl;
     cout<<endl;

    //获得4元素值
     Eigen::Quaterniond q1=Eigen::Quaterniond(roMatrix1);//旋转矩阵变为4元素
     cout<<q1.coeffs().transpose()<<endl;
    //  cout<<sqrt(2)/2<<endl;
    Eigen::Quaterniond q2=Eigen::Quaterniond(Z_pig4);//旋转向量变为4元素
     cout<<q2.coeffs().transpose()<<endl;
    Eigen::Quaterniond q3(roMatrix1);//旋转向量变为4元素
     cout<<q3.coeffs().transpose()<<endl;

     

    // Eigen::Vector3d v_3d;
    // v_3d<<1,2,3;
    // for(int j=0;j<3;j++)
    // {
    //         cout<<v_3d(j)<<endl;
    // }
    


}