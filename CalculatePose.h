#ifndef CALCULATEPOSE_H
#define CALCULATEPOSE_H
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
using namespace std;
using namespace Eigen;
#define PI 3.1415926
//I型管口内径
const double RI = 41.75;//模型管直径220mm
//O型管口内径
const double RO = 32.0;
//O型管口外径
const double ROO = 220;
//I、O型管口平面法向量夹角(ang)
const double IOangle = 6 * 3.14159 / 180;
//Calculate two couples of posture solution
const int CheckFrame = 10;//帧间验证：帧数间隔
int howBuf=-1;//-1,0~CheckFrame
Vector3d CenterBuf[CheckFrame];
Vector3d VectorBuf[CheckFrame];
//双洞标志位
bool left_or_right=0;//0-left-I type,1-right-O type


//从小到大排序、返回下标数组
vector<int> Mysort(vector<double> in) {
    int len = in.size();
    vector<int> out(len);
    for (int i = 0; i < len; i++) {//1,2,3,4
        out[i] = i;
    }

    for (int i = 0; i < len - 1; i++)//n-1次冒泡
    {
        for (int j = 0; j < len - 1 - i; j++)//每一次冒泡需要两两比较len—1-i次
        {
            //交换两数操作
            double tem1 = in[j + 1];
            int tem2 = out[j + 1];
            if (in[j] > in[j + 1])//从小到大就大于号，从大到小就小于号
            {
                in[j + 1] = in[j];
                in[j] = tem1;
                out[j + 1] = out[j];
                out[j] = tem2;
            }

        }
    }

    return  out;
}


inline int calculate_pose(array<double, 5> parameters,double R,vector<Vector3d>& cen,vector<Vector3d>& vec) {
    //calib
    //HK
    double fx= 1792.801716577363;
    double fy= 1793.181370236796;
    double cx= 1238.181736988287;//u0
    double cy= 1055.526190442365;//v0
    //Fisheye
    //double fx = 535.441421;
    //double fy = 547.906802;
    //double cx = 1199.14761;//u0
    //double cy = 1092.79105;//v0

    //ellipse
    double x=parameters[0];
    double y=parameters[1];
    double a=parameters[2];
    double b=parameters[3];
    double rad=parameters[4];//shun shi zhen rotate rad
    //double x = 300;
    //double y = 200;
    //double a = 80;
    //double b = 60;
    //double rad = 0;//shun shi zhen rotate rad

    //pixel coordinate:A1u2+B1v2+C1uv+D1u+E1v+F1=0(ellipse)
    //double temp_1=pow(a*sin(rad),2)+pow(b*cos(rad),2);
    //double temp_2=pow(a*cos(rad),2)+pow(b*sin(rad),2);
    //double temp_3=(a*a-b*b)*cos(rad)*sin(rad);
    //double A1=temp_1;
    //double B1=temp_2;
    //double C1=2*temp_3;
    //double D1=-2*x*temp_1-2*y*temp_3;
    //double E1=-2*y*temp_2-2*x*temp_3;
    //double F1=x*x*temp_1+y*y*temp_2+2*x*y*temp_3-pow(a*b,2);
//    double A1=0.333,B1=0.667,C1=0,D1=0,E1=0,F1=-1;
    //Matrix3d g;
    //Matrix<double, 3, 4> M;
    //g << A1, C1 / 2, D1 / 2,
    //    C1 / 2, B1, E1 / 2,
    //    D1 / 2, E1 / 2, F1;
 
    //1.像素坐标系x轴向右，y轴向下
    //二维标准椭圆方程系数矩阵Ez
    Matrix3d Ez;
    Ez << 1 / (a * a), 0          ,  0,
          0          , 1 / (b * b),  0,
          0          , 0          , -1;
    //二维一般椭圆方程系数矩阵g
    Matrix3d Rz, Tz;
    Rz << cos(rad), -sin(rad), 0,
          sin(rad),  cos(rad), 0,
          0       ,  0       , 1;
    Tz << 1, 0, x,
          0, 1, y,
          0, 0, 1;
    Matrix3d g = (((Tz * Rz).inverse()).transpose()) * Ez * ((Tz * Rz).inverse());

    //2.相机坐标系x轴向右，y轴向下，z轴向里
    //三维椭圆锥方程系数矩阵Q
    Matrix<double,3,4> M;
    M << fx, 0 , cx, 0,
         0 , fy, cy, 0,
         0 , 0 , 1 , 0;
    Matrix4d Qtemp=M.transpose()*g*M;
    Matrix3d Q=Qtemp.block(0,0,3,3);

    //3.求特定顺序的特征值和特征向量
    //feature value of Q:P-1QP=PTQP=diag
    SelfAdjointEigenSolver<Matrix3d> eigen_solver(Q);
    Vector3d diag=eigen_solver.eigenvalues();
    Matrix3d P=eigen_solver.eigenvectors();

    double lamta_1,lamta_2,lamta_3;
    Vector3d e1,e2,e3;

    std::ptrdiff_t i, j;
    diag.minCoeff(&i);
    diag.maxCoeff(&j);

    int s1,s2=-1,s3;
    //get lamta_3,e3
    if(diag[0]*diag[1]*diag[2]<0){//+,+,-
        s3=i;
    }
    else if(diag[0]*diag[1]*diag[2]>0){//+,-,-
        s3=j;
    }
    else{
        cout<<"lamta_1*lamta_2*lamta_3=0,wrong!"<<endl;
        return -1;
    }
    //get lamta_2,e2(order lamta_1>lamta_2)
    for(int k=0;k<3;++k){
        if(k!=s3){
            if(s2==-1){
                s2=k;
            }
            else if(abs(diag[k])<abs(diag[s2])) {
                s2=k;
            }
            else if(abs(diag[k])==abs(diag[s2])){
                cout<<"It's a perfect circle!"<<endl;
                return 0;
            }
        }
    }
    //get lamta_1
    s1=3-s2-s3;

    lamta_1=diag[s1];
    lamta_2=diag[s2];
    lamta_3=diag[s3];

    Vector3d z_dir(0,0,1);
    if(z_dir.dot(P.col(s3))>0){
        e3=P.col(s3);
    }
    else if(z_dir.dot(P.col(s3))<0){
        e3=-P.col(s3);
    }
    else{
        cout<<"The circle is a line,Wrong!";
        return -1;
    }
    e2=P.col(s2);
    e1=e2.cross(e3);
    P<<e1,e2,e3;

    //4.求解圆心和法向量
    //loacte and vector in transformed ordinate(法向量由管口平面向内)
    Vector3d center1_trans,vector1_trans,center2_trans,vector2_trans;
    double temp_8=sqrt((abs(lamta_1)- abs(lamta_2))/(abs(lamta_1)+ abs(lamta_3)));
    double temp_9=sqrt((abs(lamta_2)+ abs(lamta_3))/(abs(lamta_1)+ abs(lamta_3)));
    center1_trans<<R* sqrt(abs(lamta_3/lamta_1))*temp_8,0,R* sqrt(abs(lamta_1/lamta_3))*temp_9;
    center2_trans<<-R* sqrt(abs(lamta_3/lamta_1))*temp_8,0,R* sqrt(abs(lamta_1/lamta_3))*temp_9;
    vector1_trans << temp_8, 0, temp_9;
    vector2_trans << -temp_8, 0, temp_9;

    //loacte and vector in camera ordinate
    Vector3d center1=P*center1_trans;
    Vector3d center2=P*center2_trans;
    Vector3d vector1=P*vector1_trans;
    Vector3d vector2=P*vector2_trans;

    //相机坐标系：x-向右，y-向下，z-向里
    cen[0]={center1[0],center1[1],center1[2]};
    cen[1]={center2[0],center2[1],center2[2]};
    vec[0]={vector1[0],vector1[1],vector1[2]};
    vec[1]={vector2[0],vector2[1],vector2[2]};


    Matrix3d calib;
    calib << 0.395289658716254, -0.918285922740789, -0.0222946138806096,
        0.918362999212166, 0.394591757150395, 0.0301122378278271,
        -0.0188543732333770, -0.0323776046838182, 0.999297855158769;
    Eigen::Vector3d calib_EAngle = calib.eulerAngles(0, 1, 2);
    ////欧拉角(逆时针）
    //double x1 = vector2[0], y1 = vector2[1], z1 = vector1[2];
    //                                        
    //double arpha, beta;                     
    ////俯仰角y-z                             
    //if (y1 > 0 && z1 > 0) {
    //    arpha = 2 * PI - atan(y1 / z1);//rad
    //}
    //else if (y1 < 0 && z1 > 0)
    //    arpha = atan(-y1 / z1);//rad
    //else if (y1 < 0 && z1 < 0)
    //    arpha = PI - atan(-y1 / -z1);//rad
    //else if (y1 > 0 && z1 < 0)
    //    arpha = PI + atan(y1 / -z1);//rad
    //cout << "arpha1=" << arpha;
    Eigen::Matrix3d rotMatrix;
    Eigen::Vector3d vectorBef(0, 0, 1);
    Eigen::Vector3d vectorAft(-vector2[0], -vector2[1], -vector2[2]);
    rotMatrix = Eigen::Quaterniond::FromTwoVectors(vectorBef, vectorAft).toRotationMatrix();
    Eigen::Vector3d My_EAngle = rotMatrix.eulerAngles(0, 1, 2);

    //cout << "center1=" << center1 << endl;
    //cout << "center2=" << center2 << endl;
    //cout << "vector1=" << vector1 << endl;
    //cout << "vector2=" << vector2 << endl;
    cout << endl << "calib_EAngle=" << calib_EAngle << endl;
    cout << endl << "My_EAngle=" << My_EAngle << endl;

    return 0;
}

int Seektrue(std::vector<array<double, 5>> EParameters, Vector3d& C, Vector3d& V)
{

    std::vector<Vector3d> centerI(2);
    std::vector<Vector3d> vectorI(2);
    std::vector<Vector3d> centerO(2);
    std::vector<Vector3d> vectorO(2);
    //画面判断
    cout << "EParameters.size()=" << EParameters.size() << endl;
    switch (EParameters.size())
    {
    case(3):
    {
        //EParameters
        vector<double> dist(3);
        dist[0] = pow(EParameters[0][0] - EParameters[1][0], 2) + pow(EParameters[0][1] - EParameters[1][1], 2);
        dist[1] = pow(EParameters[1][0] - EParameters[2][0], 2) + pow(EParameters[1][1] - EParameters[2][1], 2);
        dist[2] = pow(EParameters[2][0] - EParameters[0][0], 2) + pow(EParameters[2][1] - EParameters[0][1], 2);
        vector<int> prank = Mysort(dist);
        //a,b-O内圆/外圆，c-I圆
        int a = prank[0], b = (prank[0] + 1) % 3, c = 3 - a - b;
        if (EParameters[a][2] < EParameters[b][2] && EParameters[a][3] < EParameters[b][3]) {//a-O型内圆
            calculate_pose(EParameters[a], RO, centerO, vectorO);
        }
        else if (EParameters[b][2] < EParameters[a][2] && EParameters[b][3] < EParameters[a][3]) {//b-O型内圆
            calculate_pose(EParameters[b], RO, centerO, vectorO);
        }
        calculate_pose(EParameters[c], RI, centerI, vectorI);
        //确定真解
        vector<double> vcos(4);
        vcos[0] = abs(cos(IOangle) - vectorI[0].dot(vectorO[0]) / (vectorI[0].norm() * vectorO[0].norm()));
        vcos[1] = abs(cos(IOangle) - vectorI[0].dot(vectorO[1]) / (vectorI[0].norm() * vectorO[1].norm()));
        vcos[2] = abs(cos(IOangle) - vectorI[1].dot(vectorO[0]) / (vectorI[1].norm() * vectorO[0].norm()));
        vcos[3] = abs(cos(IOangle) - vectorI[1].dot(vectorO[1]) / (vectorI[1].norm() * vectorO[1].norm()));
        vector<int> vrank = Mysort(vcos);
        //传递给下一帧
        howBuf = (howBuf + 1) % CheckFrame;
        if (left_or_right == 0) {
            CenterBuf[howBuf] = centerI[int(vrank[0] / 2)];
            VectorBuf[howBuf] = vectorI[int(vrank[0] / 2)];
        }
        else {
            CenterBuf[howBuf] = centerO[int(vrank[0] % 2)];
            VectorBuf[howBuf] = vectorO[int(vrank[0] % 2)];
        }

        C = CenterBuf[howBuf];
        V = VectorBuf[howBuf];

        break;
    }
    case(2):
    {
        cout << "执行case2" << endl;
        double dist2 = sqrt(pow(EParameters[0][0] - EParameters[1][0], 2) + pow(EParameters[0][1] - EParameters[1][1], 2));
        vector<double> r2(4);
        r2[0] = EParameters[0][2];
        r2[1] = EParameters[0][3];
        r2[2] = EParameters[1][2];
        r2[3] = EParameters[1][3];
        vector<int> rrank = Mysort(r2);

        vector<double> vcos2(4);
        if (dist2 < r2[rrank[0]]) {//内圆/外圆
            //确定真解
            vector<Vector3d> centerOO, vectorOO;
            calculate_pose(EParameters[rrank[0] / 2], RO, centerO, vectorO);
            calculate_pose(EParameters[1 - rrank[0] / 2], ROO, centerOO, vectorOO);
            vcos2[0] = abs(cos(0) - vectorI[0].dot(vectorO[0]) / (vectorI[0].norm() * vectorO[0].norm()));
            vcos2[1] = abs(cos(0) - vectorI[0].dot(vectorO[1]) / (vectorI[0].norm() * vectorO[1].norm()));
            vcos2[2] = abs(cos(0) - vectorI[1].dot(vectorO[0]) / (vectorI[1].norm() * vectorO[0].norm()));
            vcos2[3] = abs(cos(0) - vectorI[1].dot(vectorO[1]) / (vectorI[1].norm() * vectorO[1].norm()));
            vector<int> vrank2 = Mysort(vcos2);
            //传递给下一帧
            howBuf = (howBuf + 1) % CheckFrame;
            if (left_or_right == 1) {
                CenterBuf[howBuf] = centerO[int(vrank2[0] % 2)];
                VectorBuf[howBuf] = vectorO[int(vrank2[0] % 2)];
            }
        }
        else {//I圆/O圆
            //确定真解
            calculate_pose(EParameters[rrank[0] / 2], RO, centerO, vectorO);
            calculate_pose(EParameters[1 - rrank[0] / 2], RI, centerI, vectorI);
            vcos2[0] = abs(cos(IOangle) - vectorI[0].dot(vectorO[0]) / (vectorI[0].norm() * vectorO[0].norm()));
            vcos2[1] = abs(cos(IOangle) - vectorI[0].dot(vectorO[1]) / (vectorI[0].norm() * vectorO[1].norm()));
            vcos2[2] = abs(cos(IOangle) - vectorI[1].dot(vectorO[0]) / (vectorI[1].norm() * vectorO[0].norm()));
            vcos2[3] = abs(cos(IOangle) - vectorI[1].dot(vectorO[1]) / (vectorI[1].norm() * vectorO[1].norm()));
            vector<int> vrank2 = Mysort(vcos2);
            //传递给下一帧
            howBuf = (howBuf + 1) % CheckFrame;
            if (left_or_right == 0) {
                CenterBuf[howBuf] = centerI[int(vrank2[0] / 2)];
                VectorBuf[howBuf] = vectorI[int(vrank2[0] / 2)];
            }
            else {
                CenterBuf[howBuf] = centerO[int(vrank2[0] % 2)];
                VectorBuf[howBuf] = vectorO[int(vrank2[0] % 2)];
            }
        }

        C = CenterBuf[howBuf];
        V = VectorBuf[howBuf];
        break;
    }
    case(1):
    {
        cout << "执行case1" << endl;
        std::vector<Vector3d> c1(2);
        std::vector<Vector3d> v1(2);
        double R1 = (left_or_right == 0) ? RI : RO;
        calculate_pose(EParameters[0], R1, c1, v1);
        //确定真解
        if (howBuf != -1)
        {
            vector<double> vcos1(2);
            vcos1[0] = 1 - v1[0].dot(VectorBuf[howBuf]) / (v1[0].norm() * VectorBuf[howBuf].norm());
            vcos1[1] = 1 - v1[1].dot(VectorBuf[howBuf]) / (v1[1].norm() * VectorBuf[howBuf].norm());
            vector<int> vrank1 = Mysort(vcos1);
            //传递给下一帧
            howBuf = (howBuf + 1) % CheckFrame;
            CenterBuf[howBuf] = c1[vrank1[0]];
            VectorBuf[howBuf] = v1[vrank1[0]];

            C = CenterBuf[howBuf];
            V = VectorBuf[howBuf];
        }
        else {
            C = c1[0];
            V = v1[0];
        }

        break;
    }
    default:
        cout << "识别圆数目为0或大于3" << endl;
        return 0;
    }
    cout << "center=" << C << endl;
    cout << "vector=" << V << endl;
    return 1;
}

#endif // CALCULATEPOSE_H
