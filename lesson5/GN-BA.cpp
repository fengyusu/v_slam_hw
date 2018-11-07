//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>


using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "./p3d.txt";
string p2d_file = "./p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream fin_p3d(p3d_file);
    if (!fin_p3d)
    {
        cerr<<"请在有p3d.txt的目录下运行此程序"<<endl;
        return 1;
    }
    for(int i = 0; i < 76; i++)//while(fin_p3d)
    {
        double data[3] = {0};
        for ( auto& d:data )
            fin_p3d>>d;
        Eigen::Vector3d p( data[0], data[1], data[2] );
        p3d.push_back( p );
    }

    ifstream fin_p2d(p2d_file);
    if (!fin_p2d)
    {
        cerr<<"请在有p2d.txt的目录下运行此程序"<<endl;
        return 1;
    }
    for(int i = 0; i < 76; i++)//while(fin_p2d)
    {
        double data[2] = {0};
        for ( auto& d:data )
            fin_p2d>>d;
        Eigen::Vector2d p( data[0], data[1]);
        p2d.push_back( p );
    }
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Matrix3d R = Matrix3d::Identity();
    Vector3d t(0, 0, 0);
    Sophus::SE3 T_esti(R, t); // estimated pose
    Matrix4d T0 = Matrix4d::Identity();

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        Vector2d e = Vector2d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++)
        {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE 
            Vector3d p3d_pre = T_esti * p3d[i];
            double x = p3d_pre(0);
            double y = p3d_pre(1);
            double z = p3d_pre(2);
            e(0) = (p2d[i](0) - (fx * x / z + cx));
            e(1) = (p2d[i](1) - (fy * y / z + cy));

	    // END YOUR CODE HERE

	    // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE 
            J << fx/z, 0, -fx*x/(z*z), -fx*x*y/(z*z), fx+fx*x*x/(z*z), -fx*y/z, \
                 0, fy/z, -fy*y/(z*z), -fy-fy*y*y/(z*z), fy*x*y/(z*z), fy*x/z;
            J = -J;
	    // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;

            cost += e(0) * e(0) + e(1) * e(1);
        }

	// solve dx 
        Vector6d dx;
        dx = H.ldlt().solve(b);

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        T_esti = Sophus::SE3::exp(dx) * T_esti;
        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
