#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <boost/format.hpp>  // for formating strings

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

using namespace std;

// path to trajectory file
string trajectory_file = "./compare.txt";

Eigen::Isometry3d IcpSolve(const vector<Eigen::Vector3d> & points0, 
              const vector<Eigen::Vector3d> & points1);
void DrawTrajectory(const vector<Eigen::Vector3d> & points0, 
                    const vector<Eigen::Vector3d> & points1,
                    const vector<Eigen::Vector3d> & points2);
void DrawTrajectoryWithPCL(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses);



int main(int argc, char **argv) {

    vector<Eigen::Vector3d> points[3];
    Eigen::Isometry3d T;

    ifstream fin(trajectory_file);
    if (!fin)
    {
        cerr<<"请在有compare.txt的目录下运行此程序"<<endl;
        return 1;
    }

    for ( int i=0; i<600; i++ )
    {
        double data[16] = {0};
        for ( auto& d:data )
        {
            fin>>d;
        }
        Eigen::Vector3d t0( data[1], data[2], data[3] );
        Eigen::Vector3d t1( data[9], data[10], data[11] );

        points[0].push_back( t0 );
        points[1].push_back( t1 );
    }

    points[2].resize(points[0].size());

    T = IcpSolve(points[0], points[1]);

    for(int i = 0; i < points[0].size(); i++){
        points[2][i] = T * points[1][i];
    }

    // draw trajectory in pangolin
    DrawTrajectory(points[0], points[1], points[2]);


    return 0;
}

Eigen::Isometry3d IcpSolve(const vector<Eigen::Vector3d> & points0, 
              const vector<Eigen::Vector3d> & points1){
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    if (points0.empty() || points1.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return T;
    }

    Eigen::Vector3d u0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d u1 = Eigen::Vector3d::Zero();
    for(int i = 0; i < points0.size(); i++){
        u0 += points0[i];
        u1 += points1[i];
    }
    u0 /= points0.size();
    u1 /= points0.size();

    Eigen::Matrix3d w = Eigen::Matrix3d::Zero();
    for(int i = 0; i < points0.size(); i++){
        w += (points0[i] - u0) * (points1[i] - u1).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(w, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d u = svd.matrixU();
    Eigen::Matrix3d v = svd.matrixV();

    Eigen::Matrix3d r = u * v.transpose();
    Eigen::Vector3d t = u0 - r * u1;

    T.rotate(r);
    T.pretranslate(t);

    return T;
    
}


void DrawTrajectoryWithPCL(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    cv::RNG rng;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud->width = 1;
    cloud->height = poses.size();
    cloud->resize(cloud->width * cloud->height);
    for (size_t i = 0; i < poses.size() - 1; i++) {
        cloud->points[i].x = poses[i].translation()[0];
        cloud->points[i].y = poses[i].translation()[1];
        cloud->points[i].z = poses[i].translation()[2];
        uint8_t r = 255;
        uint8_t g = 0;//static_cast<uint8_t>(rng.uniform(0,255));
        uint8_t b = 0;//static_cast<uint8_t>(rng.uniform(0,255));
        uint32_t rgb = static_cast<uint32_t>(r) << 16 | \
                       static_cast<uint32_t>(r) << 8 | \
                       static_cast<uint32_t>(r) << 0 ;
        cloud->points[i].rgb = *reinterpret_cast<float *>(&rgb);
    }

    pcl::visualization::CloudViewer viewer("viewer");
    viewer.showCloud(cloud);

    while(!viewer.wasStopped())
    {
        usleep(500);

    }

}

/*******************************************************************************************/
void DrawTrajectory(const vector<Eigen::Vector3d> & points0, 
                    const vector<Eigen::Vector3d> & points1,
                    const vector<Eigen::Vector3d> & points2) {
    if (points0.empty() || points1.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < points0.size() - 1; i++) {
            // glColor3f(1 - (float) i / points0.size(), 0.0f, (float) i / points0.size());
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p0 = points0[i], p1 = points0[i + 1];
            glVertex3d(p0[0], p0[1], p0[2]);
            glVertex3d(p1[0], p1[1], p1[2]);
            glEnd();
        }

        for (size_t i = 0; i < points1.size() - 1; i++) {
            glColor3f(0.0f, 1.0f, 0.0f);
            glBegin(GL_LINES);
            auto p0 = points1[i], p1 = points1[i + 1];
            glVertex3d(p0[0], p0[1], p0[2]);
            glVertex3d(p1[0], p1[1], p1[2]);
            glEnd();
        }

        for (size_t i = 0; i < points2.size() - 1; i++) {
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            auto p0 = points2[i], p1 = points2[i + 1];
            glVertex3d(p0[0], p0[1], p0[2]);
            glVertex3d(p1[0], p1[1], p1[2]);
            glEnd();
        }


        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}
