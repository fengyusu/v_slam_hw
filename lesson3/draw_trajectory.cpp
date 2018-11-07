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
string trajectory_file = "../trajectory.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
void DrawTrajectoryWithPCL(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses);

int test(void)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

//    if(pcl::io::loadPLYFile<pcl::PointXYZRGB>("/home/fengyusu/catkin_ws/bag/cartographer_paper_deutsches_museum.bag_points.ply",*cloud) == -1)
//    {
//        return -1;
//    }

    pcl::visualization::CloudViewer viewer("viewer");
    viewer.showCloud(cloud);

    while(!viewer.wasStopped())
    {
        usleep(500);

    }
}

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    /// implement pose reading code
    // start your code here (5~10 lines)

    ifstream fin(trajectory_file);
    if (!fin)
    {
        cerr<<"请在有pose.txt的目录下运行此程序"<<endl;
        return 1;
    }

    for ( int i=0; i<400; i++ )
    {
        double data[8] = {0};
        for ( auto& d:data )
        {
            fin>>d;
        }
        Eigen::Quaterniond q( data[7], data[4], data[5], data[6] );
        Eigen::Vector3d t( data[1], data[2], data[3] );

        Sophus::SE3 SE3_qt(q,t);
        poses.push_back( SE3_qt );
    }

    // end your code here

    // draw trajectory in pangolin
    DrawTrajectoryWithPCL(poses);


    return 0;
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
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
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
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}
