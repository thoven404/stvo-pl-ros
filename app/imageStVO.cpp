#include <stereoFrame.h>
#include <stereoFrameHandler.h>
#include <boost/filesystem.hpp>

#include "dataset.h"
#include "timer.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualizer.h"
// #include <visualization_msgs/Marker.h>
// #include <tf/transform_broadcaster.h>


using namespace StVO;
using namespace std;

void showHelp();
bool getInputArgs(int argc, char **argv, std::string &dataset_name, int &frame_offset, int &frame_number, int &frame_step, std::string &config_file);

int main(int argc, char **argv){

    ros::init(argc, argv, "stvo_pl");
    ros::NodeHandle n;
    Visualizer visualizer(n);
    vector<geometry_msgs::Point> points;
    vector<geometry_msgs::Point> lines;

    string dataset_name, config_file;
    int frame_offset = 0, frame_number = 0, frame_step = 1;
    if (!getInputArgs(argc, argv, dataset_name, frame_offset, frame_number, frame_step, config_file)) {
        showHelp();
        return -1;
    }

    if (!config_file.empty()) Config::loadFromFile(config_file);
    boost::filesystem::path dataset_path(dataset_name);
    if (!boost::filesystem::exists(dataset_path) || !boost::filesystem::is_directory(dataset_path)) {
        cout << "Invalid dataset path" << endl;
        return -1;
    }

    string dataset_dir = dataset_path.string();
    PinholeStereoCamera*  cam_pin = new PinholeStereoCamera((dataset_path / "dataset_params.yaml").string());   // 读取双目相机的内参和外参
    Dataset dataset(dataset_dir, *cam_pin, frame_offset, frame_number, frame_step);     // 读取数据集图片

    // create scene
    Matrix4d Tcw, T_inc = Matrix4d::Identity();
    Vector6d cov_eig;
    Matrix6d cov;
    Tcw = Matrix4d::Identity();
    Tcw <<  1, 0, 0, 0, 
            0, 0, 1, 0, 
            0, -1, 0, 0, 
            0, 0, 0, 1;

    Timer timer;
    int frame_counter = 0;
    double t1;
    StereoFrameHandler* StVO = new StereoFrameHandler(cam_pin);
    Mat img_l, img_r;

    while( dataset.nextFrame(img_l, img_r) && ros::ok() ){
        points.clear();
        lines.clear();
        if( frame_counter == 0 ) // initialize
            StVO->initialize(img_l,img_r,0);
        else{
            timer.start();
            StVO->insertStereoPair( img_l, img_r, frame_counter );
            StVO->optimizePose();
            t1 = timer.stop();
            T_inc   = StVO->curr_frame->DT;
            cov     = StVO->curr_frame->DT_cov;
            cov_eig = StVO->curr_frame->DT_cov_eig;

            Matrix4d Twc = Tcw.inverse();
            Matrix3d Rwc = Twc.block<3,3>(0,0);
            Vector3d twc = Twc.col(3).head(3);
            cvtColor(img_l, img_l, COLOR_GRAY2BGR);
            for(auto &p: StVO->matched_pt){
                if(p->inlier){
                    Vector3d vp = Rwc * p->P + twc;
                    // Vector3d vp = p->P;
                    geometry_msgs::Point point;
                    point.x = vp(0);    point.y = vp(1);    point.z = vp(2);
                    points.push_back(point);
                    circle( img_l, Point(p->pl_obs(0),p->pl_obs(1)), 2, Scalar(0,0,255) , -1);
                }
                
            }
            for(auto &l: StVO->matched_ls){
                if(l->inlier){
                    Vector3d vp1 = Rwc * l->sP + twc;  Vector3d vp2 = Rwc * l->eP + twc;
                    // Vector3d vp1 = l->sP;  Vector3d vp2 = l->eP;
                    geometry_msgs::Point point1, point2;
                    point1.x = vp1(0);    point1.y = vp1(1);    point1.z = vp1(2);
                    point2.x = vp2(0);    point2.y = vp2(1);    point2.z = vp2(2);
                    lines.push_back(point1); lines.push_back(point2);

                    line( img_l, Point(l->spl_obs(0), l->spl_obs(1)) , Point(l->epl_obs(0), l->epl_obs(1)) , Scalar(0,255,0) , 2);
                }
            }

            visualizer.publish(Rwc, twc, points, lines);
            Tcw = T_inc.inverse() * Tcw;
            
            // console output
            cout.setf(ios::fixed,ios::floatfield); cout.precision(8);
            cout << "Frame: " << frame_counter << "\tRes.: " << StVO->curr_frame->err_norm;
            cout.setf(ios::fixed,ios::floatfield); cout.precision(3);
            cout << " \t Proc. time: " << t1 << " ms\t ";
            if( Config::adaptativeFAST() )  cout << "\t FAST: "   << StVO->orb_fast_th;
            if( Config::hasPoints())        cout << "\t Points: " << StVO->matched_pt.size() << " (" << StVO->n_inliers_pt << ") " ;
            if( Config::hasLines() )        cout << "\t Lines:  " << StVO->matched_ls.size() << " (" << StVO->n_inliers_ls << ") " ;
            cout << endl;

            // update StVO
            StVO->updateFrame();

            imshow("frame_left", img_l);
            waitKey(3);

        }
        frame_counter++;
    }

    return 0;
}


void showHelp() {
    cout << endl << "Usage: ./imagesStVO <dataset_path> <Config_file>" << endl;
}
bool getInputArgs(int argc, char **argv, std::string &dataset_name, int &frame_offset, int &frame_number, int &frame_step, std::string &config_file) {

    if( argc < 2 )
        return false;

    dataset_name = argv[1];

    config_file = argv[2];
    frame_offset = 0;
    frame_number = -1;  // 使用所有帧
    frame_step = 2;

    return true;
}

