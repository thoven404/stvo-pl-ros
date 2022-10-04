
#pragma once
#include <stereoFrame.h>
#include <stereoFeatures.h>
#include <bits/stdc++.h>    // new

typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,6,1> Vector6d;

class StereoFrame;

namespace StVO{

class StereoFrameHandler
{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    StereoFrameHandler( PinholeStereoCamera* cam_ );
    ~StereoFrameHandler();

    void initialize( const Mat img_l_, const Mat img_r_, const int idx_);
    void insertStereoPair(const Mat img_l_, const Mat img_r_, const int idx_);
    void updateFrame();

    void f2fTracking();
    void matchF2FPoints();
    void matchF2FLines();
    double f2fLineSegmentOverlap( Vector2d spl_obs, Vector2d epl_obs, Vector2d spl_proj, Vector2d epl_proj  );

    bool isGoodSolution( Matrix4d DT, Matrix6d DTcov, double err );
    void optimizePose();
    void resetOutliers();
    void setAsOutliers();

    void plotStereoFrameProjerr(Matrix4d DT, int iter);
    void plotLeftPair();

    // adaptative fast
    int orb_fast_th;
    double llength_th;

    // slam-specific functions
    bool needNewKF();
    void currFrameIsKF();

    //list< boost::shared_ptr<PointFeature> > matched_pt;
    //list< boost::shared_ptr<LineFeature>  > matched_ls;
    list< PointFeature* > matched_pt;
    list< LineFeature*  > matched_ls;

    StereoFrame* prev_frame;
    StereoFrame* curr_frame;
    PinholeStereoCamera *cam;

    int  n_inliers, n_inliers_pt, n_inliers_ls;

    // slam-specific variables
    bool     prev_f_iskf;
    double   entropy_first_prevKF;
    Matrix4d T_prevKF;
    Matrix6d cov_prevKF_currF;
    int      N_prevKF_currF;

//    bool recurse;

private:

    void prefilterOutliers( Matrix4d DT );
    void removeOutliers( Matrix4d DT );
    void gaussNewtonOptimization(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters);
    void gaussNewtonOptimizationRobust(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters);
    void gaussNewtonOptimizationRobustDebug(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters);
    void levenbergMarquardtOptimization(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters);
    void optimizeFunctions(Matrix4d DT, Matrix6d &H, Vector6d &g, double &e);
    void optimizeFunctionsRobust(Matrix4d DT, Matrix6d &H, Vector6d &g, double &e);
    void optimizePoseDebug();

};

}
