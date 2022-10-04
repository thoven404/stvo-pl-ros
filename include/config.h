
#pragma once
#include <string>

class Config
{

public:

    Config();
    ~Config();

    static void loadFromFile( const std::string &config_file );

    static Config& getInstance();

    // Keyframe selection parameters (for SLAM, if any)
    static double&  minEntropyRatio()   { return getInstance().min_entropy_ratio; }
    static double&  maxKFTDist()        { return getInstance().max_kf_t_dist; }
    static double&  maxKFRDist()        { return getInstance().max_kf_r_dist; }

    // flags
    static bool&    hasPoints()         { return getInstance().has_points; }
    static bool&    hasLines()          { return getInstance().has_lines; }
    static bool&    useFLDLines()       { return getInstance().use_fld_lines; }
    static bool&    lrInParallel()      { return getInstance().lr_in_parallel; }
    static bool&    plInParallel()      { return getInstance().pl_in_parallel; }
    static bool&    bestLRMatches()     { return getInstance().best_lr_matches; }
    static bool&    adaptativeFAST()    { return getInstance().adaptative_fast; }
    static bool&    useMotionModel()    { return getInstance().use_motion_model; }

    // points detection and matching
    static int&     matchingStrategy()  { return getInstance().matching_strategy; }
    static int&     matchingSWs()       { return getInstance().matching_s_ws; }
    static int&     matchingF2FWs()     { return getInstance().matching_f2f_ws; }

    // 保留的最大的特征数量
    static int&     orbNFeatures()      { return getInstance().orb_nfeatures; } 
    // 金字塔的降采样比例，>1
    static double&  orbScaleFactor()    { return getInstance().orb_scale_factor; }
    // 金字塔的层数
    static int&     orbNLevels()        { return getInstance().orb_nlevels; }
    // 不检测特征点的边界大小，与PatchSize匹配
    static int&     orbEdgeTh()         { return getInstance().orb_edge_th; }
    // 随机抽取多少个点进行描述子的计算
    static int&     orbWtaK()           { return getInstance().orb_wta_k; }
    // 使用什么标准来对特征进行排序
    static int&     orbScore()          { return getInstance().orb_score; }
    // 计算描述子的 patch 的大小
    static int&     orbPatchSize()      { return getInstance().orb_patch_size; }
    static int&     orbFastTh()         { return getInstance().orb_fast_th; }
    static int&     fastMinTh()         { return getInstance().fast_min_th; }
    static int&     fastMaxTh()         { return getInstance().fast_max_th; }
    static int&     fastIncTh()         { return getInstance().fast_inc_th; }
    static int&     fastFeatTh()        { return getInstance().fast_feat_th; }
    static double&  fastErrTh()         { return getInstance().fast_err_th; }
    static double&  maxDistEpip()       { return getInstance().max_dist_epip; }
    static double&  minDisp()           { return getInstance().min_disp; }
    static double&  minRatio12P()       { return getInstance().min_ratio_12_p; }

    static double&  rgbdMinDepth()      { return getInstance().rgbd_min_depth; }
    static double&  rgbdMaxDepth()      { return getInstance().rgbd_max_depth; }


    // lines detection and matching
    static int&     lsdNFeatures()      { return getInstance().lsd_nfeatures; }
    static int&     lsdRefine()         { return getInstance().lsd_refine; }
    static double&  lsdScale()          { return getInstance().lsd_scale; }
    static double&  lsdSigmaScale()     { return getInstance().lsd_sigma_scale; }
    static double&  lsdQuant()          { return getInstance().lsd_quant; }
    static double&  lsdAngTh()          { return getInstance().lsd_ang_th; }
    static double&  lsdLogEps()         { return getInstance().lsd_log_eps; }
    static double&  lsdDensityTh()      { return getInstance().lsd_density_th; }
    static int&     lsdNBins()          { return getInstance().lsd_n_bins; }
    static double&  lineHorizTh()       { return getInstance().line_horiz_th; }
    static double&  minLineLength()     { return getInstance().min_line_length; }
    static double&  minRatio12L()       { return getInstance().min_ratio_12_l; }
    static double&  stereoOverlapTh()   { return getInstance().stereo_overlap_th; }
    static double&  f2fOverlapTh()      { return getInstance().f2f_overlap_th; }
    static double&  lineSimTh()         { return getInstance().line_sim_th; }
    static double&  lsMinDispRatio()    { return getInstance().ls_min_disp_ratio; }

    // optimization
    static double&  homogTh()           { return getInstance().homog_th; }
    static int&     minFeatures()       { return getInstance().min_features; }
    static int&     maxIters()          { return getInstance().max_iters; }
    static int&     maxItersRef()       { return getInstance().max_iters_ref; }
    static double&  minError()          { return getInstance().min_error; }
    static double&  minErrorChange()    { return getInstance().min_error_change; }
    static double&  inlierK()           { return getInstance().inlier_k; }

    // SLAM parameters (keyframe selection)
    double min_entropy_ratio;
    double max_kf_t_dist;
    double max_kf_r_dist;

    // flags
    bool has_points;
    bool has_lines;
    bool lr_in_parallel;
    bool pl_in_parallel;
    bool best_lr_matches;
    bool adaptative_fast;
    bool use_fld_lines;
    bool use_motion_model;

    // points detection and matching
    int matching_strategy;
    int matching_s_ws;
    int matching_f2f_ws;

    int    orb_nfeatures;
    double orb_scale_factor;
    int    orb_nlevels;
    int    orb_edge_th;
    int    orb_wta_k;
    int    orb_score;
    int    orb_patch_size;
    int    orb_fast_th;

    int    fast_min_th;
    int    fast_max_th;
    int    fast_inc_th;
    int    fast_feat_th;
    double fast_err_th;

    double max_dist_epip;
    double min_disp;
    double min_ratio_12_p;
    double stereo_overlap_th;
    double f2f_overlap_th;
    double line_sim_th;

    double rgbd_min_depth;
    double rgbd_max_depth;

    // lines detection and matching
    int    lsd_nfeatures;
    int    lsd_refine;
    double lsd_scale;
    double lsd_sigma_scale;
    double lsd_quant;
    double lsd_ang_th;
    double lsd_log_eps;
    double lsd_density_th;
    int    lsd_n_bins;
    double line_horiz_th;
    double min_line_length;
    double min_ratio_12_l;
    double ls_min_disp_ratio;

    // optimization
    double homog_th;
    int    min_features;
    int    max_iters;
    int    max_iters_ref;
    double min_error;
    double min_error_change;
    double inlier_k;

};

