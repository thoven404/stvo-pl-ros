
#pragma once
#include <eigen3/Eigen/Core>
#include <config.h>
using namespace Eigen;

namespace StVO{

class PointFeature
{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    PointFeature( Vector3d P_, Vector2d pl_obs_);
    PointFeature( Vector2d pl_, double disp_, Vector3d P_ );
    PointFeature( Vector2d pl_, double disp_, Vector3d P_, int idx_ );
    PointFeature( Vector2d pl_, double disp_, Vector3d P_, int idx_, int level );
    PointFeature( Vector2d pl_, double disp_, Vector3d P_, int idx_, int level, Matrix3d covP_an_ );
    PointFeature( Vector2d pl_, double disp_, Vector3d P_, Vector2d pl_obs_ );
    PointFeature( Vector2d pl_, double disp_, Vector3d P_, Vector2d pl_obs_,
                  int idx_, int level_, double sigma2_, Matrix3d covP_an_, bool inlier_ );
    ~PointFeature(){};

    PointFeature* safeCopy();

    int idx;                // 保存匹配的上一帧的特征点对应的索引
    Vector2d pl, pl_obs;    // pl: 当前帧特征点二维坐标；pl_obs: 下一帧对应特征点的二维坐标
    double   disp;          // 特征点深度
    Vector3d P;             // 特征点三维坐标
    bool inlier;
    int level;              // 提取该特征点的金字塔层级
    double sigma2 = 1.0;
    Matrix3d covP_an;

};

class LineFeature
{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    LineFeature( Vector3d sP_, Vector3d eP_, Vector3d le_obs_);

    LineFeature( Vector3d sP_, Vector3d eP_, Vector3d le_obs_, Vector2d spl_obs_, Vector2d epl_obs_);

    LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                 Vector2d epl_, double edisp_, Vector3d eP_,
                 Vector3d le_,  int idx_);

    LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                 Vector2d epl_, double edisp_, Vector3d eP_,
                 Vector3d le_,  double angle_, int idx_);

    LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                 Vector2d epl_, double edisp_, Vector3d eP_,
                 Vector3d le_,  double angle_, int idx_, int level);

    /*LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                 Vector2d epl_, double edisp_, Vector3d eP_,
                 Vector3d le_,  double angle_, int idx_, int level, Matrix3d covS_an_, Matrix3d covE_an_);*/

    LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                 Vector2d epl_, double edisp_, Vector3d eP_, Vector3d le_);

    LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                 Vector2d epl_, double edisp_, Vector3d eP_,
                 Vector3d le_, Vector3d le_obs_);

    /*LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_,
                 Vector2d epl_, double edisp_, Vector3d eP_,
                 Vector3d le_,  double angle_, int idx_, int level, Vector2d spr_, Vector2d epr_);*/

    LineFeature( Vector2d spl_, double sdisp_, Vector3d sP_, Vector2d spl_obs_, double sdisp_obs_,
                 Vector2d epl_, double edisp_, Vector3d eP_, Vector2d epl_obs_, double edisp_obs_,
                 Vector3d le_, Vector3d le_obs_, double angle_, int idx_, int level_, bool inlier_, double sigma2_,
                 Matrix3d covE_an_, Matrix3d covS_an_);

    ~LineFeature(){};

    LineFeature* safeCopy();

    int idx;        // 保存匹配的上一帧的线段对应的索引
    Vector2d spl,epl, spl_obs, epl_obs;     // spl:起始点；epl:终点；
    double   sdisp, edisp, angle, sdisp_obs, edisp_obs;     // sdisp：线段起始点深度；edisp：线段终点深度；
    Vector3d sP,eP;     // sP：起始点归一化坐标；ep：终点归一化坐标
    Vector3d le, le_obs;    // le：线段的方程稀疏
    bool inlier;    // 如果下一帧匹配上了，就标记为内点

    int level;
    double sigma2 = 1.0;

    Matrix3d covE_an, covS_an;



};

}
