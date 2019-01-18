/**
 * @file seed.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 深度滤波器中种子的实现
 * @detials 在 SVO 的基础上修改.
 * @see https://github.com/uzh-rpg/rpg_svo/blob/master/svo/include/svo/depth_filter.h#L35
 * @version 0.1
 * @date 2019-01-18
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef _SSVO_SEED_HPP_
#define _SSVO_SEED_HPP_

#include "global.hpp"

namespace ssvo{

class KeyFrame;

//! modified from SVO, https://github.com/uzh-rpg/rpg_svo/blob/master/svo/include/svo/depth_filter.h#L35
/// A seed is a probabilistic depth estimate for a single pixel.

/**
 * @brief 对于单个像素进行深度估计的"种子" 这个种子类的定义
 * @detials A seed is a probabilistic depth estimate for a single pixel.
 */
class Seed
{
public:
    //NOTICE 有使用这个
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ///指向自己类的指针
    typedef std::shared_ptr<Seed> Ptr;

    ///TODO 下一个种子点的id?
    static uint64_t next_id;
    ///自己的id
    const uint64_t id;
    ///种子被创建的时候所在的关键帧
    const std::shared_ptr<KeyFrame> kf;     //!< Reference KeyFrame, where the seed created.
    ///关键帧中位于归一化平面上,应当对其估计深度的点
    const Vector3d fn_ref;                  //!< Pixel in the keyframe's normalized plane where the depth should be computed.
    ///当前帧上的匹配点
    const Vector2d px_ref;                  //!< Pixel matched in current frame
    ///fast角点的提取所在层数
    const int level_ref;                    //!< Corner detected level in refrence frame

    ///TODO 优化后的位姿?
    Vector3d optimal_pose_;

    ///收敛速率??? 卧槽这个还能自己设?  TODO
    const static double convergence_rate;

    ///TODO 啥的历史?
    std::list<std::pair<double, double> > history;

    /**
     * @brief 计算Tau
     * @details TODO a是啥,u又是啥?  均值?
     * 
     * @param[in] T_ref_cur         从当前帧到参考帧的位姿变换
     * @param[in] f                 TODO
     * @param[in] z                 TODO 深度?
     * @param[in] px_error_angle    TODO 角度估计的误差?
     * @return double               TODO Tau? 均值?
     */
    double computeTau(const SE3d &T_ref_cur, const Vector3d& f, const double z, const double px_error_angle);

    /**
     * @brief 计算var
     * @detials TODO var是啥啊,深度分布的方差?
     * 
     * @param[in] T_cur_ref     从参考帧到当前帧的位姿变换
     * @param[in] z             TODO 深度?
     * @param[in] delta         TODO ???
     * @return double           方差
     */
    double computeVar(const SE3d &T_cur_ref, const double z, const double delta);

    /**
     * @brief 更新tau?
     * @detials TODO 
     * @param[in] x         TODO 
     * @param[in] tau2      TODO
     */
    void update(const double x, const double tau2);

    /**
     * @brief 检查是否收敛
     * 
     * @return true 
     * @return false 
     */
    bool checkConvergence();

    /**
     * @brief 获取逆深度
     * 
     * @return double 逆深度
     */
    double getInvDepth();

    /**
     * @brief 获取方差
     * 
     * @return double 方差
     */
    double getVariance();

    /**
     * @brief 获取信息权重
     * @detials 这里的信息权重是啥?
     * @return double 信息权重
     */
    double getInfoWeight();

    /**
     * @brief 构造函数
     * @see Seed::Seed()
     * @param[in] kf            关键帧 NOTE 从这里可以看出,种子都是在关键帧上创建的 
     * @param[in] px            像素点的坐标 TODO 
     * @param[in] fn            TODO 
     * @param[in] level         这个特征所在的图层
     * @param[in] depth_mean    深度的均值
     * @param[in] depth_min     深度的最小值  TODO 为什么要有这个?
     * @return Ptr              实例指针
     */
    inline static Ptr create(const std::shared_ptr<KeyFrame> &kf, const Vector2d &px, const Vector3d &fn, const int level, double depth_mean, double depth_min)
    {return Ptr(new Seed(kf, px, fn, level, depth_mean, depth_min));}

private:
    ///beta 分布的参数a
    double a;                               //!< a of Beta distribution: When high, probability of inlier is large.
    ///beta 分布的参数b
    double b;                               //!< b of Beta distribution: When high, probability of outlier is large.
    ///正态分布的均值
    double mu;                              //!< Mean of normal distribution.
    ///深度的最大范围
    double z_range;                         //!< Max range of the possible depth.
    ///正态分布的方差
    double sigma2;                          //!< Variance of normal distribution.
    ///擦考图像中图像块的协方差 TODO 怎么定义?又有什么用?
    Matrix2d patch_cov;                     //!< Patch covariance in reference image.

    ///线程锁
    std::mutex mutex_seed_;

   /**
     * @brief 构造函数
     * @see Seed::Seed()
     * @param[in] kf            关键帧 NOTE 从这里可以看出,种子都是在关键帧上创建的 
     * @param[in] px            像素点的坐标 TODO 
     * @param[in] fn            TODO 
     * @param[in] level         这个特征所在的图层
     * @param[in] depth_mean    深度的均值
     * @param[in] depth_min     深度的最小值  TODO 为什么要有这个?
     * @return Ptr              实例指针
     */
    Seed(const std::shared_ptr<KeyFrame> &kf, const Vector2d &px, const Vector3d &fn, const int level, double depth_mean, double depth_min);
};

///列表类型定义
typedef std::list<Seed::Ptr> Seeds;
}

#endif //_SSVO_SEED_HPP_
