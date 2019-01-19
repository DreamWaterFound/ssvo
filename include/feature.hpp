/**
 * @file feature.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 特征处理相关
 * @version 0.1
 * @date 2019-01-19
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef _FEATURE_HPP_
#define _FEATURE_HPP_

#include <vector>
#include <opencv2/core.hpp>
#include <Eigen/Dense>

#include "global.hpp"

namespace ssvo {

class MapPoint;
class Seed;

/**
 * @brief 和特征处理有关的类
 * 
 */
class Feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ///指向自己类型的指针
    typedef std::shared_ptr<Feature> Ptr;

    ///TODO 特征点的坐标?
    Vector2d px_;
    ///TODO ????
    Vector3d fn_;
    ///TODO 特征所在的图层??
    int level_;
    ///指向它对应的地图点
    std::shared_ptr<MapPoint> mpt_;
    ///当前特征对应深度滤波器中的种子
    std::shared_ptr<Seed> seed_;
    ///特征的方向信息
    double angle;

    //如果使用闭环的话,就得需要计算描述子了
#ifdef SSVO_DBOW_ENABLE
    cv::Mat descriptors_; //! the descriptor belong to this frame
#endif

    /**
     * @brief 创建一个特征点
     * 
     * @param[in] px        坐标
     * @param[in] fn        TODO ????
     * @param[in] level     所在的图层
     * @param[in] mpt       所对应的地图点
     * @return Ptr          实例指针
     */
    inline static Ptr create(const Vector2d &px, const Vector3d &fn, int level, const std::shared_ptr<MapPoint> &mpt)
    {return std::make_shared<Feature>(Feature(px, fn, level, mpt));}

    /**
     * @brief 创建一个特征点
     * 
     * @param[in] px        坐标
     * @param[in] mpt       所对应的地图点
     * @return Ptr          实例指针
     */
    inline static Ptr create(const Vector2d &px, const std::shared_ptr<MapPoint> &mpt)
    {return std::make_shared<Feature>(Feature(px, mpt));}

    /**
     * @brief 创建一个特征点
     * 
     * @param[in] px        坐标
     * @param[in] level     所在的图层
     * @param[in] seed      深度滤波器中对应的种子
     * @return Ptr          实例指针
     */
    inline static Ptr create(const Vector2d &px, int level, const std::shared_ptr<Seed> &seed)
    {return std::make_shared<Feature>(Feature(px, level, seed));}

private:

    /**
     * @brief 创建一个特征点
     * 
     * @param[in] px        坐标
     * @param[in] fn        TODO ????
     * @param[in] level     所在的图层
     * @param[in] mpt       所对应的地图点
     */
    Feature(const Vector2d &px, const Vector3d &fn, const int level, const std::shared_ptr<MapPoint> &mpt):
        px_(px), fn_(fn), level_(level), mpt_(mpt), seed_(nullptr), angle(-1)
    {
        assert(fn[2] == 1);
        assert(mpt);
    }

    /**
     * @brief 创建一个特征点
     * 
     * @param[in] px        坐标
     * @param[in] mpt       所对应的地图点
     */
    Feature(const Vector2d &px, const std::shared_ptr<MapPoint> &mpt):
        px_(px), fn_(0,0,0), level_(0), mpt_(mpt), seed_(nullptr), angle(-1)
    {
        assert(mpt);
    }

    /**
     * @brief 创建一个特征点
     * 
     * @param[in] px        坐标
     * @param[in] level     所在的图层
     * @param[in] seed      深度滤波器中对应的种子
     */    
    Feature(const Vector2d &px, int level, const std::shared_ptr<Seed> &seed):
        px_(px), fn_(0,0,0), level_(level), mpt_(nullptr), seed_(seed), angle(-1)
    {
    }

};

///特征点集合类型
typedef std::list<Feature::Ptr> Features;

}

#endif