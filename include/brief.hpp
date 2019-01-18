/**
 * @file brief.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 目测像是实现brief描述子的文件,不过也实现了计算特征点的方向
 * @version 0.1
 * @date 2019-01-18
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef _SSVO_BRIEF_HPP_
#define _SSVO_BRIEF_HPP_

#include <memory>
#include <opencv2/core.hpp>

namespace ssvo
{

/**
 * @brief BRIEF描述子类
 * @detials 计算给定特征点在图像金字塔的某层图像上的方向和描述子
 * 
 */
class BRIEF
{
public:

    //TODO 为什么要写成这个样子?
    enum
    {
        PATCH_SIZE = 31,            //patch大小
        HALF_PATCH_SIZE = 15,       //一半的大小
        EDGE_THRESHOLD = 19,        //边界阈值
    };

    //指向当前类的智能指针类型
    typedef std::shared_ptr<BRIEF> Ptr;

    /**
     * @brief 计算指定图片上指定特征点的描述子
     * 
     * @param[in] images        给定图片        TODO 为什么是一系列的图像?
     * @param[in] keypoints     给定特征点
     * @param[out] descriptors   描述子
     */
    void compute(const std::vector<cv::Mat> &images, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors) const;

    /**
     * @brief 计算特征点的方向
     * 
     * @param[in] image     特征点在的图像
     * @param[in] pt        某个特征点
     * @param[in] u_max     TODO
     * @return float        角度
     */
    float IC_Angle(const cv::Mat &image, cv::Point2f pt, const std::vector<int> &u_max) const;

    /**
     * @brief 根据指定的pattern计算某个特征点的brief描述子
     * 
     * @param[in] kpt       给定特征点
     * @param[in] img       特征点所在的图像
     * @param[in] pattern   模式
     * @param[out] desc     指向描述子结果的指针
     */
    void compute(const cv::KeyPoint &kpt, const cv::Mat &img, const cv::Point *pattern, uchar *desc) const;

    /**
     * @brief 创建一个本类的实例
     * 
     * @param[in] scale_factor  尺度系数,和图像金字塔相关
     * @param[in] nlevels       层数
     * @return Ptr              句柄
     */
    inline static Ptr create(float scale_factor, int nlevels)
    { return Ptr(new BRIEF(scale_factor, nlevels));}

    //TODO 为什么要注释掉?
//    bool checkBorder(const double x, const double y,const int level,const bool bottom_level);

private:

    /**
     * @brief 构造函数
     * @note 为什么要吧构造函数设置为私有类型?这样做有什么目的吗?
     * 
     * @param[in] scale_factor 图像金字塔的缩放系数
     * @param[in] nlevels      当前特征点所在图像金字塔的层数
     */
    BRIEF(float scale_factor, int nlevels);//, int height, int width);

    ///图像金字塔的缩放洗漱
    const float scale_factor_;
    ///当前特征点所在图像金字塔的层数
    const int nlevels_;

    ///像素点提取模式
    std::vector<cv::Point> pattern_;
    ///TODO 
    std::vector<int> umax_;
    ///TODO 为什么会有一系列的?
    std::vector<float> scale_factors_;
    ///上面的逆
    std::vector<float> inv_scale_factors_;

    //在进行边界检测的时候用到了
//    std::vector<cv::Point2i> border_tl_;
//
//    std::vector<cv::Point2i> border_br_;
//
//    int height_, width_;
};

}

#endif //_SSVO_BRIEF_HPP_