/**
 * @file feature_alignment.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 图像对齐算法
 * @version 0.1
 * @date 2019-01-18
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef _SSVO_FEATURE_ALIGNMENT_HPP_
#define _SSVO_FEATURE_ALIGNMENT_HPP_

#include "global.hpp"
#include "config.hpp"
#include "keyframe.hpp"
#include "frame.hpp"
#include "utils.hpp"
#include "pattern.hpp"

namespace ssvo {

//! ====================== Patch align

/**
 * @brief 基于图像块的图像对齐算法
 * @detials 只有两个静态成员函数,是一个静态类
 * 
 */
class AlignPatch{
public:

    /**
     * @brief 一些大小的定义
     * 
     */
    enum {
        Size = 8,                   ///<标准大小
        Area = Size*Size,           ///<标准大小对应的面积
        HalfSize = Size/2,          ///<半大小
        SizeWithBorder = Size+2,    ///<含边界尺寸大小
    };

    /**
     * @brief TODO 对齐两帧2D图像?
     * 
     * @param[in] image_cur             当前帧图像
     * @param[in] patch_ref_with_border 携带有边界的,参考帧图像上的块  TODO
     * @param[in] estimate              TODO 一个估计值
     * @param[in] max_iterations        最大迭代次数,默认30
     * @param[in] epslion               一个接近于0的很小的正数,误差阈值,默认为0.01
     * @param[in] verbose               是否输出详细信息,默认否
     * @return true 
     * @return false 
     */
    static bool align2DI(const cv::Mat &image_cur,
                         const Matrix<float, SizeWithBorder, SizeWithBorder, RowMajor> &patch_ref_with_border,
                         Vector3d &estimate,
                         const int max_iterations = 30,
                         const double epslion = 1E-2f,
                         const bool verbose = false);

    /**
     * @brief 对齐两帧图像? TODO 
     * 
     * @param[in] image_cur         当前帧图像
     * @param[in] patch_ref         参考图像块 TODO 
     * @param[in] patch_ref_gx      TODO 
     * @param[in] patch_ref_gy      TODO 
     * @param[in] estimate          TODO 一个估计的值
     * @param[in] max_iterations    最大迭代次数,默认30
     * @param[in] epslion           误差阈值,默认0.01
     * @param[in] verbose           是否输出详细信息,默认否
     * @return true 
     * @return false 
     */
    static bool align2DI(const cv::Mat &image_cur,
                         const Matrix<float, Area, 1> &patch_ref,
                         const Matrix<float, Area, 1> &patch_ref_gx,
                         const Matrix<float, Area, 1> &patch_ref_gy,
                         Vector3d &estimate,
                         const int max_iterations = 30,
                         const double epslion = 1E-2f,
                         const bool verbose = false);
};


//! ====================== Pattern align
/**
 * @brief 基于"模式"的图像对齐方法
 * 
 */
class AlignPattern
{
public:
    //TODO 不明觉厉
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 定义一些常用的大小
     * @detials 话说为啥要写成这种形式呢? TODO  
     * 
     */
    enum {
        Num = 32,                   ///<TODO 什么的数目是32啊?
        Size = 8,                   ///<模式大小?  TODO
        HalfSize = Size/2,          ///<一半的模式大小 TODO 为什么会要用到这个
        SizeWithBorder = Size+2,    ///<考虑边界的时候的大小(边界的宽度为1)
    };

    ///一个模式对象
    static const Pattern<float, Num, Size> pattern_;

    /**
     * @brief 基于模式的方法,对2d图像进行对齐
     * 
     * @param[in] image_cur         当前图像
     * @param[in] patch_ref         参考模式(这里的patch应该是打错了吧)  TODO
     * @param[in] patch_ref_gx      TODO 
     * @param[in] patch_ref_gy      TODO 
     * @param[in] estimate          TODO 一个估计值
     * @param[in] max_iterations    最大迭代次数,默认为30
     * @param[in] epslion           误差阈值,默认为0.01
     * @param[in] verbose           是否输出详细信息,默认否
     * @return true 
     * @return false 
     */
    static bool align2DI(const cv::Mat &image_cur,
                         const Matrix<float, Num, 1> &patch_ref,
                         const Matrix<float, Num, 1> &patch_ref_gx,
                         const Matrix<float, Num, 1> &patch_ref_gy,
                         Vector3d &estimate,
                         const int max_iterations = 30,
                         const double epslion = 1E-2f,
                         const bool verbose = false);
};

/**
 * @brief TODO ????
 * 
 * @tparam T        图像的每个像素的数据类型
 * @tparam Size     图像块的大小
 */
template <typename T, int Size>
class ZSSD{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 构造函数
     * 
     * @param[in] patch_ref 参考图像块
     */
    ZSSD(const Matrix<T, Size, Size, RowMajor> &patch_ref):
        A(patch_ref)
    {
        //REVIEW
        A.array() -= A.mean();
    }

    /**
     * @brief 计算当前图像图像块的评分?
     * 
     * @param[in] patch_cur 当前的图像块 TODO
     * @return T TODO 
     */
    T compute_score(const Matrix<T, Size, Size, RowMajor> &patch_cur)
    {
        //REVIEW
        Matrix<T, Size, Size, RowMajor> B = patch_cur.array() - patch_cur.mean();

        return (A-B).norm();
    }

    /**
     * @brief 获得阈值
     * 
     * @return T 阈值
     */
    T threshold() const {return threshold_;}

private:
    ///TODO 图像块?
    Matrix<T, Size, Size, RowMajor> A;
    ///TODO 阈值???还等于带下乘500????
    const T threshold_ = Size * 500;
};

}

#endif //_SSVO_FEATURE_ALIGNMENT_HPP_
