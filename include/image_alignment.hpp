/**
 * @file image_alignment.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 实现图像对齐算法
 * @version 0.1
 * @date 2019-01-19
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef _SSVO_IMAGEALIGNMENT_HPP_
#define _SSVO_IMAGE_ALIGNMENT_HPP_

#include "global.hpp"
#include "config.hpp"
#include "keyframe.hpp"
#include "frame.hpp"
#include "utils.hpp"
#include "pattern.hpp"

namespace ssvo {

/**
 * @brief 计算仿射变换
 * 
 * @param[in] I 图像1
 * @param[in] J 图像2
 * @param[in] a TODO ?
 * @param[in] b TODO ?
 */
void calculateLightAffine(const cv::Mat &I, const cv::Mat &J, float &a, float &b);

/**
 * @brief 图像对齐类
 * @detials 这个是作为一个父类的形态出现的
 * @tparam Size 图像块的大小
 * @tparam Num  参数的个数
 */
template <int Size, int Num>
class Align{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 和图像块大小有关的参数
     * 
     */
    enum {
        PatchSize = Size,                   //图像块的大小
        PatchArea = PatchSize*PatchSize,    //图像块的面积
        HalfPatchSize = Size/2,             //半图像块的大小(1/4)/?  TODO 有什么用吗?
        Parameters = Num,                   //参数的个数
    };  

    ///记录? TODO
    std::list<std::string> logs_;

protected:
    ///参考的图像块
    Matrix<double, Dynamic, PatchArea, RowMajor> ref_patch_cache_;
    ///计算的雅克比
    Matrix<double, Dynamic, Num, RowMajor> jacbian_cache_;
    ///计算的hessain
    Matrix<double, Num, Num, RowMajor> Hessian_;
    ///TODO 
    Matrix<double, Num, 1> Jres_;
};

/**
 * @brief 图像块大小为4,具有6个参数的,计算3d SE3变换的图像对齐算法计算类
 * 
 */
class AlignSE3: public Align<4, 6>
{
public:
    /**
     * @brief 构造函数
     * 
     * @param[in] verbose 是否显示详情
     * @param[in] visible TODO 这个是否可见应该怎么理解?
     */
    AlignSE3(bool verbose=false, bool visible=false);

    /**
     * @brief 执行图像对齐算法
     * 
     * @param[in] reference_frame   参考帧
     * @param[in] current_frame     当前帧
     * @param[in] top_level         TODO 执行算法的金字塔最顶层
     * @param[in] bottom_level      TODO 执行算法的金字塔最底层
     * @param[in] max_iterations    最大迭代次数
     * @param[in] epslion           误差阈值
     * @return int                  TODO ???
     */
    int run(Frame::Ptr reference_frame, Frame::Ptr current_frame,
             int top_level, int bottom_level, int max_iterations = 30, double epslion = 1E-5f);

private:

    /**
     * @brief 计算参考的图像块 TODO 
     * 
     * @param[in] level 所在的金字塔层数
     * @param[in] fts   特征序列
     * @return int      TODO 
     */
    int computeReferencePatches(int level, std::vector<Feature::Ptr> &fts);

    /**
     * @brief 计算残差
     * 
     * @param[in] level 所在的金字塔层数
     * @param[in] N     TODO 
     * @return double   计算得到的残差
     */
    double computeResidual(int level, int N);

private:

    ///是否显示详情
    const bool verbose_;
    ///是否显示?  TODO 这个怎么理解,感觉应该是流程控制上的东西
    const bool visible_;

    ///参考帧
    Frame::Ptr ref_frame_;
    ///当前帧
    Frame::Ptr cur_frame_;

    ///TODO 什么的计数?
    int count_;
    ///每个特征的可视情况
    std::vector<bool> visiable_fts_;
    ///暂时存储参考帧中的特征? TODO 
    Matrix<double, 3, Dynamic, RowMajor> ref_feature_cache_;
    
    ///从参考帧到当前帧的位姿变换
    SE3d T_cur_from_ref_;
};


//! ========================== Utils =========================================
/**
 * @brief 这个名字空间中定义了一些小工具
 * 
 */
namespace utils{

/**
 * @brief 得到最好的搜索层??? TODO 应该怎么理解
 * 
 * @param[in] A_cur_ref     TODO 计算得到的从参考帧到当前帧的位姿变换
 * @param[in] max_level     TODO 搜索层数的范围上限?
 * @return int              最好的层
 */
int getBestSearchLevel(const Matrix2d& A_cur_ref, const int max_level);

/**
 * @brief 计算仿射变换矩阵
 * 
 * @param[in] cam_ref       参考帧的相机模型
 * @param[in] cam_cur       当前帧的相机模型
 * @param[in] px_ref        参考点的像素坐标??? TODO 
 * @param[in] f_ref         TODO 
 * @param[in] level_ref     参考帧图像所在图层
 * @param[in] depth_ref     TODO 
 * @param[in] T_cur_ref     从参考帧到当前帧的位姿 变换
 * @param[in] patch_size    图像块的大小
 * @param[out] A_cur_ref    计算得到的仿射矩阵结果
 */
void getWarpMatrixAffine(const AbstractCamera::Ptr &cam_ref,
                         const AbstractCamera::Ptr &cam_cur,
                         const Vector2d &px_ref,
                         const Vector3d &f_ref,
                         const int level_ref,
                         const double depth_ref,
                         const SE3d &T_cur_ref,
                         const int patch_size,
                         Matrix2d &A_cur_ref);

/**
 * @brief 进行仿射变换 TODO 
 * 
 * @tparam Td   图像块中的每个数据类型
 * @tparam size 图像块的大小
 * @param[in] img_ref           参考帧图像
 * @param[TODO] patch           图像块 TODO 可是为什么还需要给定图像块呢?
 * @param[in] A_cur_from_ref    从参考帧到当前帧的仿射变换
 * @param[in] px_ref            参考帧上的点
 * @param[in] level_ref         参考帧图层
 * @param[in] level_cur         当前帧图层
 */
template<typename Td, int size>
void warpAffine(const cv::Mat &img_ref,
                Matrix<Td, size, size, RowMajor> &patch,
                const Matrix2d &A_cur_from_ref,
                const Vector2d &px_ref,
                const int level_ref,
                const int level_cur)
{
    //REVIEW 代码没看
    assert(img_ref.type() == CV_8UC1);

    const Matrix2f A_ref_from_cur = A_cur_from_ref.inverse().cast<float>();
    if(std::isnan(A_ref_from_cur(0,0)))
    {
        LOG(ERROR) << "Affine warp is Nan";
        return;
    }

    const Vector2f px_ref_pyr = px_ref.cast<float>() / (1 << level_ref);
    const float half_patch_size = size * 0.5;
    const int px_pyr_scale = 1 << level_cur;
    for(int y = 0; y < size; ++y)
    {
        for(int x = 0; x < size; ++x)
        {
            Vector2f px_patch(x-half_patch_size, y-half_patch_size);
            px_patch *= px_pyr_scale;//! A_ref_from_cur is for level-0, so transform to it
            Vector2f affine = A_ref_from_cur*px_patch;
            const Vector2f px(affine + px_ref_pyr);

            if(px[0]<0 || px[1]<0 || px[0]>=img_ref.cols-1 || px[1]>=img_ref.rows-1)
                patch(y, x) = 0;
            else
                patch(y, x) = utils::interpolateMat<uchar, Td>(img_ref, px[0], px[1]);
        }
    }
} //void warpAffine()

} //namespace utils

} //namespace ssvo


#endif //SSVO_IMAGE_ALIGNMENT_H
