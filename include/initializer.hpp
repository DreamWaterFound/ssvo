/**
 * @file initializer.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 单目初始化部分
 * @version 0.1
 * @date 2019-01-19
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef _INITIALIZATION_HPP_
#define _INITIALIZATION_HPP_

#include "global.hpp"
#include "frame.hpp"
#include "config.hpp"

using namespace Eigen;

namespace ssvo{

//下面是师兄对这些帧的定义,卧槽好形象
//! fifo size = 4
//!            /last
//!   | [] [] [] [] |
//!  ref/         \cur

/**
 * @brief 候选帧类
 * @detials 可以理解成为一种特殊的帧
 */
class FrameCandidate{
public:
    ///指向自己类型的指针
    typedef std::shared_ptr<FrameCandidate> Ptr;
    
    ///帧的句柄
    Frame::Ptr frame;
    ///这个帧的什么点集啊 TODO 
    std::vector<cv::Point2f> pts;
    ///这个帧的特征点集
    std::vector<cv::Point2d> fts;
    ///这个帧....啥的层? 图像金字塔?
    std::vector<int> level;
    ///什么的索引集合? TODO
    std::vector<int64_t> idx;
    ///什么大小??? TODO 
    static int size;

    /**
     * @brief 创建特征点
     * 
     */
    void createFts();
    /**
     * @brief 获得内点
     * 
     * @param[out] inliers 提取到的内点
     * @return int         内点个数
     */
    int getInliers(std::vector<bool> &inliers);
    /**
     * @brief 更新内点
     * 
     * @param[in] inliers 源内点
     * @return int        TODO 更新后的内点个数?
     */
    int updateInliers(const std::vector<bool> &inliers);
    /**
     * @brief 追踪的检验? TODO 
     * 
     * @param[in] min_idx   什么的最小id TODO
     * @param[in] max_idx   什么的最大id TODO 
     * @param[in] min_track 最少追踪数? TODO
     * @return int          TODO 
     */
    int checkTracking(const int min_idx, const int max_idx, const int min_track);
    /**
     * @brief 得到匹配关系
     * 
     * @param[TODO] mask    掩摸? TODO
     * @param[TODO] ref_idx ????? TODO 
     */
    void getMatch(std::vector<bool> &mask, const int ref_idx);

    /**
     * @brief 构造函数
     * 
     * @param[in] frame 当前帧
     * @return Ptr      实例指针
     */
    inline static Ptr create(const Frame::Ptr &frame)
    {return std::make_shared<FrameCandidate>(FrameCandidate(frame));}
    /**
     * @brief 构造函数
     * 
     * @param[in] frame 帧
     * @param[in] cand  TODO ???
     * @return Ptr      实例指针
     */
    inline static Ptr create(const Frame::Ptr &frame, const FrameCandidate::Ptr &cand)
    {return std::make_shared<FrameCandidate>(FrameCandidate(frame, cand));}

private:
    /**
     * @brief 构造函数
     * 
     * @param[in] frame 当前帧
     */
    FrameCandidate(const Frame::Ptr &frame);
    /**
     * @brief 构造函数
     * 
     * @param[in] frame 帧
     * @param[in] cand  TODO ???
     */
    FrameCandidate(const Frame::Ptr &frame, const FrameCandidate::Ptr &cand);
};


/**
 * @brief 初始化类
 * @detials 不可被复制的类
 * 
 */
class Initializer: public noncopyable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 初始化结果
     * @detials 一共有 复位中 就绪 失败 成功 四种
     */
    enum Result {RESET=-2, READY=-1, FAILURE=0, SUCCESS=1};
    //指向自己类对象的指针
    typedef std::shared_ptr<Initializer> Ptr;
    /**
     * @brief 添加第一帧图像
     * 
     * @param[in] frame_ref     参考帧
     * @return Result           操作结果
     */
    Result addFirstImage(Frame::Ptr frame_ref);

    /**
     * @brief 添加第二帧图像
     * 
     * @param[in] frame_cur     第二帧
     * @return Result           操作结果
     */
    Result addImage(Frame::Ptr frame_cur);

    /**
     * @brief 初始化器复位
     * 
     */
    void reset();

    /**
     * @brief 创建初始地图
     * 
     * @param[in] map_scale 地图的尺度
     */
    void createInitalMap(double map_scale=1.0);

    /**
     * @brief 获取参考帧,也就是第二帧
     * 
     * @return Frame::Ptr 第二帧
     */
    Frame::Ptr getReferenceFrame(){return cand_ref_->frame;}

    /**
     * @brief 获取追踪到的点
     * 
     * @param[in] pts_ref 参考帧中的点  TODO 奇怪这里的点和特征不是一回事吗?
     * @param[in] pts_cur 当前帧中的点
     */
    void getTrackedPoints(std::vector<cv::Point2f>& pts_ref, std::vector<cv::Point2f>& pts_cur) const;

    /**
     * @brief 绘制光流
     * @detials drow 是师兄名称写错了. 它绘制的效果就是初始化的时候的那些绿线
     * @param[in] dst 画布
     */
    void drowOpticalFlow(cv::Mat& dst) const;
    /**
     * @brief 绘制光流匹配关系 TODO 和上面的有什么区别?
     * 
     * @param[in] dst 画布
     */
    void drowOpticalFlowMatch(cv::Mat& dst) const;
    /**
     * @brief 计算视差
     * 
     * @param[in] pts1          TODO 第一组点 
     * @param[in] pts2          TODO 第二组点
     * @param[in] mask          掩摸
     * @param[out] disparities   视差 TODO 但是这个数据类型我是没看懂
     */
    static void calcDisparity(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
                              const std::vector<bool> &mask, std::vector<std::pair<int, float> >& disparities);
    /**
     * @brief 找到一组最佳的RT
     * 
     * @param[in] R1        TODO 
     * @param[in] R2        TODO
     * @param[in] t         TODO 
     * @param[in] K1        参考帧的相机内参矩阵? TODO
     * @param[in] K2        当前帧的相机内参矩阵  TODO
     * @param[in] fts1      第一帧中的特征点? TODO
     * @param[in] fts2      第二帧中的特征点? TODO
     * @param[TODO] mask    掩摸? TODO 
     * @param[TODO] P3Ds    初步三角化得到的三D空间点
     * @param[out] T        计算得到的最佳的相机位姿变换矩阵
     * @return true 
     * @return false 
     */
    static bool findBestRT(const Matrix3d& R1, const Matrix3d& R2, const Vector3d& t,
                           const Matrix3d& K1, const Matrix3d& K2,
                           const std::vector<cv::Point2d>& fts1, const std::vector<cv::Point2d>& fts2,
                           std::vector<bool>& mask, std::vector<Vector3d>& P3Ds, Matrix<double, 3, 4>& T);

    /**
     * @brief 检查 投影误差
     * 
     * @param[in] pts_ref 参考帧中的点
     * @param[in] pts_cur 当前帧中的点
     * @param[in] fts_ref 参考帧中的特征
     * @param[in] fts_cur 当前帧中的特征
     * @param[in] T       相机位姿变换矩阵
     * @param[TODO] mask    掩摸  TODO
     * @param[in] p3ds    三角化得到的点
     * @param[in] sigma2  方差  TODO
     * @return int 
     */
    static int checkReprejectErr(const std::vector<cv::Point2f>& pts_ref, const std::vector<cv::Point2f>& pts_cur,
                                 const std::vector<cv::Point2d>& fts_ref, const std::vector<cv::Point2d>& fts_cur,
                                 const Matrix<double, 3, 4>& T, std::vector<bool>& mask, std::vector<Vector3d>& p3ds,
                                 const double sigma2);
    /**
     * @brief 三角化
     * 
     * @param[in] P1     TODO
     * @param[in] P2     TODO
     * @param[in] ft1    参考帧上的特征
     * @param[in] ft2    当前帧上的特征
     * @param[out] P3D   三角化之后得到的三D点
     */
    static void triangulate(const Matrix<double, 3, 4>& P1, const Matrix<double, 3, 4>& P2,
                            const cv::Point2d& ft1, const cv::Point2d& ft2, Vector4d& P3D);

    /**
     * @brief 构造函数
     *  
     * @param[in] fast_detector     特征点提取器
     * @param[in] verbose           是否输出详情
     * @return Initializer::Ptr     实例指针
     */
    inline static Initializer::Ptr create(const FastDetector::Ptr &fast_detector, bool verbose = false)
    {return Initializer::Ptr(new Initializer(fast_detector, verbose));}

private:

    /**
     * @brief 构造函数
     *  
     * @param[in] fast_detector     特征点提取器
     * @param[in] verbose           是否输出详情
     */
    Initializer(const FastDetector::Ptr &fast_detector, bool verbose = false);

    /**
     * @brief 创建新的角点
     * 
     * @param[in] candidate 预选帧
     * @return Result       执行结果
     */
    Result createNewCorners(const FrameCandidate::Ptr &candidate);

    /**
     * @brief 更改参考帧? TODO 
     * 
     * @param[in] buffer_offset TODO ? 
     * @return true 
     * @return false 
     */
    bool changeReference(int buffer_offset);

private:

    ///fast角点提取器
    FastDetector::Ptr fast_detector_;
    ///帧缓冲器,也是一个先进先出队列
    std::deque<FrameCandidate::Ptr> frame_buffer_;

    ///参考帧
    FrameCandidate::Ptr cand_ref_;//！ front of frame_buffer_
    ///当前帧
    FrameCandidate::Ptr cand_cur_;
    ///上一帧
    FrameCandidate::Ptr cand_last_;
    ///初始的地图点集合
    std::vector<Vector3d> p3ds_;
    ///视差
    std::vector<std::pair<int, float> > disparities_;
    ///内点
    std::vector<bool> inliers_;
    ///相对于什么的位姿变换矩阵?  TODO 
    Matrix<double, 3, 4> T_;

    ///是否完成 完成啥啊 TODO
    bool finished_;
    ///是否输出详情
    bool verbose_;
};

}//! namspace ssvo

#endif