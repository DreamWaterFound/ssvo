/**
 * @file feature_tracker.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 特征跟踪的实现
 * @version 0.1
 * @date 2019-01-19
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef _FEATURE_TRACKER_HPP_
#define _FEATURE_TRACKER_HPP_
#endif

#include "global.hpp"
#include "feature_detector.hpp"
#include "map.hpp"

namespace ssvo
{

/**
 * @brief 特诊跟踪器
 * @detials 不可复制
 * 
 */
class FeatureTracker : public noncopyable
{
    //为啥注释掉了嘞
//    struct Candidate {
//        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//        MapPoint::Ptr pt;    //!< 3D point.
//        Vector2d px;         //!< projected 2D pixel location.
//        Candidate(MapPoint::Ptr pt, Vector2d& px) : pt(pt), px(px) {}
//        Candidate(){}
//    };
//
//    struct Grid {
//        typedef std::list<Candidate, aligned_allocator<Candidate> > Cell;
//        int grid_size;
//        int grid_n_cols;
//        int grid_n_rows;
//        std::vector<Cell*> cells;
//        std::vector<int> grid_order;
//        std::vector<bool> occupied;
//    };

public:
    ///指向自己类型的智能指针
    typedef std::shared_ptr<FeatureTracker> Ptr;
    /**
     * @brief 从局部地图中对点进行重投影
     * 
     * @param[in] frame 当前帧
     * @return int TODO 重投影的地图点个数?
     */
    int reprojectLoaclMap(const Frame::Ptr &frame);

    /**
     * @brief 对地图点进行重投影 3D->2D
     * 
     * @param[in]   frame               帧
     * @param[in]   mpt                 要进行重投影的地图点
     * @param[out]  px_cur              投影点的坐标
     * @param[out]  level_cur           TODO
     * @param[in]   max_iterations      TODO 什么的最大迭代次数
     * @param[in]   epslion             迭代误差阈值
     * @param[in]   threshold           TODO 不知道是什么阈值
     * @param[in]   verbose             是否输出详细信息
     * @return int                      TODO 
     */ 
    static int reprojectMapPoint(const Frame::Ptr &frame, const MapPoint::Ptr& mpt, Vector2d &px_cur, int &level_cur,
                                  const int max_iterations = 30, const double epslion = 0.01, const double threshold = 4.0, bool verbose = false);

    /**
     * @brief 判断某特征点是否能够进行追踪? TODO 还是追踪某个特征点?
     * 
     * @param[in]  frame_ref        参考帧
     * @param[in]  frame_cur        当前帧
     * @param[in]  ft_ref           TODO ??? 参考帧中的特征
     * @param[out] px_cur           TODO 点在当前帧中的投影图像坐标?
     * @param[in]  level_cur        TODO 当前帧中点所在的图层?
     * @param[in]  max_iterations   最大迭代次数 TODO 啥的?
     * @param[in]  epslion          迭代误差阈值
     * @param[in]  threshold        TODO 不知道是什么阈值
     * @param[in]  verbose          是否输出详细信息
     * @return true 
     * @return false 
     */
    static bool trackFeature(const Frame::Ptr &frame_ref, const Frame::Ptr &frame_cur, const Feature::Ptr &ft_ref,
                             Vector2d &px_cur, int &level_cur, const int max_iterations = 30, const double epslion = 0.01, const double threshold = 4.0, bool verbose = false);

    /**
     * @brief 特征追踪器的构造函数
     * 
     * @param[in] width             TODO 啥的宽度
     * @param[in] height            TODO 啥的高度
     * @param[in] grid_size         网格大小
     * @param[in] border            TODO 边界宽度
     * @param[in] report            是否汇报
     * @param[in] verbose           是否输出详情
     * @return FeatureTracker::Ptr  实例指针 
     */
    inline static FeatureTracker::Ptr create(int width, int height, int grid_size, int border, bool report = false, bool verbose = false)
    {return FeatureTracker::Ptr(new FeatureTracker(width, height, grid_size, border, report, verbose));}

private:

    /**
     * @brief 特征追踪器的构造函数
     * 
     * @param[in] width             TODO 啥的宽度
     * @param[in] height            TODO 啥的高度
     * @param[in] grid_size         网格大小
     * @param[in] border            TODO 边界宽度
     * @param[in] report            是否汇报
     * @param[in] verbose           是否输出详情
     * @return FeatureTracker::Ptr  实例指针 
     */
    FeatureTracker(int width, int height, int grid_size, int border, bool report = false, bool verbose = false);

    /**
     * @brief 将地图点重投影到某个cell中
     * 
     * @param[in] frame 帧
     * @param[in] point 地图点
     * @return true 
     * @return false 
     */
    bool reprojectMapPointToCell(const Frame::Ptr &frame, const MapPoint::Ptr &point);

    /**
     * @brief 在cell中匹配地图点? TODO 
     * 
     * @param[in] frame 帧
     * @param[in] cell  指定cell
     * @return true 
     * @return false 
     */
    bool matchMapPointsFromCell(const Frame::Ptr &frame, Grid<Feature::Ptr>::Cell &cell);

    int matchMapPointsFromLastFrame(const Frame::Ptr &frame_cur, const Frame::Ptr &frame_last);

private:

    /**
     * @brief 选项
     * 
     */
    struct Option{
        int border;                 ///<图像边缘宽度? TODO
        int max_matches;            ///<最大匹配数目
        int max_track_kfs;          ///<最大追踪的关键帧
        int num_align_iter;         ///<对齐操作的最大迭代次数
        double max_align_epsilon;   ///<对齐操作的误差阈值
        double max_align_error2;    ///<TODO 
    } options_;

    ///Grid对象 TODO 
    Grid<Feature::Ptr> grid_;
    ///网格顺序,存储的是id
    std::vector<size_t> grid_order_;

    ///是否汇报
    bool report_;
    ///是否汇报详情
    bool verbose_;
    ///TODO ????
    int total_project_;
};

}//! end of ssvo