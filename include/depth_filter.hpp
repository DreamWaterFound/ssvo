/**
 * @file depth_filter.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 深度滤波器
 * @version 0.1
 * @date 2019-01-18
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef _SSVO_DEPTH_FILTER_HPP_
#define _SSVO_DEPTH_FILTER_HPP_

#include "global.hpp"
#include "map.hpp"
#include "seed.hpp"
#include "feature_detector.hpp"
#include "local_mapping.hpp"

namespace ssvo
{

/**
 * @brief 深度滤波器的实现
 * 
 */
class DepthFilter : public noncopyable
{
public:
    ///指向本类的只能指针
    ///NOTICE 学着点,几乎每个类都用到了这个
    typedef std::shared_ptr<DepthFilter> Ptr;
    ///定义了一个回调函数类型
    typedef std::function<void (const Seed::Ptr&)> Callback;

    /**
     * @brief 跟踪某一个帧
     * 
     * @param[in] frame_last 上一帧
     * @param[in] frame_cur  当前帧
     */
    void trackFrame(const Frame::Ptr &frame_last, const Frame::Ptr &frame_cur);

    /**
     * @brief 插入帧? TODO 
     * 
     * @param[in] frame     TODO
     * @param[in] keyframe  TODO 
     */
    void insertFrame(const Frame::Ptr &frame, const KeyFrame::Ptr keyframe = nullptr);

//    int getSeedsForMapping(const KeyFrame::Ptr &keyframe, const Frame::Ptr &frame);

    /**
     * @brief 通过链接关系来更新关键帧
     * 
     * @param[in] keyframe 某一个关键帧的句柄
     * @param[in] num       TODO 
     * @return int          TODO
     */
    int updateByConnectedKeyFrames(const KeyFrame::Ptr &keyframe, int num = 2);

    /**
     * @brief 使能跟踪线程
     * 
     */
    void enableTrackThread();

    /**
     * @brief 禁用跟踪线程
     * 
     */
    void disableTrackThread();

    /**
     * @brief 开启主线程
     * 
     */
    void startMainThread();


    /**
     * @brief 停止主线程
     * 
     */
    void stopMainThread();

    /**
     * @brief TODO 
     * 
     */
    void logSeedsInfo();

    /**
     * @brief 创建本类的一个实例
     * 
     * @param[in] fast_detector fast角点提取器句柄
     * @param[in] callback      回调函数指针
     * @param[in] report        是否报告 
     * @param[in] verbose       是否生成详情
     * @return Ptr 
     */
    static Ptr create(const FastDetector::Ptr &fast_detector, const Callback &callback, bool report = false, bool verbose = false)
    { return Ptr(new DepthFilter(fast_detector, callback, report, verbose)); }

private:

    /**
     * @brief 构造函数
     * 
     * @param[in] fast_detector fast提取器句柄
     * @param[in] callback      回调函数指针
     * @param[in] report        是否汇报
     * @param[in] verbose       是否生成详细信息
     */
    DepthFilter(const FastDetector::Ptr &fast_detector, const Callback &callback, bool report, bool verbose);

    /**
     * @brief 一个参数为种子对象的指针
     * 
     */
    Callback seed_coverged_callback_;

    /**
     * @brief TODO ?
     * 
     */
    void run();

    /**
     * @brief 深度滤波器停止? TODO 
     * 
     */
    void setStop();

    /**
     * @brief 是否外部请求停止本线程
     * 
     * @return true 
     * @return false 
     */
    bool isRequiredStop();

    /**
     * @brief 检查新帧
     * 
     * @param[TODO] frame       TODO
     * @param[TODO] keyframe    TODO
     * @return true 
     * @return false 
     */
    bool checkNewFrame(Frame::Ptr &frame, KeyFrame::Ptr &keyframe);

    /**
     * @brief 计算视差
     * 
     * @param[in] frame TODO
     * @return true 
     * @return false 
     */
    bool checkDisparity(const Frame::Ptr &frame);

    /**
     * @brief 创建种子对象
     * 
     * @param[in] keyframe TODO
     * @param[in] frame    TODO
     * @return int 
     */
    int createSeeds(const KeyFrame::Ptr &keyframe, const Frame::Ptr &frame = nullptr);

    /**
     * @brief 跟踪种子  TODO
     * 
     * @param[in] frame_last 上一帧
     * @param[in] frame_cur  当前帧
     * @return int 
     */
    int trackSeeds(const Frame::Ptr &frame_last, const Frame::Ptr &frame_cur) const;

    /**
     * @brief 更新种子  TODO
     * 
     * @param[in] frame 当前帧?  TODO
     * @return int              TODO
     */
    int updateSeeds(const Frame::Ptr &frame);

    /**
     * @brief 重投影所有的种子
     * 
     * @param[in] frame     当前帧? TODO 
     * @return int                 TODO 
     */
    int reprojectAllSeeds(const Frame::Ptr &frame);

    /**
     * @brief 重投影所有的种子
     * 
     * @param[in] keyframe      TODO
     * @param[in] frame         TODO
     * @param[TODO] epl_err     极线误差
     * @param[TODO] px_error    像素误差
     * @param[TODO] created     TODO
     * @return int              TODO 
     */
    int reprojectSeeds(const KeyFrame::Ptr& keyframe, const Frame::Ptr &frame, double epl_err, double px_error, bool created = true);

    /**
     * @brief 寻找极线上的匹配
     * 
     * @param[in] seed              种子? TODO
     * @param[in] keyframe          关键帧 TODO
     * @param[in] frame             当前帧? TODO
     * @param[in] T_cur_from_ref    从参考帧到当前帧的位姿变换矩阵
     * @param[out] px_matched       匹配的点? TODO
     * @param[out] level_matched    TODO
     * @return true 
     * @return false 
     */
    bool findEpipolarMatch(const Seed::Ptr &seed, const KeyFrame::Ptr &keyframe, const Frame::Ptr &frame,
                           const SE3d &T_cur_from_ref, Vector2d &px_matched, int &level_matched);

private:

    /**
     * @brief 未知
     * @note NOTICE 注意,这种使用方法已经出现了很多次了
     */
    struct Option{
        //! max keyframes for seeds tracking(exclude current keyframe)
        int max_kfs;                    ///<种子追踪的最多的关键帧数目,除了当前关键帧
        int max_features;               ///<最大的特征点个数
        double max_perprocess_kfs;      ///<最大处理的关键帧数目
        double max_epl_length;          ///<最大的极线长度
        double epl_dist2_threshold;     ///<距离极线的距离的平方的阈值
        double klt_epslion;             ///<TODO
        double align_epslion;           ///<TODO
        double pixel_error_threshold;   ///<像素误差阈值
        double min_frame_disparity;     ///<最小的帧间视差
        double min_pixel_disparity;     ///<最小的像素视差??? TODO
    } options_;

    ///fast角点提取器句柄
    FastDetector::Ptr fast_detector_;

    //这个是一种首尾都可以直接插入和删除的vector
    ///帧和关键帧的句柄对数组
    std::deque<std::pair<Frame::Ptr, KeyFrame::Ptr> > frames_buffer_;
//    std::map<uint64_t, std::tuple<int, int> > seeds_convergence_rate_;

    ///是否汇报
    const bool report_;
    ///是否汇报详情
    const bool verbose_;

    //! main thread
    ///深度滤波器主线程
    std::shared_ptr<std::thread> filter_thread_;

    ///是否已经使能了跟踪线程
    bool track_thread_enabled_;
    ///是否有终止线程的请求
    bool stop_require_;

    ///多线程锁
    std::mutex mutex_stop_;
    std::mutex mutex_frame_;


    //! track thread
    ///条件变量,多线程处理机制
    std::condition_variable cond_process_main_;
    ///多线程消息机制相关
    std::future<int> seeds_track_future_;
};

}

#endif //_SSVO_DEPTH_FILTER_HPP_
