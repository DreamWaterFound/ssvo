/**
 * @file system.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 整个ssvo主要功能的实现文件
 * @version 0.1
 * @date 2019-01-18
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef _SSVO_SYSTEM_HPP_
#define _SSVO_SYSTEM_HPP_


//REVIEW 为啥师兄不把这些文件也写在 global.hpp 中嘞?
#include "global.hpp"
#include "frame.hpp"
#include "keyframe.hpp"
#include "map.hpp"
#include "initializer.hpp"
#include "feature_detector.hpp"
#include "feature_tracker.hpp"
#include "local_mapping.hpp"
#include "depth_filter.hpp"
#include "viewer.hpp"

//姜浩师兄后加的,用于支持闭环
#ifdef SSVO_DBOW_ENABLE
#include "loop_closure.hpp"
#endif

namespace ssvo {

/**
 * @brief 系统的主要框架
 */
class System: public noncopyable
{
public:
    /**
     * @brief 系统当前所处的阶段 
     */
    enum Stage{
        STAGE_INITALIZE,        //初始化
        STAGE_NORMAL_FRAME,     //正常的帧处理
        STAGE_RELOCALIZING      //重定位 NOTICE 这个就是自己需要完成的
    };

    /**
     * @brief 当前系统的工作状态 
     */
    enum Status {
        STATUS_INITAL_RESET,    //初始化复位
        STATUS_INITAL_PROCESS,  //正在进行初始化
        STATUS_INITAL_SUCCEED,  //初始化已经成功
        STATUS_TRACKING_BAD,    //当前追踪失败
        STATUS_TRACKING_GOOD,   //当前追踪成功
    };

    /**
     * @brief ssvo系统的构造函数
     * 
     * @param[in] config_file 配置文件路径
     * @param[in] calib_flie  相机参数路径
     */
    System(std::string config_file, std::string calib_flie);

    /**
     * @brief 保存内存中的轨迹数据到文件中
     * 
     * @param[in] file_name 轨迹文件路径
     */
    void saveTrajectoryTUM(const std::string &file_name);

    /**
     * @brief 析构函数
     * 
     */
    ~System();

    /**
     * @brief 处理一帧
     * 
     * @param[in] image         当前帧的图像
     * @param[in] timestamp     时间戳
     */
    void process(const cv::Mat& image, const double timestamp);

private:

    /**
     * @brief TODO  处理一帧?
     * 
     */
    void processFrame();

    /**
     * @brief 进行追踪
     * 
     * @return Status 
     */
    Status tracking();

    /**
     * @brief 进行初始化
     * 
     * @return Status 
     */
    Status initialize();

    /**
     * @brief 重定位
     * @details TODO 这个就是我们仨需要进行的
     * 
     * @return Status 
     */
    Status relocalize();

    /**
     * @brief 创建新的关键帧
     * 
     * @return true 
     * @return false 
     */
    bool createNewKeyFrame();

    /**
     * @brief TODO 不知道哇
     * 
     */
    void finishFrame();

    /**
     * @brief TODO 计算光的仿射矩阵?
     * 
     */
    void calcLightAffine();

    /**
     * @brief 绘制当前追踪到的点
     * 
     * @param[in] frame 当前帧
     * @param[out] dst  输出对象
     * @notes 华硕这个函数名称是不是写错了啊
     */
    void drowTrackedPoints(const Frame::Ptr &frame, cv::Mat &dst);

private:

    /**
     * @brief TODO 不知道用来做什么的
     * 
     */
    struct Option{
        double min_kf_disparity;        //最小的关键帧视视差
        double min_ref_track_rate;      //最小的,在参考帧上的追踪速率?  TODO 
    } options_;

    //NOTICE 师兄这里的变量名称的最后一个字符如果是下划线的话,表示这个变量是类的成员变量

    //系统当前的阶段和状态
    Stage stage_;
    Status status_;

    //图像相关
    AbstractCamera::Ptr camera_;                //TODO ????
    FastDetector::Ptr fast_detector_;           //fast角点提取器
    FeatureTracker::Ptr feature_tracker_;       //特征追踪
    Initializer::Ptr initializer_;              //初始化
    DepthFilter::Ptr depth_filter_;             //深度滤波器

    //地图相关
    LocalMapper::Ptr mapper_;                   //局部地图

#ifdef SSVO_DBOW_ENABLE
    LoopClosure::Ptr loop_closure_;             //回环检测模块
#endif

    //可视化窗口
    Viewer::Ptr viewer_;
    std::thread viewer_thread_;

    //图像相关
    cv::Mat rgb_;                               //从数据集中读取到的rgb图像?
    Frame::Ptr last_frame_;                     //上一帧
    Frame::Ptr current_frame_;                  //当前帧
    KeyFrame::Ptr reference_keyframe_;          //参考关键帧
    KeyFrame::Ptr last_keyframe_;               //上一个关键帧, NOTICE 注意这个和上面的参考关键帧不同,这还不是一个东西

    //耗时
    double time_;
    
    //回环时添加
    uint64_t loopId_;

    //buffer
    std::list<double > frame_timestamp_buffer_;             //TODO 为什么要存储时间戳?
    std::list<Sophus::SE3d> frame_pose_buffer_;             //存储所有帧的位姿
    std::list<KeyFrame::Ptr> reference_keyframe_buffer_;    //存储所有关键帧
};

}// namespce ssvo

#endif //SSVO_SYSTEM_HPP
