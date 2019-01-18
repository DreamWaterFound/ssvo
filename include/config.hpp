/**
 * @file config.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 从配置文件中加载参数并且导入ssvo
 * @version 0.1
 * @date 2019-01-18
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef _CONFIG_HPP_
#define _CONFIG_HPP_

#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <glog/logging.h>

namespace ssvo{

using std::string;

/**
 * @brief 提供了从配置文件中读取配置文件参数到ssvo的功能
 * @detials 作为静态类被使用
 * 
 */
class Config
{
public:

    /**
     * @names 读取各种参数
     * @{
     */

    /** @brief 图像金字塔层数 */
    static int imageNLevel(){return getInstance().image_nlevel_;}
    /** @brief TODO 像素误差?  */
    static double imagePixelSigma(){return getInstance().image_sigma_;}
    /** @brief 最少角点  */
    static int initMinCorners(){return getInstance().init_min_corners_;}
    /** @brief 最少追踪到的特征点数  */
    static int initMinTracked(){return getInstance().init_min_tracked_;}
    /** @brief 相邻两帧间最小视差? TODO  */
    static int initMinDisparity(){return getInstance().init_min_disparity_;}
    /** @brief 最少的内点数目  */
    static int initMinInliers(){return getInstance().init_min_inliers_;}
    /** @brief 初始的RANSAC迭代次数  */
    static int initMaxRansacIters(){return getInstance().init_max_iters_;}
    /** @brief 网格大小  */
    static int gridSize(){return getInstance().grid_size_;}
    /** @brief 最小网格大小  */
    static int gridMinSize(){return getInstance().grid_min_size_;}
    /** @brief fast角点最大阈值 */
    static int fastMaxThreshold(){return getInstance().fast_max_threshold_;}
    /** @brief fast角点最小阈值 */
    static int fastMinThreshold(){return getInstance().fast_min_threshold_;}
    /** @brief TODO */
    static double fastMinEigen(){return getInstance().fast_min_eigen_;}
    /** @brief 地图尺度 */
    static double mapScale(){return getInstance().mapping_scale_;}
    /** @brief TODO */
    static int minConnectionObservations(){return getInstance().mapping_min_connection_observations_;}
    /** @brief 每帧最少角点个数 */
    static int minCornersPerKeyFrame(){return getInstance().mapping_min_corners_;}
    /** @brief 最大重投影关键帧 */
    static int maxReprojectKeyFrames(){return getInstance().mapping_max_reproject_kfs_;}
    /** @brief 参加局部BA时,最多参加的关键帧数目 */
    static int maxLocalBAKeyFrames(){return getInstance().mapping_max_local_ba_kfs_;}
    /** @brief TODO */
    static int minLocalBAConnectedFts(){return getInstance().mapping_min_local_ba_connected_fts_;}
    /** @brief 图像对齐时涉及到的图像金字塔中最顶层的图像层数 */
    static int alignTopLevel(){return getInstance().align_top_level_;}
    /** @brief 图像对齐时,涉及到的图像金字塔中最底层的层数 */
    static int alignBottomLevel(){return getInstance().align_bottom_level_;}
    /** @brief 图像对齐时使用的图像块大小 */
    static int alignPatchSize(){return getInstance().align_patch_size_;}
    /** @brief 最大追踪的关键帧数目 */
    static int maxTrackKeyFrames(){return getInstance().max_local_kfs_;}
    /** @brief TODO */
    static int minQualityFts(){return getInstance().min_quality_fts_;}
    /** @brief TODO */
    static int maxQualityDropFts(){return getInstance().max_quality_drop_fts_;}
    /** @brief 最大的种子buffer长度 */
    static int maxSeedsBuffer(){return getInstance().max_seeds_buffer_;}
    /** @brief 处理的最大关键帧数目 */
    static int maxPerprocessKeyFrames(){return getInstance().max_perprocess_kfs_;}
    /** @brief TODO */
    static string timeTracingDirectory(){return getInstance().time_trace_dir_;}
    /** @brief 词袋模型中字典的存放位置 */
    static std::string DBoWDirectory(){return getInstance().dbow_dir_;}

    /** @} */

private:
    /**
     * @brief 创建并返回一个config类指针
     * 
     * @return Config& 
     */
    static Config& getInstance()
    {
        static Config instance(file_name_);
        return instance;
    }

    /**
     * @brief Config类构造函数
     * 
     * @param[in] file_name 配置文件路径
     */
    Config(string& file_name)
    {
        //REVIEW 下面代码没有仔细看
        cv::FileStorage fs(file_name.c_str(), cv::FileStorage::READ);
        LOG_ASSERT(fs.isOpened()) << "Failed to open settings file at: " << file_name;

        //! Image
        image_nlevel_ = (int)fs["Image.nlevels"];
        image_sigma_ = (double)fs["Image.sigma"];
        image_sigma2_ = image_sigma_*image_sigma_;

        //! FAST detector parameters
        grid_size_ = (int)fs["FastDetector.grid_size"];
        grid_min_size_ = (int)fs["FastDetector.grid_min_size"];
        fast_max_threshold_ = (int)fs["FastDetector.fast_max_threshold"];
        fast_min_threshold_ = (int)fs["FastDetector.fast_min_threshold"];
        fast_min_eigen_ = (double)fs["FastDetector.fast_min_eigen"];

        //! initializer parameters
        init_min_corners_ = (int)fs["Initializer.min_corners"];
        init_min_tracked_ = (int)fs["Initializer.min_tracked"];
        init_min_disparity_ = (int)fs["Initializer.min_disparity"];
        init_min_inliers_ = (int)fs["Initializer.min_inliers"];
        init_max_iters_ = (int)fs["Initializer.ransac_max_iters"];

        //! map
        mapping_scale_ = (double)fs["Mapping.scale"];
        mapping_min_connection_observations_ = (int)fs["Mapping.min_connection_observations"];
        mapping_min_corners_ = (int)fs["Mapping.min_corners"];
        mapping_max_reproject_kfs_ = (int)fs["Mapping.max_reproject_kfs"];
        mapping_max_local_ba_kfs_ = (int)fs["Mapping.max_local_ba_kfs"];
        mapping_min_local_ba_connected_fts_ = (int)fs["Mapping.min_local_ba_connected_fts"];

        //! Align
        align_top_level_ = (int)fs["Align.top_level"];
        align_top_level_ = MIN(align_top_level_, image_nlevel_-1);
        align_bottom_level_ = (int)fs["Align.bottom_level"];
        align_bottom_level_ = MAX(align_bottom_level_, 0);
        align_patch_size_ = (int)fs["Align.patch_size"];

        //! Tracking
        max_local_kfs_ = (int)fs["Tracking.max_local_kfs"];
        min_quality_fts_ = (int)fs["Tracking.min_quality_fts"];
        max_quality_drop_fts_ = (int)fs["Tracking.max_quality_drop_fts"];

        max_seeds_buffer_ = (int)fs["DepthFilter.max_seeds_buffer"];
        max_perprocess_kfs_ = (int)fs["DepthFilter.max_perprocess_kfs"];

        //! glog
        if(!fs["Glog.alsologtostderr"].empty())
            fs["Glog.alsologtostderr"] >> FLAGS_alsologtostderr;

        if(!fs["Glog.colorlogtostderr"].empty())
            fs["Glog.colorlogtostderr"] >> FLAGS_colorlogtostderr;

        if(!fs["Glog.stderrthreshold"].empty())
            fs["Glog.stderrthreshold"] >> FLAGS_stderrthreshold;

        if(!fs["Glog.minloglevel"].empty())
            fs["Glog.minloglevel"] >> FLAGS_minloglevel;

        if(!fs["Glog.log_prefix"].empty())
            fs["Glog.log_prefix"] >> FLAGS_log_prefix;

        if(!fs["Glog.log_dir"].empty())
            fs["Glog.log_dir"] >> FLAGS_log_dir;

        //! Time Trace
        if(!fs["Trace.log_dir"].empty())
            fs["Trace.log_dir"] >> time_trace_dir_;

        //! DBoW
        if(!fs["DBoW.voc_dir"].empty())
            fs["DBoW.voc_dir"] >> dbow_dir_;

        fs.release();
    }

public:
    //! config file's name TODO 为什么注释写成这种形式?
    ///配置文件的路径
    static string file_name_;

private:

    int image_nlevel_;
    double image_sigma_;
    double image_sigma2_;
    double image_unsigma_;
    double image_unsigma2_;

    //! FAST detector parameters
    int grid_size_;
    int grid_min_size_;
    int fast_max_threshold_;
    int fast_min_threshold_;
    double fast_min_eigen_;

    //! initializer parameters
    int init_min_corners_;
    int init_min_tracked_;
    int init_min_disparity_;
    int init_min_inliers_;
    int init_max_iters_;

    //! map
    double mapping_scale_;
    int mapping_min_connection_observations_;
    int mapping_min_corners_;
    int mapping_max_reproject_kfs_;
    int mapping_max_local_ba_kfs_;
    int mapping_min_local_ba_connected_fts_;

    //! Align
    int align_top_level_;
    int align_bottom_level_;
    int align_patch_size_;

    //! Tracking
    int max_local_kfs_;
    int min_quality_fts_;
    int max_quality_drop_fts_;

    //! DepthFilter
    int max_seeds_buffer_;
    int max_perprocess_kfs_;

    //! TimeTrace
    string time_trace_dir_;
    
    //! DBoW
    std::string dbow_dir_;
};

}

#endif