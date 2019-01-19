/**
 * @file frame.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 帧类的实现
 * @version 0.1
 * @date 2019-01-19
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef _FRAME_HPP_
#define _FRAME_HPP_

#include "global.hpp"
#include "camera.hpp"
#include "feature.hpp"
#include "map_point.hpp"
#include "seed.hpp"
#include "feature_detector.hpp"

namespace ssvo{

class KeyFrame;

/**
 * @brief 帧
 * 
 */
class Frame : public noncopyable
{
public:

    /**
     * @brief 析构函数
     * @detials 由子类进行具体实现
     * 
     */
    virtual ~Frame() {};
    //指向自己类的指针
    typedef std::shared_ptr<Frame> Ptr;
    /**
     * @brief 获得图像金字塔
     * 
     * @return const ImgPyr 图像金字塔
     */
    const ImgPyr images() const;
    /**
     * @brief 光流图像金字塔
     * @detials 和上面的有什么不同?
     * @return const ImgPyr    图像金字塔
     */
    const ImgPyr opticalImages() const;
    /**
     * @brief 获取图像金字塔中的某层图像
     * 
     * @param[in] level      指定的图像金字塔层数
     * @return const cv::Mat 得到的图像
     */
    const cv::Mat getImage(int level) const;

    //! Transform (c)amera from (w)orld
    /**
     * @brief 帧从世界坐标系到相机坐标系的变换
     * 
     * @return SE3d Tcw
     */
    SE3d Tcw();

    //! Transform (w)orld from (c)amera
    /**
     * @brief 帧从相机坐标系到世界坐标系的变换
     * 
     * @return SE3d Twc
     */
    SE3d Twc();

    //! Transform (w)orld from (c)amera
    /**
     * @brief 获取位姿? TODO 和上面的有什么不同?
     * 
     * @return SE3d TODO 
     */
    SE3d pose();

    //! Principal ray in world frame
    /**
     * @brief 获取主光线的方向? TODO 
     * 
     * @return Vector3d TODO 什么射线的方向向量???
     */
    Vector3d ray();

    //! Set pose in world frame
    /**
     * @brief 设置帧的在世界坐标系下的位姿
     * 
     * @param[in] pose SE3表示的位姿
     */
    void setPose(const SE3d& pose);

    //! Set pose in world frame
    /**
     * @brief 设置帧的在世界坐标系下的位姿
     * 
     * @param[in] R 旋转
     * @param[in] t 平移
     */
    void setPose(const Matrix3d& R, const Vector3d& t);

    //! Set Extrinsic Matrix
    /**
     * @brief Set the Tcw 
     * 
     * @param[in] Tcw 
     */
    void setTcw(const SE3d& Tcw);

    /**
     * @brief 判断在某个空间点是否是当前帧的可视点
     * 
     * @param[in] xyz_w     空间点坐标
     * @param[in] border    TODO ????
     * @return true 
     * @return false 
     */
    bool isVisiable(const Vector3d &xyz_w, const int border = 0);

    //! Feature created by MapPoint
    /**
     * @brief TODO 通过地图点来创建 ssvo::Feature 类对象?
     * 
     * @return int TODO ? 特征对象的id?
     */
    int featureNumber();

    /**
     * @brief 获得本帧中的所有特征,其中同时也保存了对应的地图点数据
     * 
     * @return std::unordered_map<MapPoint::Ptr, Feature::Ptr> 保存在一个无序映射表中
     */
    std::unordered_map<MapPoint::Ptr, Feature::Ptr> features();
    /**
     * @brief 获取存储特征的列表
     * 
     * @return std::vector<Feature::Ptr> 存储特征的列表
     */
    std::vector<Feature::Ptr> getFeatures();

    /**
     * @brief 获取存储地图点的列表
     * 
     * @return std::vector<MapPoint::Ptr> 存储地图点的列表
     */
    std::vector<MapPoint::Ptr> getMapPoints();

    //! keep the fts and mappoints order right
    /**
     * @brief 同时获取存储特征的列表和存储地图点的列表
     * 
     * @param[out] features     保存特征的容器
     * @param[out] mappoints    保存地图点的容器
     */
    void getFeaturesAndMapPoints(std::vector<Feature::Ptr> &features, std::vector<MapPoint::Ptr> &mappoints);
    /**
     * @brief 向当前帧中添加特征
     * 
     * @param[in] ft    特征对象的句柄  TODO REVIEW 原来在这里 ft 的写法都是表示特征对象?
     * @return true 
     * @return false 
     */
    bool addFeature(const Feature::Ptr &ft);

    /**
     * @brief 从当前帧中移除特征
     * 
     * @param[in] ft    要移除的特征对象
     * @return true 
     * @return false 
     */
    bool removeFeature(const Feature::Ptr &ft);

    /**
     * @brief 从当前帧中移除地图点
     * 
     * @param[in] mpt   要移除的地图点
     * @return true 
     * @return false 
     */
    bool removeMapPoint(const MapPoint::Ptr &mpt);

    /**
     * @brief 通过地图点找到对应的特征
     * 
     * @param[in] mpt           地图点
     * @return Feature::Ptr     特征的实例句柄
     */
    Feature::Ptr getFeatureByMapPoint(const MapPoint::Ptr &mpt);

    //! Feature created by Seed
    /**
     * @brief 通过种子来生成特征? TODO 
     * 
     * @return int TODO 
     */
    int seedNumber();

    /**
     * @brief 获取种子
     * 
     * @return std::vector<Feature::Ptr> 保存有特征的容器
     */
    std::vector<Feature::Ptr> getSeeds();
    /**
     * @brief 向当前帧中添加种子
     * 
     * @param[in] ft    对应的特征
     * @return true 
     * @return false 
     */
    bool addSeed(const Feature::Ptr &ft);
    /**
     * @brief 在当前帧中移除种子
     * 
     * @param[in] seed 要移除的种子
     * @return true 
     * @return false 
     */
    bool removeSeed(const Seed::Ptr &seed);
    /**
     * @brief 确定当前帧中是否存在这个种子
     * 
     * @param[in] seed 要检查的种子
     * @return true 
     * @return false 
     */
    bool hasSeed(const Seed::Ptr &seed);
    /**
     * @brief 获取深度尺度
     * 
     * @param[TODO] depth_mean 深度的均值
     * @param[TODO] depth_min   最小深度值
     * @return true 
     * @return false 
     */
    bool getSceneDepth(double &depth_mean, double &depth_min);
    /**
     * @brief TODO 
     * 
     * @return std::map<std::shared_ptr<KeyFrame>, int> 保存有关键帧序列的容器
     */
    std::map<std::shared_ptr<KeyFrame>, int> getOverLapKeyFrames();

    /**
     * @brief 设置某个关键帧为当前帧的参考关键帧
     * 
     * @param[in] kf 要设置的参考关键帧
     */
    inline void setRefKeyFrame(const std::shared_ptr<KeyFrame> &kf) {ref_keyframe_ = kf;}
    /**
     * @brief 获取当前帧的参考关键帧
     * 
     * @return std::shared_ptr<KeyFrame> 本帧的参考关键帧
     */
    inline std::shared_ptr<KeyFrame> getRefKeyFrame() const {return ref_keyframe_;}

    /**
     * @brief 构造函数
     * 
     * @param[in] img           本帧的图片
     * @param[in] timestamp     本帧的时间戳
     * @param[in] cam           本帧的相机模型  TODO 这个为什么也要添加? 不应该是相机模型在整个SLAM的运动过程中都是保持不变的吗?
     * @return Ptr              实例指针
     */
    inline static Ptr create(const cv::Mat& img, const double timestamp, AbstractCamera::Ptr cam)
    { return Ptr(new Frame(img, timestamp, cam)); }

protected:

    /**
     * @brief 构造函数
     * 
     * @param[in] img           本帧的图片
     * @param[in] timestamp     本帧的时间戳
     * @param[in] cam           本帧的相机模型  TODO 这个为什么也要添加? 不应该是相机模型在整个SLAM的运动过程中都是保持不变的吗?
     */
    Frame(const cv::Mat& img, const double timestamp, const AbstractCamera::Ptr &cam);

    /**
     * @brief 构造函数
     * 
     * @param[in] img_pyr       图像金字塔
     * @param[in] id            当前帧的id
     * @param[in] timestamp     当前帧的时间戳
     * @param[in] cam           当前帧的相机模型
     */
    Frame(const ImgPyr& img_pyr, const uint64_t id, const double timestamp, const AbstractCamera::Ptr &cam);

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ///下一帧的id
    static uint64_t next_id_;
    ///当前帧的id
    const uint64_t id_;
    ///当前帧的时间戳
    const double timestamp_;

    ///当前帧的相机模型
    AbstractCamera::Ptr cam_;

    ///图像金字塔的最大层数?  TODO 
    const int max_level_;
    ///光流窗口的大小
    static const cv::Size optical_win_size_;
    ///TODO 参数a
    static float light_affine_a_;
    ///TODO 参数b
    static float light_affine_b_;

    ///光流得到的Tcw
    SE3d optimal_Tcw_;//! for optimization

    ///TODO 视差
    double disparity_;//! for depth filter

    ///TODO 在进行全局BA之前的Tcw
    SE3d beforeGBA_Tcw_;
    ///TODO 在进行全局BA之后的Tcw
    SE3d beforeUpdate_Tcw_;

protected:

    ///特征点和地图点对
    std::unordered_map<MapPoint::Ptr, Feature::Ptr> mpt_fts_;
    ///特征点和种子点对
    std::unordered_map<Seed::Ptr, Feature::Ptr> seed_fts_;

    ///当前帧的图像金字塔
    ImgPyr img_pyr_;

    ///从世界坐标系到当前帧相机坐标系的位姿变换
    SE3d Tcw_;
    ///从当前帧相机坐标系到世界坐标系的位姿变换
    SE3d Twc_;
    ///TODO 
    Vector3d Dw_;

    ///当前帧的参考关键帧
    std::shared_ptr<KeyFrame> ref_keyframe_;

    ///线程锁
    std::mutex mutex_pose_;
    std::mutex mutex_feature_;
    std::mutex mutex_seed_;

private:
    ///光流金字塔 TODO 和上面的图像金字塔有啥不同吗?
    ImgPyr optical_pyr_;
};

}

#endif