/**
 * @file camera.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 相机模型和基本投影,去畸变功能定义
 * @version 0.1
 * @date 2019-01-18
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef _CAMERA_HPP_
#define _CAMERA_HPP_

#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "global.hpp"

using namespace Eigen;

namespace ssvo {

// once created, never changed
/**
 * @brief 相机的抽象类
 * @detials 一旦创建就不能够被复制,也不能够被改变. 其中很多函数是虚函数,需要在子类中加以实现
 * 
 */
class AbstractCamera : public noncopyable
{
public:

    /**
     * @brief 当前相机的类型id
     * 
     */
    enum Model {
        UNKNOW      = -2,
        ABSTRACT    = -1,
        PINHOLE     = 0,
        ATAN        = 1
    };

    //TODO 为什么要加这个东西?
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //定义指向自己的智能指针类型
    typedef std::shared_ptr<AbstractCamera> Ptr;

    /**
     * @brief Construct a new Abstract Camera object
     * 
     */
    AbstractCamera();

    /**
     * @brief Construct a new Abstract Camera object
     * @notes TODO 这样写的话,如果在构造是没有给参数,是执行上面的构造函数还是执行这个构造函数呢?这样写没有问题吗?
     * 
     * @param[in] model 相机的模型id
     */
    AbstractCamera(Model model = ABSTRACT);

    /**
     * @brief Construct a new Abstract Camera object
     * 
     * @param[in] width     相机图像的宽度
     * @param[in] height    相机图像的高度
     * @param[in] model     相机模型
     */
    AbstractCamera(int width, int height, Model model = ABSTRACT);

    /**
     * @brief Construct a new Abstract Camera object
     * 
     * @param[in] width     相机图像的宽度
     * @param[in] height    相机图像的高度
     * @param[in] fx        相机的焦距x
     * @param[in] fy        焦距y
     * @param[in] cx        光心cx
     * @param[in] cy        光心cy
     * @param[in] model     相机模型id
     */
    AbstractCamera(int width, int height, double fx, double fy, double cx, double cy, Model model = ABSTRACT);

    /**
     * @brief Destroy the Abstract Camera object
     * @detials NOTICE 是虚函数,可以由子类重写
     * 
     */
    virtual ~AbstractCamera() {};

    /**
     * @brief 静态成员函数,从给定的文件中来确认相机的各项参数
     * 
     * @param[in] calib_file 相机矫正参数文件
     * @return Model         相机的类型ID
     */
    static Model checkCameraModel(std::string calib_file);

    /**
     * @names 获取相机模型的具体的某个参数
     * @{
     */

    inline const int fps() const { return fps_; }

    inline const int width() const { return width_; }

    inline const int height() const { return height_; }

    inline const double fx() const { return fx_; };

    inline const double fy() const { return fy_; };

    inline const double cx() const { return cx_; };

    inline const double cy() const { return cy_; };

    inline const cv::Mat K() const { return K_; };

    //TODO D是啥?去畸变参数?
    inline const cv::Mat D() const { return D_; };

    //TODO 这个又是啥?
    inline const cv::Mat T_BC() const { return T_BC_; };

    inline const Model model() const { return model_; }

    /** @} */

    ///TODO
    virtual Vector3d lift(const Vector2d& px) const;
    ///TODO
    virtual Vector3d lift(double x, double y) const;

    /**
     * @brief 将某个的=空间点投影到相机平面
     * 
     * @param[in] xyz  空间点坐标
     * @return Vector2d 投影到相机像素平面上面的坐标 TODO 还是到归一化平面上?
     */
    virtual Vector2d project(const Vector3d& xyz) const;

    /**
     * @brief TODO  ??? 将像素平面上的点投影到归一化相机平面上?
     * 
     * @param[in] x         x坐标
     * @param[in] y         y坐标
     * @return Vector2d     TODO ???
     */
    virtual Vector2d project(double x, double y) const;

    /**
     * @brief 对给定的特征点序列进行去畸变操作
     * 
     * @param[in] pts_dist      等待去畸变的点
     * @param[out] pts_udist    完成去畸变的点
     */
    virtual void undistortPoints(const std::vector<cv::Point2f> &pts_dist, std::vector<cv::Point2f> &pts_udist) const;

    /**
     * @brief 判断某个点是否在相机的视野内
     * 
     * @param[in] obs           点的像素坐标
     * @param[in] boundary      图像的边界
     * @return true 
     * @return false 
     */
    inline bool isInFrame(const Vector2i &obs, int boundary=0) const
    {
        //REVIEW
        if(obs[0] >= boundary && obs[0] < width() - boundary
            && obs[1] >= boundary && obs[1] < height() - boundary)
            return true;
        return false;
    }

    /**
     * @brief 判断给定的点是否在图像中
     * 
     * @param[in] obs       点的像素坐标
     * @param[in] boundary  图像的边界
     * @param[in] level     这个点是在图像金字塔中的哪一层
     * @return true 
     * @return false 
     */
    inline bool isInFrame(const Vector2i &obs, int boundary, int level) const
    {
        if(obs[0] >= boundary && obs[0] < (width() >> level) - boundary
            && obs[1] >= boundary && obs[1] < (height() >> level) - boundary)
            return true;
        return false;
    }

protected:

    ///相机模型
    const Model model_;
    ///帧率
    double fps_;
    ///画面宽度
    int width_;
    ///画面高度
    int height_;
    ///相机内参数
    double fx_, fy_, cx_, cy_;
    ///相机去畸变系数
    cv::Mat K_, D_;
    ///TODO 
    cv::Mat T_BC_;
    ///是否去畸变??? TODO
    bool distortion_;
};

/**
 * @brief 针孔相机模型类
 * 
 */
class PinholeCamera : public AbstractCamera
{

public:

    ///指向自己的智能指针类型
    typedef std::shared_ptr<PinholeCamera> Ptr;

    //TODO 
    virtual Vector3d lift(const Vector2d& px) const;
    virtual Vector3d lift(double x, double y) const;

    //TODO 作什么投影>?
    virtual Vector2d project(const Vector3d& xyz) const;
    virtual Vector2d project(double x, double y) const;

    //! all undistort points are in the normlized plane  NOTICE 
    /**
     * @brief 对点进行去畸变
     * @detials NOTICE 去畸变之后的点都是在归一化平面上
     * @param pts_dist 
     * @param pts_udist 
     */
    virtual void undistortPoints(const std::vector<cv::Point2f> &pts_dist, std::vector<cv::Point2f> &pts_udist) const;

    /**
     * @brief 创建一个针孔相机模型实例
     * 
     * @param[in] width     图像宽度
     * @param[in] height    图像高度
     * @param[in] fx        焦距x
     * @param[in] fy        焦距y
     * @param[in] cx        光心x
     * @param[in] cy        光心y
     * @param[in] k1        去畸变参数
     * @param[in] k2        去畸变参数
     * @param[in] p1        去畸变参数
     * @param[in] p2        去畸变参数
     * @return PinholeCamera::Ptr   实例句柄
     */
    inline static PinholeCamera::Ptr create(int width, int height, double fx, double fy, double cx, double cy, double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0)
    {return PinholeCamera::Ptr(new PinholeCamera(width, height, fx, fy, cx, cy, k1, k2, p1, p2));}

    /**
     * @brief 创建针孔相机实例
     * 
     * @param[in] width     图像宽度
     * @param[in] height    图像高度
     * @param[in] K         相机内参矩阵
     * @param[in] D         去畸变参数矩阵
     * @return PinholeCamera::Ptr 实例句柄
     */
    inline static PinholeCamera::Ptr create(int width, int height, const cv::Mat& K, const cv::Mat& D)
    {return PinholeCamera::Ptr(new PinholeCamera(width, height, K, D));}

    /**
     * @brief 创建针孔相机实例
     * 
     * @param[in] calib_file 相机参数文件
     * @return PinholeCamera::Ptr 实例句柄
     */
    inline static PinholeCamera::Ptr create(std::string calib_file)
    {return PinholeCamera::Ptr(new PinholeCamera(calib_file));}

private:

    /**
     * @brief 创建一个针孔相机模型实例
     * 
     * @param[in] width     图像宽度
     * @param[in] height    图像高度
     * @param[in] fx        焦距x
     * @param[in] fy        焦距y
     * @param[in] cx        光心x
     * @param[in] cy        光心y
     * @param[in] k1        去畸变参数
     * @param[in] k2        去畸变参数
     * @param[in] p1        去畸变参数
     * @param[in] p2        去畸变参数
     * @return PinholeCamera::Ptr   实例句柄
     */
    PinholeCamera(int width, int height, double fx, double fy, double cx, double cy,
           double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0);
    /**
     * @brief 创建针孔相机实例
     * 
     * @param[in] width     图像宽度
     * @param[in] height    图像高度
     * @param[in] K         相机内参矩阵
     * @param[in] D         去畸变参数矩阵
     * @return PinholeCamera::Ptr 实例句柄
     */
    PinholeCamera(int width, int height, const cv::Mat& K, const cv::Mat& D);

    /**
     * @brief 创建针孔相机实例
     * 
     * @param[in] calib_file 相机参数文件
     * @return PinholeCamera::Ptr 实例句柄
     */
    PinholeCamera(std::string calib_file);

private:

    ///针孔相机的去畸变参数
    double k1_, k2_, p1_, p2_;

};

/**
 * @brief 另外一种名为ATAN的相机模型
 * @note REVIEW 因为用不着,所以下面的就暂时不进行注解了
 * 
 */
class AtanCamera : public AbstractCamera
{

public:

    typedef std::shared_ptr<AtanCamera> Ptr;

    virtual Vector3d lift(const Vector2d& px) const;

    virtual Vector3d lift(double x, double y) const;

    virtual Vector2d project(const Vector3d& xyz) const;

    virtual Vector2d project(double x, double y) const;

    virtual void undistortPoints(const std::vector<cv::Point2f> &pts_dist, std::vector<cv::Point2f> &pts_udist) const;

    inline static AtanCamera::Ptr create(int width, int height, double fx, double fy, double cx, double cy, double s = 0.0)
    {return AtanCamera::Ptr(new AtanCamera(width, height, fx, fy, cx, cy, s));}

    inline static AtanCamera::Ptr create(int width, int height, const cv::Mat& K, const double s = 0.0)
    {return AtanCamera::Ptr(new AtanCamera(width, height, K, s));}

    inline static AtanCamera::Ptr create(std::string calib_file)
    {return AtanCamera::Ptr(new AtanCamera(calib_file));}

private:

    AtanCamera(int width, int height, double fx, double fy, double cx, double cy, double s = 0.0);

    AtanCamera(int width, int height, const cv::Mat& K, const double s = 0.0);

    AtanCamera(std::string calib_file);

private:

    double s_;
    double tans_;
};

}

#endif