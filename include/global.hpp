/**
 * @file global.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 包含常用的头文件,并且声明一些常用的功能
 * @version 0.1
 * @date 2019-01-18
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef _GLOBAL_HPP_
#define _GLOBAL_HPP_

#include <cstdlib>
#include <stdint.h>
#include <assert.h>
#include <cmath>

#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <random>
#include <list>
#include <vector>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>
#include <sophus/rxso3.hpp>
#include <glog/logging.h>

#include<Eigen/StdVector>


//NOTICE 这里是处理vector容器和Eigen对象的兼容问题的,以后自己写视觉里程计的时候也要用这个东西
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d)

using namespace Eigen;
using Sophus::SE3d;

//常用的大小函数
#ifndef MIN
    #define MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
    #define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

///图像金字塔,可以看到就是一个图像的数组
typedef std::vector<cv::Mat> ImgPyr;

/**
 * @brief 师兄定义各种功能的名字空间
 * 
 */
namespace ssvo{

//TODO
static std::mt19937_64 rd;
//TODO
static std::uniform_real_distribution<double> distribution(0.0, std::nextafter(1, std::numeric_limits<double>::max()));

//生成均匀分布的随机数
inline double Rand(double min, double max)
{ return (((double)distribution(rd) * (max - min + 1))) + min;}
inline int Rand(int min, int max)
{ return (((double)distribution(rd) * (max - min + 1))) + min;}

/**
 * @brief 基类,它的子类都不可以被复制
 * 
 */
class noncopyable
{
protected:
    noncopyable() = default;
    ~noncopyable() = default;

    noncopyable(const noncopyable&) = delete;
    noncopyable &operator=(const noncopyable&) = delete;
};

}

#endif