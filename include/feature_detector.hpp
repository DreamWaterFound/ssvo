/**
 * @file feature_detector.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 实现特征提取
 * @version 0.1
 * @date 2019-01-18
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef _FEATURE_DETECTOR_HPP_
#define _FEATURE_DETECTOR_HPP_

#include "fast/fast.h"
#include "global.hpp"
#include "grid.hpp"

//! (u,v) is in the n-level image of pyramid
//! (x,y) is in the 0-level image of pyramid
namespace ssvo
{

/** @brief 角点的数据类型 */
struct Corner{
    ///在图像中的坐标x
    float x;        //!< x-coordinate of corner in the image.
    ///在图像中的坐标y
    float y;        //!< y-coordinate of corner in the image.
    ///在图像金字塔中的哪一层被提取出来的
    int level;      //!< pyramid level of the corner.
    ///该角点的shi-tomasi评分.据说比Harris效果要更好
    float score;    //!< shi-tomasi score of the corner.
    //这里禁用掉了角点的方向信息
    //float angle;  //!< for gradient-features: dominant gradient angle.

    /** @brief 默认的构造函数 */
    Corner() : x(-1), y(-1), level(-1), score(-1) {}
    /**
     * @brief 构造函数
     * 
     * @param[in] x         坐标x
     * @param[in] y         坐标y
     * @param[in] score     评分
     * @param[in] level     图层
     */
    Corner(int x, int y, float score, int level) : x(x), y(y), level(level), score(score) {}

    /**
     * @brief 拷贝构造函数
     * 
     * @param other 引用
     */
    Corner(const Corner& other): x(other.x), y(other.y), level(other.level), score(other.score) {}
};

/**
 * @brief 在提取角点的时候使用的网格/
 * @detials 感觉是为了使得特征点分布更加均匀才设置的;并且这个类可能是多个网格的集合
 */
class FastGrid{
public:

    /**
     * @brief 常用的大小
     * @detials 话说写成这个形式究竟有什么好处?
     */
    enum {
        MAX_GRIDS = 600,        ///<最大的网格个数
        MIN_CEIL_SIZE = 30      ///<最小的网格大小
    };

    /**
     * @brief 创建一个网格类对象
     * 
     * @param[in] width             网格的宽度
     * @param[in] height            网格的高度
     * @param[in] cell_size         网格的大小      TODO 这个不是和前面的参数意义重复了吗
     * @param[in] max_threshold     最大阈值? TODO
     * @param[in] min_threshold     最小阈值? TODO
     */
    FastGrid(int width, int height, int cell_size, int max_threshold, int min_threshold);

    /**
     * @brief 获取网格的个数
     * 
     * @return int 
     */
    inline int nCells() const { return N_; }

    /**
     * @brief 根据id获得某个网格的参数
     * 
     * @param[in] id      id
     * @return cv::Rect   某个网格的参数
     */
    inline cv::Rect getCell(int id) const;

    /**
     * @brief 获取阈值
     * @detials 话说这个阈值有什么含义?有什么作用? TODO 
     * \n 注意是每一个网格都有自己的一个阈值
     * @param[in] id    某个网格的id
     * @return int      阈值
     */
    int getThreshold(int id) const;

    /**
     * @brief 设置某个网格的阈值
     * 
     * @param[in] id            网格的id
     * @param[in] threshold     阈值
     * @return true 
     * @return false 
     */
    bool setThreshold(int id, int threshold);

    /**
     * @brief 判断某个cell是否处于图像的边界上
     * @detials 真的是图像的边界上吗? 
     * @param[in] id    cell的id
     * @return true 
     * @return false 
     */
    bool inBoundary(int id) const;

public:

    const int width_;                   ///<TODO 宽度
    const int height_;                  ///<TODO 高度
    const int max_threshold_;           ///<最大阈值
    const int min_threshold_;           ///<最小阈值

private:

    int cell_size_;                     ///<cell的大小
    int cell_n_cols_;                   ///<cell的列数
    int cell_n_rows_;                   ///<cell的行数
    int N_;                             ///<总cell的个数
    std::vector<int> cells_x_;          ///<TODO
    std::vector<int> cells_y_;          ///<TODO
    std::vector<int> fast_threshold_;   ///<存储了每个cell的阈值? TODO 
};

///角点序列类型声明
typedef std::vector<Corner> Corners;

/**
 * @brief fast角点提取器
 * 
 */
class FastDetector: public noncopyable
{
public:
    ///指向当前这个类的指针
    typedef std::shared_ptr<FastDetector> Ptr;

    /**
     * @brief 探测角点
     * 
     * @param[in] img_pyr           图像金字塔 其实就是一个图像的数组
     * @param[out] new_corners      提取到的角点
     * @param[in] exist_corners     TODO 啥?已经存在的角点?
     * @param[in] N                 TODO 要提取的个数?
     * @param[in] eigen_threshold   TODO 啥阈值?
     * @return size_t               TODO 提取到的角点的个数?
     */
    size_t detect(const ImgPyr &img_pyr, Corners &new_corners, const Corners &exist_corners, const int N, const double eigen_threshold = 30.0);

    /**
     * @brief "绘制"图像网格
     * 
     * @param[in]  img      当前图像
     * @param[out] img_grid 绘制了网格的^图像? TODO
     */
    void drawGrid(const cv::Mat &img, cv::Mat &img_grid);

    /**
     * @brief 计算 shi-tomasi 评分
     * 
     * @param[in] img   角点所在的图像
     * @param[in] u     角点的坐标u
     * @param[in] v     角点的坐标v
     * @return float    计算的评分
     */
    static float shiTomasiScore(const cv::Mat &img, int u, int v);

    /**
     * @brief 在图像金字塔的某一层上检测角点
     * 
     * @param[in] img               图像金字塔中的某一层图像
     * @param[in] fast_grid         划分的网格对象
     * @param[out] corners          提取的角点
     * @param[in] eigen_threshold   TODO ????
     * @param[in] border            TODO 边界????
     * @return size_t               提取到的角点的个数
     */
    static size_t detectInLevel(const cv::Mat &img, FastGrid &fast_grid, Corners &corners, const double eigen_threshold=30, const int border=4);

    /**
     * @brief 检测fast角点..
     * @detials 所以和上面的这个函数有什么不同呢?
     * 
     * @param[in] img               要提取的图片
     * @param[out] corners          提取到的角点
     * @param[in] threshold         TODO 阈值?
     * @param[in] eigen_threshold   TODO 阈值?
     */
    static void fastDetect(const cv::Mat &img, Corners &corners, int threshold, double eigen_threshold = 30);

    /**
     * @brief 构造函数
     * @see FastDetector::FastDetector()
     * @param[in] width             图像的宽度
     * @param[in] height            图像的高度
     * @param[in] border            边界...宽度? TODO 
     * @param[in] nlevels           图像金字塔的层数
     * @param[in] grid_size         网格大小
     * @param[in] grid_min_size     网格的最小大小 TODO 为什么会有这个最小的大小?
     * @param[in] max_threshold     最大阈值 TODO 啥阈值?
     * @param[in] min_threshold     最小阈值
     * @return FastDetector::Ptr    实例指针
     */
    inline static FastDetector::Ptr create(int width, int height, int border, int nlevels, int grid_size, int grid_min_size, int max_threshold = 20, int min_threshold = 7)
    {return FastDetector::Ptr(new FastDetector(width, height, border, nlevels, grid_size, grid_min_size, max_threshold, min_threshold));}

    /**
     * @brief 获得图像的宽度
     * 
     * @return int 宽度
     */
    int getWidth(){ return width_;}

    /**
     * @brief 获得图像的高度
     * 
     * @return int 高度
     */
    int getHeight(){ return height_;}

private:

    /**
     * @brief 构造函数
     * @param[in] width             图像的宽度
     * @param[in] height            图像的高度
     * @param[in] border            边界...宽度? TODO 
     * @param[in] nlevels           图像金字塔的层数
     * @param[in] grid_size         网格大小
     * @param[in] grid_min_size     网格的最小大小 TODO 为什么会有这个最小的大小?
     * @param[in] max_threshold     最大阈值 TODO 啥阈值?
     * @param[in] min_threshold     最小阈值
     * @return FastDetector::Ptr    实例指针
     */
    FastDetector(int width, int height, int border, int nlevels, int grid_size, int grid_min_size, int max_threshold, int min_threshold);

    /**
     * @brief 设置网格的掩摸??
     * 
     * @param[in] grid      网格? TODO
     * @param[in] corners   角点?? TODO 
     */
    static void setGridMask(Grid<Corner> &grid, const Corners &corners);

    /**
     * @brief 设置角点??? 
     * @detials 这玩意儿还能够自定义设置??? TODO 
     * 
     * @param[in] grid      网格 TODO 
     * @param[in] corners   角点 TODO  
     */
    static void setCorners(Grid<Corner> &grid, const Corners &corners);

private:

    ///图像的宽度
    const int width_;
    ///图像的高度
    const int height_;
    ///图像的边界..??? TODO 
    const int border_;
    ///图像金字塔的层数 TODO 还是当前图像所在的层数哇?
    const int nlevels_;
    ///TODO 一共提取到的特征点个数吗?
    int N_;

    ///最小网格大小
    const int grid_min_size_;
    ///TODO ????
    bool size_adjust_;
    ///最大阈值 TODO
    const int max_threshold_;
    ///最小阈值 TODO
    const int min_threshold_;
    ///阈值 ...TODO
    int threshold_;

    //! 不能删除。多个线程用到了特征提取
    ///线程锁
    std::mutex mutex_fast_detector_;

    ///网各类的序列 TODO 
    std::vector<FastGrid> detect_grids_;

    ///在这一层的图像中提取到的角点的序列
    std::vector<Corners> corners_in_levels_;
    ///TODO ??
    Grid<Corner> grid_filter_;
};

}//! end of ssvo

#endif