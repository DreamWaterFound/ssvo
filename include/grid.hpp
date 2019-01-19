/**
 * @file grid.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 图像网格
 * @version 0.1
 * @date 2019-01-19
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef _SSVO_GRID_HPP_
#define _SSVO_GRID_HPP_

#include "global.hpp"

//REVIEW 整篇代码都没有看


namespace ssvo
{

/**
 * @brief 图像网格类
 * 
 * @tparam T 每个cell的类型
 */
template<typename T>
class Grid
{
public:
    ///cell列表类型
    typedef std::list<T> Cell;
    ///cell向量类型
    typedef std::vector<std::shared_ptr<Cell> > Cells;
    ///TODO 上面的两个有什么不同吗?

    ///REVIEW 查看是不是所有的属于 不可拷贝 的类,都是使用实例指针的方式来新建的

    /**
     * @brief 网格的构造函数
     * 
     * @param[in] cols 列
     * @param[in] rows 行
     * @param[in] size 大小
     */
    Grid(size_t cols, size_t rows, size_t size) :
        cols_(cols), rows_(rows), area_(cols * rows), grid_size_(size),
        grid_n_cols_(0), grid_n_rows_(0), grid_n_cells_(0),
        cells_(std::make_shared<Cells>(Cells()))
    {
        //REVIEW 代码没看
        reset(grid_size_);
    }

    /**
     * @brief 复位网格设置
     * 
     * @param[in] grid_size 网格大小
     */
    void reset(size_t grid_size)
    {
        //REVIEW 代码没看
        grid_size_ = grid_size;
        grid_n_cols_ = ceil(static_cast<double>(cols_) / grid_size_);
        grid_n_rows_ = ceil(static_cast<double>(rows_) / grid_size_);
        grid_n_cells_ = grid_n_cols_ * grid_n_rows_;

        cells_.reset(new Cells(grid_n_cells_));
        for(std::shared_ptr<Cell> &cell : *cells_)
        { cell = std::make_shared<Cell>(Cell()); }

        mask_.clear();
        mask_.resize(grid_n_cells_, false);
    }

    /**
     * @brief 清除啥啊 TODO 
     * 
     */
    void clear()
    {
        //REVIEW 代码没看
        for(std::shared_ptr<Cell> &cell : *cells_)
        { cell->clear(); }
        std::fill(mask_.begin(), mask_.end(), false);
    }

    /**
     * @brief 排序啥啊 TODO 
     * 
     */
    void sort()
    {
        //REVIEW 代码没看
        for(std::shared_ptr<Cell> &cell : *cells_)
        { cell->sort(); }
    }

    /**
     * @brief 啥大小啊 TODO 
     * 
     * @return size_t 可能获取的是每个网格的大小 TODO 
     */
    size_t size()
    {
        //REVIEW 代码没看
        return (size_t) std::count_if(cells_->begin(), cells_->end(), [](const std::shared_ptr<Cell> &cell) { return !cell->empty(); });
    }

    /**
     * @brief 重设网格大小
     * 
     * @param[in] grid_size 要重新设置的网格的大小
     */
    void resize(size_t grid_size)
    {
        if(grid_size == grid_size_)
            return;
        std::shared_ptr<Cells> old_cells = cells_;
        reset(grid_size);
        for(std::shared_ptr<Cell> &cell : *old_cells)
            for(const T &element : *cell)
                insert(element);
    }

    /**
     * @brief 插入对象? TODO 
     * 
     * @param[in] element 需要插入的对象? TODO 
     * @return size_t TODO 
     */
    inline size_t insert(const T &element)
    {
        const size_t id = getIndex(element);
        if(mask_.at(id))
            return 0;
        const std::shared_ptr<Cell> &cell = cells_->at(id);
        cell->push_back(element);
        return (size_t) cell->size();
    }

    /**
     * @brief 从图像网格中删除某个对象? TODO
     * 
     * @param[in] element 要删除的对象
     * @return size_t 删除后什么的大小?  TODO 
     */
    inline size_t remove(const T &element)
    {
        const size_t id = getIndex(element);
        if(mask_.at(id))
            return 0;
        const std::shared_ptr<Cell> &cell = cells_->at(id);
        cell->remove(element);
        return (size_t) cell->size();
    }

    /**
     * @brief 获取其中某个元素的id
     * 
     * @param[in] element 要查询的元素
     * @return size_t 它的id
     */
    size_t getIndex(const T &element)
    {
        std::cerr << "Do not use the function[ size_t getCellID(T &element) ]! Please Specialized!" << std::endl;
        std::abort();
    }

    /**
     * @brief 获取最佳的元素? TODO 
     * 
     * @param[in] out TODO ?
     */
    void getBestElement(Cell &out)
    {
        out.clear();
        for(size_t idx = 0; idx < grid_n_cells_; idx++)
        {
            if(mask_.at(idx))
                continue;
            std::shared_ptr<Cell> &cell = cells_->at(idx);
            out.push_back(*cell->rbegin());
            if(!cell->empty())
                out.push_back(*std::max_element(cell->begin(), cell->end()));
        }
    }

    /**
     * @brief 获取最佳的元素? TODO 
     * 
     * @param[in] out TODO ?
     */
    void getBestElement(std::vector<T> &out)
    {
        out.clear();
        out.reserve(size());
        for(size_t idx = 0; idx < grid_n_cells_; idx++)
        {
            if(mask_.at(idx))
                continue;
            std::shared_ptr<Cell> &cell = cells_->at(idx);
            if(!cell->empty())
                out.push_back(*std::max_element(cell->begin(), cell->end()));
        }
    }

    /**
     * @brief 设置掩摸?  TODO 
     * 
     * @param[in] mask 要设置的掩摸
     */
    void setMask(const std::vector<bool> &mask)
    {
        assert(mask.size() == grid_n_cells_);
        mask_ = mask;
    }

    /**
     * @brief 设置掩摸 TODO 
     * 
     * @param[in] index TODO 什么的索引?
     * @param[in] value 要设置的值
     */
    void setMask(size_t index, bool value = true)
    {
        assert(index >= 0 && index < grid_n_cells_);
        mask_.at(index) = value;
    }

    /**
     * @brief 查看某个索引处是否存在掩摸
     * 
     * @param[in] index 索引
     * @return true 
     * @return false 
     */
    bool isMasked(size_t index)
    {
        assert(index >= 0 && index < grid_n_cells_);
        return mask_.at(index);
    }

    /**
     * @brief 获取列数
     * 
     * @return const size_t 列数
     */
    inline const size_t cols()
    {
        return cols_;
    }

    /**
     * @brief 获取行数
     * 
     * @return const size_t 行数
     */
    inline const size_t rows()
    {
        return rows_;
    }

    /**
     * @brief 获取区域大小
     * 
     * @return const size_t 区域大小
     */
    inline const size_t area()
    {
        return area_;
    }

    /**
     * @brief 获取cell的数目
     * 
     * @return const size_t cell的数目
     */
    inline const size_t nCells()
    {
        return grid_n_cells_;
    }

    /**
     * @brief 获取grid的大小
     * 
     * @return const size_t Grid的大小
     */
    inline const size_t gridSize()
    {
        return grid_size_;
    }

    /**
     * @brief 获取一个cell对象
     * 
     * @param[in] id id
     * @return Cell& 要获取的cell对象
     */
    inline Cell &getCell(size_t id)
    {
        return *cells_->at(id);
    }

    /**
     * @brief 获取一系列的cell
     * 
     * @return Cells& 获得到cells的指针
     */
    inline Cells &getCells()
    {
        return *cells_;
    }

private:

    //TODO 下面的都是什么的行列和面积?
    ///当前cell对象的列
    const size_t cols_;
    ///当前cell对象的行
    const size_t rows_;
    ///当前cell对象的面积?? TODO
    const size_t area_;
    
    //TODO 下面的呢?
    ///当前Grid对象的面积?? TODO
    size_t grid_size_;
    ///当前Grid对象的列
    size_t grid_n_cols_;
    ///当前Grid对象的行
    size_t grid_n_rows_;
    ///当前Grid对象的cell数
    size_t grid_n_cells_;

    ///存储了cell的列表头指针
    std::shared_ptr<Cells> cells_;
    ///掩摸 TODO 不知道是做啥的
    std::vector<bool> mask_;
};  //class grid 

/**
 * @brief 重新设置网格的自适应???/
 * @detials 注意这个是个全局函数,不是Grid类的成员
 * @tparam T            网格中每个cell的数据类型
 * @param[in] grid      网格对象
 * @param[in] N         TODO 
 * @param[in] min_size  TODO 最小大小
 */
template <typename T>
void resetGridAdaptive(Grid<T> &grid, const int N, const int min_size)
{
    const int MAX_SIZE = static_cast<int>(1.1*N);
    const int MIN_SIZE = static_cast<int>(0.9*N);

    int count = 0;
//    double time0 = (double)cv::getTickCount();
    while(count++ < 5)
    {
        const int now_size = grid.size();

        if(now_size <= MAX_SIZE && now_size >= MIN_SIZE)
            break;

        const float corners_per_grid = 1.0 * now_size / (grid.nCells());
        const float n_grid = N / corners_per_grid;

        int new_size = 0;
        if(now_size > MAX_SIZE)
            new_size = ceil(std::sqrt(grid.area() / n_grid)) + 1;
        else if(now_size < MIN_SIZE)
            new_size = floor(std::sqrt(grid.area() / n_grid)) - 1;

        new_size = MAX(new_size, min_size);

        if(grid.gridSize() == new_size)
            break;

        grid.resize(new_size);

        LOG_ASSERT(new_size < grid.cols() || new_size < grid.rows()) << "Error Grid Size: " << new_size;
    }

    //TODO 下面又为什么注释掉呢
//    double time1 = (double)cv::getTickCount();
//    std::cout << "time: " << (time1-time0)/cv::getTickFrequency() << ", n:" << count-1 << std::endl;

}   //void resetGridAdaptive()

} //ssvo

#endif //_SSVO_GRID_HPP_
