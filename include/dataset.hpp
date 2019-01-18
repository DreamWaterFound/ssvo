/**
 * @file dataset.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 实现数据集的读取
 * @version 0.1
 * @date 2019-01-18
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef _SSVO_DATASET_HPP_
#define _SSVO_DATASET_HPP_

#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>

namespace ssvo {

/**
 * @brief 实现读取TUMD数据集
 * 
 */
class TUMDataReader {
public:

    /**
     * @brief Construct a new TUMDataReader
     * 
     * @param[in] dataset_path          数据集的路径
     * @param[in] association_file      关联文件,TUM数据集特有的
     * @param[in] with_ground_truth     存储了真值的文件? 
     */
    TUMDataReader(const string &dataset_path, const string &association_file, const bool with_ground_truth = false):
        dataset_path_(dataset_path), association_file_(association_file)
    {
        //REVIEW  于问题研究无关,程序代码先不看
        size_t found = dataset_path.find_last_of("/\\");
        if(found + 1 != dataset_path.size())
            dataset_path_ += dataset_path.substr(found, 1);

        std::ifstream file_stream;
        file_stream.open(association_file_.c_str());
        while (!file_stream.eof()) {
            string s;
            getline(file_stream, s);
            if (!s.empty()) {
                std::stringstream ss;
                ss << s;
                double time;
                string rgb, depth;
                ss >> time;
                timestamps_.push_back(time);
                ss >> rgb;
                rgb_images_.push_back(dataset_path_ + rgb);
                ss >> time;
                ss >> depth;
                depth_images_.push_back(dataset_path_ + depth);
                ss >> time;
                if(with_ground_truth)
                {
                    std::vector<double> ground_truth(7);
                    for(int i = 0; i < 7; ++i)
                       ss >> ground_truth[i];
                    groundtruth_data_.push_back(ground_truth);
                }
            }
        }
        file_stream.close();

        N = timestamps_.size();
        if(N == 0)
            std::cerr << "No item read! Please check association file: " << association_file << std::endl;
        else
            std::cerr << "Avaiable image items in dataset: " << N << std::endl;

    }

    /**
     * @brief 根据索引,读取图像
     * 
     * @param[in]  index         图像索引
     * @param[out] rgb_image     彩色图像文件名
     * @param[out] depth_image   深度图像文件名
     * @param[out] timestamp     时间戳
     * @return true 
     * @return false 
     */
    bool readItemByIndex(size_t index, std::string &rgb_image, std::string &depth_image, double &timestamp) const
    {
        if(index >= N)
        {
            std::cerr << " Index(" << index << ") is out of scape, max should be (0~" << N - 1 << ")";
            return false;
        }
        rgb_image = rgb_images_[index];
        depth_image = depth_images_[index];
        timestamp = timestamps_[index];
        return true;
    }

    //! used for some TUM dataset
    /**
     * @brief 根据索引获得数据集中的彩色图像
     * 
     * @param[in]  index        索引
     * @param[out] rgb_image    彩色图像
     * @param[out] timestamp    时间戳
     * @return true 
     * @return false 
     */
    bool readItemByIndex(size_t index, std::string &rgb_image, double &timestamp) const
    {
        if(index >= N)
        {
            std::cerr << " Index(" << index << ") is out of scape, max should be (0~" << N - 1 << ")";
            return false;
        }
        rgb_image = rgb_images_[index];
        timestamp = timestamps_[index];
        return true;
    }

    /**
     * @brief 根据索引,读取数据集中的图像,时间戳和轨迹真值
     * 
     * @param[in]  index            索引
     * @param[out] rgb_image        彩色图像
     * @param[out] depth_image      深度图像
     * @param[out] timestamp        时间戳
     * @param[out] ground_truth     真值
     * @return true 
     * @return false 
     */
    bool readItemWithGroundTruth(size_t index, std::string &rgb_image, std::string &depth_image, double &timestamp, std::vector<double> &ground_truth) const
    {
        if(!readItemByIndex(index, rgb_image, depth_image, timestamp))
            return false;
        ground_truth = groundtruth_data_[index];
        return true;
    }

public:
    ///图像序列个数
    size_t N;
    ///数据集路径
    std::string dataset_path_;
    ///关联文件路径
    std::string association_file_;
    ///时间戳序列
    std::vector<double> timestamps_;
    ///彩色图像路径序列
    std::vector<std::string> rgb_images_;
    ///深度图像路径序列
    std::vector<std::string> depth_images_;
    ///真值数据序列
    std::vector<std::vector<double> > groundtruth_data_;
};


/**
 * @brief EuRoc数据集读取器
 * 
 */
class EuRocDataReader {
public:

    /** @brief 自定义图像类型 */
    struct Image{
        double timestamp;   ///<时间戳
        std::string path;   ///<路径
    };

    /** @brief IMU 数据格式 */
    struct IMUData{
        double timestamp;           ///<时间戳
        std::array<double, 3> gyro; ///<陀螺仪数据          //! w_RS_S_[x,y,z] TODO ???
        std::array<double, 3> acc;  ///<加速度计数据        //! a_RS_S_[x,y,z]
    };

    /** @brief 真值数据类型 */
    
    struct GroundTruthData{
        double timestamp;           ///<时间戳
        std::array<double, 3> p;    ///<位置? TODO   //! p_RS_R_[x,y,z]
        std::array<double, 4> q;    ///<旋转? TODO   //! q_RS_[w,x,y,z]
        std::array<double, 3> v;    ///< TODO       //! v_RS_R_[x,y,z]
        std::array<double, 3> w;    ///< TODO       //! b_w_RS_S_[x,y,z]
        std::array<double, 3> a;    ///< TODO       //! b_a_RS_S_[x,y,z]
    };

    /**
     * @brief EuRoc数据集的构造函数
     * 
     * @param[in] dataset_path 数据集文件路径
     * @param[in] info          TODO 
     */
    EuRocDataReader(const std::string dataset_path, const bool info = false)
    {
        //REVIEW 代码于研究内容无关,暂时不阅读
        std::string root_path = dataset_path;
        size_t found = root_path.find_last_of("/\\");
        if(found > root_path.size())
        {
            std::cerr << "EuRocDataReader::load, Please check the path: " + dataset_path << std::endl;
            throw;
        }

        std::string divide_str = root_path.substr(found, 1);
        if (found + 1 != root_path.size())
            root_path += divide_str;

        int code = 0;

        //! for image0
        std::string cam_path0 = root_path + "cam0" + divide_str;
        std::string image_path0 = cam_path0 + "data" + divide_str;
        code = loadCameraData(cam_path0, image_path0, left_);
        if(code != 0)
        {
            std::cerr << "EuRocDataReader, Load cam0 with error " << code <<  ". Please check the cam0 path: " + cam_path0 << std::endl;
            throw;
        }

        //! for image1
        std::string cam_path1 = root_path + "cam1" + divide_str;
        std::string image_path1 = cam_path1 + "data" + divide_str;
        code = loadCameraData(cam_path1, image_path1, right_);
        if(code != 0)
        {
            std::cerr << "EuRocDataReader, Load cam1 with error " << code <<  ". Please check the cam1 path: " + cam_path1 << std::endl;
            throw;
        }

        //! for IMU
        std::string imu_path = root_path + "imu0" + divide_str;
        code = loadIMUData(imu_path, imu_);
        if(code != 0)
        {
            std::cerr << "EuRocDataReader, Load imu with error " << code <<  ". Please check the imu path: " + imu_path << std::endl;
            throw;
        }

        //! for groundtruth
        std::string groundtruth_path = root_path + "state_groundtruth_estimate0" + divide_str;
        code = loadGroundtruthData(groundtruth_path, groundtruth_);
        if(code != 0)
        {
            std::cerr << "EuRocDataReader, Load groundtruth with error " << code <<  ". Please check the groundtruth path: " + groundtruth_path << std::endl;
            throw;
        }

    }

    /**
     * @names 得到各种参数
     * @{
     */

    /** @brief 左目图像数目  */
    const size_t leftImageSize() const { return left_.size(); }
    /** @brief 右目图像数目  */
    const size_t rightImageSize() const { return right_.size(); }
    /** @brief IMU图像数目 */
    const size_t imuSize() const { return imu_.size(); }
    /** @brief  真值轨迹点个数*/
    const size_t groundtruthSize() const { return groundtruth_.size(); }
    /**
     * @brief 获取某帧左目图像
     * 
     * @param[in] idx           图像索引
     * @return const Image&     图像
     */
    const Image& leftImage(size_t idx) const { return left_.at(idx); }

    /**
     * @brief 获取某帧右目图像
     * 
     * @param[in] idx           图像索引
     * @return const Image&     图像
     */
    const Image& rightImage(size_t idx) const { return right_.at(idx); }

    /**
     * @brief 获取个索引时刻的imu数据
     * 
     * @param[in] idx           imu数据索引
     * @return const IMUData&   imu数据
     */
    const IMUData& imu(size_t idx) const { return imu_.at(idx); }

    /**
     * @brief 获取某个索引时刻的轨迹真值
     * 
     * @param[in] idx                   索引
     * @return const GroundTruthData&   轨迹真值
     */
    const GroundTruthData& groundtruth(size_t idx) const { return groundtruth_.at(idx); }

    /** @} */

private:

    /**
     * @brief 加载图像数据
     * 
     * @param[in] root_path 数据集根目录
     * @param[in] data_path 数据路径   TODO 啥数据?
     * @param[out] images   得到的图像序列
     * @return int          个数
     */
    int loadCameraData(std::string root_path, std::string data_path, std::vector<Image> &images)
    {
        //REVIEW 代码没看
        std::string camera_csv = root_path + "data.csv";

        std::ifstream file_stream(camera_csv, std::ifstream::in);
        if (!file_stream.is_open())
            return -1;

        //! image path
        std::string buffer;
        getline(file_stream, buffer);
        while (getline(file_stream, buffer))
        {
            size_t found = buffer.find_last_not_of(" \t\n\r");
            if (found != std::string::npos)
                buffer.erase(found + 1);
            else
                break;

            std::istringstream string_stream(buffer);
            std::string time_buffer;
            std::string name_buffer;
            getline(string_stream, time_buffer, ',');
            time_t time;
            std::istringstream(time_buffer) >> time;
            getline(string_stream, name_buffer);

            Image image = { time * 1e-9, data_path + name_buffer};
            images.push_back(image);
        }
        file_stream.close();

        //! config file
        std::string camera_yaml = root_path + "sensor.yaml";


        return 0;
    }

    /**
     * @brief 读取IMU数据
     * 
     * @param[in] path  数据路径
     * @param[out] imu  IMU数据
     * @return int      数据个数
     */
    int loadIMUData(std::string path, std::vector<IMUData> &imu)
    {
        //REVIEW 没有看代码
        //! csv
        std::string imu_csv = path + "data.csv";
        std::ifstream file_stream(imu_csv, std::ifstream::in);
        if (!file_stream.is_open())
            return -1;

        std::string buffer;
        getline(file_stream, buffer);
        while (getline(file_stream, buffer))
        {
            size_t found = buffer.find_last_not_of(" \t\n\r");
            if (found != std::string::npos)
                buffer.erase(found + 1);
            else
                break;

            std::istringstream string_stream(buffer);
            std::string time_buffer;
            std::string imu_buffer;
            getline(string_stream, time_buffer, ',');
            time_t time;
            std::istringstream(time_buffer) >> time;
            getline(string_stream, imu_buffer);

            IMUData data;
            data.timestamp = time * 1e-9;
            char dot;
            std::istringstream(imu_buffer)
                    >> data.gyro[0] >> dot
                    >> data.gyro[1] >> dot
                    >> data.gyro[2] >> dot
                    >> data.acc[0] >> dot
                    >> data.acc[1] >> dot
                    >> data.acc[2];

            assert(dot == ',');

            imu.push_back(data);
        }
        file_stream.close();

        return 0;
    }

    /**
     * @brief 加载轨迹真值数据
     * 
     * @param[in] path          轨迹真值数据存放路径
     * @param[out] groundtruth  轨迹真值数据
     * @return int              个数
     */
    int loadGroundtruthData(std::string path, std::vector<GroundTruthData> &groundtruth)
    {
        std::string ground_truth_csv = path + "data.csv";
        std::ifstream file_stream(ground_truth_csv, std::ifstream::in);
        if (!file_stream.is_open())
            return -1;

        std::string buffer;
        getline(file_stream, buffer);
        while (getline(file_stream, buffer))
        {
            size_t found = buffer.find_last_not_of(" \t\n\r");
            if (found != std::string::npos)
                buffer.erase(found + 1);
            else
                break;

            std::istringstream string_stream(buffer);
            std::string time_buffer;
            std::string pose_buffer;
            getline(string_stream, time_buffer, ',');
            time_t time;
            std::istringstream(time_buffer) >> time;
            getline(string_stream, pose_buffer);

            GroundTruthData data;
            data.timestamp = time * 1e-9;
            char dot;
            std::istringstream(pose_buffer)
                    >> data.p[0] >> dot
                    >> data.p[1] >> dot
                    >> data.p[2] >> dot
                    >> data.q[0] >> dot
                    >> data.q[1] >> dot
                    >> data.q[2] >> dot
                    >> data.q[3] >> dot
                    >> data.v[0] >> dot
                    >> data.v[1] >> dot
                    >> data.v[2] >> dot
                    >> data.w[0] >> dot
                    >> data.w[1] >> dot
                    >> data.w[2] >> dot
                    >> data.a[0] >> dot
                    >> data.a[1] >> dot
                    >> data.a[2];

            assert(dot == ',');

            groundtruth.push_back(data);
        }
        file_stream.close();

        return 0;
    }

private:


    std::vector<Image> left_;                   ///<左目图像序列
    std::vector<Image> right_;                  ///<右目图像序列
    std::vector<IMUData> imu_;                  ///<IMU数据序列
    std::vector<GroundTruthData> groundtruth_;  ///<轨迹真值序列
};

}

#endif //_SSVO_DATASET_HPP_
