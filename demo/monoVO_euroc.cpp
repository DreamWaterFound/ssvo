/**
 * @file monoVO_euroc.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief 跑数据集所调用的程序.本质上是调用ssvo的库来实现追踪
 * @version 0.1
 * @date 2019-01-18
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "system.hpp"
#include "dataset.hpp"
#include "time_tracing.hpp"

using namespace ssvo;

/**
 * @brief 跑euroc数据集的程序入口
 * 
 * @param argc 命令行参数个数
 * @param argv 命令行参数列表=配置文件 相机参数矫正文件 数据集路径
 * @return int 状态
 */
int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    LOG_ASSERT(argc == 4) << "\n Usage : ./monoVO_dataset config_file calib_file dataset_path";

    //新建ssvo对象
    System vo(argv[1],  //配置文件
              argv[2]); //相机参数矫正文件
    
    //新建数据读取器对象
    EuRocDataReader dataset(argv[3]);       //给的参数就是数据集的路径

    ssvo::Timer<std::micro> timer;
    //遍历数据集的每张图片
    const size_t N = dataset.leftImageSize();
    for(size_t i = 0; i < N; i++)
    {
        //读取图片
        const EuRocDataReader::Image image_data = dataset.leftImage(i);
        LOG(INFO) << "=== Load Image " << i << ": " << image_data.path << ", time: " << std::fixed <<std::setprecision(7)<< image_data.timestamp << std::endl;
        cv::Mat image = cv::imread(image_data.path, CV_LOAD_IMAGE_UNCHANGED);
        if(image.empty())
            continue;

        //对图片进行处理,并且计时
        timer.start();
        //NOTE 这里就是主要的执行部分 
        vo.process(image, image_data.timestamp);
        timer.stop();

        double time_process = timer.duration();

        //时间同步机制
        double time_wait = 0;
        if(i < N -1)
            time_wait = (dataset.leftImage(i+1).timestamp - image_data.timestamp)*1e6;
        else
            time_wait = (image_data.timestamp - dataset.leftImage(i-1).timestamp)*1e6;

        if(time_process < time_wait)
            std::this_thread::sleep_for(std::chrono::microseconds((int)(time_wait - time_process)));
    }

    //当全部都处理完成之后,保存获得到的轨迹
    vo.saveTrajectoryTUM("trajectory.txt");
    getchar();

    return 0;
}