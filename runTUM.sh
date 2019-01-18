echo -e "\e[1;35m Starting ssvo using TUM RGBD dataset. Waiting... \e[0m"

./bin/monoVO_tum ./config/euroc.yaml ./calib/TUM3_cam0.yaml /home/guoqing/Datasets/fr3_long_office /home/guoqing/Datasets/fr3_long_office/rgb.txt

echo -e ""\e[1;35m ssvo terminated. \e[0m"
