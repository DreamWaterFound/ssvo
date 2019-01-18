echo -e "\e[1;35m Starting ssvo using EuRoC dataset. Waiting... \e[0m"

./bin/monoVO_euroc ./config/euroc.yaml ./calib/euroc_cam0.yaml ~/Datasets/EuRoC/mav0/

echo -e ""\e[1;35m ssvo terminated. \e[0m"
