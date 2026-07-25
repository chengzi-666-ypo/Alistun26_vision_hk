sleep 5
cd "/home/hzc/Desktop/vision/111/sp25 V3.6hk/Alistun25_vision/Alistun25_vision_hk/"
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
   bash -c "./build/auto_aim_test"
