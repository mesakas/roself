sudo apt-get update
sudo apt-get install -y libncurses5-dev libncursesw5-dev vim


cd rosbag_tui_player
./scripts/clear.sh
./scripts/build.sh



cd ../

PROJECT_PATH=`pwd`
ENV_PARAM='export PATH="$PATH:'$PROJECT_PATH/commands\"

ISEXIST_ENV_PARAMS=$(cat ~/.bashrc | grep "$ENV_PARAM")


if [[ $ISEXIST_ENV_PARAMS ]];then
    echo ".bashrc中已存在环境变量，无需重复初始化！"
else
    echo "" >> ~/.bashrc
    echo "" >> ~/.bashrc
    echo "# roself:" >> ~/.bashrc
    echo $ENV_PARAM >> ~/.bashrc
    echo "环境变量初始化成功！重新启动终端或者source ~/.bashrc即可使用命令"
fi

