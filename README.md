# carry_my_luggage

## 環境構築

以下の環境で動くことを想定しています

- Turtlebot 2
- Rplidar A1
- Open Manipulator X
- Ubuntu 18.04
- ROS Melodic

### Rplidar A1

以下のパッケージのダウンロードとビルド

```
https://github.com/Slamtec/rplidar_ros
```

書き込み権限を与える(初めて使う場合)

```
ls /dev/ttyUSB*                 # どのデバイスファイルか確認
sudo chmod 666 /dev/ttyUSB0     # 実行権限を与える
```

### Open Manipulator X

[詳しくは公式ドキュメントを読もう](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/)

依存パッケージのインストール

```
sudo apt-get install ros-melodic-ros-controllers ros-melodic-gazebo* ros-melodic-moveit* ros-melodic-industrial-core
sudo apt-get install ros-melodic-dynamixel-sdk ros-melodic-dynamixel-workbench*
sudo apt-get install ros-melodic-robotis-manipulator
```

以下のパッケージのダウンロードとビルド

```
git clone https://github.com/ROBOTIS-GIT/open_manipulator.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
```

U2D2のセットアップ(初めて使う場合)

```
roscore
rosrun open_manipulator_controller create_udev_rules
```

### catkin_make の cmake エラー
```
rosdep install -r --from-path src --ignore-src
```

### speech_and_NLP

```
git submodule update --init
git submodule foreach git pull origin main
```
+ Ubuntuの場合 
```
cd speech_and_NLP
sh setup.sh
```
+ 手動でやる場合
```
pip install -r requirements.txt

or

pip3 ~~~ 
```

`cmake ffmpeg mecab libportaudio2 python-pyaudio` をインストール
```shell
sudo apt install cmake ffmpeg python-pyaudio mecab libportaudio2 -y
```
