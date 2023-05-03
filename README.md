# carry_my_luggage

以下の環境で動くことを想定しています

- Turtlebot 2
- Rplidar A1
- Open Manipulator X
- Ubuntu 18.04
- ROS Melodic

詳しい環境構築のやり方ほ[SETUP.md](SETUP.md)を見よう！

## Topics

### Publish

- `/control_system/cmd_vel` 制御パラメータ (geometry_msgs/Twist)
- `/image_system/person_detect/switch` 人間検出のスイッチ (std_msgs/String)
- `/image_system/paperbag_detect/switch` 紙袋検出のスイッチ (std_msgs/String)

### Subscribe

- `/image_system/person_detect/result` 人間検出の結果 (carry_my_luggage/Detect)
- `/image_system/paperbag_detect/paperbag/result` 紙袋検出の結果 (carry_my_luggage/Detect)
- `/image_system/paperbag_detect/holding/result` 紙袋の持ち手検出結果 (carry_my_luggage/Detect)

### Service

- `/image_system/hand_direction` 手の方向検出 (carry_my_luggage/HandDirection)
- `/audio_system/text_to_speech` 文字の発話 (carry_my_luggage/TextToSpeech)
- `/audio_system/speech_to_text` 音声認識の結果 (carry_my_luggage/SpeechToText)
