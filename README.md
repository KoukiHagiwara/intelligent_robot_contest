# intelligent_robot_contest
知能ロボットコンテスト用のyoloによるボール認識パッケージ
- YOLOを用いたボール認識を行う
- 得られたボールの距離と色の情報をROS2を用いてシリアル通信でArduinoへ行う

## インストール方法
以下のリポジトリをクローンしてローカル環境でコマンドを実行できるようにセットアップしてください
```
$ git clone https://github.com/KoukiHagiwara/intelligent_robot_contest.git
```

## 実行方法
実行は以下のコマンドを用いて行います。

```
$ ros2 run intelligent_robot_contest detector_node
```
```
$ ros2 run intelligent_robot_contest ball_color_node
```


## Arduinoのコード
以下がArduino側のコードです
- カメラが読み取った三色のボールに対して三色のLEDを光らせる

```
$ cat arduino.ino
```


## 動作環境
- Python 3.10
- Ubuntu 20.04 LTS
- ROS2 foxy
