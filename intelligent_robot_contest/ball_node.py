#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from ultralytics import YOLO
import serial
import time

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_color_node')
        self.get_logger().info('Ball Detector Node with LED Control has been started.')

        # --- 設定項目 ---
        # ★★★★★ あなたが学習させたモデルの正しいパスを指定！ ★★★★★
        package_share_directory = get_package_share_directory('intelligent_robot_contest')
        model_path = os.path.join(package_share_directory, 'models', 'best.pt')
        self.get_logger().info(f"Loading model from: {model_path}")

        # ★★★★★ ボールの実際の直径（cm）を正確に設定！ ★★★★★
        self.REAL_BALL_DIAMETER_CM = 6.8
        # ★★★★★ 事前にキャリブレーションして得たカメラの焦点距離を設定！ ★★★★★
        self.FOCAL_LENGTH = 718.409779
        
        # ★★★★★ Arduinoが接続されているUSBポートを指定！ ★★★★★
        self.ARDUINO_PORT = '/dev/ttyACM0' 
        
        # --- 設定項目はここまで ---

        # モデルを読み込む
        self.model = YOLO(model_path)
        self.model.to('cuda')

        # カメラを起動
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("エラー: カメラを起動できませんでした。")
            rclpy.shutdown()
            return

        self.cap.set(3, 640)
        self.cap.set(4, 480)

        # シリアル通信の初期化
        self.ser = None
        try:
            self.ser = serial.Serial(self.ARDUINO_PORT, 9600, timeout=1)
            time.sleep(2) # Arduinoの起動を待つ
            self.get_logger().info(f"Arduino on port {self.ARDUINO_PORT} connected.")
        except serial.SerialException as e:
            self.get_logger().error(f"エラー: シリアルポート {self.ARDUINO_PORT} を開けませんでした: {e}")
       
        # 描画用の色設定 (クラスID 0, 1, 2... に対応)
        # BGR形式: 0:赤, 1:青, 2:黄 と仮定
        self.colors = [(0, 0, 255), (255, 0, 0), (0, 255, 255)] 

        # 最後に送信したコマンドを保存する変数
        self.last_command_sent = ''

        # 定期的に処理を実行するタイマーを作成
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz (0.1秒ごと)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("エラー: フレームを読み込めませんでした。")
            return

        results = self.model.predict(frame, conf=0.5, verbose=False)
        
        detected_balls = []
        for res in results:
            boxes = res.boxes.cpu().numpy()
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                pixel_diameter = ((x2 - x1) + (y2 - y1)) / 2.0
                
                distance_cm = 0
                if pixel_diameter > 0:
                    distance_cm = (self.REAL_BALL_DIAMETER_CM * self.FOCAL_LENGTH) / pixel_diameter
                
                detected_balls.append({
                    'distance': distance_cm, 
                    'box': (x1, y1, x2, y2), 
                    'class_id': int(box.cls[0])
                })
        
        # デフォルトのコマンドを 'N' (消灯) に設定
        command_to_send = 'N'

        # カメラに最も近いボールを選択
        if detected_balls:
            nearest_ball = min(detected_balls, key=lambda b: b['distance'])
            class_id = nearest_ball['class_id']
            
            # ★★★ ここからが主な変更点 ★★★
            # クラスIDに基づいて送信するコマンドを決定
            # class_id 0 -> 赤 (R)
            # class_id 1 -> 青 (B)
            # class_id 2 -> 黄 (Y)
            # ※お使いのモデルのクラスIDと色が合っているか確認してください
            if class_id == 0:
                command_to_send = 'R'
            elif class_id == 1:
                command_to_send = 'B'
            elif class_id == 2:
                command_to_send = 'Y'
            
            # 描画処理
            x1, y1, x2, y2 = nearest_ball['box']
            class_name = self.model.names[class_id]
            color = self.colors[class_id % len(self.colors)]
            label = f"{class_name}: {nearest_ball['distance'] / 100:.2f} m"
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # ★★★ Arduinoへのコマンド送信部分 ★★★
        # 状態が変化した場合のみコマンドを送信
        if command_to_send != self.last_command_sent:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(command_to_send.encode('utf-8'))
                    self.get_logger().info(f"Sent command to Arduino: '{command_to_send}'")
                    self.last_command_sent = command_to_send
                except serial.SerialException as e:
                    self.get_logger().warn(f"Arduinoへの書き込みに失敗しました: {e}")

        # 映像の表示と終了処理
        cv2.imshow('ROS2 Ball Detector', frame)
        if cv2.waitKey(1) & 0xFF == 27: # ESCキー
            # 終了時にLEDを消灯する
            if self.ser and self.ser.is_open:
                self.ser.write('N'.encode('utf-8'))
                self.ser.close()
            self.timer.cancel()
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    detector_node = BallDetectorNode()
    rclpy.spin(detector_node)
    # クリーンアップ
    detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()