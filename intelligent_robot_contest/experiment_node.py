# 検出した距離をcsvファイルに保存するコード

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
import csv  # ★追加: CSV保存用

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_color_node')
        self.get_logger().info('Ball Detector Node (Data Collection Mode) Started.')

        # --- 設定項目 ---
        # ★★★★★ モデルパス ★★★★★
        try:
            package_share_directory = get_package_share_directory('intelligent_robot_contest')
            model_path = os.path.join(package_share_directory, 'models', 'best.pt')
        except:
            model_path = 'best.pt' # フォールバック
            
        self.get_logger().info(f"Loading model from: {model_path}")

        # ★★★★★ パラメータ設定 ★★★★★
        self.REAL_BALL_DIAMETER_CM = 6.8
        self.FOCAL_LENGTH = 718.409779
        self.ARDUINO_PORT = '/dev/ttyACM0' 
        
        # ★追加: データ収集用の設定
        self.measured_data = []      # データを保存するリスト
        self.TARGET_DATA_COUNT = 100 # 集めるデータの数
        self.csv_filename = 'ball_distances.csv' # 保存するファイル名
        
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
            time.sleep(2)
            self.get_logger().info(f"Arduino on port {self.ARDUINO_PORT} connected.")
        except serial.SerialException as e:
            self.get_logger().error(f"エラー: シリアルポート {self.ARDUINO_PORT} を開けませんでした: {e}")
       
        self.colors = [(0, 0, 255), (255, 0, 0), (0, 255, 255)] 
        self.last_command_sent = ''

        # 定期的に処理を実行するタイマーを作成 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

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
        
        command_to_send = 'N'

        # カメラに最も近いボールを選択
        if detected_balls:
            nearest_ball = min(detected_balls, key=lambda b: b['distance'])
            class_id = nearest_ball['class_id']
            dist_cm = nearest_ball['distance']

            # ★追加: データを記録する処理
            dist_meter = dist_cm / 100.0  # cm を m に変換
            self.measured_data.append(dist_meter)
            
            # 進捗を表示
            current_count = len(self.measured_data)
            self.get_logger().info(f"Data collected: {current_count}/{self.TARGET_DATA_COUNT} (Last: {dist_meter:.3f}m)")

            # 目標数に達したら保存して終了
            if current_count >= self.TARGET_DATA_COUNT:
                self.save_and_exit()
                return # 以降の処理はしない

            # --- 既存のArduino送信ロジック ---
            if class_id == 0: command_to_send = 'R'
            elif class_id == 1: command_to_send = 'B'
            elif class_id == 2: command_to_send = 'Y'
            
            # 描画処理
            x1, y1, x2, y2 = nearest_ball['box']
            class_name = self.model.names[class_id]
            color = self.colors[class_id % len(self.colors)]
            label = f"{class_name}: {dist_meter:.2f} m"
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Arduinoへのコマンド送信
        if command_to_send != self.last_command_sent:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(command_to_send.encode('utf-8'))
                    self.last_command_sent = command_to_send
                except serial.SerialException as e:
                    pass

        # 映像の表示
        cv2.imshow('ROS2 Ball Detector', frame)
        if cv2.waitKey(1) & 0xFF == 27:
            self.cleanup()

    # ★追加: CSV保存と終了処理
    def save_and_exit(self):
        self.get_logger().info(f"Target count reached! Saving to {self.csv_filename}...")
        
        try:
            with open(self.csv_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Distance_Meter"]) # ヘッダー
                for data in self.measured_data:
                    writer.writerow([data])
            self.get_logger().info("Save complete.")
        except Exception as e:
            self.get_logger().error(f"Failed to save CSV: {e}")
        
        self.cleanup()

    def cleanup(self):
        if self.ser and self.ser.is_open:
            self.ser.write('N'.encode('utf-8'))
            self.ser.close()
        self.timer.cancel()
        self.cap.release()
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    detector_node = BallDetectorNode()
    try:
        rclpy.spin(detector_node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        # cleanupがまだ呼ばれていない場合のために念の為
        if rclpy.ok():
            detector_node.cleanup()

if __name__ == '__main__':
    main()