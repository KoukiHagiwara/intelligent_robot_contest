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
        self.get_logger().info('Ball Detector Node has been started.')

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
        self.frame_width = int(self.cap.get(3))
        self.frame_center_x = self.frame_width // 2

        # シリアル通信の初期化
        try:
            self.ser = serial.Serial(self.ARDUINO_PORT, 9600, timeout=1)
            time.sleep(2) # Arduinoの起動を待つ
            self.get_logger().info(f"Arduino on port {self.ARDUINO_PORT} connected.")
        except serial.SerialException as e:
            self.get_logger().error(f"エラー: シリアルポート {self.ARDUINO_PORT} を開けませんでした: {e}")
            self.ser = None
        
        # 描画用の色設定 (クラスID 0, 1, 2... に対応)
        self.colors = [(0, 0, 255), (255, 0, 0), (0, 255, 255)] # BGR形式: 赤, 青, 黄

        # 定期的に処理を実行するタイマーを作成
        self.timer = self.create_timer(0.4, self.timer_callback) # 10Hz (0.1秒ごと)

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
                center_x = (x1 + x2) // 2
                pixel_diameter = ((x2 - x1) + (y2 - y1)) / 2.0
                
                distance_cm = 0
                if pixel_diameter > 0:
                    distance_cm = (self.REAL_BALL_DIAMETER_CM * self.FOCAL_LENGTH) / pixel_diameter
                
                detected_balls.append({
                    'center_x': center_x, 
                    'distance': distance_cm, 
                    'box': (x1, y1, x2, y2), 
                    'class_id': int(box.cls[0])
                })
        
        # カメラに最も近いボールを選択
        if detected_balls:
            nearest_ball = min(detected_balls, key=lambda b: b['distance'])
            
            # Arduinoに情報を送信
            if self.ser and self.ser.is_open:
                # 送信するデータを準備
                center_x_to_send = int(nearest_ball['center_x'])
                distance_cm_to_send = int(nearest_ball['distance'])
                class_id_to_send = nearest_ball['class_id']
                
                # データを "<X座標,距離(cm),クラスID>\n" の形式で送信
                data_to_send = f"<{center_x_to_send},{distance_cm_to_send},{class_id_to_send}>\n"
                self.ser.write(data_to_send.encode('utf-8'))
                
                # ★★★ ここを修正しました ★★★
                # ログにはメートル単位の距離と送信データを両方表示
                distance_m = nearest_ball['distance'] / 100.0
                class_name_log = self.model.names[class_id_to_send]
                self.get_logger().info(
                    f"ターゲット: {class_name_log}, "
                    f"距離: {distance_m:.2f} m, "
                    f"送信データ: {data_to_send.strip()}"
                )

            # 描画処理
            x1, y1, x2, y2 = nearest_ball['box']
            class_id = nearest_ball['class_id']
            # モデルのクラス名を取得
            class_name = self.model.names[class_id]
            color = self.colors[class_id % len(self.colors)]
            label = f"{class_name}: {nearest_ball['distance'] / 100:.2f} m"
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        cv2.imshow('ROS2 Ball Detector', frame)
        if cv2.waitKey(1) & 0xFF == 27: # ESCキー
            self.timer.cancel()
            self.cap.release()
            cv2.destroyAllWindows()
            if self.ser and self.ser.is_open:
                self.ser.close()
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

