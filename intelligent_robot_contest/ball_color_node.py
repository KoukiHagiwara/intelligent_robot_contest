#!/usr/bin/env python3
#距離、x座標、色の3つを送る
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
        self.get_logger().info('Ball Detector Node (Traffic Reduced) Started.')

        # --- 設定項目 ---
        try:
            package_share_directory = get_package_share_directory('intelligent_robot_contest')
            model_path = os.path.join(package_share_directory, 'models', 'best.pt')
        except:
            model_path = 'best.pt'
            self.get_logger().warn("Package not found, using 'best.pt'.")

        self.get_logger().info(f"Loading model from: {model_path}")

        # パラメータ設定
        self.REAL_BALL_DIAMETER_CM = 6.8
        self.FOCAL_LENGTH = 718.409779
        self.ARDUINO_PORT = '/dev/ttyACM0' 
        self.MAX_CHASE_DISTANCE = 80.0

        # --- 初期化 ---
        self.model = YOLO(model_path)
        self.model.to('cuda')

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("エラー: カメラを起動できませんでした。")
            rclpy.shutdown()
            return

        self.cap.set(3, 640)
        self.cap.set(4, 480)

        # シリアル通信
        self.ser = None
        try:
            self.ser = serial.Serial(self.ARDUINO_PORT, 9600, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"Arduino connected: {self.ARDUINO_PORT}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial Error: {e}")
       
        self.colors = [(0, 0, 255), (255, 0, 0), (0, 255, 255)] 

        # ★ロックオン制御用の変数
        self.locked_target_id = None
        self.lost_count = 0
        
        # ★通信量削減用の変数（前回送った内容を覚えておく）
        self.last_sent_str = ""

        # タイマー (0.1秒 = 10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        results = self.model.predict(frame, conf=0.5, verbose=False)
        
        detected_balls = []
        for res in results:
            boxes = res.boxes.cpu().numpy()
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x = (x1 + x2) / 2.0
                pixel_diameter = ((x2 - x1) + (y2 - y1)) / 2.0
                
                distance_cm = 0
                if pixel_diameter > 0:
                    distance_cm = (self.REAL_BALL_DIAMETER_CM * self.FOCAL_LENGTH) / pixel_diameter
                
                detected_balls.append({
                    'distance': distance_cm, 
                    'center_x': center_x,
                    'box': (x1, y1, x2, y2), 
                    'class_id': int(box.cls[0])
                })
        
        # --- ターゲット選択 ---
        final_target = None
        if self.locked_target_id is not None:
            same_id_balls = [b for b in detected_balls if b['class_id'] == self.locked_target_id]
            if same_id_balls:
                final_target = min(same_id_balls, key=lambda b: b['distance'])
                self.lost_count = 0
            else:
                self.lost_count += 1
                if self.lost_count > 10:
                    self.locked_target_id = None
                    self.lost_count = 0
                    self.get_logger().info("Target Lost")
        else:
            valid_balls = [b for b in detected_balls if b['distance'] <= self.MAX_CHASE_DISTANCE]
            if valid_balls:
                final_target = min(valid_balls, key=lambda b: b['distance'])
                self.locked_target_id = final_target['class_id']
                self.get_logger().info(f"Locked on ID: {self.locked_target_id}")

        # --- 送信データ生成 ---
        command_code = 'N'
        dist_val_cm = 0
        center_x_val = 320 # デフォルトは中央
        
        if final_target:
            class_id = final_target['class_id']
            # ★ポイント: 数値を整数(int)にして細かい変動を無視する
            dist_val_cm = int(final_target['distance'])
            center_x_val = int(final_target['center_x'])
            # ★変更点2: 画面表示用にメートルに変換する
            dist_val_meter = dist_val_cm / 100.0

            x1, y1, x2, y2 = final_target['box']
            
            if dist_val_cm <= self.MAX_CHASE_DISTANCE:
                if class_id == 0: command_code = 'R'
                elif class_id == 1: command_code = 'B'
                elif class_id == 2: command_code = 'Y'
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                text = f"LOCK {command_code}:{dist_val_meter:.2f}m"
                cv2.putText(frame, text, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            else:
                command_code = 'N'
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                text = f"WAIT {dist_val_meter:.2f}m"
                cv2.putText(frame, text, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

        elif self.locked_target_id is not None:
            cv2.putText(frame, "Searching...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        # --- ★★★ Arduinoへのコマンド送信部分 (変更済み) ★★★ ---
        
        # データを文字列にする (例: "R:45:320\n")
        # 整数に丸めているので、0.9cm以下の変化や0.9ピクセル以下の変化では文字列が変わりません
        send_str = f"{command_code}:{dist_val_cm}:{center_x_val}\n"
        
        # 前回送信した文字列と「完全に同じ」なら送信しない
        if send_str != self.last_sent_str:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(send_str.encode('utf-8'))
                    # 送信した内容を記憶
                    self.last_sent_str = send_str
                    # デバッグ用に送信時のみ表示
                    # self.get_logger().info(f"Send: {send_str.strip()}")
                except serial.SerialException as e:
                    self.get_logger().warn(f"Serial Write Fail: {e}")

        # 映像表示
        cv2.imshow('ROS2 Ball Detector', frame)
        if cv2.waitKey(1) & 0xFF == 27:
            self.cleanup()

    def cleanup(self):
        if self.ser and self.ser.is_open:
            self.ser.write("N:0:320\n".encode('utf-8'))
            self.ser.close()
        self.cap.release()
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()

if __name__ == '__main__':
    main()