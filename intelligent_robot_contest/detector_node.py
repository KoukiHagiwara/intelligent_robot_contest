#!/usr/bin/env python3
#arduino接続なしでyolo動かせるか確認できるコード
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from ultralytics import YOLO
# import serial  # 実験用なのでシリアル通信はオフ
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_color_node')
        self.get_logger().info('Ball Detector Node (Experiment Mode: Show ALL) started.')

        # --- 設定項目 ---
        try:
            package_share_directory = get_package_share_directory('intelligent_robot_contest')
            model_path = os.path.join(package_share_directory, 'models', 'best.pt')
        except:
            model_path = 'best.pt' # パッケージが見つからない場合の対策

        self.get_logger().info(f"Loading model from: {model_path}")

        # ★★★★★ パラメータ設定 ★★★★★
        self.REAL_BALL_DIAMETER_CM = 6.8
        self.FOCAL_LENGTH = 718.409779
        
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

        # 実験用なのでシリアル通信は無効化します
        self.ser = None
        # try:
        #     self.ARDUINO_PORT = '/dev/ttyACM0' 
        #     self.ser = serial.Serial(self.ARDUINO_PORT, 9600, timeout=1)
        #     time.sleep(2)
        #     self.get_logger().info(f"Arduino connected.")
        # except Exception as e:
        #     self.get_logger().warn(f"Arduino not connected (Experiment Mode): {e}")
       
        # 描画用の色設定 (クラスID 0, 1, 2... に対応)
        self.colors = [(0, 0, 255), (255, 0, 0), (0, 255, 255)] 

        # ★★★ 変更点1: 画像配信用のパブリッシャーと変換器を作成 ★★★
        # トピック名: 'processed_image'
        self.image_pub = self.create_publisher(Image, 'processed_image', 10)
        self.bridge = CvBridge()
        # 定期的に処理を実行するタイマー (10Hz)
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
        
        # ★★★ 変更点: すべての検出されたボールを描画するループ ★★★
        for ball in detected_balls:
            x1, y1, x2, y2 = ball['box']
            class_id = ball['class_id']
            dist_cm = ball['distance']
            dist_m = dist_cm / 100.0 # メートル変換

            # クラス名取得（モデルに名前定義があればそれを使う、なければID）
            if hasattr(self.model, 'names'):
                class_name = self.model.names[class_id]
            else:
                class_name = str(class_id)

            # 色決定
            color = self.colors[class_id % len(self.colors)]
            
            # ラベル作成 (例: "Red: 1.25 m")
            label = f"{class_name}: {dist_m:.2f} m"
            
            # 描画
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        # ★★★ 変更点2: imshowを廃止し、ROSトピックとして配信 ★★★
        try:
            # OpenCV画像(BGR)をROSメッセージに変換
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            # トピック 'processed_image' に配信
            self.image_pub.publish(ros_image_msg)
        except Exception as e:
            self.get_logger().error(f"画像の配信に失敗: {e}")

        # 映像の表示と終了処理
        cv2.imshow('Experiment Ball Detector', frame)
        if cv2.waitKey(1) & 0xFF == 27: # ESCキー
           self.cleanup()

    def cleanup(self):
        if self.ser and self.ser.is_open:
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
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            detector_node.cleanup()

if __name__ == '__main__':
    main()
