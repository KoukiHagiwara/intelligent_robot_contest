#!/usr/bin/python3

import cv2
import numpy as np
from ultralytics import YOLO

# --- ここから設定項目 ---

# ★★★★★ あなたが学習させたモデルの正しいパスを指定してください！ ★★★★★
model_path = 'runs/detect/train13/weights/best.pt'

# ★★★★★ ボールの実際の直径（cm）を正確に設定してください ★★★★★
REAL_BALL_DIAMETER_CM = 6.8 # 例: ボールの直径が6.8cmの場合

# ★★★★★ 事前にキャリブレーションして得たカメラの焦点距離を設定してください ★★★★★
FOCAL_LENGTH = 718.409779

# --- 設定項目はここまで ---

# モデルを読み込む
model = YOLO(model_path)

# GPUが利用可能ならGPUを使う
model.to('cuda')

# カメラを起動
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("エラー: カメラを起動できませんでした。")
    exit()

# カメラの解像度を設定
cap.set(3, 640)
cap.set(4, 480)

# 検出結果をきれいに見せるための色設定 (クラスIDに対応)
# 例: class 0 = 青, class 1 = 緑, class 2 = 赤 ...
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255), (255, 0, 255)]

while True:
    # カメラからフレームを1枚読み込む
    ret, frame = cap.read()
    if not ret:
        print("エラー: フレームを読み込めませんでした。")
        break

    # モデルで推論を実行（信頼度が50%以上のものだけを検出）
    results = model.predict(frame, conf=0.5)

    # 検出されたオブジェクトの情報をループで処理
    for res in results:
        # 検出結果のバウンディングボックスを取得
        boxes = res.boxes.cpu().numpy()
        
        for box in boxes:
            # バウンディングボックスの座標を取得
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            
            # クラスIDと信頼度を取得
            class_id = int(box.cls[0])
            conf = box.conf[0]
            class_name = model.names[class_id]
            
            # バウンディングボックスの幅と高さを計算
            pixel_width = x2 - x1
            pixel_height = y2 - y1
            
            # 見かけの直径を計算 (幅と高さの平均)
            pixel_diameter = (pixel_width + pixel_height) / 2.0
            
            # 距離を計算
            if pixel_diameter > 0:
                distance_cm = (REAL_BALL_DIAMETER_CM * FOCAL_LENGTH) / pixel_diameter
                
                # --- ここから描画処理 ---
                # バウンディングボックスを描画
                color = colors[class_id % len(colors)]
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                
                # ラベルと距離を描画
                label = f"{class_name}: {distance_cm/100:.2f} m"
                
                # ラベルの背景を塗りつぶし
                (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(frame, (x1, y1 - h - 15), (x1 + w, y1), color, -1)
                
                # ラベルのテキストを描画
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


    # 結果を表示
    cv2.imshow('Real-time Ball Detection with Distance', frame)

    # 'ESC'キーが押されたらループを抜ける (キーコード 27 は ESC)
    k = cv2.waitKey(1)
    if k == 27:
        break

# リソースを解放
cap.release()
cv2.destroyAllWindows()
