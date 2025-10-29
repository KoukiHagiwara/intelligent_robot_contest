// --- Arduino側 (受信側) スケッチ (赤・青・黄) ---// 各LEDを接続するピン番号を定義
//カメラで読み取った三色のボールの色に対応して三色のLEDをつけるコード

const int RED_LED_PIN = 9;
const int YELLOW_LED_PIN = 10; 
const int BLUE_LED_PIN = 11;

void setup() {
  // シリアル通信を開始 (ボーレートは9600に設定)
  Serial.begin(9600);  // LEDピンを出力モードに設定
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT); // 黄色ピンを設定
  pinMode(BLUE_LED_PIN, OUTPUT);  // 起動時にすべてのLEDを消灯しておく
  allLedsOff();
  Serial.println("Arduino Ready (R/B/Y). Waiting for command...");
}

void loop() {
  // シリアルポートにデータが送信されてきたか確認
  if (Serial.available() > 0) {
    // データを1バイト（1文字）読み込む
    char command = Serial.read();    // まず全てのLEDを消灯する
    allLedsOff();    // 受け取った文字に応じて、対応するLEDを点灯
    switch (command) {
      case 'R': // 'R' を受け取ったら赤を点灯
        digitalWrite(RED_LED_PIN, HIGH);
        Serial.println("Received: R -> Red LED ON");
        break;      
	
　　　case 'Y': // 'Y' を受け取ったら黄を点灯 (Gから変更)
        digitalWrite(YELLOW_LED_PIN, HIGH);
        Serial.println("Received: Y -> Yellow LED ON");
        break;      

　　　case 'B': // 'B' を受け取ったら青を点灯
        digitalWrite(BLUE_LED_PIN, HIGH);
        Serial.println("Received: B -> Blue LED ON");
        break;      

　　　case 'N': // 'N' (None) を受け取ったら消灯
        Serial.println("Received: N -> All LEDs OFF");
        break;
    }
  }
}

// すべてのLEDを消灯する関数
void allLedsOff() {
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW); // 黄色ピンを消灯
  digitalWrite(BLUE_LED_PIN, LOW);
}
