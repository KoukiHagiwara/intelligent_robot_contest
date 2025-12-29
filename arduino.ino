// --- Arduino側 (受信側) スケッチ (赤・青・黄) ---// 各LEDを接続するピン番号を定義
//カメラで読み取った三色のボールの色に対応して三色のLEDをつけるコード

const int RED_LED_PIN = 9;
const int YELLOW_LED_PIN = 10; 
const int BLUE_LED_PIN = 11;

// データ受信用変数
String inputString = "";
boolean stringComplete = false;

void setup() {
  Serial.begin(9600);
  
  // 読み込みのタイムアウトを短く設定（重要：これでブロックされるのを防ぐ）
  Serial.setTimeout(10); 

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  
  allLedsOff();
}

void loop() {
  // データが来ているか確認
  if (Serial.available() > 0) {
    // ★重要: バッファに溜まっている古いデータを読み捨てて、最新の1行だけを取る工夫
    // python側で通信量を減らしていれば、単純に readStringUntil でOKですが、
    // 万が一溜まってしまった場合、ここで最新の '\n' まで読み進める処理を入れるとさらに良いです。
    // 今回はシンプルに「区切り文字(\n)まで一気に読む」ことで対応します。
    
    String receivedData = Serial.readStringUntil('\n');
    receivedData.trim(); // 余計な空白や改行を除去

    // データが空でなければ解析開始
    if (receivedData.length() > 0) {
      parseAndExecute(receivedData);
    }
  }
}

// データ解析実行関数
// 受信データ例: "R:45:320" (コマンド:距離:X座標)
void parseAndExecute(String data) {
  
  // 区切り文字 ':' の位置を探す
  int firstColon = data.indexOf(':');
  int secondColon = data.lastIndexOf(':');

  // データが正しい形式かチェック（コロンが2つあるか）
  if (firstColon > 0 && secondColon > 0 && secondColon > firstColon) {
    
    // 1. コマンド文字を取り出す (例: "R")
    String cmdStr = data.substring(0, firstColon);
    char command = cmdStr.charAt(0);

    // 2. 距離を取り出す (例: "45") -> 今回はLED制御には使いませんが、モータ制御で使えます
    String distStr = data.substring(firstColon + 1, secondColon);
    int distance = distStr.toInt();

    // 3. X座標を取り出す (例: "320") -> モータ制御(左右旋回)で使えます
    String xStr = data.substring(secondColon + 1);
    int x_pos = xStr.toInt();

    // LED制御 (モータ制御もここに書く)
    controlLEDs(command);

    // デバッグ用: 受け取った数値を確認したい場合のみコメントアウトを外す
    // Serial.print("CMD:"); Serial.print(command);
    // Serial.print(" Dist:"); Serial.print(distance);
    // Serial.print(" X:"); Serial.println(x_pos);
  }
}

void controlLEDs(char command) {
  // 状態が変わる前に一度消す（必要に応じて残してもOK）
  allLedsOff();

  switch (command) {
    case 'R':
      digitalWrite(RED_LED_PIN, HIGH);
      break;
    case 'B':
      digitalWrite(BLUE_LED_PIN, HIGH);
      break;
    case 'Y':
      digitalWrite(YELLOW_LED_PIN, HIGH);
      break;
    case 'N':
      // 何もしない（allLedsOffで消えているため）
      break;
  }
}

void allLedsOff() {
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
}