#include <math.h>
#include <Servo.h>
#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
MCP_CAN CAN0(10);
void process_X_axis(float Ax);
// void process_Z_axis(float Az); 
void process_Pitch(float pitch);
void apply_complementary_filter();

// ========== 移動平均用の変数 ==========
// X軸加速度（0x1780000）
// int msg_cnt_x = 0; // 不要になります
// float ax_sum = 0.00; // sumは維持しますが、使い方が変わります
float avg_ax = 0.00;

// Z軸加速度（0x17C0000）
/*
int msg_cnt_z = 0;
float az_sum = 0.00;
float avg_az = 0.00;
*/

// Pitchジャイロ（0x17C0000）
// int msg_cnt_pitch = 0; // 不要になります
// float pitch_sum = 0.00; // sumは維持しますが、使い方が変わります
float avg_pitch = 0.00;

const int SAMPLE_SIZE_X = 10;
//const int SAMPLE_SIZE_Z = 100;
const int SAMPLE_SIZE_PITCH = 10;

// --- ここから新しい移動平均用の変数 ---
float ax_readings[SAMPLE_SIZE_X]; // X軸加速度の履歴を保持する配列
int ax_currentIndex = 0;           // X軸加速度配列の現在のインデックス
long double ax_current_sum = 0.0;  // X軸加速度の合計 (より高精度なlong doubleに)
bool ax_buffer_filled = false;     // X軸バッファが一度満たされたか

float pitch_readings[SAMPLE_SIZE_PITCH]; // Pitch角速度の履歴を保持する配列
int pitch_currentIndex = 0;                // Pitch角速度配列の現在のインデックス
long double pitch_current_sum = 0.0;       // Pitch角速度の合計 (より高精度なlong doubleに)
bool pitch_buffer_filled = false;          // Pitchバッファが一度満たされたか
// --- ここまで新しい移動平均用の変数 ---


// ========== 相補フィルタ用の変数 ==========
float tilt_accel = 0.00;
float tilt_gyro = 0.00;
float tilt_filtered = 0.00;

// タイミング管理
unsigned long prev_time = 0;
float dt = 0.01;

// 相補フィルタ係数
const float ALPHA = 0.98;  // change ratio up to the application needs

// サーボモーター
Servo servo;
int output_pin = 8;

// 初回フラグ
bool first_calculation = true;

void setup() {
  Serial.begin(115200);
  
  if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) 
    Serial.println("MCP2515 初期化成功！");
  else 
    Serial.println("MCP2515 初期化失敗！");
  
  CAN0.init_Mask(0, 0, 0x010F0000);
  CAN0.init_Filt(0, 0, 0x01780000);
  CAN0.init_Filt(1, 0, 0x017C0000);
  
  CAN0.init_Mask(1, 0, 0x010F0000);
  CAN0.init_Filt(2, 0, 0x01780000);
  CAN0.init_Filt(3, 0, 0x017C0000);
  CAN0.init_Filt(4, 0, 0x01780000);
  CAN0.init_Filt(5, 0, 0x017C0000);
  
  CAN0.setMode(MCP_NORMAL);
  servo.attach(output_pin, 500, 2500);
  servo.write(90);
  delay(1000);
  
  prev_time = millis();
  
  // --- 移動平均バッファの初期化 ---
  for (int i = 0; i < SAMPLE_SIZE_X; i++) {
    ax_readings[i] = 0.0;
  }
  for (int i = 0; i < SAMPLE_SIZE_PITCH; i++) {
    pitch_readings[i] = 0.0;
  }
  // --- 初期化ここまで ---

  Serial.println("╔═══════════════════════════════════════════╗");
  Serial.println("║   相補フィルタあり版                       ║");
  Serial.println("║   - センサー側15Hzフィルタ                ║");
  Serial.println("║   - 移動平均フィルタ（指定サンプル数）      ║"); // 記述を修正
  Serial.println("║   - 相補フィルタ（ALPHA=0.95）            ║");
  Serial.println("╚═══════════════════════════════════════════╝\n");
}

void loop() {
  // 時間差分の計算
  unsigned long current_time = millis();
  dt = (current_time - prev_time) / 1000.0;
  //Serial.println(current_time); // デバッグ用なのでコメントアウト
  prev_time = current_time;
  
  // CANデータ取得
  if(CAN0.checkReceive() == CAN_MSGAVAIL) {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    
    // シリアル出力は情報量が多くなるので、必要に応じてコメントアウトしてください
    // Serial.print("ID: 0x");
    // Serial.print(rxId, HEX);
    // Serial.print(" Data: ");
    // for(int i = 0; i < len; i++) {
    //   if(rxBuf[i] < 0x10) Serial.print("0");
    //   Serial.print(rxBuf[i], HEX);
    //   Serial.print(" ");
    // }
    // Serial.println();

    // ========== ID別に処理 ==========
    if(rxId == 0x178) {
      // X軸加速度
      unsigned int rawAx = (rxBuf[5] << 8) | rxBuf[4];
      float Ax = (rawAx - 32768) * 0.00125;
      
      // Serial.print("  → X軸 Raw: "); // デバッグ用なのでコメントアウト
      // Serial.print(rawAx);
      // Serial.print(", Ax: ");
      // Serial.print(Ax, 4);
      // Serial.println(" m/s^2");
      
      process_X_axis(Ax);
    }
    else if(rxId == 0x17C) {
      // Pitch角速度
      unsigned int rawPitch = (rxBuf[1] << 8) | rxBuf[0];
      float pitch = (rawPitch - 32768) * 0.005;
      
      // Serial.print("  → Pitch Raw: "); // デバッグ用なのでコメントアウト
      // Serial.print(rawPitch);
      // Serial.print(", Pitch: ");
      // Serial.print(pitch, 4);
      // Serial.println(" deg/s");
      
      process_Pitch(pitch);
    }
  }
  
  // 相補フィルタの適用
  apply_complementary_filter();
}

// ========== X軸加速度データ処理（移動平均） ==========
void process_X_axis(float Ax) {
  // 1. 合計値から最も古い値を引く
  //    (バッファがまだ満たされていない場合は、初期値の0を引くことになる)
  ax_current_sum -= ax_readings[ax_currentIndex];

  // 2. 配列の現在の位置に新しい値を格納する
  ax_readings[ax_currentIndex] = Ax;

  // 3. 合計値に新しい値を加える
  ax_current_sum += Ax;

  // 4. 次の書き込み位置を更新する (リングバッファの原理)
  ax_currentIndex = (ax_currentIndex + 1) % SAMPLE_SIZE_X;

  // 5. バッファが一度でも満たされたかチェック
  if (ax_currentIndex == 0 && !ax_buffer_filled) {
    ax_buffer_filled = true;
  }

  // 6. 移動平均を計算する (バッファが満たされていればSAMPLE_SIZE_Xで割る)
  if (ax_buffer_filled) {
    avg_ax = (float)ax_current_sum / SAMPLE_SIZE_X;
  } else {
    // バッファがまだ満たされていない場合は、現在入っているデータ数で割る
    avg_ax = (float)ax_current_sum / ax_currentIndex;
  }
  
  // デバッグ出力
  Serial.print("[X軸加速度] 移動平均 (");
  if(ax_buffer_filled) {
    Serial.print(SAMPLE_SIZE_X);
  } else {
    Serial.print(ax_currentIndex);
  }
  Serial.print(" samples): ");
  Serial.print(avg_ax, 4);
  Serial.println(" m/s^2");
}

// ========== Z軸加速度データ処理（移動平均） ==========
/*
... Z軸も同様のロジックで修正します。
*/

// ========== Pitchジャイロデータ処理（移動平均） ==========
void process_Pitch(float pitch) {
  // 1. 合計値から最も古い値を引く
  pitch_current_sum -= pitch_readings[pitch_currentIndex];

  // 2. 配列の現在の位置に新しい値を格納する
  pitch_readings[pitch_currentIndex] = pitch;

  // 3. 合計値に新しい値を加える
  pitch_current_sum += pitch;

  // 4. 次の書き込み位置を更新する (リングバッファの原理)
  pitch_currentIndex = (pitch_currentIndex + 1) % SAMPLE_SIZE_PITCH;

  // 5. バッファが一度でも満たされたかチェック
  if (pitch_currentIndex == 0 && !pitch_buffer_filled) {
    pitch_buffer_filled = true;
  }

  // 6. 移動平均を計算する
  if (pitch_buffer_filled) {
    avg_pitch = (float)pitch_current_sum / SAMPLE_SIZE_PITCH;
  } else {
    avg_pitch = (float)pitch_current_sum / pitch_currentIndex;
  }

  // デバッグ出力
  Serial.print("[Pitch角速度] 移動平均 (");
  if(pitch_buffer_filled) {
    Serial.print(SAMPLE_SIZE_PITCH);
  } else {
    Serial.print(pitch_currentIndex);
  }
  Serial.print(" samples): ");
  Serial.print(avg_pitch, 4);
  Serial.println(" deg/s");
}

// ========== 相補フィルタの適用 ==========
void apply_complementary_filter() {
  // 移動平均バッファが満たされていることを確認
  // `ax_buffer_filled`と`pitch_buffer_filled`がtrueになるまでは、
  // 計算は行わない方が安定します。
  if(ax_buffer_filled && pitch_buffer_filled) { // 条件を修正
    
    // ========== ステップ1: 加速度から傾きを計算 ==========
    //Z軸加速度が利用できないため、X軸のみで傾きを計算
    float ax_norm = avg_ax / 9.81;
    if(abs(ax_norm) <= 1.0) {
      tilt_accel = asin(ax_norm) * 180.0 / PI;
    } else {
      tilt_accel = (ax_norm > 0) ? 90.0 : -90.0;
    }
    
    // ========== ステップ2 & 3: 相補フィルタ適用 ==========
    if(first_calculation) {
      tilt_filtered = tilt_accel;
      tilt_gyro = tilt_accel;
      first_calculation = false;
      
      Serial.println("╔═══════════════════════════════════════════╗");
      Serial.println("║   初回計算：加速度値を初期値として設定      ║");
      Serial.print("║   初期傾き: ");
      Serial.print(tilt_accel, 2);
      Serial.println(" 度                      ║");
      Serial.println("╚═══════════════════════════════════════════╝");
    } 
    else {
      // ジャイロで傾きを積分
      tilt_gyro = tilt_filtered + avg_pitch * dt;
      
      // 相補フィルタ
      tilt_filtered = ALPHA * tilt_gyro + (1.0 - ALPHA) * tilt_accel;
    }
    
    // デバッグ出力
    Serial.println("╔═══════════════════════════════════════════╗");
    Serial.println("║     相補フィルタ計算結果                   ║");
    Serial.println("╠═══════════════════════════════════════════╣");
    Serial.print("║ dt: ");
    Serial.print(dt, 4);
    Serial.print(" 秒 │ Pitch: ");
    Serial.print(avg_pitch, 2);
    Serial.println(" deg/s    ║");
    Serial.print("║ [加速度] ");
    Serial.print(tilt_accel, 2);
    Serial.print("° │ [ジャイロ積分] ");
    Serial.print(tilt_gyro, 2);
    Serial.println("°  ║");
    Serial.print("║ [相補フィルタ後] ");
    Serial.print(tilt_filtered, 2);
    Serial.print("° (差分:");
    Serial.print(abs(tilt_filtered - tilt_accel), 2);
    Serial.println("°) ║");
    Serial.println("╚═══════════════════════════════════════════╝");
    
    // サーボ制御
    int servo_angle = constrain(tilt_filtered + 90, 0, 180);
    servo.write(servo_angle);
    Serial.print(">>> サーボ角度: ");
    Serial.print(servo_angle);
    Serial.println(" 度\n");
    
    // apply_complementary_filter()内でフラグをリセットする必要はなくなります
    // なぜなら、移動平均は常に最新のデータを保持するためです。
  }
}
