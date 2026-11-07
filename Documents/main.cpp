#include <Arduino.h>
#include "driver/pcnt.h" 
#include "driver/gpio.h" 

// ================== CẤU HÌNH HỆ THỐNG ==================
#define MAX_SPEED_MS 0.40   
#define PWM_MAX_VAL 255     
#define RC_DEADZONE 30      
#define ROS_TIMEOUT_MS 1000 

// ================== CẤU HÌNH BỘ LỌC NHIỄU PCNT ==================
#define PCNT_FILTER_VAL 300

// ================== CẤU HÌNH TỰ HỒI PHỤC (WATCHDOG) ==================
#define STUCK_CHECK_INTERVAL 500 // Kiểm tra lỗi mỗi 500ms
#define PWM_THRESHOLD_STUCK 80   // Nếu PWM > 80 mà encoder đứng im thì coi là lỗi

// ================== KHAI BÁO BIẾN VÀ CHÂN ==================
long enc1Count = 0, enc2Count = 0, enc3Count = 0, enc4Count = 0;
// Biến lưu trạng thái cũ để kiểm tra treo
long lastEnc1 = 0, lastEnc2 = 0, lastEnc3 = 0, lastEnc4 = 0;
// Biến lưu giá trị PWM hiện tại đang xuất ra motor
int currentPwm1 = 0, currentPwm2 = 0, currentPwm3 = 0, currentPwm4 = 0;

// CHÂN ENCODER
int ENC1_A = 12, ENC1_B = 13;
int ENC2_A = 26, ENC2_B = 25;
int ENC3_A = 27, ENC3_B = 14; 
int ENC4_A = 33, ENC4_B = 4;

// BIẾN ĐIỀU KHIỂN RC VÀ MOTOR
long batdau1 = 0, batdau2 = 0, batdau3 = 0;
int dodai1 = 0, dodai2 = 0, dodai3 = 0;
int xungRc1 = 1500, xungRc2 = 1500, xungRc3 = 1500;
bool sansang1, sansang2, sansang3;
int rc1 = 35, rc2 = 34, rc3 = 32; 
int r1=23, l1=22, r2=2, l2=15, r3=21, l3=19, r4=18, l4=17;

// BIẾN ROS
float ros_vx = 0.0;
float ros_vy = 0.0;
float ros_wz = 0.0;
unsigned long last_ros_command_time = 0;

// THÔNG SỐ ĐỘNG HỌC
const float Lx = 0.13;
const float Ly = 0.155;
const float R_WHEEL = 0.05; 
const int ENCODER_PPR = 200; 

// ================== NGUYÊN MẪU HÀM ==================
void docxung1(); void docxung2(); void docxung3();
void dieukhien();
void docEncoderVaGuiROS();
void docLenhTuROS();
void setMotor(int motorId, int chA, int chB, int pwm, int min_pwm);
void initPCNT(pcnt_unit_t unit, int pinA, int pinB);
int16_t readPCNT(pcnt_unit_t unit);
void checkEncoderHealth(); // Hàm mới: Kiểm tra và tự sửa lỗi

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10); 

  pinMode(rc1, INPUT); pinMode(rc2, INPUT); pinMode(rc3, INPUT);
  attachInterrupt(digitalPinToInterrupt(rc1), docxung1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rc2), docxung2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rc3), docxung3, CHANGE);

  // --- KHỞI TẠO BỘ ĐẾM PCNT ---
  initPCNT(PCNT_UNIT_0, ENC1_A, ENC1_B);
  initPCNT(PCNT_UNIT_1, ENC2_A, ENC2_B);
  initPCNT(PCNT_UNIT_2, ENC3_A, ENC3_B);
  initPCNT(PCNT_UNIT_3, ENC4_A, ENC4_B);

  // Cấu hình PWM
  for(int i=0; i<8; i++) ledcSetup(i, 20000, 8);
  ledcAttachPin(r1, 0); ledcAttachPin(l1, 1);
  ledcAttachPin(r2, 2); ledcAttachPin(l2, 3);
  ledcAttachPin(r3, 4); ledcAttachPin(l3, 5);
  ledcAttachPin(r4, 6); ledcAttachPin(l4, 7);
  
  // Dừng motor ban đầu
  for(int i=0; i<8; i++) ledcWrite(i, 0);
}

// ================== LOOP ==================
void loop() {
  // 1. Đọc xung RC
  if (sansang1 || sansang2 || sansang3) {
    noInterrupts();
    long t1 = dodai1, t2 = dodai2, t3 = dodai3;
    sansang1 = sansang2 = sansang3 = false;
    interrupts();

    if(t1 >= 900 && t1 <= 2100) xungRc1 = t1;
    if(t2 >= 900 && t2 <= 2100) xungRc2 = t2;
    if(t3 >= 900 && t3 <= 2100) xungRc3 = t3;
  }

  // 2. Nhận lệnh từ ROS
  docLenhTuROS();

  // 3. Gửi data Encoder lên ROS
  docEncoderVaGuiROS();

  // 4. Kiểm tra sức khỏe Encoder (Tự động fix lỗi treo)
  checkEncoderHealth();

  // 5. Điều khiển Motor
  dieukhien();
}

// ================== HÀM HỖ TRỢ PCNT ==================

void initPCNT(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = pinA,
    .ctrl_gpio_num = pinB,
    .lctrl_mode = PCNT_MODE_REVERSE,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DEC,
    .counter_h_lim = 30000,
    .counter_l_lim = -30000,
    .unit = unit,
    .channel = PCNT_CHANNEL_0,
  };
  
  pcnt_unit_config(&pcnt_config);

  pcnt_config.pulse_gpio_num = pinB;
  pcnt_config.ctrl_gpio_num = pinA;
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.hctrl_mode = PCNT_MODE_REVERSE;
  pcnt_config.channel = PCNT_CHANNEL_1; 
  pcnt_unit_config(&pcnt_config);

  // Kích hoạt trở treo nội để tín hiệu ổn định hơn
  gpio_set_pull_mode((gpio_num_t)pinA, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)pinB, GPIO_PULLUP_ONLY);

  pcnt_set_filter_value(unit, PCNT_FILTER_VAL);
  pcnt_filter_enable(unit);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

int16_t readPCNT(pcnt_unit_t unit) {
  int16_t count = 0;
  pcnt_get_counter_value(unit, &count);
  pcnt_counter_clear(unit); 
  return count;
}

// === HÀM MỚI: TỰ ĐỘNG KHÔI PHỤC ENCODER BỊ TREO ===
void checkEncoderHealth() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > STUCK_CHECK_INTERVAL) {
    lastCheck = millis();

    // Kiem tra Motor 1
    // Neu PWM dang cap dien lon (>80) MA Encoder khong thay doi gia tri -> COI NHU BI TREO
    if (abs(currentPwm1) > PWM_THRESHOLD_STUCK && enc1Count == lastEnc1) {
      initPCNT(PCNT_UNIT_0, ENC1_A, ENC1_B); // Khoi dong lai Driver Encoder nay
    }
    lastEnc1 = enc1Count; // Cap nhat trang thai cu

    // Kiem tra Motor 2
    if (abs(currentPwm2) > PWM_THRESHOLD_STUCK && enc2Count == lastEnc2) {
      initPCNT(PCNT_UNIT_1, ENC2_A, ENC2_B);
    }
    lastEnc2 = enc2Count;

    // Kiem tra Motor 3
    if (abs(currentPwm3) > PWM_THRESHOLD_STUCK && enc3Count == lastEnc3) {
      initPCNT(PCNT_UNIT_2, ENC3_A, ENC3_B);
    }
    lastEnc3 = enc3Count;

    // Kiem tra Motor 4
    if (abs(currentPwm4) > PWM_THRESHOLD_STUCK && enc4Count == lastEnc4) {
      initPCNT(PCNT_UNIT_3, ENC4_A, ENC4_B);
    }
    lastEnc4 = enc4Count;
  }
}

// ================== CÁC HÀM KHÁC ==================

void docEncoderVaGuiROS() {
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 50) { 
    lastPrint = millis();
    
    long delta1 = readPCNT(PCNT_UNIT_0);
    long delta2 = readPCNT(PCNT_UNIT_1);
    long delta3 = readPCNT(PCNT_UNIT_2);
    long delta4 = readPCNT(PCNT_UNIT_3);

    enc1Count += delta1;
    enc2Count += delta2;
    enc3Count += delta3;
    enc4Count += delta4;

    Serial.printf("E,%ld,%ld,%ld,%ld\n", enc1Count/4, enc2Count/4, enc3Count/4, enc4Count/4);
  }
}

void docLenhTuROS() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("V,")) {
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);
      int thirdComma = input.indexOf(',', secondComma + 1);

      if (firstComma > 0 && secondComma > 0 && thirdComma > 0) {
        String s_vx = input.substring(firstComma + 1, secondComma);
        String s_vy = input.substring(secondComma + 1, thirdComma);
        String s_wz = input.substring(thirdComma + 1);

        ros_vx = s_vx.toFloat();
        ros_vy = s_vy.toFloat();
        ros_wz = s_wz.toFloat();
        last_ros_command_time = millis();
      }
    }
  }
}

void dieukhien() {
  float target_Vx = 0;
  float target_Vy = 0;
  float target_Wz = 0;

  bool rc_active = (abs(xungRc1 - 1500) > RC_DEADZONE) || 
                   (abs(xungRc2 - 1500) > RC_DEADZONE) || 
                   (abs(xungRc3 - 1500) > RC_DEADZONE);

  if (rc_active) {
    target_Vy = map(xungRc1, 1000, 2000, -MAX_SPEED_MS * 1000, MAX_SPEED_MS * 1000) / 1000.0;
    target_Vx = map(xungRc2, 1000, 2000, -MAX_SPEED_MS * 1000, MAX_SPEED_MS * 1000) / 1000.0;
    target_Wz = map(xungRc3, 1000, 2000, -MAX_SPEED_MS * 1000, MAX_SPEED_MS * 1000) / 1000.0 * 2.5; 
  } 
  else {
    if (millis() - last_ros_command_time < ROS_TIMEOUT_MS) {
      target_Vx = ros_vx;
      target_Vy = ros_vy;
      target_Wz = ros_wz;
    } else {
      target_Vx = 0; target_Vy = 0; target_Wz = 0;
    }
  }

  target_Vx = constrain(target_Vx, -MAX_SPEED_MS, MAX_SPEED_MS);
  target_Vy = constrain(target_Vy, -MAX_SPEED_MS, MAX_SPEED_MS);

  float v1 = target_Vx + target_Vy + (Lx + Ly) * target_Wz; 
  float v2 = target_Vx - target_Vy - (Lx + Ly) * target_Wz; 
  float v3 = target_Vx - target_Vy + (Lx + Ly) * target_Wz; 
  float v4 = target_Vx + target_Vy - (Lx + Ly) * target_Wz; 

  int pwm1 = (v1 / MAX_SPEED_MS) * PWM_MAX_VAL;
  int pwm2 = (v2 / MAX_SPEED_MS) * PWM_MAX_VAL;
  int pwm3 = (v3 / MAX_SPEED_MS) * PWM_MAX_VAL;
  int pwm4 = (v4 / MAX_SPEED_MS) * PWM_MAX_VAL;

  pwm1 = constrain(pwm1, -255, 255);
  pwm2 = constrain(pwm2, -255, 255);
  pwm3 = constrain(pwm3, -255, 255);
  pwm4 = constrain(pwm4, -255, 255);

  setMotor(1, 0, 1, pwm1, 40); 
  setMotor(2, 2, 3, pwm2, 45); 
  setMotor(3, 4, 5, pwm3, 30); 
  setMotor(4, 6, 7, pwm4, 48); 
}

// Cập nhật hàm setMotor để lưu giá trị PWM hiện tại cho Watchdog kiểm tra
void setMotor(int motorId, int chA, int chB, int pwm, int min_pwm) {
  // Lưu PWM vào biến toàn cục để hàm checkEncoderHealth sử dụng
  if (motorId == 1) currentPwm1 = pwm;
  else if (motorId == 2) currentPwm2 = pwm;
  else if (motorId == 3) currentPwm3 = pwm;
  else if (motorId == 4) currentPwm4 = pwm;

  if (pwm > 0) {
    int output = map(pwm, 0, 255, min_pwm, 255);
    ledcWrite(chA, output);
    ledcWrite(chB, 0);
  } else if (pwm < 0) {
    int output = map(abs(pwm), 0, 255, min_pwm, 255);
    ledcWrite(chA, 0);
    ledcWrite(chB, output);
  } else {
    ledcWrite(chA, 0);
    ledcWrite(chB, 0);
  }
}

void docxung1() { if (digitalRead(rc1) == HIGH) batdau1 = micros(); else { dodai1 = micros() - batdau1; sansang1 = true; } }
void docxung2() { if (digitalRead(rc2) == HIGH) batdau2 = micros(); else { dodai2 = micros() - batdau2; sansang2 = true; } }
void docxung3() { if (digitalRead(rc3) == HIGH) batdau3 = micros(); else { dodai3 = micros() - batdau3; sansang3 = true; } }