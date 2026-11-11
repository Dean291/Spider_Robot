// ============================================================
//  Robot nhện 4 chân - NÂNG NGƯỜI V23 + ĐI 1 -> 4 -> 2 -> 3 + TRÁNH VẬT CẢN (HY-SRF05)
//  Board: Blue Pill STM32F103 (I2C1: PB6 = SCL, PB7 = SDA)
//  V24: đi nhanh hơn ~20%, thân cân bằng hơn (bù chân sau phải nặng), 2 chân sau nâng người mạnh hơn.
//  V25: đứng dậy xong -> DƠ CHÂN TRƯỚC TRÁI CHÀO (thân nghiêng về sau phải trước) -> rồi mới đi.
//  Đã sửa nhận diện trái/phải thực tế: C=FL, D=RL, A=FR, B=RR.
// ============================================================
#include <Wire.h>

#define PCA9685_ADDR        0x40
#define PCA9685_MODE1       0x00
#define PCA9685_MODE2       0x01
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06

// Các kênh PCA9685 VẬT LÝ giữ nguyên, chỉ đổi tên trái/phải.
// Chân A: FR (trước phải)
#define CH_A_COXA   2
#define CH_A_FEMUR  0
#define CH_A_TIBIA  1
// Chân B: RR (sau phải)
#define CH_B_COXA   5
#define CH_B_FEMUR  3
#define CH_B_TIBIA  4
// Chân C: FL (trước trái)
#define CH_C_COXA   8
#define CH_C_FEMUR  6
#define CH_C_TIBIA  7
// Chân D: RL (sau trái)
#define CH_D_COXA   11
#define CH_D_FEMUR  9
#define CH_D_TIBIA  10

// Thứ tự LOGIC: 0 = FL (chân 1), 1 = RL (chân 3),
//               2 = FR (chân 2), 3 = RR (chân 4).
// Sắp xếp lại kênh để sửa trái/phải; KHÔNG đổi dây và góc servo.
const uint8_t leg_ch[4][3] = {
  {CH_C_COXA, CH_C_FEMUR, CH_C_TIBIA}, // 0: FL (C, chân 1)
  {CH_D_COXA, CH_D_FEMUR, CH_D_TIBIA}, // 1: RL (D, chân 3)
  {CH_A_COXA, CH_A_FEMUR, CH_A_TIBIA}, // 2: FR (A, chân 2)
  {CH_B_COXA, CH_B_FEMUR, CH_B_TIBIA}  // 3: RR (B, chân 4)
};

// Vẫn phát lệnh theo thứ tự kênh V23: A -> B -> C -> D.
const uint8_t servo_order[4] = {2, 3, 0, 1};

// Riêng đùi RR (B) gập vào thân thêm 5°: 160° -> 155°.
// Cẳng RR giữ 173°; tất cả các khớp khác giữ nguyên.
float stand_pose[4][3] = {
  {100, 140, 140}, // FL (C): như hàng C gốc
  {130, 80, 30},   // RL (D): như hàng D gốc
  {150, 150, 100}, // FR (A): như hàng A gốc
  {120, 155, 173}  // RR (B): đùi gập vào thân thêm 5°
};

// ---- TƯ THẾ DÙNG RIÊNG CHO ĐỨNG DẬY: COPY NGUYÊN từ robot_nhen_v23 ----
// Chỉ dùng trong sit_start() / pop_up(). Dáng đi vẫn dùng stand_pose ở trên.
// Khác stand_pose đúng ở chân RR (B): V23 = {120, 160, 180}.
const float v23_stand_pose[4][3] = {
  {100, 140, 140}, // FL (C)
  {130, 80, 30},   // RL (D)
  {150, 150, 100}, // FR (A)
  {120, 160, 180}  // RR (B): đùi 160°, cẳng 180° như V23
};
// Chiều co cẳng của V23 theo kênh A,B,C,D = {-1,-1,1,-1} -> thứ tự mới C,D,A,B.
const int V23_TIBIA_TUCK_DIR[4] = { 1, -1, -1, -1 };
const float V23_TUCK_DEG = 8.0f;   // V23 co cả 4 cẳng đều 8°

// Đảo THỨ TỰ mảng cùng với chân vật lý, không đảo hướng servo.
const int COXA_DIR[4] = { -1, -1, 1, 1 };
const int WALK_SIGN   = 1;
const int LIFT_DIR[4] = { 1, -1, -1, 1 };

// ---------- THÔNG SỐ CHỈNH ----------
// --- Nâng người: lấy đúng từ file robot_nhen_v23_gap_ca_dui_va_cang ---
const float SIT_OFFSET      = 8.0;
const int   POP_TIME        = 230;   // giữ nguyên khai báo V23
const float POP_OVERSHOOT   = 12.0;  // giữ nguyên khai báo V23
const int   STARTUP_STAGGER = 15;
// Dùng cho cẳng RR lúc BƯỚC (gait_pose). Đứng dậy dùng V23_TIBIA_TUCK_DIR.
const int   TIBIA_TUCK_DIR[4] = { 1, -1, -1, 1 };
// Gốc A,B,C,D = {1,-1,-1,1} -> thứ tự mới C,D,A,B.
const int   FEMUR_TUCK_DIR[4] = { -1, 1, 1, -1 };
const float RAISE_DEG       = 5.0f;
// --- Tăng lực nâng 2 chân sau (RL + RR) lúc đứng dậy ---
// Phần mông nặng: 2 đùi sau được lệnh ép xuống THÊM rồi trả về mức V23 sau khi thân đã lên.
// Lệnh lệch nhiều hơn = servo ra lực mạnh hơn. Tăng dần 10 -> 12 -> 15 nếu vẫn yếu.
const float REAR_EXTRA_DEG  = 10.0f;  // ép thêm cho đùi sau (độ)
const int   REAR_HOLD_MS    = 300;    // giữ ở đỉnh cho thân lên hẳn
const int   REAR_SETTLE_MS  = 500;    // trả đùi sau về mức nâng V23 (êm)

// ============================================================
//  MÀN CHÀO (chạy 1 lần sau khi đứng dậy, xong mới đi)
//  Dơ chân trước trái (FL) lên cao và vẫy. Trước khi nhấc, thân nghiêng về phía
//  chân ĐỐI DIỆN CHÉO (sau phải RR) để trọng tâm nằm gọn trong tam giác 3 chân còn lại.
// ============================================================
const bool  WAVE_AFTER_STANDUP = true;  // false = bỏ chào, đứng dậy xong đi luôn như V24
const int   WAVE_LEG        = 0;        // chân chào: 0=FL (trước trái), 1=RL, 2=FR, 3=RR
const float WAVE_LIFT_DEG   = 38.0f;    // nhấc đùi (FL chỉ còn ~40° hành trình tới 180°, tự giới hạn)
const float WAVE_TIBIA_DEG  = 20.0f;    // gập thêm cẳng để bàn chân giơ cao hơn (0 = không gập)
const float WAVE_SWING_DEG  = 20.0f;    // biên độ vẫy (hông) mỗi bên
const int   WAVE_COUNT      = 4;        // số lần vẫy qua-lại
const int   WAVE_PERIOD_MS  = 650;      // thời gian 1 lần vẫy qua-lại
// Độ nghiêng về phía chân chéo: chân chéo (RR) hạ thân xuống L/2, 2 chân kề (FR, RL) ép lên L/2.
const float WAVE_LEAN_DEG   = 5.0f;     // tăng nếu khi dơ chân thân vẫn hụt về phía chân chào
const int   WAVE_LEAN_MS    = 900;      // thời gian nghiêng sang
const int   WAVE_SETTLE_MS  = 500;      // đứng yên sau khi nghiêng, rồi mới nhấc chân

// --- Dáng đi: giữ nguyên góc đứng, biên độ, chu kỳ và tốc độ cập nhật ---
#define GAIT_TROT   0
#define GAIT_CRAWL  1
const int   GAIT        = GAIT_CRAWL;

const float STRIDE_AMP  = 22.0;   // V24: 20 -> 22 (bước dài hơn 10%)
const float LIFT_HEIGHT = 28.0;
const float BODY_RAISE  = 0.0;
const float TOUCHDOWN_PUSH = 4.0;
const float PUSH_PULSE_FRAC = 0.07;
const int   CYCLE_MS    = 1000;   // V24: 1100 -> 1000 (nhanh hơn ~21% cùng với bước dài hơn)
const int   UPDATE_MS   = 15;
const int   WALK_CYCLES = 0;

// --- Bù cân bằng khi chân 3/sau trái (RL, kênh D) nhấc lên ---
// Chống thêm bằng chân 1 (trước trái) và chân 4 (sau phải).
// Chỉ áp dụng trong dáng ĐI; không thay đổi động tác đứng dậy V23.
const float RL_PRELOAD_FRAC  = 0.08f; // chuẩn bị chống trước khi RL rời sàn (~88ms)
const float RL_LIFTOFF_DELAY_FRAC = 0.02f; // RL chờ ~22ms sau pha gốc để trụ vững
const float RL_SUPPORT_FL_DEG = 7.0f; // tăng nhẹ chống phía trái để giảm đổ trái
const float RL_SUPPORT_RR_DEG = 5.0f; // giảm lực đẩy phía phải để bớt quay trái

// --- V24: CÂN BẰNG THÂN KHI ĐI (bù phần mông nặng, nhất là sau phải) ---
// Chỉnh độ cao nền từng chân LÚC ĐI. Thứ tự: FL, RL, FR, RR.
// Số dương = chân ép xuống (chỗ đó của thân cao lên). Mặc định ép thêm 2 chân sau, nhiều nhất RR.
// Thân vẫn chúi/nghiêng về sau-phải -> tăng RR (mỗi lần 1°); bị ngược lại thì giảm.
const float STAND_TRIM[4] = { 0.0f, 2.0f, 0.0f, 3.0f };

// Khi RR (góc nặng nhất) nhấc lên, thân dễ đổ về phía RR: chuyển tải sang 2 chân kề RR
// là RL (sau trái) và FR (trước phải) trước khi RR rời sàn, giống cách đã làm cho RL.
const float RR_PRELOAD_FRAC    = 0.08f; // chuẩn bị chống trước khi RR rời sàn
const float RR_SUPPORT_RL_DEG  = 5.0f;  // RL ép thêm khi RR nhấc
const float RR_SUPPORT_FR_DEG  = 4.0f;  // FR ép thêm khi RR nhấc
const float RR_LIFT_MAX        = 24.0f; // giữ độ nhấc RR như cũ dù đã ép thêm đùi

// Chân 4/sau phải (RR, kênh B) đùi gốc 155° nên còn 24° hành trình nhấc an toàn.
// Cẳng RR/B gập theo chiều TĂNG góc. Giới hạn 179° tránh ép servo về 180°.
const float RR_TIBIA_SWING_DEG = 6.0f; // co từ 173° lên ~179° ở đỉnh bước
const float RR_COXA_HOLD_FRAC = 0.05f;  // hông RR đưa tới đều hơn, giảm giật
const float RR_STRIDE_SCALE = 0.92f;     // V24: 0.85 -> 0.92, RR đẩy đủ hơn để đỡ lệch về phía sau-phải.
                                         // Nếu giờ lại lệch ngược sang trái thì giảm về 0.85

// --- Chế độ kiểm tra chiều hông ---
const bool  TEST_COXA   = false;
const int   MOVE_SPEED  = 10;

// ============================================================
//  TRÁNH VẬT CẢN (HY-SRF05)
//  TRIG -> PA3 (A3), ECHO -> PA2 (A2), VCC -> 5V, GND -> G
//  Đi thẳng -> thấy vật gần thì: DỪNG -> LÙI -> RẼ (quay tại chỗ) cho tới khi
//  phía trước thoáng -> ĐI THẲNG tiếp theo hướng mới.
//  Mỗi lần gặp vật đổi bên rẽ (trái rồi phải rồi trái...) để khỏi kẹt góc.
//  Đèn PC13 trên board: KHÔNG có vật = SÁNG, CÓ vật cản = TẮT.
//  Lùi = cả 4 chân quét hông ngược chiều, vẫn dùng dáng bò 1->4->2->3.
// ============================================================
#define TRIG_PIN  PA3
#define ECHO_PIN  PA2
#define LED_PIN   PC13

// Đèn PC13 trên Blue Pill / Black Pill sáng khi chân ở mức LOW
#define LED_ON    LOW
#define LED_OFF   HIGH

const bool     OBSTACLE_AVOID  = true;   // false = tắt tránh vật cản (đi thẳng như cũ)
const float    OBSTACLE_CM     = 20.0f;  // gần hơn mức này = có vật cản
const float    CLEAR_CM        = 32.0f;  // xa hơn mức này mới coi là thoáng, hết lùi
const uint32_t PING_MS         = 120;    // bao lâu đo 1 lần
const uint32_t ECHO_TIMEOUT_US = 12000;  // ~2m, quá thì coi là không có vật
const uint32_t STOP_MS         = 400;    // dừng bước tiến trước khi lùi
const uint32_t BACK_MIN_MS     = 1800;   // lùi tối thiểu (~2 chu kỳ bước)
const uint32_t BACK_MAX_MS     = 6000;   // lùi tối đa, quá thì thôi lùi để khỏi lùi mãi
const bool     TURN_AFTER_BACK = true;   // true = lùi xong RẼ sang hướng khác; false = chỉ lùi rồi đi thẳng lại
const uint32_t TURN_MIN_MS     = 1500;   // quay tối thiểu (~1,5 chu kỳ bước)
const uint32_t TURN_EXTRA_MS   = 900;    // sau khi thấy thoáng, quay thêm chừng này cho thân lọt qua vật
const uint32_t TURN_MAX_MS     = 8000;   // quay mãi vẫn vướng -> lùi lại, đổi chiều rẽ
const int      TURN_SIDE_INIT  = 1;      // 1 = rẽ trái, -1 = rẽ phải (lần đầu)
const float    BLEND_MS        = 350.0f; // thời gian chuyển đổi chiều (êm, không giật hông)

enum State { ST_WALK, ST_STOP, ST_BACK, ST_TURN };

// Hệ số chiều quét hông từng chân: +1 = đi tới, -1 = đi lùi, 0 = đứng yên.
// Thứ tự chân: 0 = FL, 1 = RL (bên trái), 2 = FR, 3 = RR (bên phải).
float dir_blend[4]  = {1.0f, 1.0f, 1.0f, 1.0f};
float dir_target[4] = {1.0f, 1.0f, 1.0f, 1.0f};

float current_pose[4][3];
float last_deg[16];

// ---------- PCA9685 ----------
void pca_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

void pca_init() {
  pca_write(PCA9685_MODE1, 0x10);
  pca_write(PCA9685_PRESCALE, 121);
  pca_write(PCA9685_MODE1, 0x20);
  delay(2);
  pca_write(PCA9685_MODE2, 0x04);
  pca_write(PCA9685_MODE1, 0xA0);
  for (int i = 0; i < 16; i++) last_deg[i] = -1000.0f;
}

void servo_raw(uint8_t ch, float deg) {
  if (deg < 0.0f)   deg = 0.0f;
  if (deg > 180.0f) deg = 180.0f;
  if (fabs(deg - last_deg[ch]) < 0.2f) return;
  last_deg[ch] = deg;

  float us = 500.0f + deg * 2000.0f / 180.0f;
  uint16_t ticks = (uint16_t)(us * 4096.0f * 50.0f / 1000000.0f + 0.5f);
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(PCA9685_LED0_ON_L + 4 * ch);
  Wire.write(0); Wire.write(0);
  Wire.write(ticks & 0xFF); Wire.write(ticks >> 8);
  Wire.endTransmission();
}

void update_servos() {
  for (int i = 0; i < 4; i++) {
    int leg = servo_order[i];
    for (int joint = 0; joint < 3; joint++) {
      servo_raw(leg_ch[leg][joint], current_pose[leg][joint]);
    }
  }
}

void smooth_move(float target_pose[4][3], int duration_ms) {
  float start_pose[4][3];
  memcpy(start_pose, current_pose, sizeof(start_pose));

  int steps = duration_ms / MOVE_SPEED;
  if (steps <= 0) steps = 1;

  for (int step = 1; step <= steps; step++) {
    float t = (float)step / steps;
    float k = 0.5f * (1.0f - cos(t * PI));

    for (int leg = 0; leg < 4; leg++) {
      for (int joint = 0; joint < 3; joint++) {
        current_pose[leg][joint] = start_pose[leg][joint] + (target_pose[leg][joint] - start_pose[leg][joint]) * k;
      }
    }
    update_servos();
    delay(MOVE_SPEED);
  }
  memcpy(current_pose, target_pose, sizeof(current_pose));
  update_servos();
}

// ---------- NÂNG NGƯỜI V23 (giữ thời gian/góc đùi, sửa chiều cẳng RR) ----------
void get_sit_pose(float target[4][3]) {
  memcpy(target, v23_stand_pose, sizeof(v23_stand_pose));
  for (int leg = 0; leg < 4; leg++) {
    target[leg][1] += SIT_OFFSET * FEMUR_TUCK_DIR[leg];
    float a = target[leg][2] + V23_TUCK_DEG * V23_TIBIA_TUCK_DIR[leg];
    if (a < 0.0f) a = 0.0f;
    if (a > 180.0f) a = 180.0f;
    target[leg][2] = a;
  }
}

void sit_start() {
  float sit[4][3];
  get_sit_pose(sit);
  memcpy(current_pose, sit, sizeof(current_pose));
  for (int i = 0; i < 4; i++) {
    int leg = servo_order[i];
    for (int j = 0; j < 3; j++) {
      servo_raw(leg_ch[leg][j], current_pose[leg][j]);
    }
    delay(STARTUP_STAGGER);
  }
  delay(300);
}

void pop_up() {
  // Đứng dậy = COPY NGUYÊN V23 (góc, thời gian, thứ tự) kể cả chân RR 160°/180°.
  // V23 - bước 1: co cẳng trong get_sit_pose().
  // V23 - bước 2: giữ đùi thấp, đưa 4 cẳng về góc chống nền ban đầu.
  float brace[4][3];
  memcpy(brace, current_pose, sizeof(brace));
  for (int leg = 0; leg < 4; leg++) brace[leg][2] = v23_stand_pose[leg][2];
  smooth_move(brace, 900);
  delay(500);

  // V23 - bước 3: cả 4 đùi nâng đồng thời, không bật giật.
  float target[4][3];
  memcpy(target, brace, sizeof(target));
  for (int leg = 0; leg < 4; leg++) {
    // Chân sau (1 = RL, 3 = RR) ép thêm REAR_EXTRA_DEG để thắng phần mông nặng.
    float raise = RAISE_DEG + ((leg == 1 || leg == 3) ? REAR_EXTRA_DEG : 0.0f);
    target[leg][1] = v23_stand_pose[leg][1] - raise * LIFT_DIR[leg];
    if (target[leg][1] < 0.0f) target[leg][1] = 0.0f;
    if (target[leg][1] > 180.0f) target[leg][1] = 180.0f;
  }
  smooth_move(target, 1500);

  // Giữ ngắn ở đỉnh, rồi trả 2 đùi sau về đúng mức nâng V23 (chân trước giữ nguyên).
  if (REAR_EXTRA_DEG > 0.0f) {
    delay(REAR_HOLD_MS);
    float settle[4][3];
    memcpy(settle, target, sizeof(settle));
    for (int leg = 1; leg < 4; leg += 2) {
      settle[leg][1] = v23_stand_pose[leg][1] - RAISE_DEG * LIFT_DIR[leg];
    }
    smooth_move(settle, REAR_SETTLE_MS);
  }
  // Bản đi tiếp tục vào dáng trong setup() theo đúng 400ms của file đi.
}

// ---------- DÁNG ĐI THEO PHA ----------
// Crawl: 0.23 thay vì 0.25 để giữa hai lượt nhấc có ~22ms đủ 4 chân tì đất.
// Chu kỳ CYCLE_MS và thứ tự chân vẫn giữ nguyên.
float swing_frac() { return (GAIT == GAIT_TROT) ? 0.45f : 0.23f; }

float phase_offset(int leg) {
  // Thứ tự nhấc thực tế: 1 (FL/C) -> 4 (RR/B) -> 2 (FR/A) -> 3 (RL/D).
  // Vẫn giữ nguyên pha 0, 0.25, 0.5, 0.75 của file đi.
  static const float off[4] = {0.0f, 0.25f, 0.5f, 0.75f};
  return off[leg];
}

// Hệ số chống [0..1] cho chân đang chuẩn bị nhấc: tăng êm trước khi nhấc,
// giữ tới 60% pha bay, giảm êm trước khi chạm đất.
float support_env(float p, float swing_start, float S, float preload) {
  const float hold_end  = swing_start + 0.60f * S;
  const float touchdown = swing_start + S;
  if (p >= 1.0f - preload) {
    float u = (p - (1.0f - preload)) / preload;
    return 0.5f * (1.0f - cosf(PI * u));
  } else if (p < hold_end) {
    return 1.0f;
  } else if (p < touchdown) {
    float u = (p - hold_end) / (touchdown - hold_end);
    return 0.5f * (1.0f + cosf(PI * u));
  }
  return 0.0f;
}

void gait_pose(float t, float out[4][3]) {
  const float S = swing_frac();

  // Chuyển tải sang FL + RR trước khi chân RL nhấc, giữ mức hỗ trợ ở đầu
  // và giữa bước rồi giảm êm trước khi RL chạm nền.
  float rl_p = t + phase_offset(1);
  rl_p -= floorf(rl_p);
  float rl_support = 0.0f;
  if (GAIT == GAIT_CRAWL) {
    const float hold_end = RL_LIFTOFF_DELAY_FRAC + 0.60f * S;
    const float touchdown = RL_LIFTOFF_DELAY_FRAC + S;
    if (rl_p >= 1.0f - RL_PRELOAD_FRAC) {
      float u = (rl_p - (1.0f - RL_PRELOAD_FRAC)) / RL_PRELOAD_FRAC;
      rl_support = 0.5f * (1.0f - cosf(PI * u));
    } else if (rl_p < hold_end) {
      rl_support = 1.0f;
    } else if (rl_p < touchdown) {
      float u = (rl_p - hold_end) / (touchdown - hold_end);
      rl_support = 0.5f * (1.0f + cosf(PI * u));
    }
  }

  // V24: chuyển tải sang RL + FR trước khi RR (góc nặng) nhấc.
  float rr_p = t + phase_offset(3);
  rr_p -= floorf(rr_p);
  float rr_support = (GAIT == GAIT_CRAWL) ? support_env(rr_p, 0.0f, S, RR_PRELOAD_FRAC) : 0.0f;

  // Chân 1 (FL) chạm đất tại t=0.25; chân 2 (FR) tại t=0.75.
  float push = 0.0f;
  const float touchdown_times[2] = {0.25f, 0.75f};
  for (int i = 0; i < 2; i++) {
    float d = t - touchdown_times[i];
    if (d < 0.0f) d += 1.0f;
    if (d < PUSH_PULSE_FRAC) {
      float q = d / PUSH_PULSE_FRAC;
      float pulse = 0.5f * (1.0f + cosf(PI * q));
      if (pulse > push) push = pulse;
    }
  }

  for (int leg = 0; leg < 4; leg++) {
    float p = t + phase_offset(leg);
    p -= floorf(p);

    // Giảm riêng tầm quét hông RR: hạn chế lực đẩy làm robot quay trái.
    // Chân 1, 2, 3 vẫn dùng biên độ STRIDE_AMP gốc.
    const float stride = (leg == 3) ? STRIDE_AMP * RR_STRIDE_SCALE : STRIDE_AMP;

    // V24: cộng độ cao nền riêng từng chân (STAND_TRIM) để thân đứng bằng khi đi.
    float base_femur = stand_pose[leg][1] - (BODY_RAISE + STAND_TRIM[leg]) * LIFT_DIR[leg];
    float room = (LIFT_DIR[leg] > 0) ? (180.0f - base_femur) : base_femur;
    float lift_cap = (leg == 3) ? RR_LIFT_MAX : LIFT_HEIGHT;
    float lift_max = min(lift_cap, room - 1.0f);
    if (lift_max < 0.0f) lift_max = 0.0f;

    // RL có một khoảng chống thêm trước khi nhấc; thời gian cả chu kỳ không đổi.
    const float swing_start = (GAIT == GAIT_CRAWL && leg == 1)
                                 ? RL_LIFTOFF_DELAY_FRAC : 0.0f;
    const float swing_end = swing_start + S;
    const bool swinging = (p >= swing_start && p < swing_end);

    float x, lift;
    if (swinging) {
      float u = (p - swing_start) / S;
      float e = 0.5f * (1.0f - cosf(PI * u));

      if (leg == 3) {
        // RR: co/nhấc trước, hông bắt đầu tiến sau 10% pha bay để ít rê chân.
        float move_u = (u - RR_COXA_HOLD_FRAC) / (1.0f - 2.0f * RR_COXA_HOLD_FRAC);
        if (move_u < 0.0f) move_u = 0.0f;
        if (move_u > 1.0f) move_u = 1.0f;
        e = 0.5f * (1.0f - cosf(PI * move_u));
        lift = lift_max * sinf(PI * u);
      } else if (leg == 1) {
        // RL nhấc/hạ êm ở hai đầu pha: hạn chế giật tải làm thân nghiêng.
        float wave = sinf(PI * u);
        lift = lift_max * wave * wave;
      } else {
        lift = lift_max * sinf(PI * u);
      }
      x = -stride + 2.0f * stride * e;
    } else if (p < swing_start) {
      // RL giữ chân chống tại vị trí sau trước khi nhấc, không giật coxa.
      x = -stride;
      lift = 0.0f;
    } else {
      float u = (p - swing_end) / (1.0f - swing_end);
      x = stride - 2.0f * stride * u;
      lift = 0.0f;
    }

    // dir_blend: +1 đi tới, -1 đi lùi (lùi hoặc quay tại chỗ), chuyển dần qua 0 cho êm
    out[leg][0] = stand_pose[leg][0] + WALK_SIGN * x * COXA_DIR[leg] * dir_blend[leg];

    // Xung chống chỉ dành cho chân đang tì đất, không ghìm chân vừa nhấc.
    float stance_push = swinging ? 0.0f : push;
    if (GAIT == GAIT_CRAWL && leg == 1 &&
        (p >= 1.0f - RL_PRELOAD_FRAC || p < swing_start)) {
      // Không đè RL xuống ngay trước lúc nhấc; chuyển tải sang FL + RR.
      stance_push = 0.0f;
    }
    float support_push_deg = TOUCHDOWN_PUSH * stance_push;

    // Dùng mức chống lớn hơn trong hai mức, KHÔNG cộng dồn xung đẩy 4°
    // với bù cân bằng để tránh ép chân FL và RR xuống quá mạnh.
    if (leg == 0) {
      float balance_deg = RL_SUPPORT_FL_DEG * rl_support;
      if (balance_deg > support_push_deg) support_push_deg = balance_deg;
    }
    if (leg == 3) {
      float balance_deg = RL_SUPPORT_RR_DEG * rl_support;
      if (balance_deg > support_push_deg) support_push_deg = balance_deg;
    }
    // V24: khi RR sắp nhấc/đang bay, RL và FR ép thêm để thân không đổ về phía RR.
    if (leg == 1) {
      float balance_deg = RR_SUPPORT_RL_DEG * rr_support;
      if (balance_deg > support_push_deg) support_push_deg = balance_deg;
    }
    if (leg == 2) {
      float balance_deg = RR_SUPPORT_FR_DEG * rr_support;
      if (balance_deg > support_push_deg) support_push_deg = balance_deg;
    }

    float femur = base_femur + lift * LIFT_DIR[leg]
                  - support_push_deg * LIFT_DIR[leg];
    if (femur < 0.0f) femur = 0.0f;
    if (femur > 180.0f) femur = 180.0f;
    out[leg][1] = femur;

    out[leg][2] = stand_pose[leg][2];
    if (leg == 3 && swinging) {
      // Cẳng RR/B: gập từ 173° tới khoảng 179° khi bay, duỗi về 173° khi hạ.
      float u = (p - swing_start) / S;
      float tibia = stand_pose[leg][2]
                    + RR_TIBIA_SWING_DEG * sinf(PI * u) * TIBIA_TUCK_DIR[leg];
      if (tibia < 0.0f) tibia = 0.0f;
      if (tibia > 180.0f) tibia = 180.0f;
      out[leg][2] = tibia;
    }
  }
}

void coxa_test() {
  // Cặp trái (FL, RL) và phải (FR, RR) đã khớp dây thực tế.
  const int pairs[2][2] = { {0, 1}, {2, 3} };
  for (int k = 0; k < 2; k++) {
    float t[4][3];
    memcpy(t, stand_pose, sizeof(t));
    for (int j = 0; j < 2; j++) {
      int leg = pairs[k][j];
      t[leg][0] += 20.0f * WALK_SIGN * COXA_DIR[leg];
    }
    smooth_move(t, 500);
    delay(1500);
    smooth_move(stand_pose, 500);
    delay(800);
  }
}

// ---------- DỰNG TƯ THẾ TỪ "LỆCH HÔNG" VÀ "ĐỘ CAO THÂN" ----------
// hx[leg]: lệch hông (độ), > 0 = bàn chân đưa về phía trước.
// h[leg] : độ cao thân tại chân đó so với stand_pose (độ), > 0 = chân ép xuống (thân cao lên),
//          < 0 = đùi nhấc lên (hạ thân / nhấc chân khỏi sàn).
// tibia_flex: độ gập thêm cẳng của chân chào (độ), theo chiều co cẳng TIBIA_TUCK_DIR.
void wave_pose(const float hx[4], const float h[4], float tibia_flex, float out[4][3]) {
  for (int leg = 0; leg < 4; leg++) {
    float c = stand_pose[leg][0] + WALK_SIGN * hx[leg] * COXA_DIR[leg];
    float f = stand_pose[leg][1] - h[leg] * LIFT_DIR[leg];
    float a = stand_pose[leg][2];
    if (leg == WAVE_LEG) a += tibia_flex * TIBIA_TUCK_DIR[leg];
    if (c < 0.0f) c = 0.0f;
    if (c > 180.0f) c = 180.0f;
    if (f < 0.0f) f = 0.0f;
    if (f > 180.0f) f = 180.0f;
    if (a < 0.0f) a = 0.0f;
    if (a > 180.0f) a = 180.0f;
    out[leg][0] = c;
    out[leg][1] = f;
    out[leg][2] = a;
  }
}

void wave_apply_now(const float hx[4], const float h[4], float tibia_flex) {
  float pose[4][3];
  wave_pose(hx, h, tibia_flex, pose);
  memcpy(current_pose, pose, sizeof(current_pose));
  update_servos();
}

// ---------- MÀN CHÀO: dơ chân trước trái ----------
void wave_hello() {
  const int diag = 3 - WAVE_LEG;       // chân chéo với chân chào (FL -> RR)
  float hx[4]   = {0, 0, 0, 0};
  float base[4];                       // độ cao nền = đúng như lúc đi (STAND_TRIM)
  float h[4];
  float pose[4][3];
  for (int leg = 0; leg < 4; leg++) base[leg] = STAND_TRIM[leg];

  // Độ nhấc thật của đùi chân chào, không để vượt hành trình servo.
  float base_femur = stand_pose[WAVE_LEG][1] - base[WAVE_LEG] * LIFT_DIR[WAVE_LEG];
  float room = (LIFT_DIR[WAVE_LEG] > 0) ? (180.0f - base_femur) : base_femur;
  float lift_deg = min(WAVE_LIFT_DEG, room - 1.0f);
  if (lift_deg < 0.0f) lift_deg = 0.0f;

  // 0. Từ tư thế vừa nâng người, về độ cao nền, đứng yên cho 4 chân chắc.
  memcpy(h, base, sizeof(h));
  wave_pose(hx, h, 0.0f, pose);
  smooth_move(pose, 700);
  delay(500);

  // 1. NGHIÊNG THÂN về phía chân chéo (RR) bằng cả 3 chân còn lại:
  //    chân chéo hạ thân xuống L/2, hai chân kề ép lên L/2.
  for (int leg = 0; leg < 4; leg++) {
    if (leg == WAVE_LEG) continue;
    h[leg] = base[leg] + ((leg == diag) ? -0.5f : 0.5f) * WAVE_LEAN_DEG;
  }
  wave_pose(hx, h, 0.0f, pose);
  smooth_move(pose, WAVE_LEAN_MS);
  delay(WAVE_SETTLE_MS);

  // 2. Nhấc chân chào lên từ từ: đùi nhấc + gập cẳng cho bàn chân giơ cao.
  float lean_h[4];
  memcpy(lean_h, h, sizeof(lean_h));
  h[WAVE_LEG] = base[WAVE_LEG] - lift_deg;
  wave_pose(hx, h, WAVE_TIBIA_DEG, pose);
  smooth_move(pose, 900);
  delay(250);

  // 3. Vẫy chân (quét hông qua lại), bắt đầu và kết thúc ở vị trí giữa.
  uint32_t t0 = millis();
  uint32_t total = (uint32_t)WAVE_COUNT * WAVE_PERIOD_MS;
  uint32_t el;
  while ((el = millis() - t0) < total) {
    float ph = 2.0f * PI * (float)el / (float)WAVE_PERIOD_MS;
    hx[WAVE_LEG] = WAVE_SWING_DEG * sinf(ph);
    wave_apply_now(hx, h, WAVE_TIBIA_DEG);
    delay(UPDATE_MS);
  }
  hx[WAVE_LEG] = 0.0f;
  wave_apply_now(hx, h, WAVE_TIBIA_DEG);
  delay(250);

  // 4. Hạ chân xuống, chờ chạm sàn rồi mới nhả nghiêng.
  h[WAVE_LEG] = base[WAVE_LEG];
  wave_pose(hx, h, 0.0f, pose);
  smooth_move(pose, 900);
  delay(400);

  // 5. Trả thân về ngang, về độ cao nền như lúc đi.
  memcpy(h, base, sizeof(h));
  wave_pose(hx, h, 0.0f, pose);
  smooth_move(pose, 700);
  delay(300);
}

// Đọc khoảng cách (cm). Không thấy vật / nhiễu -> trả 999.
float readCm() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  uint32_t us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return 999.0f;
  return us * 0.0343f / 2.0f;
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_ON);      // mới bật: chưa có vật -> sáng
  digitalWrite(TRIG_PIN, LOW);
  Wire.begin();
  Wire.setClock(400000);
  pca_init();
  delay(500);

  sit_start();   // 1. Co đùi và cẳng theo đúng V23
  pop_up();      // 2. Chống cẳng, nâng người theo đúng V23

  if (TEST_COXA) { coxa_test(); delay(1500); }

  if (WAVE_AFTER_STANDUP) wave_hello();   // 3. Dơ chân trước trái chào (nghiêng sau phải trước), xong mới đi

  float start[4][3];
  gait_pose(0.0f, start);
  smooth_move(start, 400);   // 4. Giữ nguyên chuyển tiếp của file đi
}

void loop() {
  static uint32_t t0 = millis();
  static bool done = false;
  if (done) return;

  uint32_t el = millis() - t0;
  if (WALK_CYCLES > 0 && el >= (uint32_t)WALK_CYCLES * CYCLE_MS) {
    smooth_move(stand_pose, 300);
    done = true;
    return;
  }

  // ---- Máy trạng thái: ĐI THẲNG -> DỪNG -> LÙI -> (quay) -> đi thẳng ----
  static State    state = ST_WALK;
  static uint32_t last_ping = 0, state_start = 0;
  static uint8_t  near_cnt = 0;
  static int      turn_side = TURN_SIDE_INIT;
  static float    d = 999.0f;               // khoảng cách đo gần nhất
  static uint32_t clear_start = 0;          // lúc bắt đầu thấy thoáng khi đang quay (0 = chưa thoáng)

  uint32_t now = millis();

  if (OBSTACLE_AVOID) {
    // Đo định kỳ ở MỌI trạng thái để cập nhật đèn và biết khi nào hết vật
    if (now - last_ping >= PING_MS) {
      last_ping = now;
      d = readCm();
      // Đèn: CÓ vật cản (gần hơn ngưỡng) = TẮT, không có vật = SÁNG
      digitalWrite(LED_PIN, (d < OBSTACLE_CM) ? LED_OFF : LED_ON);

      if (state == ST_WALK) {
        // Cần 2 lần đo liên tiếp gần mới dừng (lọc nhiễu)
        if (d < OBSTACLE_CM) {
          if (++near_cnt >= 2) { state = ST_STOP; state_start = now; near_cnt = 0; }
        } else {
          near_cnt = 0;
        }
      }
    }

    if (state == ST_STOP) {
      if (now - state_start >= STOP_MS) { state = ST_BACK; state_start = now; }
    } else if (state == ST_BACK) {
      uint32_t el_back = now - state_start;
      // Lùi tối thiểu BACK_MIN_MS, rồi lùi tiếp cho tới khi thoáng (hoặc hết BACK_MAX_MS)
      if ((el_back >= BACK_MIN_MS && d > CLEAR_CM) || el_back >= BACK_MAX_MS) {
        if (TURN_AFTER_BACK) { state = ST_TURN; clear_start = 0; }
        else                 { state = ST_WALK; near_cnt = 0; }
        state_start = now;
      }
    } else if (state == ST_TURN) {
      uint32_t el_turn = now - state_start;
      // Quay tối thiểu TURN_MIN_MS; sau đó chờ phía trước thoáng liên tục TURN_EXTRA_MS
      if (el_turn >= TURN_MIN_MS && d > CLEAR_CM) {
        if (clear_start == 0) clear_start = now;
      } else {
        clear_start = 0;
      }
      if (clear_start != 0 && now - clear_start >= TURN_EXTRA_MS) {
        state = ST_WALK; near_cnt = 0; state_start = now;
        turn_side = -turn_side;           // lần gặp vật sau rẽ bên kia
      } else if (el_turn >= TURN_MAX_MS) {
        turn_side = -turn_side;           // quay mãi vẫn vướng: lùi lại, rẽ bên kia
        state = ST_BACK; state_start = now;
      }
    }
  }

  // Chiều quét hông từng chân theo trạng thái. Thứ tự: 0 FL, 1 RL (trái), 2 FR, 3 RR (phải)
  for (int leg = 0; leg < 4; leg++) {
    float target = 1.0f;                                       // đi thẳng
    if (state == ST_STOP) target = 0.0f;                       // dừng: không quét hông tới
    if (state == ST_BACK) target = -1.0f;                      // lùi: cả 4 chân ngược
    if (state == ST_TURN) target = (leg < 2) ? -(float)turn_side : (float)turn_side;
    dir_target[leg] = target;

    float step = (float)UPDATE_MS / BLEND_MS * 2.0f;  // *2 vì đi từ +1 sang -1 là quãng 2
    if (dir_blend[leg] < dir_target[leg]) {
      dir_blend[leg] += step;
      if (dir_blend[leg] > dir_target[leg]) dir_blend[leg] = dir_target[leg];
    } else if (dir_blend[leg] > dir_target[leg]) {
      dir_blend[leg] -= step;
      if (dir_blend[leg] < dir_target[leg]) dir_blend[leg] = dir_target[leg];
    }
  }

  float t = (float)(el % CYCLE_MS) / (float)CYCLE_MS;
  gait_pose(t, current_pose);
  update_servos();
  delay(UPDATE_MS);
}

// GHI CHÚ:
// - Chân B vật lý (nay đúng là RR/chân 4) có đùi đứng ở 160 độ nên gần 180.
//   Code vẫn tự giới hạn độ nhấc của chân B như file đi gốc.
// - Nếu robot đi lùi: đặt WALK_SIGN = -1.
