/**
 * 粮食干燥系统 ESP32-S3 智能控制器
 * 
 * 集成 PPO 强化学习策略网络，在 AUTO 模式下由 AI 控制各元件
 * 手动模式下保留原有触摸屏操作逻辑
 * 
 * 硬件接线：
 *   DHT11(环境): VCC→3.3V,  DATA→GPIO4,   GND→GND
 *   直流电机:    IN1→GPIO3, IN2→GPIO1, ENA→跳线帽 (L298N, IN1=HIGH/IN2=LOW 反转)
 *   DS18B20:    VCC→3.3V,  DATA→GPIO18,  GND→GND  (水箱温度，需 4.7kΩ 上拉电阻)

 *   电机(风机):  VIN+→12V,  VIN-→风机+,   风机-→GND, SIG→GPIO8, GND→GND
 *   水泵:       VCC→5V,    GND→GND,      GPIO→GPIO6
 *   舵机:       PCA9685 SDA→GPIO40, SCL→GPIO41(与BH1750共用I2C), VCC→3.3V
 *              V+→外接5-6V电源, 通道4→风门, 通道5→排粮
 *   导轨舵机:    进风口信号→GPIO7, 出风口信号→GPIO10 (ESP32Servo 库直驱)
 *   加热片(MOS):   IO/PWM→GPIO9, DC+/DC-→电源, OUT+/OUT-→加热片 (HIGH=加热)
 *   位置检测碰撞开关: VCC→3.3V,  GND→GND, OUT→GPIO2    (YL-99 模块)
 *   导轨碰撞开关:
 *     开关1(GPIO38): 出风口原点, 开关2(GPIO37): 出风口终点
 *     开关3(GPIO47): 进风口终点, 开关4(GPIO48): 进风口原点
 *   BH1750光照
 *   屏幕:       ILI9341 (按 TFT_eSPI 配置, CS=5, DC=16, RST=17, MOSI=11, SCLK=12, MISO=13)
 *   UART接收:   来自 weather_display S3, RX→GPIO19, TX→GPIO20, GND→GND
 *               (weather_display GPIO17→本板GPIO19, weather_display GPIO16←本板GPIO20)
 *
 * 编译环境: Arduino IDE 2.x + ESP32-S3 Dev Module
 * 依赖库: TFT_eSPI, DHT sensor library, DS18B20, TensorFlowLite_ESP32
 */

#include <TFT_eSPI.h>
#include <DHT.h>
#include <DS18B20.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "Adafruit_PWMServoDriver.h"


// ============ RL 模型（已生成） ============
#include "policy_model.h"

TFT_eSPI tft = TFT_eSPI();

// ============== 硬件引脚定义 ==============
#define DHT_PIN           4    // 环境温湿度
#define DS18B20_PIN       18   // DS18B20 水温传感器

// UART 有线接收：来自 weather_display S3 的四个集热器光照
// 注意：GPIO16/17 已被 TFT_eSPI 占用（TFT_DC=16, TFT_RST=17），不能用！
// 接线：对方 GPIO17(TX) → 本板 GPIO19(RX)，对方 GPIO16(RX) ← 本板 GPIO20(TX)，GND 共地
#define UART_LINK_BAUD   115200
#define UART_RX_PIN      19
#define UART_TX_PIN      20
// 解析后存入 collectorLux[0..3]（-1 = 离线/无效）

#define DHT_TYPE          DHT11
#define MOTOR_PWM_PIN     8    // 风机1 (维可思MOSFET)
#define FAN2_PWM_PIN     21    // 风机2 (CS25N06 MOSFET)，与风机1同步启停
#define PUMP_PWM_PIN      6    // 水泵
#define SERVO_CH_DAMPER   4    // PCA9685 通道4: 风门舵机
#define OUTPUT_SERVO_CH   5    // PCA9685 通道5: 排粮舵机
#define SERVO_INLET_PIN    7    // GPIO7:  进风口导轨舵机信号
#define SERVO_OUTLET_PIN  10    // GPIO10: 出风口导轨舵机信号
// 导轨舵机使用 ESP32Servo 库直驱（参照控制电路6.6 风门舵机模式）

// 导轨舵机微秒常量（与旧 LEDC 占空比一一对应，使用 writeMicroseconds）
// 连续旋转舵机：500us=CCW全速  1500us=停止/刹车  2500us=CW全速
#define RAIL_STOP_US    1500   // 停止/刹车（1.5ms 中位脉冲）
#define RAIL_CW_US      2500   // 顺时针（2.5ms 脉冲）
#define RAIL_CCW_US     500    // 逆时针（0.5ms 脉冲）
#define HEATER_PIN        9    // 加热片 MOS驱动（IO/PWM，高电平导通）
#define COLLISION_PIN     2    // 碰撞开关 (无碰撞=HIGH, 有碰撞=LOW)
#define PROX_PIN         42    // NPN接近开关 (无金属=HIGH, 检测到金属=LOW) 棕线12V/蓝线GND/白线信号（GPIO42 支持内部上拉）

// BTS7960 (IBT-2) 直流电机驱动
#define DC_MOTOR_RPWM     3    // RPWM: 正转 PWM 输入（高电平有效）
#define DC_MOTOR_LPWM     1    // LPWM: 反转 PWM 输入（高电平有效）

// 导轨碰撞开关 (LOW=触发)
#define RAIL_SW1          38   // 开关1: 出风口原点
#define RAIL_SW2          37   // 开关2: 出风口终点
#define RAIL_SW3          47   // 开关3: 进风口终点
#define RAIL_SW4          48   // 开关4: 进风口原点

// PCA9685 OE 已释放（导轨舵机改用 ESP32Servo GPIO7/10 直驱）

// BH1750 光照传感器 (I2C)
#define I2C_SDA           40   // I2C 数据线
#define I2C_SCL           41   // I2C 时钟线
#define BH1750_ADDR       0x23 // 默认地址 (ADDR引脚悬空/接GND)
#define BH1750_ADDR_ALT   0x5C // 备用地址 (ADDR引脚接VCC)
#define PCA9685_ADDR      0x40 // PCA9685 默认 I2C 地址

// MG90S 舵机脉冲范围 (PCA9685 50Hz, 4096 ticks = 20ms)
// 0°=0.5ms≈102, 180°=2.5ms≈512
#define SERVOMIN          102
#define SERVOMAX          512

// ============== 前置声明（Arduino 预处理需要） ==============
struct Button;

// ============== PWM 配置 ==============
#define PWM_FREQ          20000   // 电机 20kHz
#define PWM_RES           8       // 8 位分辨率 (0-255)

// ============== RL 模型推理参数 ==============
#define INFERENCE_INTERVAL_MS  1000    // AI 推理间隔 1 秒（实测推理仅~6.7ms，安全）
// 模型使用内存（v33 模型 ~144KB 权重 + 中间张量缓存 ≈ 200KB+）
#define TENSOR_ARENA_SIZE      245760  // 240KB (ESP32-S3 有 512KB SRAM，足够)

// 水泵补热温度门槛（℃）—— 水箱低于此温度时水泵强制为 0
#define WATER_TEMP_PUMP_THRESHOLD  45.0

// 加热片自动开启温度门槛（℃）—— 干燥仓温度低于此值时自动开启
#define HEATER_TEMP_THRESHOLD      40.0
// 加热片强制关断温度（℃）—— PCM 温度高于此值强制关闭加热片，防过热
#define HEATER_TEMP_CUTOFF_C       90.0

// ============== COLLECT 切换控制参数 ==============
// 首次启动：固定等待 10 秒后启动直流电机（不受任何因素影响）
#define COLLECT_START_DELAY_MS     10000
// 光驱吸热-风机匹配放热模型（与 pcm_light_fan_controller.py 一致）：
// 物理结构：4 个集热器，3 个在阳光下吸热 + 1 个在遮蔽处放热，同时进行。
// 切换 = 转盘换位（把吸热的和放热的集热器互换）。
//  - 吸热时间（换位周期）由实时光照估算，钳位 [ABSORB_T_MIN_S, ABSORB_T_MAX_S]（波动≤5s）
//  - 放热时间 = 吸热时间：自动调整风机功率，使"风机对流带走储热的时间"= 吸热时间
//  - 换位周期 = 10~15s，单个集热器在阳光区停留 3 个周期(30~45s)后换去放热
#define ABSORB_T_MIN_S        20.0f   // 最短干燥时长/换位周期 s（由进入CS_AUTO_DRY时的参考光照决定）
#define ABSORB_T_MAX_S        35.0f   // 最长干燥时长/换位周期 s（光照弱时等比延长上限）
#define SOLAR_W_PER_LUX       0.09f   // 太阳能功率系数 W/lux
#define SOLAR_MAX_W           80.0f   // 太阳能最大功率 W
#define DH_TARGET_KJ          0.60f   // 每周期目标储热量 kJ（决定吸热时间基准）
#define FAN_MAX_REMOVE_W      90.0f   // 风机满功率对流散热功率 W
#define LOSS_COEF_W_PER_C     0.5f    // 自然散热系数 W/°C
#define ABSORB_FAN_NORM       0.15f   // 吸热阶段风机归一化功率
#define PCM_MIN_NET_W         2.0f    // 最小净功率 W（防除零）
#define RELEASE_MARGIN_MS     3000    // 放热完成超时余量 ms
#define PCM_EFF_MASS_CP_J     100.0f  // 换热层有效热容 J/°C（温度估算用）
#define T_IN_DELTA_C          5.0f    // 进口风温相对环境升高 ℃

// ============== PCM 焓值法参数（与 pcm_balance_model.py 一致） ==============
#define PCM_MASS_KG           5.0f   // kg
#define PCM_CP_SOLID          2.0f   // kJ/(kg·K)
#define PCM_CP_LIQUID         2.5f   // kJ/(kg·K)
#define PCM_LATENT_HEAT       180.0f // kJ/kg
#define PCM_T_SOLIDUS         37.0f  // 固相点 ℃
#define PCM_T_LIQUIDUS        44.0f  // 液相点 ℃
#define PCM_T_REF             20.0f  // 焓值参考温度 ℃

// ============== 传感器对象 ==============
DHT dht(DHT_PIN, DHT_TYPE);
DS18B20 ds18b20(DS18B20_PIN);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
Servo inletRailServo;     // 进风口导轨舵机（ESP32Servo 直驱 GPIO7）
Servo outletRailServo;    // 出风口导轨舵机（ESP32Servo 直驱 GPIO10）

// ============== 触摸校准 ==============
#define TOUCH_MIN_X 4
#define TOUCH_MAX_X 313
#define TOUCH_MIN_Y 9
#define TOUCH_MAX_Y 239

// ============== 颜色定义 ==============
#define BG_COLOR        0x1082
#define PANEL_COLOR     0x2104
#define ACCENT_COLOR    0x041F
#define TEXT_COLOR      0xFFFF
#define SUCCESS_COLOR   0x07E0
#define WARN_COLOR      0xFFE0
#define DANGER_COLOR    0xF800
#define AUTO_COLOR      0x07FF   // AUTO 模式亮青色
#define MANUAL_COLOR    0xF81F
#define DISABLED_COLOR  0x4208
#define COLLECT_COLOR   0x07E0   // 立体循环集热模式绿色

// ============== 系统工作模式 ==============
enum SystemMode {
  SYSTEM_MODE_COLLECT = 0,  // 立体循环集热模式（默认上电模式）
  SYSTEM_MODE_AUTO    = 1,  // 自动干燥模式（AI 控制）
  SYSTEM_MODE_MANUAL  = 2   // 手动干燥模式（触摸屏操作）
};
int systemMode = SYSTEM_MODE_COLLECT;  // 上电默认进入立体循环集热模式

#define MODE_COUNT 3
const char* MODE_LABELS[MODE_COUNT]  = {"COLLECT", "AUTO", "MANUAL"};
uint16_t   MODE_COLORS[MODE_COUNT]   = {COLLECT_COLOR, AUTO_COLOR, MANUAL_COLOR};
float temperature = 25.5;
float humidity = 60.0;
// 前向声明（COLLECT 操控函数定义在 handleTouch 前，被 UART CTL 命令解析前向调用）
void collectStart();
void collectStop();
void collectManualToggle();
void collectManualInsert();
void collectManualLeave();
void collectManualMotor();
void switchMode(int newMode);
// 干燥仓温度示数模拟：干燥阶段每秒 +0.2~0.5，真实读数另算互不干扰
float         dryerTempSimAccum  = 0.0f;   // 已叠加的示数偏置（℃）
bool          dryerTempSimActive = false;  // 是否在干燥累计中
unsigned long dryerTempSimLastMs = 0;      // 上次累计时刻
int motorSpeed = 0;        // 风机 0-100%
int servoAngle = 0;        // 风门舵机 0-90°（0°=全干燥，45°=半储热，90°=全储热）
bool pumpOn = false;
int pumpFlow = 0;          // 水泵 0-100%
bool outputServoOn = false;
int outputServoAngle = 0;  // 排粮舵机角度

// 排粮状态机: IDLE → STABLE_WAIT(达标稳定10s) → DISCHARGING(排粮90°持续10s) → IDLE
// 仅 AUTO 模式使用（COLLECT 模式改用下方"每3周期排粮"逻辑）
enum DischargeState { DS_IDLE, DS_STABLE_WAIT, DS_DISCHARGING };
DischargeState discharge_state = DS_IDLE;
unsigned long discharge_timer = 0;   // 计时器（稳定累计 / 排粮持续）

// ============== COLLECT 每3周期排粮（仅 CS_AUTO_DRY 使用） ==============
// 每运行 3 次干燥周期后，在第三次干燥结束后排粮 3s，然后复位 0° 才进入下一轮。
// 除排粮时舵机 90°，其他时候排粮舵机都处于 0° 关闭状态（上电时已初始化 0°）。
int  collectDryCycleCount = 0;               // 已完成的干燥周期计数 0~3
bool collectDischarging = false;             // COLLECT 排粮进行中（舵机 90°）
unsigned long collectDischargeStartMs = 0;   // 排粮开始时间 ms
#define COLLECT_DISCHARGE_MS  10000          // 排粮持续 10s

// ============== RL 模型相关 ==============
#ifdef POLICY_MODEL_H
  #include <TensorFlowLite_ESP32.h>
  #include "tensorflow/lite/micro/all_ops_resolver.h"
  #include "tensorflow/lite/micro/micro_interpreter.h"
  #include "tensorflow/lite/micro/micro_error_reporter.h"
  #include "tensorflow/lite/schema/schema_generated.h"

  const tflite::Model* rl_model = nullptr;
  uint8_t* tensor_arena = nullptr;  // 运行时分配到 PSRAM（避免挤占 DRAM）
  bool rl_model_loaded = false;
  unsigned long last_rl_inference = 0;
#else
  bool rl_model_loaded = false;
  #define RL_MODEL_AVAILABLE false
#endif

// ============== 触摸消抖 ==============
uint32_t lastTouchTime = 0;
bool wasPressed = false;
int lastX = 0, lastY = 0;

// ============== 按钮结构 ==============
struct Button {
  int x, y, w, h;
  const char* label;
  uint16_t color;
  bool state;
};

Button buttons[10];

// ============== 自动模式变量 ==============
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 2000;
bool sensorError = false;

// ============== RL 辅助变量 ==============
// 虚拟传感器数据（当没有真实传感器时，用于维持状态估计）
float pcm_temperature = 25.5;     // PCM 温度 (℃)
float ambient_temperature = 25.5; // 环境温度 (℃)
float ambient_humidity = 60.0;    // 环境湿度 (%)
float inlet_air_temp = 26.0;      // 进口风温 (℃)
float outlet_air_temp = 26.0;     // 出口风温 (℃)
float dryer_humidity = 60.0;      // 干燥室湿度 (%)
float water_temperature = 25.5;   // 水箱温度 (℃)
bool  ds18b20Online = false;      // DS18B20 是否在线（断线检测）
bool  ds18b20Checked = false;     // DS18B20 是否已检测过（未检测前不算故障）
bool  heaterOn = false;            // 加热片启停（数字 HIGH/LOW）
bool  collisionDetected = false;   // 碰撞开关当前状态
bool  collisionWasReleased = false; // START 后碰撞开关是否已松开过
bool  dcMotorOn = false;           // 直流电机状态
bool  collectProcessActive = false; // 循环集热模式一键开始
float lightIntensity = 0.0;         // 光照强度 (lux)
bool  bh1750Ready = false;          // BH1750 是否初始化成功

// ============== UART 接收：四个集热器光照（来自 weather_display S3） ==============
float collectorLux[4] = {-1, -1, -1, -1};   // 集热器 1~4 光照，-1=无效
bool  collectorLuxValid[4] = {false, false, false, false};
String uartLineBuf = "";                     // 行缓冲（按 \n 切分）

// ============== UART 接收：天气预报数据（来自 weather_display S3） ==============
bool  wxValid = false;         // 天气数据是否已收到
float wxTemp = 25.0f;          // 环境温度 °C
int   wxHum  = 60;             // 环境湿度 %
int   wxCode = 8;              // 天气码 1晴2多云3阴4雨5雪6雷阵雨7雾8未知
int   wxRainProb = 0;          // 降雨概率 %
int   wxRainHour = -1;         // 降雨开始小时（-1=无雨）
unsigned long wxLastMs = 0;    // 天气数据更新时间

// ============== 导轨舵机 & 碰撞开关 ==============
enum CollectSubState {
  CS_HOMING,        // 回原点：进气管舵机CCW→开关3/4，出气管舵机CW→开关1/2
  CS_READY,         // 原点就绪，等待一键开始
  CS_WAIT_LIGHT,    // 等待光照强度 >= 400 lux
  CS_MOTOR_GO,      // 直流电机运行中，等待位置碰撞开关松开→再压
  CS_PIPES_INSERT,  // 导轨舵机相向而行插入集热器
  CS_AUTO_DRY,      // 自动干燥运行（AI控制温湿度）
  CS_DRY_HOMING,    // 自动循环：导轨回原点
  CS_DRY_MOTOR,     // 自动循环：直流电机 → 等待碰撞
  CS_DRY_INSERT     // 自动循环：插入集热器 → 回 CS_AUTO_DRY
};
CollectSubState collectSubState = CS_HOMING;
// 导轨舵机：仅用方向控制（SERVOMIN=CCW, SERVOMAX=CW, 0=停），不追踪角度
bool railHomingDone = false;      // 导轨回原点是否完成
bool collisionJustPressed = false; // 位置碰撞开关刚被压下的边沿（用于导轮回原点检测）

// 导轨舵机逐台独立控制
bool inletReversing = false;      // 进气管舵机是否正在反转
bool outletReversing = false;     // 出气管舵机是否正在反转
bool inletDone = false;           // 进气管舵机本阶段完成
bool outletDone = false;          // 出气管舵机本阶段完成
static bool inletLastSW = false;  // 进气管控制开关上一状态(3|4)
static bool outletLastSW = false; // 出气管控制开关上一状态(1|2)
bool pipesInserted = false;         // 导轨是否已插入集热器终点位（干燥前提）

// COLLECT 定时切换变量
unsigned long waitLightStartMs = 0;   // 首次启动计时起点（进入 CS_WAIT_LIGHT 时）
unsigned long autoDryStartMs = 0;     // 干燥中换位计时起点（进入 CS_AUTO_DRY 时）

// PCM 焓值追踪变量（用于显示与 AI 观测）
float pcmEnthalpy = 0.0f;         // PCM 当前焓值 kJ

// ============== 光驱集热统一周期控制（动态模型） ==============
// 4 集热器流水线：每个换位周期(10~15s)内，3 个集热器吸热 + 1 个放热
// 同时进行、同时结束，不分先后。换位 = 转盘转动，把吸热/放热的集热器互换。
float absorbRemainSec = 0.0f;              // 换位剩余时间 s（=干燥剩余时间显示）
float collectorFan = 0.0f;                 // 集热循环控制的风机功率 0~1
float absorbDurationSec = ABSORB_T_MIN_S;  // 换位周期 s（光照驱动，放热匹配目标）
bool  collectorForceApply = true;          // 换位完成后强制重新下发元件（导轨归零后需重发）
float collectRefLightIntensity = 0.0f;     // 进入CS_AUTO_DRY时的参考光照 lux，以此为最短干燥时间基准
int   dischargingCollector = -1;           // 正在放热的集热器索引 0~3（按1→2→3→4轮换），-1=未知
// 每个集热器有效光照（直接用其内置光照，用于换位周期/风机）
float collectorEffectiveLux[4] = {0,0,0,0};
float collectorAmbientFactor = 0.3f;       // 环境光照占比（BH1750），集热器光照占 0.7

// ============== COLLECT 手动操控（仅 CS_READY 阶段） ==============
bool manualControlActive = false;   // 手动操控模式是否激活（仅 CS_READY 可用）
bool manualPipeInserted = false;    // 手动进/出气舵机状态：false=离开原点, true=插入集热器
bool manualPipeMoving = false;      // 手动气管舵机是否正在移动（碰撞限位中）
bool manualMotorRunning = false;    // 手动直流电机状态
// 手动舵机到位检测状态（与自动模式 CS_PIPES_INSERT/CS_DRY_HOMING 完全一致的"压到→反转→松开→停止"）
bool manualInletDone = false;       // 手动进气管到位完成
bool manualOutletDone = false;      // 手动出气管到位完成
bool manualInletReversing = false;  // 手动进气管反转中
bool manualOutletReversing = false; // 手动出气管反转中
bool manualInletLastSW = false;     // 手动进气管上次开关状态（边沿检测）
bool manualOutletLastSW = false;    // 手动出气管上次开关状态（边沿检测）
bool manualExitHoming = false;      // 退出手动模式后自动回原点进行中


// 上一次的动作（用于状态估计和平滑）
float lat_fan = 0.0;
float last_fan = 0.0;
float last_damper = 0.5;
float last_pump = 0.0;
float last_discharge = 0.0;
bool  last_heater = false;   // 加热片上次开关状态

// 温度趋势追踪（用于智能分热）
float prev_inlet_temp = 26.0;     // 上一时刻进口风温
float temp_rise_rate = 0.0;       // 温升速率 (℃/步)

// 注：Arduino IDE 会自动为 .ino 中的函数生成原型，无需手动前置声明
// （手动声明易与实际定义签名冲突，如 bh1750Read() 返回值类型等）

void setup() {
  Serial.begin(115200);

  // UART 有线接收（四个集热器光照，来自 weather_display S3）
  Serial1.begin(UART_LINK_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("UART 链路就绪 (RX=GPIO19, TX=GPIO20)");

  initHardware();

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(BG_COLOR);

  // 初始化 RL 模型（tensor arena 分配到 PSRAM，节约 DRAM）
#ifdef POLICY_MODEL_H
  tensor_arena = (uint8_t*)heap_caps_malloc(TENSOR_ARENA_SIZE, MALLOC_CAP_SPIRAM);
  if (!tensor_arena) {
    Serial.println("PSRAM alloc failed! Falling back to DRAM...");
    tensor_arena = (uint8_t*)malloc(TENSOR_ARENA_SIZE);
  }
  if (!tensor_arena) {
    Serial.println("FATAL: Cannot allocate tensor arena!");
  } else {
    Serial.printf("Tensor arena: %d bytes allocated\n", TENSOR_ARENA_SIZE);
  }
  initRLModel();
#else
  Serial.println("WARNING: policy_model.h not included!");
  rl_model_loaded = false;
#endif

  initButtons();
  drawFullScreen();

  Serial.println("Dryer Controller Started");
  Serial.printf("  Mode: %s\n", MODE_LABELS[systemMode]);
}

void loop() {
  unsigned long now = millis();

  // UART 有线接收：解析 weather_display 发来的四个集热器光照 + 天气数据
  parseUartLux();

  // UART 发送：AI 干燥建议发回 weather_display
  // 天气刷新后(收到WX)立即回一条；之后每 60 秒定时刷新
  static unsigned long lastAdviceSendMs = 0;
  static bool lastWxSent = false;
  if ((wxValid && wxLastMs > lastAdviceSendMs && !lastWxSent) ||   // 新天气到达
      (now - lastAdviceSendMs >= 60000)) {                          // 或 60 秒定时
    lastAdviceSendMs = now;
    lastWxSent = true;
    sendAdviceToDisplay();
  }
  if (!wxValid) lastWxSent = false;

  // UART 发送：放热集热器编号 + 干燥剩余时间（每秒发送；干燥期间发有效值，其余阶段发 0/-1）
  static unsigned long lastDisSendMs = 0;
  if (now - lastDisSendMs >= 1000) {
    lastDisSendMs = now;
    if (systemMode == SYSTEM_MODE_COLLECT && collectSubState == CS_AUTO_DRY &&
        dischargingCollector >= 0) {
      // 干燥剩余时间每个周期都发（平板实时显示倒计时）
      Serial1.printf("RMN:%.1f\n", absorbRemainSec);
      // 本次干燥预计时长（由光照计算，供平板显示）
      Serial1.printf("DRT:%.1f\n", absorbDurationSec);
      // "X号正在放热！"字样只在第3次干燥（排粮轮）显示，其余干燥阶段不显示
      if (collectDryCycleCount >= 2) {
        Serial1.printf("DIS:%d\n", dischargingCollector + 1);
      } else {
        Serial1.printf("DIS:0\n");
      }
    } else {
      Serial1.printf("DIS:0\n");
      Serial1.printf("RMN:-1\n");
    }
    // 元件工作状态上报（每秒）：风机% / 水泵% / 加热 / 风门角度
    Serial1.printf("STT:m=%d;p=%d;h=%d;d=%d\n",
                   motorSpeed, pumpFlow, heaterOn ? 1 : 0, servoAngle);
    // COLLECT 阶段上报（供平板在无剩余时间时显示当前阶段；非 COLLECT 发 -1）
    if (systemMode == SYSTEM_MODE_COLLECT) {
      Serial1.printf("STA:%d\n", (int)collectSubState);
    } else {
      Serial1.printf("STA:-1\n");
    }
    // 系统模式上报（供平板退出 MANUAL 后复位风机/水泵滑条）
    Serial1.printf("SYS:%d\n", (int)systemMode);
    // BH1750 实时光照上报（供平板显示当前环境光照；无传感器发 -1）
    Serial1.printf("LUX:%.0f\n", bh1750Ready ? lightIntensity : -1.0f);
    // 设备故障上报（供平板显示是否有故障）
    Serial1.printf("FLT:%d\n", systemFault() ? 1 : 0);
  }

  // 传感器实时读取（COLLECT 模式仅 CS_AUTO_DRY 时读取全部传感器）
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    if (systemMode != SYSTEM_MODE_COLLECT || collectSubState == CS_AUTO_DRY) {
      readSensors();
      updateStateEstimate();
      updateSensorDisplay();
    } else {
      // COLLECT 非干燥阶段：仍定期检测 DS18B20 在线状态，
      // 避免故障指示因"从未检测"而停在初始 false 误报 FAULT
      static unsigned long lastDsCheck = 0;
      if (now - lastDsCheck >= 5000) {
        lastDsCheck = now;
        checkDs18b20();
      }
    }
  }

  if (systemMode == SYSTEM_MODE_COLLECT) {
    // ---- 立体循环集热模式：读取光照强度（早期阶段刷新独立光照面板）----
    static unsigned long lastLightRead = 0;
    if (now - lastLightRead >= 1000) {
      lastLightRead = now;
      float val = bh1750Read();
      if (val >= 0) {
        lightIntensity = val;   // BH1750 读成功才更新光照值
      }
      // 光照面板（含 CS_WAIT_LIGHT 倒计时）每秒刷新，不依赖 BH1750 读取成败
      if (collectSubState != CS_AUTO_DRY) {
        updateLightSensorDisplay();
      }
      updateCollectorLight();   // 结合环境光 + UART 四路集热器光照
      static unsigned long lastLog = 0;
      if (now - lastLog >= 10000) {
        lastLog = now;
        Serial.printf("Light: %.1f lux | Collector: %.0f;%.0f;%.0f;%.0f | Discharging: #%d\n",
                      lightIntensity,
                      collectorEffectiveLux[0], collectorEffectiveLux[1],
                      collectorEffectiveLux[2], collectorEffectiveLux[3],
                      dischargingCollector + 1);
      }
    }

    // ---- 自动干燥模块（CS_AUTO_DRY）：4 集热器 3吸1放，统一换位周期 ----
    if (collectSubState == CS_AUTO_DRY) {
      // ---- COLLECT 排粮：每 3 个干燥周期后排粮 3s，复位 0° 后才退出 CS_AUTO_DRY ----
      if (collectDischarging) {
        if (millis() - collectDischargeStartMs >= COLLECT_DISCHARGE_MS) {
          // 排粮完成：先复位 0°，然后才退出 CS_AUTO_DRY 进入换位阶段（下一轮）
          setOutputServoAngle(0);
          collectDischarging = false;
          collectDryCycleCount = 0;
          Serial.println("COLLECT DISCHARGE: 排粮完成，复位0°，退出干燥阶段");
          // 退出 CS_AUTO_DRY → 导轨回原点换位
          setHeater(false);
          setMotorSpeed(0);
          setPumpFlow(0);
          inletDone = false; outletDone = false;
          inletReversing = false; outletReversing = false;
          inletLastSW = false; outletLastSW = false;
          inletRailServo.writeMicroseconds(RAIL_STOP_US);
          outletRailServo.writeMicroseconds(RAIL_STOP_US);
          pipesInserted = false;                  // 导轨即将离开终点回原点
          collectSubState = CS_DRY_HOMING;
          drawFullScreen();
          Serial.printf("AUTO DRY: cycle done -> homing\n");
        } else {
          // 排粮中：舵机保持 90°，暂停干燥（仍处于 CS_AUTO_DRY 阶段）
          setOutputServoAngle(90);
          updateControlDisplay();
        }
      } else {
        // 统一周期：光照 → 实时估算换位剩余时间（=干燥剩余时间，10~20s）。
        // 周期内 3 吸 1 放同时进行、同时结束；换位 = 转盘转动。
        updateDryModel();

        // 周期性刷新 controls 面板显示（每秒一次，不依赖 AI 推理/整屏重绘）
        // 显示 MOTOR/SERVO/HEAT/PUMP/OUT + 干燥剩余时间(updateRemainDisplay 在其内)
        // motorSpeed 总是最后一次实际下发的值，因此 MOTOR 示数与真实 PWM 一致
        static unsigned long lastRemainDisp = 0;
        if (millis() - lastRemainDisp >= 1000) {
          lastRemainDisp = millis();
          updateControlDisplay();
          drawFaultIndicator();   // 干燥期间每秒刷新设备故障指示
        }

        unsigned long dryElapsedMs = millis() - autoDryStartMs;
        // 换位触发：倒计时到 0（已持续 ≥ 换位周期，周期钳位 [t_min,t_max]）；
        // 或超时最长周期（异常/无光照时保证换位）
        bool maxElapsed = (dryElapsedMs >= (unsigned long)(ABSORB_T_MAX_S * 1000.0f));
        bool cycleDone = (absorbRemainSec <= 0.0f) || maxElapsed;

        if (cycleDone) {
          // 干燥周期结束：温度示数模拟停止并清零（下一轮干燥重新开始）
          dryerTempSimActive = false;
          dryerTempSimAccum = 0.0f;
          // 周期完成；先停掉所有其他元件（吸热/放热同时完成）
          // 注意：周期计数已在进入 CS_AUTO_DRY 时 +1（collectDryCycleCount=1,2,3）
          setHeater(false);
          setMotorSpeed(0);
          setPumpFlow(0);

          if (collectDryCycleCount >= 2) {
            // 第 2 次干燥结束：不退出，先在 CS_AUTO_DRY 内排粮 3s，复位后才换位
            // 注意：collectDryCycleCount 保持 3（排粮提示显示到复位为止）
            collectDischarging = true;
            collectDischargeStartMs = millis();
            setOutputServoAngle(90);   // 排粮舵机打开
            drawFullScreen();
            Serial.println("COLLECT DISCHARGE: 第3次干燥完成，开始排粮 3s");
            // 注意：collectSubState 保持 CS_AUTO_DRY，排粮复位后才进入 CS_DRY_HOMING
          } else {
            // 未到 3 次：直接退出 CS_AUTO_DRY → 换位
            inletDone = false; outletDone = false;
            inletReversing = false; outletReversing = false;
            inletLastSW = false; outletLastSW = false;
            inletRailServo.writeMicroseconds(RAIL_STOP_US);
            outletRailServo.writeMicroseconds(RAIL_STOP_US);
            pipesInserted = false;                  // 导轨即将离开终点回原点
            collectSubState = CS_DRY_HOMING;
            drawFullScreen();   // 整屏重绘为光照面板，清除 CS_AUTO_DRY 残留的传感器/控件
            Serial.printf("AUTO DRY: cycle done (remain=%.1fs, T_pcm=%.1fC, H=%.0fkJ) -> homing\n",
                          computeDryRemainingTime(), pcm_temperature, pcmEnthalpy);
          }
        } else {
          // 周期进行中：本来的自动干燥模型(AI) 与 光驱集热统一控制 融合
          // AI 控制风机/风门/水泵/加热片(干燥维度)；集热控制统一调整四类元件
          // （风机=放热匹配功率，放热时间与光照决定的换位周期一致）
          // 干燥仓温度示数：每秒 +0.3~0.5 随机增长（真实读数另算，互不干扰）
          if (dryerTempSimActive) {
            unsigned long nowSim = millis();
            if (nowSim - dryerTempSimLastMs >= 1000) {
              dryerTempSimLastMs = nowSim;
              float inc = random(30, 51) / 100.0f;   // 0.30~0.50
              dryerTempSimAccum += inc;
              Serial.printf("DRY TEMP SIM: +%.2f -> T示数=%.1fC\n", inc, temperature + dryerTempSimAccum);
            }
          }
          runAIDryModel();
          applyCollectorCycle();
        }
      }
    }

  } else if (systemMode == SYSTEM_MODE_AUTO) {
    updateDischarge();           // 排粮状态机（每 loop 运行，50ms 粒度）

    // v34: 加热片改由 AI 决策（动作第5维），不再强制常开
    // 模型未加载时回退为手动恒开，保证基本干燥能力
#ifndef POLICY_MODEL_H
    if (!heaterOn) setHeater(true);
#endif

#ifdef POLICY_MODEL_H
    if (rl_model_loaded) {
      runAIInference();       // AI 推理控制（模型已加载，每 5 秒一次，含加热片）
    } else {
      // 模型未加载，打印诊断信息（包含 arena 使用情况）
      static unsigned long lastDiag = 0;
      if (now - lastDiag >= 10000) {
        lastDiag = now;
        Serial.printf("AUTO mode - model FAILED to load! Arena=%d bytes (need >150KB for v34)\n",
                      TENSOR_ARENA_SIZE);
        Serial.println("  Check initRLModel() error messages above for details.");
      }
    }
#else
    static unsigned long lastDiag2 = 0;
    if (now - lastDiag2 >= 10000) {
      lastDiag2 = now;
      Serial.println("AUTO mode - policy_model.h NOT compiled!");
    }
#endif
  } else {  // SYSTEM_MODE_MANUAL
    applyManualControls();
  }

  // 导轨碰撞开关实时检测（NC常闭型：未压=LOW，已压=HIGH，反转后 LOW=触发）
  static bool lastSW1 = HIGH, lastSW2 = HIGH, lastSW3 = HIGH, lastSW4 = HIGH;
  bool sw1 = !digitalRead(RAIL_SW1);  // NC开关取反
  bool sw2 = !digitalRead(RAIL_SW2);
  bool sw3 = !digitalRead(RAIL_SW3);
  bool sw4 = !digitalRead(RAIL_SW4);
  if (sw1 != lastSW1 || sw2 != lastSW2 || sw3 != lastSW3 || sw4 != lastSW4) {
    lastSW1 = sw1; lastSW2 = sw2; lastSW3 = sw3; lastSW4 = sw4;
    if (sw1 == LOW || sw2 == LOW || sw3 == LOW || sw4 == LOW) {
      Serial.printf("RAIL SW: 1=%d 2=%d 3=%d 4=%d\n", !sw1, !sw2, !sw3, !sw4);
    }
  }

  // ============== COLLECT 模式：导轨状态机 + 转盘电机 ==============
  if (systemMode == SYSTEM_MODE_COLLECT) {
    static unsigned long lastRailStep = 0;
    // 导轨舵机步进（每 40ms 执行一次）
    bool railNeedUpdate = false;
    if (now - lastRailStep >= 40) {
      lastRailStep = now;
      railNeedUpdate = true;
    }

    switch (collectSubState) {
      case CS_HOMING: {
        // 逐台独立控制：进气管 CCW→压到原点SW4→反转CW→松开→停止
        //                出气管 CW→压到原点SW1→反转CCW→松开→停止
        // v34.17: 只检测原点开关（SW4=进风口原点, SW1=出风口原点），
        //   避免舵机停在插入侧压着终点开关时误判"已到位"，确保真正回原点
        bool inletSwNow  = (sw4 == LOW);  // 仅进风口原点
        bool outletSwNow = (sw1 == LOW);  // 仅出风口原点

        if (railNeedUpdate) {
          
          if (!inletDone && !inletRailServo.attached()) inletRailServo.attach(SERVO_INLET_PIN);
          if (!outletDone && !outletRailServo.attached()) outletRailServo.attach(SERVO_OUTLET_PIN);
          // === 进气管舵机 ===
          if (!inletDone) {
            if (!inletReversing) {
              // 前进：CCW
              if (inletSwNow && !inletLastSW) {
                inletReversing = true;
                inletRailServo.writeMicroseconds(RAIL_CW_US); // 反转CW
              } else {
                inletRailServo.writeMicroseconds(RAIL_CCW_US); // 持续CCW
              }
            } else {
              // 反转：CW，开关松开即完成
              if (!inletSwNow && inletLastSW) {
                inletDone = true;
                inletRailServo.writeMicroseconds(RAIL_STOP_US);
              } else {
                inletRailServo.writeMicroseconds(RAIL_CW_US); // 持续CW
              }
            }
          }

          // === 出气管舵机 ===
          if (!outletDone) {
            if (!outletReversing) {
              // 前进：CW
              if (outletSwNow && !outletLastSW) {
                outletReversing = true;
                outletRailServo.writeMicroseconds(RAIL_CCW_US); // 反转CCW
              } else {
                outletRailServo.writeMicroseconds(RAIL_CW_US); // 持续CW
              }
            } else {
              // 反转：CCW，开关松开即完成
              if (!outletSwNow && outletLastSW) {
                outletDone = true;
                outletRailServo.writeMicroseconds(RAIL_STOP_US);
              } else {
                outletRailServo.writeMicroseconds(RAIL_CCW_US); // 持续CCW
              }
            }
          }

          inletLastSW = inletSwNow;
          outletLastSW = outletSwNow;

          // 调试
          Serial.printf("HOMING | SW:1=%d 2=%d 3=%d 4=%d | "
                        "inSw=%d lastIn=%d rev=%d done=%d | "
                        "outSw=%d lastOut=%d rev=%d done=%d\n",
            !sw1, !sw2, !sw3, !sw4,
            inletSwNow, inletLastSW, inletReversing, inletDone,
            outletSwNow, outletLastSW, outletReversing, outletDone);
        }

        // 两个都完成 → 进入READY
        if (inletDone && outletDone) {
          inletDone = false; outletDone = false;
          inletReversing = false; outletReversing = false;
          inletLastSW = false; outletLastSW = false;
          railHomingDone = true;
          collectSubState = CS_READY;
          // 进入 CS_READY：复位手动操控状态（起始为非手动，START 可点）
          manualControlActive = false;
          manualPipeInserted = false;
          manualPipeMoving = false;
          manualMotorRunning = false;
          manualInletDone = false; manualOutletDone = false;
          manualInletReversing = false; manualOutletReversing = false;
          manualInletLastSW = false; manualOutletLastSW = false;
          Serial.println("Rail home done. Ready for START.");
          drawFullScreen();
        }
        break;
      }

      case CS_READY:
        // 等待一键开始按钮
        // 手动操控模式下：驱动气管舵机（插入/离开），与自动模式 CS_PIPES_INSERT/CS_DRY_HOMING
        // 完全一致的"压到开关 → 反转 → 松开 → 停止"防过头逻辑
        // manualExitHoming: 退出手动模式后的自动回原点（无论舵机状态，都执行回初始状态程序）
        // 退出手动时 manualControlActive 保持 true，回原点完成后才置 false 回到 ready
        if ((manualControlActive && manualPipeMoving) || manualExitHoming) {
          // 读取碰撞开关（NC常闭型，取反后 LOW=触发，与主循环一致）
          bool msw1 = !digitalRead(RAIL_SW1);  // 出风口原点触发=LOW
          bool msw2 = !digitalRead(RAIL_SW2);  // 出风口终点触发=LOW
          bool msw3 = !digitalRead(RAIL_SW3);  // 进风口终点触发=LOW
          bool msw4 = !digitalRead(RAIL_SW4);  // 进风口原点触发=LOW

          if (!inletRailServo.attached()) inletRailServo.attach(SERVO_INLET_PIN);
          if (!outletRailServo.attached()) outletRailServo.attach(SERVO_OUTLET_PIN);

          if (manualPipeInserted) {
            // ===== 插入集热器（同 CS_PIPES_INSERT）=====
            // v34.17: 只检测终点开关（进=msw3终点, 出=msw2终点）
            bool mInSw  = (msw3 == LOW);  // 仅进风口终点
            bool mOutSw = (msw2 == LOW);  // 仅出风口终点

            // === 进气管舵机 ===
            if (!manualInletDone) {
              if (!manualInletReversing) {
                if (mInSw && !manualInletLastSW) {
                  manualInletReversing = true;
                  inletRailServo.writeMicroseconds(RAIL_CCW_US); // 反转CCW
                } else {
                  inletRailServo.writeMicroseconds(RAIL_CW_US); // 前进CW
                }
              } else {
                if (!mInSw && manualInletLastSW) {
                  manualInletDone = true;
                  inletRailServo.writeMicroseconds(RAIL_STOP_US);
                } else {
                  inletRailServo.writeMicroseconds(RAIL_CCW_US); // 持续CCW
                }
              }
            }

            // === 出气管舵机 ===
            if (!manualOutletDone) {
              if (!manualOutletReversing) {
                if (mOutSw && !manualOutletLastSW) {
                  manualOutletReversing = true;
                  outletRailServo.writeMicroseconds(RAIL_CW_US); // 反转CW
                } else {
                  outletRailServo.writeMicroseconds(RAIL_CCW_US); // 前进CCW
                }
              } else {
                if (!mOutSw && manualOutletLastSW) {
                  manualOutletDone = true;
                  outletRailServo.writeMicroseconds(RAIL_STOP_US);
                } else {
                  outletRailServo.writeMicroseconds(RAIL_CW_US); // 持续CW
                }
              }
            }

            manualInletLastSW = mInSw;
            manualOutletLastSW = mOutSw;
          } else {
            // ===== 离开回原点（同 CS_DRY_HOMING）=====
            // v34.17: 只检测原点开关（进=msw4原点, 出=msw1原点）
            bool mInSw  = (msw4 == LOW);  // 仅进风口原点
            bool mOutSw = (msw1 == LOW);  // 仅出风口原点

            // === 进气管舵机 ===
            if (!manualInletDone) {
              if (!manualInletReversing) {
                if (mInSw && !manualInletLastSW) {
                  manualInletReversing = true;
                  inletRailServo.writeMicroseconds(RAIL_CW_US); // 反转CW
                } else {
                  inletRailServo.writeMicroseconds(RAIL_CCW_US); // 前进CCW
                }
              } else {
                if (!mInSw && manualInletLastSW) {
                  manualInletDone = true;
                  inletRailServo.writeMicroseconds(RAIL_STOP_US);
                } else {
                  inletRailServo.writeMicroseconds(RAIL_CW_US); // 持续CW
                }
              }
            }

            // === 出气管舵机 ===
            if (!manualOutletDone) {
              if (!manualOutletReversing) {
                if (mOutSw && !manualOutletLastSW) {
                  manualOutletReversing = true;
                  outletRailServo.writeMicroseconds(RAIL_CCW_US); // 反转CCW
                } else {
                  outletRailServo.writeMicroseconds(RAIL_CW_US); // 前进CW
                }
              } else {
                if (!mOutSw && manualOutletLastSW) {
                  manualOutletDone = true;
                  outletRailServo.writeMicroseconds(RAIL_STOP_US);
                } else {
                  outletRailServo.writeMicroseconds(RAIL_CCW_US); // 持续CCW
                }
              }
            }

            manualInletLastSW = mInSw;
            manualOutletLastSW = mOutSw;
          }

          // 两个都完成 → 停止移动
          if (manualInletDone && manualOutletDone) {
            inletRailServo.writeMicroseconds(RAIL_STOP_US);
            outletRailServo.writeMicroseconds(RAIL_STOP_US);
            manualPipeMoving = false;
            manualInletDone = false;
            manualOutletDone = false;
            manualInletReversing = false;
            manualOutletReversing = false;
            manualInletLastSW = false;
            manualOutletLastSW = false;
            if (manualExitHoming) {
              // 退出手动模式中的自动回原点已完成 → 真正退出手动模式，回到初始状态
              manualExitHoming = false;
              manualControlActive = false;
              manualPipeInserted = false;
              // 刷新界面：手动面板恢复"MANUAL"按钮，START 恢复可点
              drawCollectManualPanel();
              drawCollectPanel();
              Serial.println("COLLECT: Manual override OFF (pipes homed)");
            } else {
              // 正常手动插入/离开完成：刷新按钮显示（IN>> → INSERTED / OUT>> → LEAVED）
              drawCollectManualPanel();
              Serial.println(manualPipeInserted
                ? "MANUAL PIPE: Inserted (reverse-release done)"
                : "MANUAL PIPE: Left/Homed (reverse-release done)");
            }
          }
        }
        break;

      case CS_WAIT_LIGHT: {
        // 固定等待 10 秒后启动直流电机（不受光照等任何因素影响）
        if (millis() - waitLightStartMs >= COLLECT_START_DELAY_MS) {
          collisionWasReleased = false;
          collisionDetected = collisionActive();
          setDCMotor(true);
          collectSubState = CS_MOTOR_GO;
          Serial.println("COLLECT: 10s delay done -> Motor ON");
          drawCollectPanel();
        }
        break;
      }

      case CS_MOTOR_GO: {
        // 直流电机运行中，等待碰撞开关松开→再压
        bool newCollision = collisionActive();
        if (newCollision != collisionDetected) {
          collisionDetected = newCollision;
          if (!collisionDetected) {
            collisionWasReleased = true;
            Serial.println("COLLISION released - armed");
          } else if (collisionWasReleased) {
            // 再次压下 → 停电机 → 转 PIPES_INSERT
            setDCMotor(false);
            Serial.println("COLLISION! Re-pressed - Motor STOP -> Insert pipes");
            collisionWasReleased = false;
            railHomingDone = false;               // 导轨即将离开原点
            pipesInserted = false;                // 导轨离开终点
            collectSubState = CS_PIPES_INSERT;
            inletDone = false; outletDone = false;
            inletReversing = false; outletReversing = false;
            inletLastSW = false; outletLastSW = false;
            drawCollectPanel();
          }
        }
        break;
      }

      case CS_PIPES_INSERT: {
        // 逐台独立控制：进气管 CW→压到终点SW3→反转CCW→松开→停止
        //                出气管 CCW→压到终点SW2→反转CW→松开→停止
        // v34.17: 只检测终点开关（SW3=进风口终点, SW2=出风口终点），
        //   确保插入动作准确到位，不误判原点开关
        bool inletSwNow  = (sw3 == LOW);  // 仅进风口终点
        bool outletSwNow = (sw2 == LOW);  // 仅出风口终点

        if (railNeedUpdate) {
          
          if (!inletDone && !inletRailServo.attached()) inletRailServo.attach(SERVO_INLET_PIN);
          if (!outletDone && !outletRailServo.attached()) outletRailServo.attach(SERVO_OUTLET_PIN);
          // === 进气管舵机 ===
          if (!inletDone) {
            if (!inletReversing) {
              if (inletSwNow && !inletLastSW) {
                inletReversing = true;
                inletRailServo.writeMicroseconds(RAIL_CCW_US); // 反转CCW
              } else {
                inletRailServo.writeMicroseconds(RAIL_CW_US); // 前进CW
              }
            } else {
              if (!inletSwNow && inletLastSW) {
                inletDone = true;
                inletRailServo.writeMicroseconds(RAIL_STOP_US);
              } else {
                inletRailServo.writeMicroseconds(RAIL_CCW_US); // 持续CCW
              }
            }
          }

          // === 出气管舵机 ===
          if (!outletDone) {
            if (!outletReversing) {
              if (outletSwNow && !outletLastSW) {
                outletReversing = true;
                outletRailServo.writeMicroseconds(RAIL_CW_US); // 反转CW
              } else {
                outletRailServo.writeMicroseconds(RAIL_CCW_US); // 前进CCW
              }
            } else {
              if (!outletSwNow && outletLastSW) {
                outletDone = true;
                outletRailServo.writeMicroseconds(RAIL_STOP_US);
              } else {
                outletRailServo.writeMicroseconds(RAIL_CW_US); // 持续CW
              }
            }
          }

          inletLastSW = inletSwNow;
          outletLastSW = outletSwNow;

          Serial.printf("INSERT | SW:1=%d 2=%d 3=%d 4=%d | "
                        "inSw=%d lastIn=%d rev=%d done=%d | "
                        "outSw=%d lastOut=%d rev=%d done=%d\n",
            !sw1, !sw2, !sw3, !sw4,
            inletSwNow, inletLastSW, inletReversing, inletDone,
            outletSwNow, outletLastSW, outletReversing, outletDone);
        }

        // 两个都完成 → 进入自动干燥模块
        if (inletDone && outletDone) {
          inletDone = false; outletDone = false;
          inletReversing = false; outletReversing = false;
          inletLastSW = false; outletLastSW = false;
          pipesInserted = true;               // 导轨已到终点，允许干燥
          inletRailServo.writeMicroseconds(RAIL_STOP_US);
          outletRailServo.writeMicroseconds(RAIL_STOP_US);
          collectSubState = CS_AUTO_DRY;
          autoDryStartMs = millis();          // 记录干燥计时起点
          // 干燥开始：温度示数模拟从 0 重新累计（真实读数不动）
          dryerTempSimAccum = 0.0f;
          dryerTempSimActive = true;
          dryerTempSimLastMs = millis();
          // 统一换位周期：重置剩余时间，并强制重新下发元件
          absorbRemainSec = 0.0f;
          collectorForceApply = true;
          collectDryCycleCount++;             // 新周期开始计数（1,2,3；复位后重计）
          // 记录参考光照（吸热侧有效光照），作为最短干燥时间基准
          updateCollectorLight();
          dischargingCollector = 0;  // 首次进干燥：固定起点为 1 号集热器
          collectRefLightIntensity = absorbingLight();
          Serial.printf("Pipes inserted. Starting AUTO DRY module. RefLight: %.1f lux (absorbing)\n", collectRefLightIntensity);
          drawFullScreen();
        }
        break;
      }

      // ====== 自动循环：导轨回原点（与 CS_HOMING 逻辑相同） ======
      case CS_DRY_HOMING: {
        // v34.17: 只检测原点开关（SW4=进风口原点, SW1=出风口原点）
        bool inletSwNow  = (sw4 == LOW);  // 仅进风口原点
        bool outletSwNow = (sw1 == LOW);  // 仅出风口原点

        if (railNeedUpdate) {
          
          if (!inletDone && !inletRailServo.attached()) inletRailServo.attach(SERVO_INLET_PIN);
          if (!outletDone && !outletRailServo.attached()) outletRailServo.attach(SERVO_OUTLET_PIN);
          // 进气管：CCW → 压 SW3/SW4 → 反转 CW
          if (!inletDone) {
            if (!inletReversing) {
              if (inletSwNow && !inletLastSW) {
                inletReversing = true;
                inletRailServo.writeMicroseconds(RAIL_CW_US); // 反转CW
              } else {
                inletRailServo.writeMicroseconds(RAIL_CCW_US); // 持续CCW
              }
            } else {
              if (!inletSwNow && inletLastSW) {
                inletDone = true;
                inletRailServo.writeMicroseconds(RAIL_STOP_US);
              } else {
                inletRailServo.writeMicroseconds(RAIL_CW_US); // 持续CW
              }
            }
          }
          // 出气管：CW → 压 SW1/SW2 → 反转 CCW
          if (!outletDone) {
            if (!outletReversing) {
              if (outletSwNow && !outletLastSW) {
                outletReversing = true;
                outletRailServo.writeMicroseconds(RAIL_CCW_US); // 反转CCW
              } else {
                outletRailServo.writeMicroseconds(RAIL_CW_US); // 持续CW
              }
            } else {
              if (!outletSwNow && outletLastSW) {
                outletDone = true;
                outletRailServo.writeMicroseconds(RAIL_STOP_US);
              } else {
                outletRailServo.writeMicroseconds(RAIL_CCW_US); // 持续CCW
              }
            }
          }
          inletLastSW = inletSwNow;
          outletLastSW = outletSwNow;
          Serial.printf("DRY_HOME | SW:1=%d 2=%d 3=%d 4=%d | "
                        "inSw=%d lastIn=%d rev=%d done=%d | "
                        "outSw=%d lastOut=%d rev=%d done=%d\n",
            !sw1, !sw2, !sw3, !sw4,
            inletSwNow, inletLastSW, inletReversing, inletDone,
            outletSwNow, outletLastSW, outletReversing, outletDone);
        }
        if (inletDone && outletDone) {
          inletDone = false; outletDone = false;
          inletReversing = false; outletReversing = false;
          inletLastSW = false; outletLastSW = false;
          railHomingDone = true;                  // 标记原点已回，允许启动直流电机
          // 启动直流电机，等待碰撞开关
          collisionWasReleased = false;
          collisionDetected = collisionActive();
          setDCMotor(true);
          collectSubState = CS_DRY_MOTOR;
          drawCollectPanel();
          Serial.println("DRY: Home done -> Motor ON");
        }
        break;
      }

      // ====== 自动循环：直流电机 → 等待碰撞（松开→压回） ======
      case CS_DRY_MOTOR: {
        bool newCollision = collisionActive();
        if (newCollision != collisionDetected) {
          collisionDetected = newCollision;
          if (!collisionDetected) {
            collisionWasReleased = true;
            Serial.println("DRY: Collision released - armed");
          } else if (collisionWasReleased) {
            setDCMotor(false);
            // 压到碰撞开关 → 换位完成：放热台按 1→2→3→4→1 顺序轮换
            updateCollectorLight();
            dischargingCollector = (dischargingCollector + 1) % 4;
            Serial.printf("DRY: Re-pressed! Discharging collector = #%d (cycle rotation)\n",
                          dischargingCollector + 1);
            collisionWasReleased = false;
            railHomingDone = false;               // 导轨即将离开原点
            pipesInserted = false;                // 导轨离开终点
            collectSubState = CS_DRY_INSERT;
            inletDone = false; outletDone = false;
            inletReversing = false; outletReversing = false;
            inletLastSW = false; outletLastSW = false;
            drawCollectPanel();
          }
        }
        break;
      }

      // ====== 自动循环：插入集热器（与 CS_PIPES_INSERT 逻辑相同） ======
      case CS_DRY_INSERT: {
        // v34.17: 只检测终点开关（SW3=进风口终点, SW2=出风口终点）
        bool inletSwNow  = (sw3 == LOW);  // 仅进风口终点
        bool outletSwNow = (sw2 == LOW);  // 仅出风口终点

        if (railNeedUpdate) {
          // 预计算脉宽，最后一次性发送，保证两舵机同步启动
          int inletPulse  = RAIL_STOP_US;
          int outletPulse = RAIL_STOP_US;

          // 进气管：CW → 压 SW3/SW4 → 反转 CCW
          if (!inletDone) {
            if (!inletReversing) {
              if (inletSwNow && !inletLastSW) {
                inletReversing = true;
                inletPulse = RAIL_CCW_US; // 反转
              } else {
                inletPulse = RAIL_CW_US;  // 前进
              }
            } else {
              if (!inletSwNow && inletLastSW) {
                inletDone = true;
              } else {
                inletPulse = RAIL_CCW_US; // 持续反转
              }
            }
          }
          // 出气管：CCW → 压 SW1/SW2 → 反转 CW
          if (!outletDone) {
            if (!outletReversing) {
              if (outletSwNow && !outletLastSW) {
                outletReversing = true;
                outletPulse = RAIL_CW_US;  // 反转
              } else {
                outletPulse = RAIL_CCW_US; // 前进
              }
            } else {
              if (!outletSwNow && outletLastSW) {
                outletDone = true;
              } else {
                outletPulse = RAIL_CW_US;  // 持续反转
              }
            }
          }

          // 同时发送两路脉宽
          inletRailServo.writeMicroseconds(inletPulse);
          outletRailServo.writeMicroseconds(outletPulse);

          inletLastSW = inletSwNow;
          outletLastSW = outletSwNow;
          Serial.printf("DRY_INSERT | SW:1=%d 2=%d 3=%d 4=%d | "
                        "iP=%d oP=%d | "
                        "iSw=%d iLst=%d iRv=%d iDn=%d | "
                        "oSw=%d oLst=%d oRv=%d oDn=%d\n",
            !sw1, !sw2, !sw3, !sw4,
            inletPulse, outletPulse,
            inletSwNow, inletLastSW, inletReversing, inletDone,
            outletSwNow, outletLastSW, outletReversing, outletDone);
        }
        // 两个都完成 → 回 CS_AUTO_DRY，等待下次光照触发
        if (inletDone && outletDone) {
          inletDone = false; outletDone = false;
          inletReversing = false; outletReversing = false;
          inletLastSW = false; outletLastSW = false;
          pipesInserted = true;               // 导轨已到终点，允许干燥
          inletRailServo.writeMicroseconds(RAIL_STOP_US);
          outletRailServo.writeMicroseconds(RAIL_STOP_US);
          collectSubState = CS_AUTO_DRY;
          autoDryStartMs = millis();          // 记录干燥计时起点（重新计时）
          // 干燥开始：温度示数模拟从 0 重新累计（真实读数不动）
          dryerTempSimAccum = 0.0f;
          dryerTempSimActive = true;
          dryerTempSimLastMs = millis();
          // 统一换位周期：重置剩余时间，并强制重新下发元件
          absorbRemainSec = 0.0f;
          collectorForceApply = true;
          collectDryCycleCount++;             // 新周期开始计数（1,2,3；复位后重计）
          // 刷新各台光照（放热台已在换位时轮换，此处不重选）
          updateCollectorLight();
          // 不更新 collectRefLightIntensity，保持首次 START 时的参考光照不变
          Serial.printf("DRY: Insert done -> Back to AUTO DRY. Discharging #%d. RefLight unchanged: %.1f lux\n",
                        dischargingCollector + 1, collectRefLightIntensity);
          drawFullScreen();
        }
        break;
      }
    }

    // 手动模式安全保护：接近开关"松开→再压"后停止（同自动模式 CS_MOTOR_GO 的转圈定位逻辑）
    if (manualMotorRunning) {
      bool newCollision = collisionActive();
      if (newCollision != collisionDetected) {
        collisionDetected = newCollision;
        if (!collisionDetected) {
          collisionWasReleased = true;
          Serial.println("MANUAL MOTOR: Proximity released - armed");
        } else if (collisionWasReleased) {
          setDCMotor(false);
          manualMotorRunning = false;
          collisionWasReleased = false;
          Serial.println("MANUAL MOTOR: STOP - proximity re-triggered!");
          drawCollectManualPanel();
        }
      }
    }

    // 碰撞开关指示灯
    tft.fillCircle(130, 52, 5, collisionDetected ? DANGER_COLOR : SUCCESS_COLOR);
  } else {
    // 非 COLLECT 模式：电机始终关闭
    if (dcMotorOn) setDCMotor(false);
    bool newCollision = collisionActive();
    if (newCollision != collisionDetected) {
      collisionDetected = newCollision;
      updateSensorDisplay();
    }
  }

  // ============== 非 COLLECT 模式：导轨舵机完全禁止工作（持续拉停） ==============
  if (systemMode != SYSTEM_MODE_COLLECT) {
    inletRailServo.writeMicroseconds(RAIL_STOP_US);
    outletRailServo.writeMicroseconds(RAIL_STOP_US);
  }

  handleTouch();
  delay(50);
}


// ==================== 硬件初始化 ====================

void initHardware() {
  dht.begin();
  Serial.printf("DS18B20 传感器就绪 (GPIO%d)\n", DS18B20_PIN);
  checkDs18b20();   // 开机立即检测在线状态，避免故障指示初始误报


  // 电机 PWM（风机1 与 风机2 同步启停，独立引脚/通道互不干扰）
  ledcAttach(MOTOR_PWM_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(FAN2_PWM_PIN, PWM_FREQ, PWM_RES);

  // 水泵 PWM
  ledcAttach(PUMP_PWM_PIN, PWM_FREQ, PWM_RES);

  // 加热片 MOS驱动模块（IO/PWM → GPIO9, HIGH=加热, LOW=关闭）
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);   // 初始关闭

  // 碰撞开关 GPIO
  pinMode(COLLISION_PIN, INPUT_PULLUP);
  // NPN常闭接近开关 GPIO（开集电极输出，需上拉）
  pinMode(PROX_PIN, INPUT_PULLUP);

  // 导轨碰撞开关 (LOW=触发)
  pinMode(RAIL_SW1, INPUT_PULLUP);
  pinMode(RAIL_SW2, INPUT_PULLUP);
  pinMode(RAIL_SW3, INPUT_PULLUP);
  pinMode(RAIL_SW4, INPUT_PULLUP);

  // BTS7960 直流电机（仅 COLLECT 模式工作）：RPWM/LPWM 用 LEDC PWM 控制
  ledcAttach(DC_MOTOR_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(DC_MOTOR_LPWM, PWM_FREQ, PWM_RES);
  ledcWrite(DC_MOTOR_RPWM, 0);
  ledcWrite(DC_MOTOR_LPWM, 0);
  dcMotorOn = false;

  // I2C 总线初始化（BH1750 + PCA9685 共用）
  Wire.begin(I2C_SDA, I2C_SCL);

  // BH1750 光照传感器初始化
  bh1750Ready = bh1750Init();
  if (bh1750Ready) {
    Serial.println("BH1750 光照传感器初始化成功");
  } else {
    Serial.println("BH1750 初始化失败，请检查 I2C 接线 (SDA=GPIO40, SCL=GPIO41)");
  }

  // PCA9685 舵机驱动板初始化（风门 CH4、排粮 CH5，共用 I2C）
  pwm.begin();
  pwm.setPWMFreq(50);  // 50Hz 舵机频率
  Serial.println("PCA9685 初始化成功 (0x40, 50Hz, CH4=风门, CH5=排粮)");

  // 导轨舵机初始化（先写停止脉宽再 attach，PWM 启动瞬间即是 1500us 无抖动）
  Serial.println("导轨舵机初始化（ESP32Servo）...");
  inletRailServo.writeMicroseconds(RAIL_STOP_US);     // 先缓存停止值
  outletRailServo.writeMicroseconds(RAIL_STOP_US);
  inletRailServo.attach(SERVO_INLET_PIN);     // GPIO7:  进风口导轨（激活时直接用缓存值）
  outletRailServo.attach(SERVO_OUTLET_PIN);   // GPIO10: 出风口导轨
  Serial.printf("  进风口导轨: GPIO%d  出风口导轨: GPIO%d\n", SERVO_INLET_PIN, SERVO_OUTLET_PIN);


  // 初始状态: 风机停、水泵停、风门全开向干燥仓(0°)、排粮口关闭、加热片关闭
  setMotorSpeed(0);
  setPumpFlow(0);
  setServoAngle(0);
  setOutputServoAngle(0);
}


// ==================== RL 模型初始化 ====================

#ifdef POLICY_MODEL_H
// TensorFlowLite_ESP32 要求 AllOpsResolver 和 MicroInterpreter 在栈上声明
static tflite::AllOpsResolver* rl_resolver = nullptr;
static tflite::MicroInterpreter* rl_interpreter = nullptr;
static TfLiteTensor* rl_input = nullptr;
static TfLiteTensor* rl_output = nullptr;

void initRLModel() {
  rl_model = tflite::GetModel(policy_model_data);
  if (rl_model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.printf("模型版本错误! 期望:%d 实际:%d\n", TFLITE_SCHEMA_VERSION, rl_model->version());
    rl_model_loaded = false;
    return;
  }

  // 使用 static 确保对象生命周期与程序相同（栈分配）
  static tflite::AllOpsResolver resolver;
  rl_resolver = &resolver;

  static tflite::MicroErrorReporter error_reporter;
  static tflite::MicroInterpreter static_interpreter(
      rl_model, *rl_resolver, tensor_arena, TENSOR_ARENA_SIZE,
      &error_reporter);
  rl_interpreter = &static_interpreter;

  if (rl_interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("TFLite 张量分配失败!");
    rl_model_loaded = false;
    return;
  }

  rl_input  = rl_interpreter->input(0);
  rl_output = rl_interpreter->output(0);

  rl_model_loaded = true;
  Serial.printf("RL 模型加载成功\n");
  Serial.printf("  输入: [%d x %d]\n", rl_input->dims->data[0], rl_input->dims->data[1]);
  Serial.printf("  输出: [%d x %d]\n", rl_output->dims->data[0], rl_output->dims->data[1]);
  Serial.printf("  Arena: %d / %d bytes\n", rl_interpreter->arena_used_bytes(), TENSOR_ARENA_SIZE);
}
#endif


// ==================== 硬件控制函数 ====================

void setMotorSpeed(int speedPercent) {
  speedPercent = constrain(speedPercent, 0, 100);
  motorSpeed = speedPercent;
  int pwmValue = map(speedPercent, 0, 100, 0, 255);
  ledcWrite(MOTOR_PWM_PIN, pwmValue);   // 风机1
  ledcWrite(FAN2_PWM_PIN, pwmValue);    // 风机2（同步）
  Serial.printf("Motor: %d%% -> PWM:%d (FAN1+2)\n", speedPercent, pwmValue);
}

void setPumpFlow(int flowPercent) {
  flowPercent = constrain(flowPercent, 0, 100);
  pumpFlow = flowPercent;
  pumpOn = (flowPercent > 0);
  int pwmValue = map(flowPercent, 0, 100, 0, 255);
  ledcWrite(PUMP_PWM_PIN, pwmValue);
  Serial.printf("Pump: %d%% -> PWM:%d\n", flowPercent, pwmValue);
}

void setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  servoAngle = angle;
  uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(SERVO_CH_DAMPER, 0, pulse);
  Serial.printf("Servo(Damper CH%d): %d° -> pulse:%d\n", SERVO_CH_DAMPER, angle, pulse);
}

void setOutputServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  outputServoAngle = angle;
  uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(OUTPUT_SERVO_CH, 0, pulse);
  Serial.printf("Output Servo(CH%d): %d° -> pulse:%d\n", OUTPUT_SERVO_CH, angle, pulse);
}

// 导轨舵机（方向控制：SERVOMIN=CCW, SERVOMAX=CW, 0=停）

// 加热片 MOS驱动：高电平导通加热，低电平关闭
void setHeater(bool on) {
  // COLLECT 模式安全保护：导轨未插入集热器终点时禁止加热
  if (on && systemMode == SYSTEM_MODE_COLLECT && !pipesInserted) {
    if (heaterOn) {
      heaterOn = false;
      digitalWrite(HEATER_PIN, LOW);
    }
    Serial.println("Heater: BLOCKED - pipes not inserted!");
    return;
  }
  heaterOn = on;
  digitalWrite(HEATER_PIN, on ? HIGH : LOW);
  Serial.printf("Heater: %s\n", on ? "ON" : "OFF");
}

// ============ 直流电机控制 ============
void setDCMotor(bool on) {
  // 非 COLLECT 模式下直流电机完全禁止工作
  if (on && systemMode != SYSTEM_MODE_COLLECT) {
    if (dcMotorOn) setDCMotor(false);
    return;
  }
  // 安全保护：导轨未回原点时禁止启动直流电机
  if (on && !railHomingDone) {
    if (dcMotorOn) setDCMotor(false);
    Serial.println("DC Motor: BLOCKED - rails not homed!");
    return;
  }
  // 安全保护（v34.16）：手动模式下，气管舵机已插入集热器时禁止启动直流电机
  // 只有舵机回到原点才能启动电机，防止插入时移动破坏结构
  if (on && manualControlActive && manualPipeInserted) {
    if (dcMotorOn) setDCMotor(false);
    Serial.println("DC Motor: BLOCKED - pipes inserted! Return to origin first.");
    return;
  }
  // 安全保护（v34.16）：手动模式下，气管舵机正在移动时禁止启动直流电机（互斥）
  if (on && manualControlActive && manualPipeMoving) {
    if (dcMotorOn) setDCMotor(false);
    Serial.println("DC Motor: BLOCKED - pipes moving! Wait for servo stop.");
    return;
  }
  dcMotorOn = on;
  if (on) {
    // 正转：RPWM 满占空比，LPWM 低（低边导通）
    ledcWrite(DC_MOTOR_RPWM, 255);
    ledcWrite(DC_MOTOR_LPWM, 0);
  } else {
    // 停止（低边刹车）：两路都低 → 电机两端接 GND 短路制动
    ledcWrite(DC_MOTOR_RPWM, 0);
    ledcWrite(DC_MOTOR_LPWM, 0);
  }
  Serial.printf("DC Motor: %s (REVERSE)\n", on ? "ON" : "OFF");
}

// 碰撞检测：仅使用 NPN 接近开关（检测到金属=HIGH 触发），位置碰撞开关已屏蔽
bool collisionActive() {
  return (digitalRead(PROX_PIN) == HIGH);   // NPN接近开关触发（检测到金属=HIGH）
}

// ==================== BH1750 光照传感器驱动（原生 Wire，无需外部库） ====================

bool bh1750Init() {
  Wire.beginTransmission(BH1750_ADDR);
  Wire.write(0x01);  // Power ON
  if (Wire.endTransmission() != 0) return false;
  delay(10);
  Wire.beginTransmission(BH1750_ADDR);
  Wire.write(0x10);  // Continuous High-Resolution Mode (1lux, 120ms)
  if (Wire.endTransmission() != 0) return false;
  delay(180);
  return true;
}

float bh1750Read() {
  if (!bh1750Ready) return -1.0;
  Wire.requestFrom(BH1750_ADDR, 2);
  if (Wire.available() < 2) return -1.0;
  uint16_t raw = (Wire.read() << 8) | Wire.read();
  return raw / 1.2;
}

// ==================== UART 接收：四个集热器光照解析 ====================
// 协议：weather_display 每 2 秒发送 "LUX:v1;v2;v3;v4\n"（v=-1 表示该节点离线）
void parseUartLux() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      // 诊断：打印收到的原始行（验证是否收到干净数据）
      if (uartLineBuf.length() > 0) {
        Serial.printf("[UART] got: '%s'\n", uartLineBuf.c_str());
      }
      // 处理完整一行
      if (uartLineBuf.startsWith("LUX:")) {
        String vals = uartLineBuf.substring(4);
        int idx = 0;
        int start = 0;
        for (int i = 0; i <= vals.length(); i++) {
          if (i == vals.length() || vals.charAt(i) == ';') {
            if (idx < 4) {
              String s = vals.substring(start, i);
              s.trim();
              float v = s.toFloat();
              collectorLux[idx] = v;
              collectorLuxValid[idx] = (v >= 0);
            }
            idx++;
            start = i + 1;
          }
        }
      } else if (uartLineBuf.startsWith("WX:")) {
        // WX:温度;湿度;天气码;降雨概率;降雨开始小时
        String vals = uartLineBuf.substring(3);
        float f[5]; int idx = 0; int start = 0;
        for (int i = 0; i <= vals.length(); i++) {
          if (i == vals.length() || vals.charAt(i) == ';') {
            if (idx < 5) f[idx++] = vals.substring(start, i).toFloat();
            start = i + 1;
          }
        }
        wxValid = true;
        wxTemp = f[0];
        wxHum = (int)f[1];
        wxCode = (int)f[2];
        wxRainProb = (int)f[3];
        wxRainHour = (int)f[4];
        wxLastMs = millis();
        Serial.printf("Weather<- temp=%.1f hum=%d code=%d rain=%d%%@%dh\n",
                      wxTemp, wxHum, wxCode, wxRainProb, wxRainHour);
      } else if (uartLineBuf.startsWith("CTL:")) {
        // 平板经天气屏转发的远程操控指令（复用各执行器的自带安全保护）
        String c = uartLineBuf.substring(4);
        if (c == "START") {
          collectStart();
          drawFullScreen();
          Serial.println("CTL-> START");
        } else if (c == "STOP") {
          collectStop();
          drawFullScreen();
          Serial.println("CTL-> STOP");
        } else if (c == "MANUAL") {
          collectManualToggle();
          Serial.println("CTL-> MANUAL");
        } else if (c == "INSERT") {
          collectManualInsert();
          Serial.println("CTL-> INSERT");
        } else if (c == "LEAVE") {
          collectManualLeave();
          Serial.println("CTL-> LEAVE");
        } else if (c == "MOTOR") {
          collectManualMotor();
          Serial.println("CTL-> MOTOR");
        } else if (c.startsWith("MODE=")) {
          // 远程切换系统模式（带 switchMode 的安全保护）
          String m = c.substring(5);
          int newMode = -1;
          if (m == "COLLECT") newMode = SYSTEM_MODE_COLLECT;
          else if (m == "AUTO") newMode = SYSTEM_MODE_AUTO;
          else if (m == "MANUAL") newMode = SYSTEM_MODE_MANUAL;
          if (newMode >= 0) {
            switchMode(newMode);
            Serial.printf("CTL-> MODE=%s\n", MODE_LABELS[systemMode]);
          } else {
            Serial.printf("CTL-> unknown mode: %s\n", m.c_str());
          }
        } else if (c.startsWith("FAN=") || c.startsWith("PUMP=") ||
                   c.startsWith("HEATER=") || c.startsWith("DAMPER=")) {
          // 干燥元件（风机/水泵/加热片/风门）：仅 MANUAL 模式可远程操控
          if (systemMode != SYSTEM_MODE_MANUAL) {
            Serial.printf("CTL-> %s BLOCKED: only in MANUAL mode\n", c.c_str());
          } else if (c.startsWith("FAN=")) {
            int v = c.substring(4).toInt();
            setMotorSpeed(v);
            Serial.printf("CTL-> FAN=%d%%\n", v);
          } else if (c.startsWith("PUMP=")) {
            int v = c.substring(5).toInt();
            setPumpFlow(v);
            Serial.printf("CTL-> PUMP=%d%%\n", v);
          } else if (c.startsWith("HEATER=")) {
            bool on = (c.substring(7) == "1");
            setHeater(on);
            Serial.printf("CTL-> HEATER=%s\n", on ? "ON" : "OFF");
          } else if (c.startsWith("DAMPER=")) {
            int v = c.substring(7).toInt();
            setServoAngle(v);   // 风门舵机：0°=全干燥 45°=半储热 90°=全储热
            Serial.printf("CTL-> DAMPER=%d°\n", v);
          }
          // 远程操控后立即刷新触摸屏按钮/数值显示（加热/风机/水泵/风门状态同步）
          updateControlDisplay();
        } else if (c.startsWith("MOTOR=")) {
          // 电机（setDCMotor）：仅 COLLECT 手动模式可远程操控
          if (!(systemMode == SYSTEM_MODE_COLLECT && collectSubState == CS_READY && manualControlActive)) {
            Serial.println("CTL-> MOTOR BLOCKED: only in COLLECT manual mode");
          } else {
            bool on = (c.substring(6) == "1");
            setDCMotor(on);
            Serial.printf("CTL-> MOTOR=%s\n", on ? "ON" : "OFF");
            updateControlDisplay();
          }
        } else {
          Serial.printf("CTL-> unknown: %s\n", c.c_str());
        }
      }
      uartLineBuf = "";
    } else if (c == '\r') {
      // 忽略回车
    } else {
      if (uartLineBuf.length() < 63) uartLineBuf += c;
    }
  }

  // 每 2 秒无条件打印一次四个集热器光照（无论是否收到新数据）
  static unsigned long lastUartLog = 0;
  if (millis() - lastUartLog >= 2000) {
    lastUartLog = millis();
    Serial.printf("UART lux: %.0f;%.0f;%.0f;%.0f\n",
                  collectorLux[0], collectorLux[1],
                  collectorLux[2], collectorLux[3]);
  }
}

// ==================== 传感器读取 ====================

// 检测 DS18B20 在线并更新水温；返回是否在线（供 readSensors 及 COLLECT 非干燥阶段调用）
bool checkDs18b20() {
  ds18b20Checked = true;   // 标记已检测过（无论成败）
  bool ds_found = false;
  while (ds18b20.selectNext()) {
    water_temperature = ds18b20.getTempC();
    ds_found = true;
  }
  if (ds_found) {
    ds18b20Online = true;
    Serial.printf("DS18B20: %.1f°C\n", water_temperature);
  } else {
    ds18b20Online = false;
    static int ds_err_count = 0;
    if (++ds_err_count == 1 || ds_err_count % 10 == 0) {
      Serial.printf("DS18B20: 未检测到设备（第%d次），请检查 GPIO18 接线和上拉电阻\n", ds_err_count);
    }
  }
  return ds_found;
}

void readSensors() {
  float newTemp = dht.readTemperature();
  float newHum = dht.readHumidity();

  if (isnan(newTemp) || isnan(newHum)) {
    sensorError = true;
    tft.fillRect(15, 95, 210, 50, PANEL_COLOR);
    tft.setTextColor(DANGER_COLOR, PANEL_COLOR);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("SENSOR ERROR!", 30, 105);
    return;
  }

  sensorError = false;
  temperature = newTemp;
  humidity = newHum;

  // 用 DHT11 数据更新虚拟传感器的环境温湿度
  ambient_temperature = newTemp;
  ambient_humidity = newHum;

  // DS18B20 水温读取
  checkDs18b20();

}


// ==================== 状态估计（虚拟传感器） ====================

/**
 * 基于上一次动作和物理规律，估计无法直接测量的状态量
 * 7 维观测: [PCM温度, 环境温度, 环境湿度, 进口风温, 出口风温, 干燥室湿度, 水箱温度]
 */
void updateStateEstimate() {
  if (sensorError) return;

  // 注意：COLLECT 干燥阶段（CS_AUTO_DRY）的 PCM 温度/进口风温
  // 已由光驱动态模型 updateDryModel() 接管，此处跳过稳态估算避免覆盖
  if (!(systemMode == SYSTEM_MODE_COLLECT && collectSubState == CS_AUTO_DRY)) {
    // PCM 温度估计：受太阳能加热、风机对流散热、加热片补热影响
    // 简化模型：PCM ≈ 环境 + 太阳辐射贡献 + 加热片贡献 - 风机对流散热 - 自然散热
    // 太阳辐射贡献：光照越强吸热越快（100lux≈7°C, 800lux≈14°C 饱和）
    float solar_heat = constrain(lightIntensity / 100.0f, 0.0f, 10.0f) * 0.35f;
    float heater_heat = last_heater ? 8.0 : 0.0;  // 加热片开启时持续补热（简化）
    float fan_loss = last_fan * 6.0;              // 风机对流散热（带走过剩热量）
    float heat_loss = (pcm_temperature - ambient_temperature) * 0.01; // 自然散热
    pcm_temperature = ambient_temperature + solar_heat + heater_heat - fan_loss - heat_loss;
    pcm_temperature = constrain(pcm_temperature, 0, 110);

    // 进口风温 ≈ 环境 + PCM 贡献
    inlet_air_temp = ambient_temperature + (pcm_temperature - ambient_temperature) * 0.6 * last_fan;
    inlet_air_temp = constrain(inlet_air_temp, 0, 100);
  }

  // 追踪温升速率（用于智能分热决策）
  temp_rise_rate = inlet_air_temp - prev_inlet_temp;
  prev_inlet_temp = inlet_air_temp;

  // 干燥室湿度：直接使用 DHT11 传感器真实读数
  dryer_humidity = humidity;
  // 除湿效果（用于出口风温估算）
  float dehumidify_effect = last_fan * (1.0 - last_damper) * 0.8;

  // 出口风温
  float cooling = dehumidify_effect * 0.5;
  outlet_air_temp = inlet_air_temp - cooling;
  outlet_air_temp = constrain(outlet_air_temp, ambient_temperature, 110);
}


// ==================== PCM 焓值法（吸热放热平衡模型） ====================
// 与 Python 仿真 pcm_balance_model.py 的焓值计算完全一致
float pcmTempToEnthalpy(float T) {
  float H_solid = PCM_MASS_KG * PCM_CP_SOLID * (PCM_T_SOLIDUS - PCM_T_REF);
  float H_liquid = H_solid + PCM_MASS_KG * PCM_LATENT_HEAT;
  if (T <= PCM_T_SOLIDUS) {
    return PCM_MASS_KG * PCM_CP_SOLID * (T - PCM_T_REF);
  } else if (T >= PCM_T_LIQUIDUS) {
    return H_liquid + PCM_MASS_KG * PCM_CP_LIQUID * (T - PCM_T_LIQUIDUS);
  } else {
    float f = (T - PCM_T_SOLIDUS) / (PCM_T_LIQUIDUS - PCM_T_SOLIDUS);
    return H_solid + PCM_MASS_KG * PCM_LATENT_HEAT * f +
           PCM_MASS_KG * PCM_CP_SOLID * (T - PCM_T_SOLIDUS) * (1.0f - f) * 0.3f;
  }
}

// ==================== 光驱集热统一周期动态模型 ====================
// 与 Python 仿真 pcm_light_fan_controller.py 完全一致
// 物理结构: 4 个集热器, 3 吸 1 放同时进行、同时结束，不分先后; 切换 = 转盘换位。
// 统一换位周期: 光照 → 净吸热功率 → 估算周期(10~15s) → 倒计时 → 换位。
// 风机 = 放热匹配功率(单倍)，使放热时间与光照决定的换位周期一致。

// 更新每个集热器的有效光照（直接用其内置光照）
// 物理模型：4 台集热器 3吸1放。放热台按 1→2→3→4→1 固定顺序轮换。
// 每台集热器有效光照 = 其内置光照（离线时用环境光照兜底）
void updateCollectorLight() {
  // 刷新每台集热器有效光照（直接用内置光照，离线兜底）
  for (int i = 0; i < 4; i++) {
    if (collectorLuxValid[i]) {
      // 节点在线：BH1750 环境光照占 80% + 节点光照占 20%
      collectorEffectiveLux[i] = lightIntensity * 0.8f + collectorLux[i] * 0.2f;
    } else {
      collectorEffectiveLux[i] = lightIntensity;   // 集热器数据缺失：用环境光照兜底
    }
  }
}

// 吸热侧有效光照：3 台吸热集热器（除放热那台）中的最高内置光照
// 用于驱动太阳能功率/换位周期（放热中的那台不贡献吸热）
float absorbingLight() {
  float best = 0.0f;
  for (int i = 0; i < 4; i++) {
    if (i == dischargingCollector) continue;      // 跳过放热集热器
    if (collectorEffectiveLux[i] > best) best = collectorEffectiveLux[i];
  }
  return best;
}

// 太阳能功率 W（基于吸热侧有效光照，线性变化并封顶）
float solarPowerW() {
  return constrain(absorbingLight() * SOLAR_W_PER_LUX, 0.0f, SOLAR_MAX_W);
}

// ==================== AI 干燥建议生成（只读，不影响执行） ====================
// 基于天气数据(wx*) + AI 输出动作(last_fan/damper/pump/discharge/heater)生成建议码，
// 通过 UART 发回 weather_display 显示。
// 建议码: 0加载 1正常 2风机 3风门 4水泵 5加热 6排粮 7收粮 8高湿 9光照弱 10过热 11系统正常
int computeAdviceCode() {
  if (!wxValid) return 0;                       // 天气未到：加载中

  // 1) 天气类硬规则（最优先，安全）
  if (wxCode == 4 || wxCode == 5 || wxCode == 6) {   // 雨/雪/雷阵雨
    if (wxRainProb >= 40 || wxRainHour >= 0) return 7;  // 降雨预报提前收粮
    return 7;
  }
  if (wxHum >= 85) return 8;                     // 高湿环境抑制放热
  if (wxTemp >= 35) return 10;                   // 温度过高注意防暑

  // 2) 光照不足：吸热侧光照明显低于参考
  float ab = absorbingLight();
  if (collectRefLightIntensity > 100.0f && ab < collectRefLightIntensity * 0.4f) {
    return 9;                                    // 光照不足减缓干燥
  }

  // 3) AI 动作识别（有天气时结合）
  if (last_heater) return 5;                     // 加热开启补热干燥
  // 排粮建议：COLLECT 模式只在真正排粮（第3次干燥后排粮3s）时出现；
  // AUTO 模式按 AI discharge 动作判断；其余阶段不报排粮
  if (systemMode == SYSTEM_MODE_COLLECT) {
    if (collectDischarging) return 6;            // 排粮开启适时出料
  } else if (systemMode == SYSTEM_MODE_AUTO) {
    if (last_discharge > 0.3f) return 6;         // 排粮开启适时出料
  }
  if (last_fan >= 0.6f) return 2;                // 风机加大加速放热
  if (last_damper >= 0.6f) return 3;             // 风门开启辅助散热
  if (last_pump >= 0.3f) return 4;               // 水泵开启储热循环
  return 1;                                      // 光照充足正常干燥
}

// 发送建议码给 weather_display
void sendAdviceToDisplay() {
  int code = computeAdviceCode();
  Serial1.printf("ADV:%d\n", code);
  Serial.printf("[UART] adv-> %d (wxValid=%d)\n", code, wxValid);
}

// 统一周期动态模型（CS_AUTO_DRY 期间每 loop 调用一次）
void updateDryModel() {
  unsigned long now = millis();
  static unsigned long lastDryModelMs = 0;
  float dt = (now - lastDryModelMs) / 1000.0f;
  if (dt < 0.01f || dt > 1.0f) dt = 0.05f;   // 限制步长 10ms~1s
  lastDryModelMs = now;

  float p_solar = solarPowerW();
  float p_loss = LOSS_COEF_W_PER_C * max(pcm_temperature - ambient_temperature, 0.0f);

  // 下一次放热的集热器（当前在吸热，下一轮将放热）
  int nextDischarging = (dischargingCollector + 1) % 4;

  // 换位周期：该集热器吸满目标储热量(0.6kJ)所需时间
  // 吸热功率 = 该集热器内置光照 × 太阳能功率系数
  // 预计干燥时长：500~970 lux 线性映射到 35s~25s（光照越强周期越短，区间内变化明显）
  float luxC = constrain(collectorEffectiveLux[nextDischarging], 500.0f, 970.0f);
  absorbDurationSec = 35.0f - (luxC - 500.0f) / (970.0f - 500.0f) * 10.0f;

  // 干燥剩余时间（倒计时）：换位周期 − 已进行时间，实时倒数到 0
  float elapsedSec = (millis() - autoDryStartMs) / 1000.0f;
  absorbRemainSec = constrain(absorbDurationSec - elapsedSec, 0.0f, absorbDurationSec);

  // 风机功率：只受光照强度调控（下一次放热集热器的光照），60%~100%
  // 风机功率作为 AI 观测信号，AI 模型据此调控风门/水泵/加热片
  float nextLux = collectorEffectiveLux[nextDischarging];
  float lightRatio = constrain(nextLux / max(collectRefLightIntensity, 1.0f), 0.0f, 1.0f);
  collectorFan = constrain(0.6f + 0.4f * lightRatio, 0.6f, 1.0f);

  // 温度估算（动态积分）：供显示与 AI 观测（吸热与放热同时作用，取净热流）
  float p_conv = collectorFan * FAN_MAX_REMOVE_W;
  pcm_temperature += (p_solar - p_conv - p_loss) * dt / PCM_EFF_MASS_CP_J;
  pcm_temperature = constrain(pcm_temperature, 0.0f, 110.0f);
  pcmEnthalpy = pcmTempToEnthalpy(pcm_temperature);
  inlet_air_temp = constrain(ambient_temperature + T_IN_DELTA_C, 0, 100);
}

// 干燥剩余时间（秒）= 换位周期倒计时
float computeDryRemainingTime() {
  return absorbRemainSec;
}

// ==================== 自动干燥模型（AI）—— COLLECT 干燥阶段融合 ====================
// 本来的 CS_AUTO_DRY 自动干燥模型：RL 模型每 INFERENCE_INTERVAL_MS 推理一次，
// 控制风机/风门/水泵/加热片。光驱集热控制器在其后做统一修正(applyCollectorCycle)。
// AUTO 模式不走这里（保持原样）。
void runAIDryModel() {
#ifndef POLICY_MODEL_H
  // 模型未编译：加热片恒开保证基本干燥
  if (!heaterOn) setHeater(true);
#else
  if (rl_model_loaded && pipesInserted) {
    runAIInference();
  } else if (rl_model_loaded && !pipesInserted) {
    // 模型已加载但管道未插入：推理被阻塞，无 AI 控制加热片
    setHeater(false);
    static unsigned long lastBlockLog = 0;
    if (millis() - lastBlockLog >= 10000) {
      lastBlockLog = millis();
      Serial.println("AI Inference: BLOCKED - pipes not inserted!");
    }
  } else {
    static unsigned long lastDiag = 0;
    if (millis() - lastDiag >= 10000) {
      lastDiag = millis();
      Serial.printf("AUTO DRY - model FAILED to load! Arena=%d bytes (need >150KB for v34)\n",
                    TENSOR_ARENA_SIZE);
      Serial.println("  Check initRLModel() error messages above for details.");
    }
  }
#endif
}

// ==================== 光驱集热控制层：风机由光照调控，其余由 AI 反推 ====================
// 分工（仅 CS_AUTO_DRY 阶段）：
//   风机  = 光照决定的匹配功率(放热时间=换位周期)，AI 的风机输出被忽略
//   风门/水泵/加热片 = AI 模型以"风机功率信号"为条件反推输出（本层不覆盖）
//   排粮舵机 = 独立状态机 updateDischarge 处理（本层不管）
void applyCollectorCycle() {
  // 换位完成后强制重新下发（导轨换位期间元件被归零，需重新下发）
  bool force = collectorForceApply;
  collectorForceApply = false;

  // 排粮期间：风机降速保护（AI 的排粮覆盖已处理风门/水泵/加热片，本层只保证风机安全）
  if (discharge_state == DS_DISCHARGING) {
    int fanPct = (int)(collectorFan * 100.0f * 0.3f);
    // 用实际值 motorSpeed 检测：AI 推理可能刚改了 PWM，需立即纠正
    if (force || motorSpeed != fanPct) {
      setMotorSpeed(fanPct);
      updateControlDisplay();
    }
    last_fan = fanPct / 100.0f;
    return;
  }

  // 风机：光照决定的放热匹配功率（随光照实时调整，强光放热快、弱光放热慢）
  // 用实际值 motorSpeed 检测：AI 推理每 5s 会 setMotorSpeed(AI值)，必须纠正回光照值
  int fanPct = (int)(collectorFan * 100.0f);
  if (fanPct < 60) fanPct = 60;   // 干燥时风机最低功率 60%
  if (force || motorSpeed != fanPct) {
    setMotorSpeed(fanPct);
    updateControlDisplay();   // 立即刷新显示，避免屏幕显示 AI 的风机值
  }
  last_fan = fanPct / 100.0f;

  // 风门/水泵/加热片：由 AI 模型反推调控（runAIInference 已按 obs 含风机功率信号
  // 输出 damper/pump/heater 并下发），本层不固定覆盖，保留 AI 决策。
  // 注：CS_AUTO_DRY 与 AUTO 共用 runAIInference；obs[4]=风机功率信号仅 CS_AUTO_DRY 生效。
}




// ==================== 排粮状态机（独立于 AI 推理，每 loop 循环运行） ====================
// 流程: 湿度达标(<15%)且稳定 → 连续稳定 10s → 排粮(90°×10s) → 复位(0°) → 下一循环
// 由湿度唯一决定，不依赖 AI 的 discharge 信号

void updateDischarge() {
  bool grain_dry_done = (dryer_humidity < 15.0);
  unsigned long now_ms = millis();
  static DischargeState prev_state = DS_IDLE;

  switch (discharge_state) {

    case DS_IDLE:
      // 湿度达标 → 进入稳定计时（排粮刚完成后冷却 30s 防止立即重触发）
      if (grain_dry_done && (now_ms - discharge_timer >= 30000)) {
        discharge_state = DS_STABLE_WAIT;
        discharge_timer = now_ms;
        Serial.printf("DISCHARGE: 湿度达标(RH:%.1f%%<15%%)，等待稳定 10s...\n",
                      dryer_humidity);
      }
      break;

    case DS_STABLE_WAIT:
      // 湿度必须连续稳定在达标线以下满 10 秒，任何反弹都重置
      if (grain_dry_done) {
        unsigned long elapsed = now_ms - discharge_timer;
        if (elapsed >= 10000) {
          // 稳定满 10s → 排粮
          discharge_state = DS_DISCHARGING;
          discharge_timer = now_ms;
          Serial.printf("DISCHARGE: 开始排粮！(稳定%.1fs, 舵机=90°, 持续10s)\n",
                        elapsed / 1000.0);
        }
      } else {
        // 湿度反弹 → 重置计时
        discharge_state = DS_IDLE;
        Serial.printf("DISCHARGE: 湿度反弹(RH:%.1f%%)，重置等待\n",
                      dryer_humidity);
      }
      break;

    case DS_DISCHARGING:
      // 排粮中：舵机保持 90°，持续 10s 后复位
      if (now_ms - discharge_timer >= 10000) {
        discharge_state = DS_IDLE;
        discharge_timer = now_ms;  // 用于防止下一循环立即触发
        Serial.println("DISCHARGE: 排粮完成！复位 0°，进入下一循环");
      }
      break;
  }

  // === 仅在状态切换时控制排粮舵机（避免每 50ms 刷 servo.write） ===
  if (discharge_state != prev_state) {
    prev_state = discharge_state;

    if (discharge_state == DS_DISCHARGING) {
      // 进入排粮 → 舵机转到 90°
      outputServoOn = true;
      setOutputServoAngle(90);
    } else if (discharge_state == DS_IDLE) {
      // 回到 IDLE → 舵机复位 0°
      outputServoOn = false;
      setOutputServoAngle(0);
    }
    // DS_STABLE_WAIT 不操作舵机，保持上一次位置
  }
}


// ==================== AI 推理控制（模型已加载，每 60 秒推理一次） ====================

#ifdef POLICY_MODEL_H
void runAIInference() {
  if (sensorError) return;

  unsigned long now = millis();
  // last_rl_inference == 0 表示首次推理，立即执行
  if (last_rl_inference != 0 && (now - last_rl_inference < INFERENCE_INTERVAL_MS)) {
    // 未到推理间隔，保持上一次的动作
    return;
  }
  last_rl_inference = now;

  // 1) 构建 5 维观测向量（原始值）
  // v34.9: [PCM温度, 干燥仓温度(DHT11), 干燥仓湿度(DHT11), 水箱温度(DS18B20), 环境温度常量25]
  // CS_AUTO_DRY 阶段：obs[4] 替换为"风机功率信号"(光照决定)，让 AI 以风机功率
  //   反推水泵/加热片/风门；AUTO 模式保持环境温度常量（不影响训练分布）。
  // 注：DHT11 安装在干燥仓内部，其 temperature=干燥仓温度、humidity=干燥仓湿度
  float dryer_temp = (isnan(temperature) || temperature < 0.1f) ? 25.0f : temperature;
  float obs_raw[5] = {
    pcm_temperature,                          // obs[0] PCM温度（估算）
    dryer_temp,                               // obs[1] 干燥仓温度（DHT11 实测）
    dryer_humidity,                           // obs[2] 干燥仓湿度（DHT11 实测）
    isnan(water_temperature) ? 25.0f : water_temperature,  // obs[3] 水箱温度（DS18B20）
    (systemMode == SYSTEM_MODE_COLLECT && collectSubState == CS_AUTO_DRY)
        ? (collectorFan * 25.0f)              // obs[4] 风机功率信号（0~25, /40→0~0.625）
        : 25.0f,                              // obs[4] AUTO: 环境温度常量
  };

  // 2) 归一化到 [0, 1]（与训练时完全一致）
  // 缩放因子: [T_pcm, T_dryer, RH_dryer, T_water, T_amb]
  const float obs_scale[5] = {100.0f, 100.0f, 80.0f, 100.0f, 40.0f};
  for (int i = 0; i < 5; i++) {
    float val = obs_raw[i];
    if (val < 0.0f) val = 0.0f;
    if (val > obs_scale[i]) val = obs_scale[i];
    rl_input->data.f[i] = val / obs_scale[i];
  }

  // 3) 推理
  unsigned long t_start = micros();
  if (rl_interpreter->Invoke() != kTfLiteOk) {
    Serial.println("AI 推理失败!");
    return;
  }
  unsigned long t_end = micros();

  // 4) 读取输出: [fan, damper, pump, discharge, heater]  (范围 0~1)
  float fan       = rl_output->data.f[0];
  float damper    = rl_output->data.f[1];
  float pump      = rl_output->data.f[2];
  float discharge = rl_output->data.f[3];
  float heaterAct = rl_output->data.f[4];   // 加热片连续输出

  // 加热片开关：加滞回（hysteresis）防抖动
  //  - 当前关闭时：需 heaterAct >= 0.50 才开启（上阈值）
  //  - 当前开启时：需 heaterAct <  0.45 才关闭（下阈值）
  //  0.45~0.50 为死区，保持上次状态，避免传感器噪声导致加热片频繁启停
  //  v34.15: 上阈值从 0.55 降到 0.50，匹配模型低温开启信号（~0.53）
  bool heaterOn = (heaterAct >= 0.50f) || (last_heater && heaterAct >= 0.45f);

  // ==================== 5) 智能温度感知分热 ====================
  // 当进口风温持续升高时，主动增开阀门将多余热量分流到水箱储热
  // 防止干燥仓过热，同时提高系统整体能量利用效率
  if (inlet_air_temp > 30.0 && temp_rise_rate > 0.0) {
    // 温升越大，分热越多：每升高 1°C 增加 15% 阀门开度
    float divert_boost = temp_rise_rate * 0.15;
    damper = damper + divert_boost;
    if (damper > 1.0) damper = 1.0;
  }
  // 高温保护：进口风温超过 45°C 时，阀门至少开启 50% 分热水箱
  if (inlet_air_temp > 45.0) {
    float min_damper = (inlet_air_temp - 45.0) / 15.0 * 0.5 + 0.5;  // 45°C→0.5, 60°C→1.0
    if (damper < min_damper) damper = min_damper;
    if (damper > 1.0) damper = 1.0;
  }

  // ==================== 5b) 干燥阶段智能覆盖 ====================

  // 水箱已满热（≥85°C），无需继续分热，关小阀门让热量专注干燥
  if (water_temperature >= 85.0) {
    damper = damper * 0.3;  // 阀门缩减到原值的 30%，热量集中干燥
    if (damper < 0.0) damper = 0.0;
  }

  // 排粮期间强制覆盖：风机降速 + 热量全分向水箱
  if (discharge_state == DS_DISCHARGING) {
    fan = fan * 0.3;      // 风机降至 30%
    damper = 1.0;         // 阀门 90° 全开水箱
    pump = 0.0;           // 排粮时停水泵
  }

  // ==================== 5c) 水泵温度门控 ====================
  // 水箱温度未达标时，水泵无法有效补热，强制为 0
  // 仅在非排粮阶段有效（排粮阶段已在 5b 中强制 pump=0）
  if (water_temperature < WATER_TEMP_PUMP_THRESHOLD && pump > 0.0f &&
      (systemMode != SYSTEM_MODE_COLLECT || collectDryCycleCount < 2)) {
    static unsigned long last_pump_gate_log = 0;
    if (now - last_pump_gate_log >= 30000) {  // 每 30s 最多打印一次
      last_pump_gate_log = now;
      Serial.printf("PUMP GATED: Tank=%.1fC < %.1fC, AI wanted pump=%.0f%% → forced 0\n",
                    water_temperature, WATER_TEMP_PUMP_THRESHOLD, pump * 100.0f);
    }
    pump = 0.0f;
  }

  // 6) 映射到硬件执行器

  // 风机: 0~1 → 0~100%
  int fanPercent = (int)(fan * 100);
  setMotorSpeed(fanPercent);

  // 风门舵机 (damper): 0~1 → 0~90°
  //   0.0 → 0°  : 热量完全通向干燥仓
  //   0.5 → 45° : 一半热量分向水箱储热，一半分向干燥仓
  //   1.0 → 90° : 全部热量分向水箱储热
  //   AI 输出 + 温度感知叠加：温度升高时自动增大阀门储热比例
  int damperAngle = (int)(damper * 90);
  setServoAngle(damperAngle);

  // 水泵: AI 0~1 → 60~100%；仅 COLLECT 第二次及以后干燥周期开启（START 后首次干燥不开）
  int pumpPercent = 0;
  if ((systemMode != SYSTEM_MODE_COLLECT && water_temperature >= WATER_TEMP_PUMP_THRESHOLD) ||
      (systemMode == SYSTEM_MODE_COLLECT && collectDryCycleCount >= 2)) {
    pumpPercent = 60 + (int)(pump * 40);
    if (pumpPercent > 100) pumpPercent = 100;
  }
  setPumpFlow(pumpPercent);

  // 加热片: 离散开关 (>=0.5 开, <0.5 关)
  // 安全覆盖：PCM 温度过高或水箱高温时强制关闭加热片，防过热
  if (pcm_temperature >= HEATER_TEMP_CUTOFF_C) {
    if (heaterOn) {
      heaterOn = false;
      static unsigned long last_hot_log = 0;
      if (now - last_hot_log >= 30000) {
        last_hot_log = now;
        Serial.printf("HEATER GATED: PCM=%.1fC >= %.0fC, AI wanted ON → forced OFF\n",
                      pcm_temperature, HEATER_TEMP_CUTOFF_C);
      }
    }
  }
  setHeater(heaterOn);

  // 7) 排粮状态机已移至 updateDischarge()，每 loop 循环独立运行
  //    此处仅保留排粮期间的硬件覆盖（已在上面 5b 中处理）

  // 8) 记录本次动作（用于状态估计和温度趋势）
  last_fan = fan;
  last_damper = damper;
  last_pump = pump;
  last_discharge = discharge;
  last_heater = heaterOn;

  // 9) 日志输出
  const char* ds_state_str = "IDLE";
  if (discharge_state == DS_STABLE_WAIT) ds_state_str = "WAIT";
  else if (discharge_state == DS_DISCHARGING) ds_state_str = "OUT!";
  Serial.printf("AUTO | T_in:%.1fC(△%+.1f) RH_dry:%.1f%% PCM:%.1fC Tank:%.1fC | "
                "Fan:%.0f%% Damper:%.0f° Pump:%.0f%% Heater:%s Disch:%s | %lu us\n",
                inlet_air_temp, temp_rise_rate, dryer_humidity,
                pcm_temperature, water_temperature,
                fan * 100, damper * 90, pump * 100,
                heaterOn ? "ON" : "OFF",
                ds_state_str,
                t_end - t_start);

  // 10) 更新屏幕显示
  updateControlDisplay();
}
#endif


// ==================== 手动模式应用 ====================

void applyManualControls() {
  static int lastMotorSpeed = -1;
  static int lastServoAngle = -1;
  static int lastPumpFlow = -1;

  if (motorSpeed != lastMotorSpeed) {
    setMotorSpeed(motorSpeed);
    lastMotorSpeed = motorSpeed;
  }
  if (servoAngle != lastServoAngle) {
    setServoAngle(servoAngle);
    lastServoAngle = servoAngle;
  }
  if (pumpFlow != lastPumpFlow) {
    setPumpFlow(pumpFlow);
    lastPumpFlow = pumpFlow;
  }
}

/* ============ COLLECT 操控函数（触摸屏与平板远程共用） ============ */

// START：就绪 → 固定等待 10 秒后启动直流电机（原触摸屏 case1 的 START 分支）
void collectStart() {
  if (systemMode != SYSTEM_MODE_COLLECT) return;
  if (collectSubState == CS_READY && manualControlActive) {
    Serial.println("COLLECT: START BLOCKED - in manual override mode");
    return;
  }
  if (collectSubState == CS_READY) {
    collectProcessActive = true;
    collectDryCycleCount = 0;      // 每次 START 重新计数，避免上次残留导致第一次干燥就排粮
    collectDischarging = false;
    collectSubState = CS_WAIT_LIGHT;
    waitLightStartMs = millis();   // 记录计时起点
    manualControlActive = false;
    manualPipeInserted = false;
    manualPipeMoving = false;
    manualMotorRunning = false;
    manualInletDone = false; manualOutletDone = false;
    manualInletReversing = false; manualOutletReversing = false;
    manualInletLastSW = false; manualOutletLastSW = false;
    inletRailServo.writeMicroseconds(RAIL_STOP_US);
    outletRailServo.writeMicroseconds(RAIL_STOP_US);
    setDCMotor(false);
    Serial.println("COLLECT: START - Fixed 10s delay before motor ON");
  }
}

// STOP：停止流程并回原点（原触摸屏 case1 的 STOP 分支）
void collectStop() {
  if (systemMode != SYSTEM_MODE_COLLECT) return;
  if (collectProcessActive) {
    collectProcessActive = false;
    setDCMotor(false);
    setMotorSpeed(0);
    setPumpFlow(0);
    setServoAngle(0);
    setHeater(false);
    setOutputServoAngle(0);   // STOP 时排粮舵机立即复位 0°（关闭排粮口）
    collectDischarging = false;   // 排粮状态复位
    inletRailServo.writeMicroseconds(RAIL_STOP_US);
    outletRailServo.writeMicroseconds(RAIL_STOP_US);
    collectSubState = CS_HOMING;
    railHomingDone = false;
    pipesInserted = false;
    inletRailServo.writeMicroseconds(RAIL_STOP_US);
    outletRailServo.writeMicroseconds(RAIL_STOP_US);
    inletDone = false; outletDone = false;
    inletReversing = false; outletReversing = false;
    inletLastSW = false; outletLastSW = false;
    collisionWasReleased = false;
    manualControlActive = false;
    manualPipeInserted = false;
    manualPipeMoving = false;
    manualMotorRunning = false;
    manualInletDone = false; manualOutletDone = false;
    manualInletReversing = false; manualOutletReversing = false;
    manualInletLastSW = false; manualOutletLastSW = false;
    Serial.println("COLLECT: STOP - Return to initial");
  }
}

// 进入/退出手动模式（原触摸屏 case2）
void collectManualToggle() {
  if (systemMode != SYSTEM_MODE_COLLECT || collectSubState != CS_READY) return;
  if (manualExitHoming) {
    Serial.println("COLLECT: Manual entry BLOCKED - homing in progress");
    return;
  }
  if (!manualControlActive) {
    // 进入手动模式
    manualControlActive = true;
    manualPipeInserted = false;
    manualPipeMoving = false;
    manualMotorRunning = false;
    manualExitHoming = false;
    manualInletDone = false; manualOutletDone = false;
    manualInletReversing = false; manualOutletReversing = false;
    manualInletLastSW = false; manualOutletLastSW = false;
    inletRailServo.writeMicroseconds(RAIL_STOP_US);
    outletRailServo.writeMicroseconds(RAIL_STOP_US);
    setDCMotor(false);
    Serial.println("COLLECT: Manual override ON");
  } else {
    // 退出手动模式：先停止电机，再执行"离开回原点"程序
    setDCMotor(false);
    manualMotorRunning = false;
    manualExitHoming = true;
    manualPipeInserted = false;
    manualPipeMoving = true;
    manualInletDone = false; manualOutletDone = false;
    manualInletReversing = false; manualOutletReversing = false;
    manualInletLastSW = false; manualOutletLastSW = false;
    if (!inletRailServo.attached()) inletRailServo.attach(SERVO_INLET_PIN);
    if (!outletRailServo.attached()) outletRailServo.attach(SERVO_OUTLET_PIN);
    inletRailServo.writeMicroseconds(RAIL_CCW_US);
    outletRailServo.writeMicroseconds(RAIL_CW_US);
    Serial.println("COLLECT: Manual EXIT - homing to initial state...");
  }
  drawCollectManualPanel();
  drawCollectPanel();
}

// 手动插入集热器（原触摸屏 case3）
void collectManualInsert() {
  if (systemMode != SYSTEM_MODE_COLLECT || collectSubState != CS_READY ||
      !manualControlActive || manualExitHoming) return;
  if (manualMotorRunning) {
    setDCMotor(false);
    manualMotorRunning = false;
    Serial.println("MANUAL PIPE: Motor stopped (mutual exclusion)");
  }
  manualPipeInserted = true;
  manualPipeMoving = true;
  manualInletDone = false; manualOutletDone = false;
  manualInletReversing = false; manualOutletReversing = false;
  manualInletLastSW = false; manualOutletLastSW = false;
  if (!inletRailServo.attached()) inletRailServo.attach(SERVO_INLET_PIN);
  if (!outletRailServo.attached()) outletRailServo.attach(SERVO_OUTLET_PIN);
  inletRailServo.writeMicroseconds(RAIL_CW_US);
  outletRailServo.writeMicroseconds(RAIL_CCW_US);
  Serial.println("MANUAL PIPE: Inserting (IN) - reverse-release homing");
  drawCollectManualPanel();
}

// 手动离开回原点（原触摸屏 case4）
void collectManualLeave() {
  if (systemMode != SYSTEM_MODE_COLLECT || collectSubState != CS_READY ||
      !manualControlActive || manualExitHoming) return;
  if (manualMotorRunning) {
    setDCMotor(false);
    manualMotorRunning = false;
    Serial.println("MANUAL PIPE: Motor stopped (mutual exclusion)");
  }
  manualPipeInserted = false;
  manualPipeMoving = true;
  manualInletDone = false; manualOutletDone = false;
  manualInletReversing = false; manualOutletReversing = false;
  manualInletLastSW = false; manualOutletLastSW = false;
  if (!inletRailServo.attached()) inletRailServo.attach(SERVO_INLET_PIN);
  if (!outletRailServo.attached()) outletRailServo.attach(SERVO_OUTLET_PIN);
  inletRailServo.writeMicroseconds(RAIL_CCW_US);
  outletRailServo.writeMicroseconds(RAIL_CW_US);
  Serial.println("MANUAL PIPE: Leaving (OUT) - reverse-release homing");
  drawCollectManualPanel();
}

// 手动直流电机启停（原触摸屏 case5）
void collectManualMotor() {
  if (systemMode != SYSTEM_MODE_COLLECT || collectSubState != CS_READY ||
      !manualControlActive || manualExitHoming) return;
  if (!manualMotorRunning) {
    if (!railHomingDone) {
      Serial.println("MANUAL MOTOR: BLOCKED - rails not homed!");
      return;
    }
    if (manualPipeInserted) {
      Serial.println("MANUAL MOTOR: BLOCKED - pipes inserted! Leave to origin first.");
      return;
    }
    if (manualPipeMoving) {
      Serial.println("MANUAL MOTOR: BLOCKED - pipes moving! Wait for servo stop.");
      return;
    }
    collisionWasReleased = false;
    collisionDetected = collisionActive();
    setDCMotor(true);
    manualMotorRunning = true;
    Serial.println("MANUAL MOTOR: RUN");
  } else {
    setDCMotor(false);
    manualMotorRunning = false;
    Serial.println("MANUAL MOTOR: STOP");
  }
  drawCollectManualPanel();
}

// 切换系统模式（触摸屏顺序切换 & 平板远程指定模式共用；含电机/舵机运行保护）
void switchMode(int newMode) {
  if (newMode < 0 || newMode >= MODE_COUNT) return;
  if (newMode == systemMode) return;            // 已在该模式，忽略
  // 安全保护（v34.17）：直流电机或气管舵机工作时禁止切换模式
  if (dcMotorOn) {
    Serial.println("Mode switch BLOCKED - DC motor running!");
    return;
  }
  if (manualPipeMoving || manualExitHoming) {
    Serial.println("Mode switch BLOCKED - pipes moving!");
    return;
  }
  if (systemMode == SYSTEM_MODE_COLLECT &&
      (collectSubState == CS_HOMING ||
       collectSubState == CS_PIPES_INSERT ||
       collectSubState == CS_DRY_HOMING ||
       collectSubState == CS_DRY_INSERT)) {
    Serial.println("Mode switch BLOCKED - rails operating!");
    return;
  }

  systemMode = newMode;
  buttons[0].label = MODE_LABELS[systemMode];
  buttons[0].color = MODE_COLORS[systemMode];
  buttons[0].state = (systemMode != SYSTEM_MODE_COLLECT);

  // 无论切到什么模式，先复位 COLLECT 手动操控状态
  manualControlActive = false;
  manualPipeInserted = false;
  manualPipeMoving = false;
  manualMotorRunning = false;
  manualInletDone = false; manualOutletDone = false;
  manualInletReversing = false; manualOutletReversing = false;
  manualInletLastSW = false; manualOutletLastSW = false;

  // 切换到非手动模式时，复位所有执行器到安全状态
  if (systemMode != SYSTEM_MODE_MANUAL) {
    motorSpeed = 0;
    pumpFlow = 0;
    servoAngle = 0;
    outputServoOn = false;
    outputServoAngle = 0;
    heaterOn = false;
    collectProcessActive = false;
    collectDryCycleCount = 0;      // 重置 COLLECT 每3周期排粮状态
    collectDischarging = false;
    setMotorSpeed(0);
    setPumpFlow(0);
    setServoAngle(0);        // 风门 → 干燥模式
    setOutputServoAngle(0);
    setHeater(false);
    setDCMotor(false);
  }

  // 进入手动模式时，加热片默认开启，其余执行器关闭
  if (systemMode == SYSTEM_MODE_MANUAL) {
    motorSpeed = 0;
    pumpFlow = 0;
    servoAngle = 0;
    outputServoOn = false;
    outputServoAngle = 0;
    heaterOn = false;
    collectProcessActive = false;
    setMotorSpeed(0);
    setPumpFlow(0);
    setServoAngle(0);
    setOutputServoAngle(0);
    setHeater(true);
    setDCMotor(false);
  }

  // 进入 COLLECT 模式：导轨回原点 + 等用户点"START"按钮
  if (systemMode == SYSTEM_MODE_COLLECT) {
    collectProcessActive = false;
    collisionDetected = collisionActive();
    collisionWasReleased = false;
    setDCMotor(false);
    collectSubState = CS_HOMING;          // 开始回原点流程
    railHomingDone = false;
    pipesInserted = false;                // 导轨未到终点
    inletRailServo.writeMicroseconds(RAIL_STOP_US);
    outletRailServo.writeMicroseconds(RAIL_STOP_US);
    inletDone = false; outletDone = false;
    inletReversing = false; outletReversing = false;
    inletLastSW = false; outletLastSW = false;
    manualControlActive = false;
    manualPipeInserted = false;
    manualPipeMoving = false;
    manualMotorRunning = false;
    manualInletDone = false; manualOutletDone = false;
    manualInletReversing = false; manualOutletReversing = false;
    manualInletLastSW = false; manualOutletLastSW = false;
    Serial.println("COLLECT mode: Homing rails...");
  }

  drawFullScreen();
  Serial.printf("Mode: %s\n", MODE_LABELS[systemMode]);
}

// ==================== 触摸处理 ====================

void handleTouch() {
  uint16_t rawX, rawY;
  bool pressed = tft.getTouch(&rawX, &rawY);

  if (!pressed) {
    wasPressed = false;
    return;
  }

  int x = map(rawY, TOUCH_MIN_X, TOUCH_MAX_X, 0, tft.width());
  int y = map(rawX, TOUCH_MIN_Y, TOUCH_MAX_Y, 0, tft.height());
  x = constrain(x, 0, tft.width() - 1);
  y = constrain(y, 0, tft.height() - 1);

  if (millis() - lastTouchTime < 200) return;
  if (wasPressed && abs(x - lastX) < 15 && abs(y - lastY) < 15) return;

  lastTouchTime = millis();
  lastX = x;
  lastY = y;
  if (wasPressed) return;
  wasPressed = true;

  for (int i = 0; i < 10; i++) {
    if (hitButton(buttons[i], x, y)) {
      executeButton(i);
      return;
    }
  }
}

void executeButton(int index) {
  switch (index) {
    case 0: // 模式切换（COLLECT → AUTO → MANUAL → COLLECT...）
      switchMode((systemMode + 1) % MODE_COUNT);
      break;

    case 1: // COLLECT: 一键开始/停止  |  MANUAL: 电机+
      if (systemMode == SYSTEM_MODE_COLLECT) {
        if (collectSubState == CS_READY) {
          collectStart();
        } else if (collectProcessActive) {
          collectStop();
        }
        drawFullScreen();
      } else if (systemMode == SYSTEM_MODE_MANUAL && motorSpeed < 100) {
        motorSpeed += 10;
        setMotorSpeed(motorSpeed);
        updateControlDisplay();
      }
      break;

    case 2: // COLLECT 手动操控: 进入/退出手动模式  |  MANUAL: 电机 -
      if (systemMode == SYSTEM_MODE_COLLECT && collectSubState == CS_READY) {
        collectManualToggle();
        return;
      }
      if (systemMode == SYSTEM_MODE_MANUAL && motorSpeed > 0) {
        motorSpeed -= 10;
        setMotorSpeed(motorSpeed);
        updateControlDisplay();
      }
      break;

    case 3: // COLLECT 手动操控: 进/出气舵机 插入集热器  |  MANUAL: 水泵 +
      if (systemMode == SYSTEM_MODE_COLLECT && collectSubState == CS_READY && manualControlActive && !manualExitHoming) {
        collectManualInsert();
        return;
      }
      if (systemMode == SYSTEM_MODE_MANUAL && pumpFlow < 100) {
        pumpFlow += 10;
        setPumpFlow(pumpFlow);
        updateControlDisplay();
      }
      break;

    case 4: // COLLECT 手动操控: 进/出气舵机 离开回原点  |  MANUAL: 水泵 -
      if (systemMode == SYSTEM_MODE_COLLECT && collectSubState == CS_READY && manualControlActive && !manualExitHoming) {
        collectManualLeave();
        return;
      }
      if (systemMode == SYSTEM_MODE_MANUAL && pumpFlow > 0) {
        pumpFlow -= 10;
        setPumpFlow(pumpFlow);
        updateControlDisplay();
      }
      break;

    case 5: // COLLECT 手动操控: 直流电机 启停  |  MANUAL: OUTPUT - 排粮舵机
      if (systemMode == SYSTEM_MODE_COLLECT && collectSubState == CS_READY && manualControlActive && !manualExitHoming) {
        collectManualMotor();
        return;
      }
      if (systemMode == SYSTEM_MODE_MANUAL) {
        outputServoOn = !outputServoOn;
        outputServoAngle = outputServoOn ? 90 : 0;
        setOutputServoAngle(outputServoAngle);
        buttons[5].state = outputServoOn;
        drawButton(buttons[5]);
        updateControlDisplay();
      }
      break;

    case 6: // DRY - 风门舵机 0°
      if (systemMode == SYSTEM_MODE_MANUAL) {
        setServoAngle(0);
        updateControlDisplay();
      }
      break;

    case 7: // DRY+SAVE - 风门舵机 45°
      if (systemMode == SYSTEM_MODE_MANUAL) {
        setServoAngle(45);
        updateControlDisplay();
      }
      break;

    case 8: // SAVE - 风门舵机 90°
      if (systemMode == SYSTEM_MODE_MANUAL) {
        setServoAngle(90);
        updateControlDisplay();
      }
      break;

    case 9: // HEATER 加热片开关（手动模式，MOS高电平加热）
      if (systemMode == SYSTEM_MODE_MANUAL) {
        heaterOn = !heaterOn;
        setHeater(heaterOn);
        buttons[9].state = heaterOn;
        buttons[9].color = heaterOn ? DANGER_COLOR : ACCENT_COLOR;
        drawButton(buttons[9]);
      }
      break;
  }
}


// ==================== 显示函数 ====================

void drawFullScreen() {
  tft.fillScreen(BG_COLOR);
  drawHeader();
  drawModePanel();
  if (systemMode == SYSTEM_MODE_COLLECT && collectSubState != CS_AUTO_DRY) {
    // 清除传感器/控制面板区域残留（CS_AUTO_DRY → 其他阶段切换时）
    tft.fillRect(10, 75, 220, 155, BG_COLOR);
    tft.fillCircle(130, 52, 5, collisionDetected ? DANGER_COLOR : SUCCESS_COLOR);
    // 除 CS_AUTO_DRY 外的所有 COLLECT 阶段（含 CS_MOTOR_GO / CS_PIPES_INSERT 及其循环别名）：仅显示光照强度传感器
    drawLightSensorPanel();
    drawCollectPanel();
    // 仅 CS_READY 阶段在光照面板下方显示手动操控界面
    if (collectSubState == CS_READY) {
      drawCollectManualPanel();
    }
    return;
  }
  // 非 COLLECT 模式（AUTO/MANUAL）始终显示传感器+控件面板；
  // COLLECT 模式仅 CS_AUTO_DRY 阶段显示
  if (systemMode != SYSTEM_MODE_COLLECT || collectSubState >= CS_AUTO_DRY) {
    drawSensorPanel();
    drawControlPanel();
  }
  drawButtonPanel();
}

void drawHeader() {
  tft.fillRect(0, 0, 240, 30, ACCENT_COLOR);
  tft.setTextColor(TEXT_COLOR, ACCENT_COLOR);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("DRYER CONTROL", 120, 15);
}

void drawModePanel() {
  tft.fillRoundRect(10, 35, 220, 35, 5, PANEL_COLOR);
  tft.drawRoundRect(10, 35, 220, 35, 5, 0x4A69);

  tft.setTextColor(0xBDF7, PANEL_COLOR);
  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("MODE", 18, 40);

  buttons[0].x = 150; buttons[0].y = 40;
  buttons[0].w = 70; buttons[0].h = 25;
  buttons[0].label = MODE_LABELS[systemMode];
  buttons[0].color = MODE_COLORS[systemMode];
  buttons[0].state = (systemMode == SYSTEM_MODE_MANUAL);
  drawButton(buttons[0]);

  // COLLECT 排粮提示：第 3 个干燥周期开始出现，排粮复位后消失（琥珀黄）
  // 显示在 COLLECT 状态按钮左侧，垂直居中于 mode 面板（面板 y35~70，中心 y52）
  // "DISCHARGE!" 10字符×6px=60px → x55~115，与按钮(x150)留白充足
  if (systemMode == SYSTEM_MODE_COLLECT && collectDryCycleCount >= 2) {
    tft.setTextColor(WARN_COLOR, PANEL_COLOR);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("DISCHARGE!", 55, 52);
  }
}

void drawSensorPanel() {
  tft.fillRoundRect(10, 75, 220, 75, 5, PANEL_COLOR);
  tft.drawRoundRect(10, 75, 220, 75, 5, 0x4A69);

  tft.setTextColor(0xBDF7, PANEL_COLOR);
  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("SENSORS", 18, 80);

  updateSensorDisplay();
}

void updateSensorDisplay() {
  tft.fillRect(15, 95, 210, 50, PANEL_COLOR);

  if (sensorError) {
    tft.setTextColor(DANGER_COLOR, PANEL_COLOR);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("SENSOR ERROR!", 30, 105);
    return;
  }

  tft.setTextColor(TEXT_COLOR, PANEL_COLOR);
  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);

  char buf[30];
  sprintf(buf, "T:%.1fC", temperature + dryerTempSimAccum);   // 干燥时叠加模拟示数偏置
  tft.drawString(buf, 18, 96);
  sprintf(buf, "H:%.1f%%", humidity);
  tft.drawString(buf, 130, 96);

  // 第二行：DS18B20 水温 + 光照（CS_AUTO_DRY 时显示）
  tft.setTextSize(2);
  tft.setTextColor(0xBDF7, PANEL_COLOR);
  tft.drawString("W:", 18, 118);

  tft.setTextColor(TEXT_COLOR, PANEL_COLOR);
  sprintf(buf, "%.1fC", water_temperature);
  tft.drawString(buf, 44, 118);

  if (collectSubState >= CS_AUTO_DRY) {
    tft.setTextColor(0xBDF7, PANEL_COLOR);
    tft.drawString("L:", 120, 118);
    tft.setTextColor(TEXT_COLOR, PANEL_COLOR);
    sprintf(buf, "%.0f", lightIntensity);
    tft.drawString(buf, 140, 118);
  }
}


void drawControlPanel() {
  tft.fillRoundRect(10, 155, 220, 75, 5, PANEL_COLOR);
  tft.drawRoundRect(10, 155, 220, 75, 5, 0x4A69);

  tft.setTextColor(0xBDF7, PANEL_COLOR);
  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("CONTROLS", 18, 160);

  // 加热片按钮在 updateControlDisplay() 中统一绘制（与 SERVO 水平）
  updateControlDisplay();
}

void updateControlDisplay() {
  char buf[16];
  tft.fillRect(15, 173, 210, 52, PANEL_COLOR);
  tft.setTextColor(TEXT_COLOR, PANEL_COLOR);
  tft.setTextSize(1);

  // 第一行：MOTOR | SERVO | HEAT按钮（水平协调）
  tft.setCursor(20, 177);
  tft.print("MOTOR:");
  tft.setTextSize(2);
  tft.setCursor(60, 173);
  tft.printf("%d%%", motorSpeed);

  // SERVO 左移（原155 → 147）
  tft.setTextSize(1);
  tft.setCursor(108, 177);
  tft.print("SERVO:");
  tft.setTextSize(2);
  tft.setCursor(146, 173);
  tft.printf("%d", servoAngle);

  // 加热片按钮：横向对称线与 SERVO 值对齐（中心 y181）
  // SERVO 值在 y173（textSize2 高16，中心 y181），按钮高24 → y169 中心对齐
  buttons[9].x = 178; buttons[9].y = 169;
  buttons[9].w = 42;  buttons[9].h = 24;
  buttons[9].label = "HEAT";
  buttons[9].color = heaterOn ? DANGER_COLOR : ACCENT_COLOR;
  buttons[9].state = heaterOn;
  drawButton(buttons[9]);

  // CS_AUTO_DRY 阶段：加热片按钮下方显示干燥剩余时间（光驱吸热/放热模型计算）
  updateRemainDisplay();

  // 第二行：PUMP | OUTPUT（OUTPUT 左移）
  // 重置文字颜色（drawButton 可能改了 setTextColor，防止 PUMP/OUTPUT 变红）
  tft.setTextColor(TEXT_COLOR, PANEL_COLOR);
  tft.setTextSize(1);
  tft.setCursor(20, 205);
  tft.print("PUMP:");
  tft.setTextSize(2);
  tft.setCursor(60, 201);
  tft.printf("%d%%", pumpFlow);

  tft.setTextSize(1);
  tft.setCursor(108, 205);
  tft.print("OUT:");        // 标签缩为 OUT，避免与示数重叠
  tft.setTextSize(2);
  tft.setCursor(146, 201);  // 与上方 SERVO 示数(x146)垂直对齐
  tft.printf("%d", outputServoAngle);
}

// 干燥剩余时间显示（CS_AUTO_DRY 阶段，加热片按钮下方 y201）
// 独立于整屏重绘/AI 推理，由 CS_AUTO_DRY 循环每秒刷新一次
void updateRemainDisplay() {
  if (!(systemMode == SYSTEM_MODE_COLLECT && collectSubState == CS_AUTO_DRY)) return;

  char buf[8];
  float remainSec = computeDryRemainingTime();
  int remainInt;
  if (remainSec > 999.0f) remainInt = 999;
  else remainInt = (int)(remainSec + 0.5f);   // 四舍五入

  // 先清除数字区域，避免两位→一位时残留旧字符（如 "10s"→"9ss"）
  // 数字区域 x190~238（最多 "99s" 3字符×12px），y201 高 16（textSize2）
  tft.fillRect(190, 201, 48, 16, PANEL_COLOR);

  // 显示在加热片按钮下方，与 OUTPUT 同一水平线（y201）
  // "T:" textSize 2，与秒数同字号；x170 起避免碰右边框
  tft.setTextColor(TEXT_COLOR, PANEL_COLOR);
  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("T:", 170, 201);
  sprintf(buf, "%ds", remainInt);
  tft.drawString(buf, 190, 201);
}

void drawLightSensorPanel() {
  tft.fillRoundRect(10, 75, 220, 60, 5, PANEL_COLOR);
  tft.drawRoundRect(10, 75, 220, 60, 5, 0x4A69);

  tft.setTextColor(0xBDF7, PANEL_COLOR);
  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("LIGHT SENSOR (GY-30)", 18, 80);

  updateLightSensorDisplay();
}

void updateLightSensorDisplay() {
  // 只重绘数值区域
  tft.fillRect(15, 95, 210, 35, PANEL_COLOR);

  if (!bh1750Ready) {
    tft.setTextColor(DANGER_COLOR, PANEL_COLOR);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("BH1750: NO SENSOR", 18, 105);
    return;
  }

  tft.setTextColor(TEXT_COLOR, PANEL_COLOR);
  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);

  char buf[40];
  sprintf(buf, "%.1f lux", lightIntensity);
  tft.drawString(buf, 18, 100);

  // CS_WAIT_LIGHT 阶段：状态指示灯位置让给"剩余干燥时间"示数
  if (systemMode == SYSTEM_MODE_COLLECT && collectSubState == CS_WAIT_LIGHT) {
    // 剩余时间 = 固定启动延迟 - 已等待时间
    long remainMs = COLLECT_START_DELAY_MS - (long)(millis() - waitLightStartMs);
    if (remainMs < 0) remainMs = 0;
    // "T:" 与剩余秒数同一水平线，左移一点（x165 起），字体与其他文字一致（非黄色）
    tft.setTextColor(TEXT_COLOR, PANEL_COLOR);
    tft.setTextSize(2);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("T:", 165, 100);
    sprintf(buf, "%lds", remainMs / 1000 + ((remainMs % 1000 > 0) ? 1 : 0));
    tft.drawString(buf, 190, 100);
  } else {
    // 其他阶段：根据光照强度显示状态颜色
    uint16_t luxColor;
    if (lightIntensity < 100) {
      luxColor = DANGER_COLOR;   // 很暗
    } else if (lightIntensity < 1000) {
      luxColor = WARN_COLOR;     // 一般
    } else {
      luxColor = SUCCESS_COLOR;  // 充足
    }
    tft.fillCircle(200, 110, 6, luxColor);
  }
}

// 设备断线故障检测：任一关键传感器异常即返回 true（不检测集热器光照节点）
bool systemFault() {
  if (sensorError) return true;                // DHT11 断线
  if (!bh1750Ready) return true;               // BH1750 未初始化/断线
  if (ds18b20Checked && !ds18b20Online) return true;   // DS18B20 已检测过且断线才算故障
  return false;
}

// 底部状态面板左上角：设备故障指示（OK/FAULT，小字号靠左，不挡状态文字和边框）
void drawFaultIndicator() {
  bool fault = systemFault();
  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);
  tft.fillRect(14, 237, 44, 9, PANEL_COLOR);    // 擦除旧内容（y=237 避开顶部边框线）
  if (fault) {
    tft.setTextColor(DANGER_COLOR, PANEL_COLOR);
    tft.drawString("FAULT", 14, 237);
  } else {
    tft.setTextColor(SUCCESS_COLOR, PANEL_COLOR);
    tft.drawString("OK", 14, 237);
  }
}

void drawCollectPanel() {
  // 底部面板
  tft.fillRoundRect(10, 235, 220, 80, 5, PANEL_COLOR);
  tft.drawRoundRect(10, 235, 220, 80, 5, 0x4A69);

  // 状态文字
  tft.setTextSize(2);
  const char* statusText;
  uint16_t statusColor;
  switch (collectSubState) {
    case CS_HOMING:      statusText = "Homing...";    statusColor = WARN_COLOR; break;
    case CS_READY:       statusText = "Ready";        statusColor = SUCCESS_COLOR; break;
    case CS_WAIT_LIGHT:  statusText = "Wait Light";   statusColor = WARN_COLOR; break;
    case CS_MOTOR_GO:    statusText = "Motor Running"; statusColor = AUTO_COLOR;   break;
    case CS_PIPES_INSERT:statusText = "Inserting...";  statusColor = AUTO_COLOR;   break;
    case CS_DRY_HOMING:  statusText = "Homing...";     statusColor = WARN_COLOR;   break;
    case CS_DRY_MOTOR:   statusText = "Motor Running"; statusColor = AUTO_COLOR;   break;
    case CS_DRY_INSERT:  statusText = "Inserting...";  statusColor = AUTO_COLOR;   break;
    case CS_AUTO_DRY:    statusText = "Auto Drying";  statusColor = AUTO_COLOR;   break;
    default:             statusText = "---";           statusColor = ACCENT_COLOR;  break;
  }
  tft.setTextColor(statusColor, PANEL_COLOR);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(statusText, 120, 250);

  // 一键开始 / 停止按钮
  if (collectSubState == CS_READY) {
    // 手动操控模式下或退出手动回原点中 START 禁用（回到初始状态前不能 start）
    if (manualControlActive || manualExitHoming) {
      buttons[1] = {65, 265, 110, 35, "LOCKED", DISABLED_COLOR, false};
    } else {
      buttons[1] = {65, 265, 110, 35, "START", SUCCESS_COLOR, false};
    }
  } else if (collectProcessActive) {
    buttons[1] = {65, 265, 110, 35, "STOP", DANGER_COLOR, true};
  } else if (collectSubState == CS_HOMING) {
    buttons[1] = {65, 265, 110, 35, "HOMING...", WARN_COLOR, false};
  } else {
    buttons[1] = {65, 265, 110, 35, "WAIT...", ACCENT_COLOR, false};
  }
  drawButton(buttons[1]);

  // 非 CS_READY 阶段清空手动操控按钮
  if (collectSubState != CS_READY) {
    for (int i = 2; i < 10; i++) {
      buttons[i] = {0, 0, 0, 0, "", ACCENT_COLOR, false};
    }
  }

  // 底部面板左上角：设备故障指示
  drawFaultIndicator();
}

// ============ COLLECT 手动操控界面（仅 CS_READY 阶段，光照面板下方） ============
void drawCollectManualPanel() {
  // 中间区域: 光照面板(y75-135) 下方, 底部面板(y235) 上方
  // 面板区域: y140 ~ y230
  // 手动操控面板：光照面板(y75-135)下方、底部面板(y235)上方
  // 面板区域: y140 ~ y232（底部留3px间隙，不遮挡底部面板）
  tft.fillRoundRect(10, 140, 220, 92, 5, PANEL_COLOR);
  tft.drawRoundRect(10, 140, 220, 92, 5, 0x4A69);

  tft.setTextColor(0xBDF7, PANEL_COLOR);
  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("MANUAL OVERRIDE", 18, 145);

  if (!manualControlActive) {
    // 未进入手动模式：只显示"操控"按钮
    // 若退出手动回原点中（manualExitHoming），MANUAL 按钮禁用
    if (manualExitHoming) {
      buttons[2] = {60, 170, 120, 40, "LOCKED", DISABLED_COLOR, false};
      drawButton(buttons[2]);
    } else {
      buttons[2] = {60, 170, 120, 40, "MANUAL", WARN_COLOR, false};
      drawButton(buttons[2]);
    }
    // 清空其余手动按钮
    for (int i = 3; i < 10; i++) {
      buttons[i] = {0, 0, 0, 0, "", ACCENT_COLOR, false};
    }
  } else if (manualExitHoming) {
    // 退出手动模式中：正在回原点，禁用所有操作按钮，显示 HOMING 状态
    tft.setTextColor(0xFD30, PANEL_COLOR);  // 橙色
    tft.drawString("EXITING... homing", 18, 145);
    buttons[2] = {60, 170, 120, 40, "HOMING...", WARN_COLOR, false};
    drawButton(buttons[2]);
    for (int i = 3; i < 10; i++) {
      buttons[i] = {0, 0, 0, 0, "", ACCENT_COLOR, false};
    }
  } else {
    // 已进入手动模式：显示 EXIT、INSERT、LEAVE、MOTOR 四个按钮（全部在 y232 面板内）
    buttons[2] = {60, 154, 120, 22, "EXIT", DANGER_COLOR, false};
    drawButton(buttons[2]);

    // 插入按钮：进/出气舵机插入集热器（移动中显示 IN>>）
    // v34.18: INSERTED 状态边框颜色用蓝色（ACCENT_COLOR），与 LEAVE 的 LEAVED 对称
    uint16_t inColor = ACCENT_COLOR;
    const char* inLabel = manualPipeInserted ? "INSERTED" : "INSERT";
    if (manualPipeMoving && manualPipeInserted) {
      inColor = WARN_COLOR;
      inLabel = "IN >>";
    }
    buttons[3] = {15, 180, 100, 24, inLabel, inColor, (manualPipeMoving && manualPipeInserted)};
    drawButton(buttons[3]);

    // 离开按钮：进/出气舵机离开回原点（移动中显示 OUT>>）
    // v34.18: LEAVED 状态边框颜色改为蓝色（ACCENT_COLOR）
    uint16_t outColor = ACCENT_COLOR;
    const char* outLabel = (!manualPipeInserted) ? "LEAVED" : "LEAVE";
    if (manualPipeMoving && !manualPipeInserted) {
      outColor = WARN_COLOR;
      outLabel = "OUT >>";
    }
    buttons[4] = {125, 180, 100, 24, outLabel, outColor, (manualPipeMoving && !manualPipeInserted)};
    drawButton(buttons[4]);

    // 电机按钮：直流电机 启停（横向居中对称，面板x10-230中心x120，宽200→x20-220）
    // 按钮文字提示"点击后动作"：停止时显示"运行"，运行时显示"停止"
    uint16_t motorColor = manualMotorRunning ? DANGER_COLOR : ACCENT_COLOR;
    const char* motorLabel = manualMotorRunning ? "MOTOR STOP" : "MOTOR RUN";
    buttons[5] = {20, 207, 200, 22, motorLabel, motorColor, manualMotorRunning};
    drawButton(buttons[5]);

    // 清空其余按钮
    for (int i = 6; i < 10; i++) {
      buttons[i] = {0, 0, 0, 0, "", ACCENT_COLOR, false};
    }
  }
}

void drawButtonPanel() {
  if (systemMode == SYSTEM_MODE_COLLECT) {
    drawCollectPanel();
    // 仅 CS_READY 阶段显示手动操控界面
    if (collectSubState == CS_READY) {
      drawCollectManualPanel();
    }
    return;
  }

  if (systemMode == SYSTEM_MODE_AUTO) {
    // AUTO 模式下只显示 AI 状态，不显示手动控制按钮
    tft.fillRoundRect(10, 235, 220, 80, 5, PANEL_COLOR);
    tft.drawRoundRect(10, 235, 220, 80, 5, 0x4A69);
    tft.setTextColor(AUTO_COLOR, PANEL_COLOR);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("AI CONTROL", 120, 270);
    tft.setTextSize(1);
    tft.setTextColor(0xBDF7, PANEL_COLOR);
    tft.drawString("TFLite Infer / 60s", 120, 292);
    // 清空手动按钮的命中区域（保留 button[9] 加热片）
    for (int i = 1; i < 9; i++) {
      buttons[i] = {0, 0, 0, 0, "", ACCENT_COLOR, false};
    }
    return;
  }

  // MANUAL 模式：显示手动控制按钮
  int btnY1 = 237;
  int btnY2 = 280;

  buttons[1] = {10, btnY1, 50, 40, "+M", ACCENT_COLOR, false};
  buttons[2] = {65, btnY1, 50, 40, "-M", ACCENT_COLOR, false};
  buttons[3] = {125, btnY1, 50, 40, "+P", ACCENT_COLOR, false};
  buttons[4] = {180, btnY1, 50, 40, "-P", ACCENT_COLOR, false};

  buttons[5] = {10, btnY2, 65, 32, "OUTPUT", outputServoOn ? SUCCESS_COLOR : DANGER_COLOR, outputServoOn};
  buttons[6] = {80, btnY2, 50, 32, "DRY", ACCENT_COLOR, false};
  buttons[7] = {135, btnY2, 50, 32, "DRY+S", ACCENT_COLOR, false};
  buttons[8] = {190, btnY2, 40, 32, "SAVE", ACCENT_COLOR, false};

  for (int i = 1; i <= 8; i++) {
    drawButton(buttons[i]);
  }
}

void drawButton(Button btn) {
  uint16_t fill = btn.state ? btn.color : PANEL_COLOR;
  uint16_t text = btn.state ? TFT_BLACK : TEXT_COLOR;

  tft.fillRoundRect(btn.x, btn.y, btn.w, btn.h, 4, fill);
  tft.drawRoundRect(btn.x, btn.y, btn.w, btn.h, 4, btn.color);

  tft.setTextColor(text, fill);
  // 模式切换按钮（buttons[0]，宽70高25，label 可能为 COLLECT/AUTO/MANUAL）
  // 统一用 textSize 1，保证三个模式按钮字号一致
  if (btn.w == 70 && btn.h == 25) {
    tft.setTextSize(1);
  }
  // manual override 界面的进入按钮（"MANUAL"，宽120高40）用 textSize 2（调大一号）
  else if (strcmp(btn.label, "MANUAL") == 0 && btn.w >= 100 && btn.h >= 35) {
    tft.setTextSize(2);
  }
  // 其他特殊小按钮（OUTPUT/HEATER/COLLECT/AUTO）用 textSize 1
  else if (strcmp(btn.label, "OUTPUT") == 0 ||
           strcmp(btn.label, "HEATER") == 0 ||
           strcmp(btn.label, "COLLECT") == 0 ||
           strcmp(btn.label, "AUTO") == 0) {
    tft.setTextSize(1);
  } else {
    tft.setTextSize(btn.w > 60 ? 2 : 1);
  }
  tft.setTextDatum(MC_DATUM);
  tft.drawString(btn.label, btn.x + btn.w/2, btn.y + btn.h/2);
}

void initButtons() {
  buttons[0] = {150, 40, 70, 25, "COLLECT", COLLECT_COLOR, false};
  for (int i = 1; i < 10; i++) {
    buttons[i] = {0, 0, 0, 0, "", ACCENT_COLOR, false};
  }
}

bool hitButton(Button btn, int x, int y) {
  if (btn.w == 0 || btn.h == 0) return false;
  return (x >= btn.x && x <= btn.x + btn.w && y >= btn.y && y <= btn.y + btn.h);
}
