//#############################################################################
//
//  Dual-module single-phase direct AC-AC Buck-Boost controller (F280049)
//
//  This file is shared by both CCS projects.  Change UNIT_ROLE only:
//      UNIT_1: model 1 = voltage control, model 2 = current sharing
//      UNIT_2: model 2 = voltage control
//
//  IMPORTANT: the delivered build is deliberately gate-locked.  ADC, OLED,
//  keys, PLL, protection and control calculations execute, but the OST latch
//  cannot be cleared while POWER_STAGE_ENABLE is zero.
//
//#############################################################################

#include "F28x_Project.h"
#include "driverlib.h"
#include "math.h"
#include "string.h"
#include "oled.h"

//********** 编译期安全开关 **********//
// 首版只做低压、无功率联调：POWER_STAGE_ENABLE 必须保持 0。
// 需要正式放开功率级时，必须先完成电流标定、栅极波形、CMPSS 和 ISR 实测。
#define UNIT_1                         1
#define UNIT_2                         2
#define PROFILE_HALF                   1
#define PROFILE_FULL                   2
#define HALF_SAFE                      1
#define HALF_IHI                       2
#define AUX_SHARED_FORMAL              1
#define AUX_SHARED_LAB_DEBUG           2

#define UNIT_ROLE UNIT_1
#define POWER_STAGE_ENABLE          1U
#define POWER_PROFILE               PROFILE_FULL
#define HALF_CURRENT_STAGE          HALF_SAFE
#define HALF_IHI_TEST_PASSED_ACK       1
#define FORMAL_HW_RELEASE_ACK           1
#define DIRECTION_DIAG_8V              0
#define AUX_TOPOLOGY                   AUX_SHARED_FORMAL
#define CURRENT_LOOP_INTEGRAL_ENABLE   1U
#define GATE_OPEN_LOOP_TEST           0U
#define OPEN_LOOP_DUTY                0.10f

// CMPSS7 digital deglitch filter. At 100 MHz SYSCLK this samples every
// 100 ns and requires 3 of 5 over-limit samples, rejecting narrow bipolar
// switching spikes while keeping the hardware trip response below 1 us.
#define IL_CMPSS_FILTER_PRESCALE        9U
#define IL_CMPSS_FILTER_WINDOW          5U
#define IL_CMPSS_FILTER_THRESHOLD       3U

#if UNIT_ROLE != UNIT_1 && UNIT_ROLE != UNIT_2
#error invalid UNIT_ROLE
#endif
#if POWER_STAGE_ENABLE != 0 && POWER_STAGE_ENABLE != 1
#error invalid POWER_STAGE_ENABLE
#endif
#if POWER_PROFILE != PROFILE_HALF && POWER_PROFILE != PROFILE_FULL
#error invalid POWER_PROFILE
#endif
#if HALF_CURRENT_STAGE != HALF_SAFE && HALF_CURRENT_STAGE != HALF_IHI
#error invalid HALF_CURRENT_STAGE
#endif
#if AUX_TOPOLOGY != AUX_SHARED_FORMAL && AUX_TOPOLOGY != AUX_SHARED_LAB_DEBUG
#error invalid AUX_TOPOLOGY
#endif
#if HALF_IHI_TEST_PASSED_ACK != 0 && HALF_IHI_TEST_PASSED_ACK != 1
#error invalid HALF_IHI_TEST_PASSED_ACK
#endif
#if FORMAL_HW_RELEASE_ACK != 0 && FORMAL_HW_RELEASE_ACK != 1
#error invalid FORMAL_HW_RELEASE_ACK
#endif
#if DIRECTION_DIAG_8V != 0 && DIRECTION_DIAG_8V != 1
#error invalid DIRECTION_DIAG_8V
#endif
#if CURRENT_LOOP_INTEGRAL_ENABLE != 0 && CURRENT_LOOP_INTEGRAL_ENABLE != 1
#error invalid CURRENT_LOOP_INTEGRAL_ENABLE
#endif
#if DIRECTION_DIAG_8V && \
   (POWER_PROFILE != PROFILE_HALF || HALF_CURRENT_STAGE != HALF_SAFE || \
    AUX_TOPOLOGY != AUX_SHARED_LAB_DEBUG || HALF_IHI_TEST_PASSED_ACK != 0)
#error 8V direction diagnostic requires HALF SAFE and lab auxiliary
#endif
#if GATE_OPEN_LOOP_TEST && \
   ((POWER_STAGE_ENABLE != 1U) || \
    (DIRECTION_DIAG_8V != 1U) || \
    (POWER_PROFILE != PROFILE_HALF) || \
    (HALF_CURRENT_STAGE != HALF_SAFE) || \
    (AUX_TOPOLOGY != AUX_SHARED_LAB_DEBUG))
#error Open-loop gate test requires isolated 8V HALF_SAFE configuration
#endif
#if POWER_PROFILE == PROFILE_FULL && AUX_TOPOLOGY != AUX_SHARED_FORMAL
#error FULL profile requires approved transformer-fed auxiliary supply
#endif
#if POWER_PROFILE == PROFILE_FULL && FORMAL_HW_RELEASE_ACK != 1
#error FULL profile requires explicit formal hardware release
#endif
#if POWER_PROFILE == PROFILE_FULL && HALF_IHI_TEST_PASSED_ACK != 1
#error FULL profile requires completed HALF rated-current tests
#endif

//********** 固定硬件参数 **********//
// 20 kHz 载波、1 mH 电感、Ci=11 uF、Co=21 uF。所有电流限值都必须按实测
// 传感器标定值重新确认；这里的宏只是当前工程的初始保护值。
#define PI_F                            3.14159265358979323846f
#define TWO_PI_F                       (2.0f * PI_F)
#define SQRT2_F                         1.4142135623730950488f
#define INV_SQRT2_F                     0.7071067811865475244f
#define CONTROL_HZ                      20000.0f
#define CONTROL_TS                     0.00005f
#define PWM_TBPRD                       2500U
#define PWM_DEAD_TICKS                  25U
#define PWM_ZC_TICKS                    25U
#define PWM_D_MIN                       0.02f
#define PWM_D_MAX                       0.85f
#define RMS_WINDOW                      400U
#define L_HENRY                         0.001f
#define CO_FARAD                        21.0e-6f
#define IO_FF_A                         0.881911f
#define FUND_LPF_A                      0.993737f
#define IL_TO_OUTPUT_SIGN              (1.0f)

//********** ADC 标定参数 (physical = (raw - offset) / gain) **********//
#define UI_ADC_OFFSET                    2066.2f
#define UI_ADC_GAIN                      36.443f
#define UO_ADC_OFFSET                    2067.0f
#define UO_ADC_GAIN                      36.378f
#define IL_ADC_OFFSET                    2077.3f
#define IL_ADC_GAIN                      164.61f
#define IO_ADC_OFFSET                    2097.6f
#define IO_ADC_GAIN                      333.10f
#define IT_ADC_OFFSET                    2081.8f
#define IT_ADC_GAIN                      162.97f
// 极性(Polarity)：#1，取决于差分前端接线方向
#define UI_ADC_POLARITY                  1.0f
#define UO_ADC_POLARITY                  1.0f
#define IL_ADC_POLARITY                  1.0f
#define IO_ADC_POLARITY                  1.0f
#define IT_ADC_POLARITY                  1.0f

// ADC 断线/饱和保护：只把接近 12-bit 电源轨的原始码判为无效。
// 连续 20 个载波周期（1 ms）越界才触发 F09，抑制并机切换时的瞬态干扰。
#define ADC_RAW_RAIL_MIN                  8U
#define ADC_RAW_RAIL_MAX                  4087U
#define ADC_RAIL_BAD_LIMIT                20U

//********** 电压范围与保护阈值 (V) **********//
#if POWER_PROFILE == PROFILE_HALF
// 36 V RMS 输入
#define UI_START_MIN                     34.0f
#define UI_START_MAX                     38.0f
#define UI_RUN_MIN                       32.0f
#define UI_RUN_MAX                       39.0f

// 输出 1~35 V，并机额定目标 30 V
#define UO_SINGLE_MIN                    1.0f
#define UO_SINGLE_MAX                   35.0f
#define UO_PARALLEL_MAX                 35.0f
#define UO_DEFAULT                      30.0f

// 输出过压保护
#define UO_RMS_HARD_MAX                  36.5f
#define UO_ABS_PK_TRIP                   52.0f
#define UOV_MARGIN                       1.0f
#else
// 输入电压启动/运行窗口
#define UI_START_MIN                     33.0f
#define UI_START_MAX                     38.0f
#define UI_RUN_MIN                       30.0f
#define UI_RUN_MAX                       38.0f
#define UO_SINGLE_MAX                    35.0f
#define UO_PARALLEL_MAX                  30.0f
#define UO_DEFAULT                       30.0f
#define UO_RMS_HARD_MAX                  37.0f
#define UO_ABS_PK_TRIP                   53.0f
#define UOV_MARGIN                       2.0f
#endif

//********** 电流保护阈值 (A) **********//
#if POWER_PROFILE == PROFILE_FULL || HALF_CURRENT_STAGE == HALF_IHI
#define I_BRANCH_CMD_MAX                 2.05f
#define IO_PK_TRIP                       3.50f
#define IO_RMS_TRIP                      2.25f
#define IT_PK_TRIP                       7.00f
#define IT_RMS_TRIP                      4.50f
#define IL_REF_PK_MAX                    5.80f
#define IL_SW_FAST_LIMIT                 6.00f
#define IL_CMPSS_TRIP                    6.50f
#define IL_RMS_CONT                      4.00f
#define IL_RMS_TRIP                      4.20f
#else
#define I_BRANCH_CMD_MAX                 1.15f
#define IO_PK_TRIP                       2.00f
#define IO_RMS_TRIP                      1.30f
#define IT_PK_TRIP                       4.00f
#define IT_RMS_TRIP                      2.50f
#define IL_REF_PK_MAX                    3.10f
#define IL_SW_FAST_LIMIT                 3.40f
#define IL_CMPSS_TRIP                    3.80f
#define IL_RMS_CONT                      2.20f
#define IL_RMS_TRIP                      2.40f
#endif

#if DIRECTION_DIAG_8V
#undef UI_START_MIN
#undef UI_START_MAX
#undef UI_RUN_MIN
#undef UI_RUN_MAX
#undef UO_DEFAULT
#undef IL_REF_PK_MAX
#undef IL_SW_FAST_LIMIT
#undef IL_CMPSS_TRIP
// 输入电压启动/运行窗口
#define UI_START_MIN                     4.0f
#define UI_START_MAX                     8.0f
#define UI_RUN_MIN                       4.0f
#define UI_RUN_MAX                       8.0f
#define UO_DEFAULT                       1.0f
#define IL_REF_PK_MAX                    0.45f
#define IL_SW_FAST_LIMIT                 0.60f
#define IL_CMPSS_TRIP                    0.80f
#endif

//********** 过零迟滞换向 **********//
#define ZC_ENTER_V                      0.20f
#define ZC_EXIT_V                       0.20f
#define ZC_BLOCK_SAMPLES                160U
#define ZC_ADVANCE_SEC                  75.0e-6f
#define ZC_CURRENT_FADE_V               0.40f

//********** PLL 和控制增益（调试初值）**********//
#define PLL_KP                           88.9f
#define PLL_KI                           3948.0f
#define PLL_W_MIN                        (TWO_PI_F * 45.0f)
#define PLL_W_MAX                        (TWO_PI_F * 55.0f)
#define PLL_SOGI_K                       1.41421356237f
#define CURRENT_KP                       6.0f
#define CURRENT_KI                       160.0f
#define VOLTAGE_KP                       1.0e-3f
#define VOLTAGE_KI                       5.0e-2f
#define SHARE_KP                         0.70f
#define SHARE_KI                         30.0f
#define DIRECT_VOLTAGE_DUTY_TEST         1U
#define UO_SET_MIN                      1.0f
#define UO_SET_MAX                     35.0f
#define UO_SET_STEP                     0.5f
#define DUTY_IO_FF_GAIN                 0.002f
#define DIAG_DISABLE_IL_SW_TRIP         0U
#define DIAG_DISABLE_CMPSS_TRIP         0U

#define KEY1_IN                         (GpioDataRegs.GPADAT.bit.GPIO27 == 0U)
#define KEY2_IN                         (GpioDataRegs.GPADAT.bit.GPIO25 == 0U)
#define KEY3_IN                         (GpioDataRegs.GPADAT.bit.GPIO17 == 0U)
#define KEY4_IN                         (GpioDataRegs.GPADAT.bit.GPIO26 == 0U)
#define KEY5_IN                         (GpioDataRegs.GPADAT.bit.GPIO16 == 0U)
#define KEY6_IN                         (GpioDataRegs.GPBDAT.bit.GPIO39 == 0U)

typedef enum
{
    MODEL_SAFE = 0,
    MODEL_VOLTAGE = 1,
    MODEL_PARALLEL = 2
} ModelCode;

typedef enum
{
    ST_BOOT = 0, ST_SAFE, ST_PRECHECK, ST_INPUT_PLL, ST_WAIT_UO,
    ST_OUTPUT_PLL, ST_SS5_RAMP, ST_SS5_HOLD, ST_SHARE_RAMP,
    ST_VOLT_RAMP, ST_RUN, ST_STOP_RAMP, ST_WAIT_MASTER_OFF,
    ST_DISCHARGE, ST_GATE_TEST, ST_FAULT
} RunState;

typedef enum
{
    GATE_BLOCKED = 0, GATE_ZC_A, GATE_ZC_B, GATE_ACTIVE
} GateState;

typedef enum
{
    HALF_POS = 1, HALF_NEG = -1
} HalfPolarity;

typedef enum
{
    FAULT_OK = 0, FAULT_IL_PK = 1, FAULT_IL_RMS = 2,
    FAULT_IO = 3, FAULT_IT = 4, FAULT_UO_OV = 5,
    FAULT_UI = 6, FAULT_BUS_LOST = 7, FAULT_PLL = 8,
    FAULT_ADC = 9, FAULT_START = 10, FAULT_DC_OFFSET = 11,
    FAULT_MODE = 12, FAULT_PHASE = 13, FAULT_DISCHARGE = 14,
    FAULT_CPU = 15, FAULT_TEMP = 16
} FaultCode;

typedef struct
{
    float alpha;
    float beta;
    float v_prev;
    float dalpha;
} SogiState;

typedef struct
{
    SogiState sogi;
    float theta;
    float omega;
    float integrator;
    float amplitude;
    float amplitude_published;
    float q_norm;
    Uint16 locked;
    Uint32 lock_samples;
    float sin_theta;
    float cos_theta;
    float sin_dtheta;
    float cos_dtheta;
    float dtheta_pending;
    float sin_dtheta_pending;
    float cos_dtheta_pending;
} SinglePhasePll;

typedef struct
{
    float kp;
    float ki;
    float integral;
    float out_min;
    float out_max;
} PiController;

typedef struct
{
    float data[RMS_WINDOW];
    float sum_sq;
    float sum;
    Uint16 index;
    Uint16 filled;
    float rms;
    float mean;
} SlidingRms;

typedef struct
{
    Uint16 raw_ui;
    Uint16 raw_uo;
    Uint16 raw_il;
    Uint16 raw_io;
    Uint16 raw_it;
    float ui;
    float uo;
    float il;
    float io;
    float it;
    float ui_rms;
    float uo_rms;
    float il_rms;
    float io_rms;
    float it_rms;
    float il_mean;
    float uo_mean;
} Measurements;

typedef struct
{
    Uint16 stable;
    Uint16 count;
    Uint16 held_ms;
    Uint16 repeat_ms;
    Uint16 long_fired;
    Uint16 press_edge;
    Uint16 release_edge;
    Uint16 repeat_edge;
} KeyState;

typedef struct
{
    float d;
    float q;
} DqLowPass;

//********** 运行状态 **********//
// ADC/PLL/RMS/PI 状态在 ISR 中更新；按键、OLED 和状态机在主循环中低速运行。
volatile Measurements meas;
SinglePhasePll pll_ui;
SinglePhasePll pll_uo;
SogiState uo_observer;
SlidingRms rms_il;
SlidingRms rms_io;
SlidingRms rms_it;
SlidingRms rms_uo;
PiController pi_id;
PiController pi_vd;
PiController pi_vq;
PiController pi_sd;
PiController pi_sq;
PiController pi_u_duty;
DqLowPass it_dq;
DqLowPass io_dq;

volatile RunState run_state = ST_BOOT;
volatile GateState gate_state = GATE_BLOCKED;
volatile HalfPolarity half = HALF_POS;
volatile HalfPolarity next_half = HALF_POS;
volatile FaultCode fault_code = FAULT_OK;
volatile FaultCode warning_code = FAULT_OK;
volatile Measurements fault_snapshot;
volatile RunState fault_state = ST_BOOT;
volatile float fault_duty = PWM_D_MIN;
volatile Uint16 model_cmd = MODEL_SAFE;
volatile Uint16 model_active = MODEL_SAFE;
volatile Uint16 start_request = 0U;
volatile Uint16 stop_request = 0U;
volatile Uint16 clear_request = 0U;
volatile Uint16 tick_1ms = 0U;
volatile Uint16 tick_5ms = 0U;
volatile Uint16 tick_100ms = 0U;
volatile Uint16 tick_div_5ms = 0U;
volatile Uint16 tick_div_100ms = 0U;
volatile Uint32 isr_heartbeat = 0UL;
volatile Uint32 main_heartbeat = 0UL;
volatile Uint32 state_ms = 0UL;
volatile Uint32 isr_overrun_count = 0UL;
// 调试观测量：用于区分 WDT 复位、ADCINT 溢出和 ISR 实际超时。
volatile Uint32 debug_reset_cause = 0UL;
volatile Uint16 debug_last_isr_ticks = 0U;
volatile Uint16 debug_max_isr_ticks = 0U;
volatile Uint16 debug_adc_ovf_count = 0U;
volatile Uint16 debug_isr_slot = 0U;
volatile Uint16 debug_adc_ovf_slot = 0U;
volatile Uint16 debug_dead_bus_block = 0U;
volatile Uint16 debug_dead_bus_limit_mA = 300U;

float u_ref_cmd = UO_DEFAULT;
float u_ref_active = 0.0f;
float trim_low = 0.0f;
float trim_high = 0.0f;
float k_cmd = 1.0f;
float k_active = 1.0f;
float share_alpha_ramp = 0.0f;
float io_ff = 0.0f;
float duty = PWM_D_MIN;
float il_ref = 0.0f;
float il_ref_prev = 0.0f;
float it_fund_rms = 0.0f;
float io_fund_rms = 0.0f;
float io2_fund_rms = 0.0f;
float share_error_pct = 0.0f;
float iconv_ref_held = 0.0f;
float iconv_ref_target = 0.0f;
float dff_held = PWM_D_MIN;
float dff_target = PWM_D_MIN;
float direct_duty_target = PWM_D_MIN;
float uo_amp_held = 0.0f;
Uint16 k_limited = 0U;
Uint16 bus_valid = 0U;
Uint16 gate_start_pending = 0U;
Uint16 adc_bad_count = 0U;
Uint16 il_fast_count = 0U;
Uint16 io_pk_count = 0U;
Uint16 it_pk_count = 0U;
Uint16 uo_pk_count = 0U;
Uint16 uo_rms_ov_count = 0U;
volatile float debug_dynamic_ov = 0.0f;
volatile float debug_u_ref_active = 0.0f;
Uint16 il_rms_count = 0U;
Uint16 io_rms_count = 0U;
Uint16 it_rms_count = 0U;
Uint16 ui_bad_count = 0U;
Uint16 pll_bad_count = 0U;
Uint16 bus_lost_count = 0U;
Uint16 dc_bad_count = 0U;
Uint16 zc_stage_periods = 0U;
Uint16 sample_div = 0U;
Uint16 oled_line = 0U;
Uint16 ss5_stable_ms = 0U;
Uint16 output_pll_stable_ms = 0U;
Uint16 voltage_stable_ms = 0U;
Uint16 voltage_error_ms = 0U;
Uint16 current_limit_ms = 0U;
Uint16 current_release_ms = 0U;
Uint16 current_limit_active = 0U;
Uint16 fault_clear_safe_ms = 0U;
Uint16 trip_clear_safe_count = 0U;
Uint16 open_loop_test_ms = 0U;
float ui_alpha_prev = 0.0f;
volatile Uint32 debug_half_change_count = 0UL;
typedef enum
{
    ZC_REGION_POS = 1,
    ZC_REGION_ZERO = 0,
    ZC_REGION_NEG = -1
} ZcRegion;
volatile ZcRegion zc_region = ZC_REGION_ZERO;
Uint16 zc_block_count = 0U;
volatile Uint32 debug_rejected_zc_count = 0UL;
volatile float debug_uo_d = 0.0f;
volatile float debug_uo_q = 0.0f;
volatile float debug_vd_error = 0.0f;
volatile float debug_vq_error = 0.0f;
volatile Uint32 debug_zc_a_count = 0UL;
volatile Uint32 debug_zc_b_count = 0UL;
volatile Uint32 debug_zc_done_count = 0UL;
volatile Uint16 debug_il_fault_source = 0U;
volatile Uint16 debug_cmpss_status_at_fault = 0U;
volatile Uint16 debug_epwm1_tz_at_fault = 0U;
volatile Uint16 debug_epwm2_tz_at_fault = 0U;
KeyState keys_state[6];

// 函数持久状态（原 static 局部变量，统一风格提升至文件域）
Uint16 dead_bus_count = 0U;
Uint16 adc_isr_overrun_consecutive = 0U;
Uint32 main_isr_seen = 0UL;
Uint32 main_main_seen = 0UL;

#ifdef _FLASH
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;
#endif

// 将每个20 kHz周期都执行的计算热点放入RAM。慢速数学函数已通过载波分槽
// 控制峰值预算，保留在Flash，避免挤占有限的RAMLS0。
#ifdef _FLASH
#pragma CODE_SECTION(is_voltage_role, ".TI.ramfunc")
#pragma CODE_SECTION(share_signal_processing_active, ".TI.ramfunc")
#pragma CODE_SECTION(clampf_local, ".TI.ramfunc")
#pragma CODE_SECTION(slew, ".TI.ramfunc")
#pragma CODE_SECTION(sogi_update, ".TI.ramfunc")
#pragma CODE_SECTION(pll_update_fast, ".TI.ramfunc")
#pragma CODE_SECTION(rms_update_fast, ".TI.ramfunc")
#pragma CODE_SECTION(mean_update_fast, ".TI.ramfunc")
#pragma CODE_SECTION(pi_reset, ".TI.ramfunc")
#pragma CODE_SECTION(dq_lpf_update, ".TI.ramfunc")
#pragma CODE_SECTION(adcB1ISR, ".TI.ramfunc")
#pragma CODE_SECTION(sample_and_calibrate, ".TI.ramfunc")
#pragma CODE_SECTION(signal_processing_fast, ".TI.ramfunc")
#pragma CODE_SECTION(control_update_fast, ".TI.ramfunc")
#endif

//********** 函数声明 **********//
__interrupt void adcB1ISR(void);
void hardware_init(void);
void gpio_init_local(void);
void gpio_enable_pwm_mux(void);
void keys_init(void);
void adc_init_local(void);
void adc_soc_init(void);
void pwm_init_local(void);
void pwm_module_init(volatile struct EPWM_REGS *pwm, Uint16 master);
void cmpss_trip_init(void);
void controllers_init(void);
void pwm_force_off(void);
Uint16 trip_clear_qualify_fast(void);
Uint16 pwm_clear_ost(void);
void pwm_set_normal(HalfPolarity half, float duty);
void pwm_set_zc_a(HalfPolarity next_half);
void pwm_set_zc_b(HalfPolarity next_half);
void pwm_set_aq_active(volatile struct EPWM_REGS *pwm);
void pwm_set_aq_delayed_high(volatile struct EPWM_REGS *pwm, Uint16 output_a,
                                    Uint16 ticks);
void pwm_force_pair(volatile struct EPWM_REGS *pwm, Uint16 a_force,
                           Uint16 b_force);
void gate_sequence_update(void);
void request_half_change(HalfPolarity next_half);
void sample_and_calibrate(void);
void signal_processing_fast(void);
void signal_processing_it_dq_publish_slow(void);
void signal_processing_io_dq_publish_slow(void);
void signal_processing_io2_dq_publish_slow(void);
void signal_processing_il_publish_slow(void);
void signal_processing_io_publish_slow(void);
void signal_processing_it_publish_slow(void);
void signal_processing_voltage_publish_slow(void);
void software_protection_fast(void);
void software_protection_slow(void);
void control_update_fast(void);
void control_update_slow(void);
void slow_state_machine_1ms(void);
Uint16 voltage_current_limit_1ms(void);
void keys_update_5ms(void);
void key_action(Uint16 key, Uint16 repeat_event, Uint16 release_event,
                       Uint16 long_event);
void oled_update_one_line(void);
void latch_fault(FaultCode fault);
Uint16 fault_clear_conditions(void);
Uint16 is_voltage_role(void);
Uint16 is_share_role(void);
Uint16 share_signal_processing_active(void);
void reset_share_signal_processing(void);
Uint16 model_is_legal(Uint16 model);
Uint16 dead_bus(void);
Uint16 input_start_ok(void);
Uint16 input_run_ok(void);
Uint16 predicted_zero(void);
float effective_u_ref(void);
float active_u_max(void);
float clampf_local(float x, float lo, float hi);
float slew(float x, float target, float step);
float wrap_pi(float x);
void sogi_update(SogiState *s, float input, float omega);
void pll_update_fast(SinglePhasePll *pll, float input, float amp_min);
void pll_slow_amplitude_update(SinglePhasePll *pll, float amp_min);
void pll_slow_normalize_update(SinglePhasePll *pll);
void pll_slow_theta_update(SinglePhasePll *pll);
void pll_slow_sin_dtheta_update(SinglePhasePll *pll);
void pll_slow_cos_dtheta_update(SinglePhasePll *pll);
void pll_slow_commit_dtheta_update(SinglePhasePll *pll);
void rms_update_fast(SlidingRms *r, float x);
void rms_slow_update(SlidingRms *r);
void mean_update_fast(SlidingRms *r, float x);
void mean_slow_update(SlidingRms *r);
float pi_update(PiController *pi, float error, Uint16 integrate);
float pi_update_dt(PiController *pi, float error, Uint16 integrate, float dt);
void pi_reset(PiController *pi);
void dq_lpf_update(DqLowPass *f, float x, float sn, float cs);
void clear_line(char *line);
void put_text(char *line, Uint16 pos, const char *s);
void put_u2(char *line, Uint16 pos, Uint16 v);
void put_fixed(char *line, Uint16 pos, float x, Uint16 decimals,
                      Uint16 width);
const char *state_text(RunState state);
const char *profile_text(void);

//********** 主循环 **********//
// 主循环不直接产生 PWM，只处理模式、按键、OLED、看门狗和故障复位。
void main(void)
    {
    Uint32 reset_cause;

#ifdef _FLASH
    // 将时间关键代码从 Flash 复制到 RAM（零等待执行）
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif

    // 初始化顺序很重要：先建立系统时钟，再把门极保持为 GPIO 低电平，
    // 然后初始化 ADC/ePWM/CMPSS，最后才开放中断和低速任务。
    InitSysCtrl();        // 系统时钟初始化（PLL、外设时钟、看门狗）
    reset_cause = SysCtl_getResetCause();
    debug_reset_cause = reset_cause;
    hardware_init();    // 外设初始化（GPIO/ADC/PWM/CMPSS/OLED）
    controllers_init();  // 控制器/PLL/RMS 状态清零

    run_state = ST_SAFE;       // 初始化完成，进入安全待机
    pwm_force_off();           // 确保所有门极驱动信号为低
    if ((reset_cause & (SYSCTL_CAUSE_WDRS | SYSCTL_CAUSE_NMIWDRS)) != 0UL)
    {
        latch_fault(FAULT_CPU);
    }
    SysCtl_clearResetCause(reset_cause);

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1U;
    EDIS;

    IER |= M_INT1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 0U;  // ADCA1 关闭
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1U;  // ADCB1 开启
    EINT;
    ERTM;

    SysCtl_setWatchdogMode(SYSCTL_WD_MODE_RESET);
    // Reset defaults give only about 13 ms.  A 16-character software-I2C
    // OLED line can take tens of milliseconds, so retain fault detection while
    // allowing the documented low-speed task to complete.
    SysCtl_setWatchdogPredivider(SYSCTL_WD_PREDIV_512);
    SysCtl_setWatchdogPrescaler(SYSCTL_WD_PRESCALE_64);
    SysCtl_serviceWatchdog();
    SysCtl_enableWatchdog();

    // 1 ms 状态机、5 ms 按键、100 ms OLED 都由 ISR 产生的计数节拍驱动。
    // 使用计数而不是单比特标志，避免 OLED 阻塞时丢失低速节拍。
    for (;;)
    {
        Uint16 do_1ms = 0U;
        Uint16 do_5ms = 0U;
        Uint16 do_100ms = 0U;
        Uint32 isr_now;
        Uint32 isr_seen = main_isr_seen;
        Uint32 main_seen = main_main_seen;

        main_heartbeat++;           // 主循环心跳（WDT 服务判断用）
        DINT;                      // 关中断，原子读取 ISR 定时标志
        if (tick_1ms != 0U) { tick_1ms--; do_1ms = 1U; }
        if (tick_5ms != 0U) { tick_5ms--; do_5ms = 1U; }
        if (tick_100ms != 0U) { tick_100ms--; do_100ms = 1U; }
        isr_now = isr_heartbeat;
        EINT;                      // 恢复中断

        // ---- 1 ms 任务：状态机、限流、启动/停止时序 ----
        if (do_1ms != 0U)
        {
            slow_state_machine_1ms();
            if ((isr_now != isr_seen) && (main_heartbeat != main_seen))
            {
                isr_seen = isr_now;
                main_seen = main_heartbeat;
                main_isr_seen = isr_seen;
                main_main_seen = main_seen;
                SysCtl_serviceWatchdog();
            }
        }
        // ---- 5 ms 任务：按键扫描 ----
        if (do_5ms != 0U)
        {
            keys_update_5ms();
        }
        // ---- 100 ms 任务：OLED 显示刷新（4 行轮显）----
        if (do_100ms != 0U)
        {
            oled_update_one_line();
        }
    }
}

//********** OLED 显示 **********//
// OLED 精简版：单字母标签，保留标志性区分。
void oled_update_one_line(void)
{
    char line[17];
    Uint16 model = (model_active != MODEL_SAFE) ? model_active : model_cmd;

    clear_line(line);
    if (oled_line == 0U)
    {
        put_text(line, 0U, profile_text());
#if UNIT_ROLE == UNIT_1
        put_text(line, 6U, (model == MODEL_PARALLEL) ? "M2SH" :
                            ((model == MODEL_VOLTAGE) ? "M1VC" : "M0--"));
#else
        put_text(line, 6U, (model == MODEL_PARALLEL) ? "M2VC" : "M0--");
#endif
        put_text(line, 11U, state_text(run_state));
    }
    else if (oled_line == 1U)
    {
        if (run_state == ST_FAULT)
        {
            put_text(line, 0U, "F");
            put_u2(line, 2U, (Uint16)fault_code);
            put_text(line, 5U, "OST");
        }
        else
        {
            put_text(line, 0U, "U");
            put_fixed(line, 1U, (run_state == ST_FAULT) ?
                      fault_snapshot.uo_rms : meas.uo_rms, 1U, 5U);
            put_text(line, 7U, "r");
            put_fixed(line, 8U, effective_u_ref(), 1U, 5U);
        }
    }
    else if (oled_line == 2U)
    {
        put_text(line, 0U, "L");
        put_fixed(line, 1U, (run_state == ST_FAULT) ?
                  fault_snapshot.il_rms : meas.il_rms, 2U, 5U);
        put_text(line, 7U, "o");
        put_fixed(line, 8U, (run_state == ST_FAULT) ?
                  fault_snapshot.io_rms : meas.io_rms, 2U, 5U);
    }
    else
    {
        if (model == MODEL_PARALLEL)
        {
            // Model2并机页优先显示实际生效K及按键设定K；PLL资格已由
            // 状态机和保护持续监控，不再占用OLED第四行。
            put_text(line, 0U, "K");
            put_fixed(line, 1U, k_active, 2U, 4U);
            put_text(line, 6U, "c");
            put_fixed(line, 7U, k_cmd, 2U, 4U);
            put_text(line, 12U, "w");
            put_u2(line, 13U, (Uint16)((fault_code != FAULT_OK) ? fault_code : warning_code));
        }
        else
        {
            put_text(line, 0U, "P");
            put_text(line, 1U, pll_ui.locked ? "Y" : "N");
            put_text(line, 4U, "Hz");
            put_fixed(line, 6U, pll_ui.omega / TWO_PI_F, 1U, 5U);
            put_text(line, 12U, "w");
            put_u2(line, 13U, (Uint16)((fault_code != FAULT_OK) ? fault_code : warning_code));
        }
    }

    OLED_ShowString(0U, (Uchar)oled_line, line);
    oled_line = (oled_line + 1U) & 3U;
}

//********** 安全条件 **********//
// 这里的条件只负责“允许启动/允许清故障”，最终硬件过流仍由 CMPSS→ePWM OST
// 异步链路完成，不能用软件轮询替代。
void latch_fault(FaultCode fault)
{
    // 只记录第一个故障，保存当时的采样值、状态和占空比，避免后续故障覆盖根因。
    if ((fault != FAULT_OK) && (fault_code == FAULT_OK))
    {
        // Freeze the first fault context before changing the state or outputs.
        // Subsequent protection sources must not overwrite the root event.
        fault_snapshot = meas;
        fault_state = run_state;
        fault_duty = duty;
        fault_clear_safe_ms = 0U;
        fault_code = fault;
        run_state = ST_FAULT;
        model_cmd = MODEL_SAFE;
        start_request = 0U;
        stop_request = 0U;
        pwm_force_off();           // 确保所有门极驱动信号为低
    }
}

Uint16 fault_clear_conditions(void)
{
    // 故障清除必须同时满足：ADC 原始码有效、母线已放电、CMPSS 未动作、
    // 电流低于恢复阈值，并连续保持 100 ms；否则继续保持 OST。
    Uint16 raw_ok = ((meas.raw_ui >= ADC_RAW_RAIL_MIN) &&
                     (meas.raw_ui <= ADC_RAW_RAIL_MAX) &&
                     (meas.raw_uo >= ADC_RAW_RAIL_MIN) &&
                     (meas.raw_uo <= ADC_RAW_RAIL_MAX) &&
                     (meas.raw_il >= ADC_RAW_RAIL_MIN) &&
                     (meas.raw_il <= ADC_RAW_RAIL_MAX) &&
                     (meas.raw_io >= ADC_RAW_RAIL_MIN) &&
                     (meas.raw_io <= ADC_RAW_RAIL_MAX));
    Uint16 comparator_active;
#if UNIT_ROLE == UNIT_1
    raw_ok = raw_ok && (meas.raw_it >= ADC_RAW_RAIL_MIN) &&
             (meas.raw_it <= ADC_RAW_RAIL_MAX);
#endif

    if (!raw_ok || !dead_bus() ||
        (fabsf(meas.il) > 0.8f * IL_CMPSS_TRIP))
    {
        fault_clear_safe_ms = 0U;
        return 0U;
    }
    if ((fault_code == FAULT_IL_RMS) && (meas.il_rms > 0.8f * IL_RMS_TRIP))
    {
        fault_clear_safe_ms = 0U; return 0U;
    }
    if ((fault_code == FAULT_IO) && (meas.io_rms > 0.8f * IO_RMS_TRIP))
    {
        fault_clear_safe_ms = 0U; return 0U;
    }
#if UNIT_ROLE == UNIT_1
    if ((fault_code == FAULT_IT) && (meas.it_rms > 0.8f * IT_RMS_TRIP))
    {
        fault_clear_safe_ms = 0U; return 0U;
    }
#endif

    // 只在100 ms资格窗口起点清除旧锁存。窗口建立后只读不清，任何再次
    // 越流都会留下锁存并把计时归零，下一次安全尝试才重新武装。
    if (fault_clear_safe_ms == 0U)
    {
        CMPSS_clearFilterLatchHigh(CMPSS7_BASE);
        CMPSS_clearFilterLatchLow(CMPSS7_BASE);
    }
    comparator_active = CMPSS_getStatus(CMPSS7_BASE) &
                        (CMPSS_STS_HI_FILTOUT | CMPSS_STS_LO_FILTOUT);
    if (comparator_active != 0U)
    {
        fault_clear_safe_ms = 0U;
        return 0U;
    }
    if (fault_clear_safe_ms < 100U) fault_clear_safe_ms++;
    return (fault_clear_safe_ms >= 100U) ? 1U : 0U;
}

Uint16 is_voltage_role(void)
{
#if UNIT_ROLE == UNIT_1
    return (model_active == MODEL_VOLTAGE) ? 1U : 0U;
#else
    return (model_active == MODEL_PARALLEL) ? 1U : 0U;
#endif
}

Uint16 is_share_role(void)
{
#if UNIT_ROLE == UNIT_1
    return (model_active == MODEL_PARALLEL) ? 1U : 0U;
#else
    return 0U;
#endif
}

Uint16 share_signal_processing_active(void)
{
#if UNIT_ROLE == UNIT_1
    if (model_active != MODEL_PARALLEL) return 0U;
    return ((run_state == ST_OUTPUT_PLL) ||
            (run_state == ST_SHARE_RAMP) ||
            (run_state == ST_RUN) ||
            (run_state == ST_STOP_RAMP)) ? 1U : 0U;
#else
    return 0U;
#endif
}

void reset_share_signal_processing(void)
{
    // WAIT期间输出PLL处理已停用；清除全部历史，确保母线重新出现后必须
    // 从零开始满足100 ms锁定资格，不能沿用上一次的locked/theta/dq。
    memset(&pll_uo, 0, sizeof(pll_uo));
    memset(&it_dq, 0, sizeof(it_dq));
    memset(&io_dq, 0, sizeof(io_dq));
    memset(&rms_it, 0, sizeof(rms_it));

    pll_uo.omega = TWO_PI_F * 50.0f;
    pll_uo.cos_theta = 1.0f;
    pll_uo.sin_theta = 0.0f;
    pll_uo.sin_dtheta = 0.0157073f;
    pll_uo.cos_dtheta = 0.9998766f;
    pll_uo.sin_dtheta_pending = pll_uo.sin_dtheta;
    pll_uo.cos_dtheta_pending = pll_uo.cos_dtheta;
    pll_uo.amplitude_published = 2.0f;

    it_fund_rms = 0.0f;
    io_fund_rms = 0.0f;
    io2_fund_rms = 0.0f;
    meas.it_rms = 0.0f;
}

Uint16 model_is_legal(Uint16 model)
{
#if UNIT_ROLE == UNIT_1
    return ((model == MODEL_SAFE) || (model == MODEL_VOLTAGE) ||
            (model == MODEL_PARALLEL)) ? 1U : 0U;
#else
    return ((model == MODEL_SAFE) || (model == MODEL_PARALLEL)) ? 1U : 0U;
#endif
}

Uint16 dead_bus(void)
{
    // Uo 负责判断母线已放电；iL/Io 给传感器零点噪声保留 0.30 A 裕量。
    // debug_dead_bus_block: 0=通过，1=Uo，2=iL，3=Io。
    Uint16 count = dead_bus_count;
    if (meas.uo_rms >= 2.0f)
    {
        debug_dead_bus_block = 1U;
        count = 0U;
    }
    else if (meas.il_rms >= 0.30f)
    {
        debug_dead_bus_block = 2U;
        count = 0U;
    }
    else if (meas.io_rms >= 0.30f)
    {
        debug_dead_bus_block = 3U;
        count = 0U;
    }
    else
    {
        debug_dead_bus_block = 0U;
        if (count < 100U) count++;
    }
    dead_bus_count = count;
    return (count >= 100U) ? 1U : 0U;
}

Uint16 input_start_ok(void)
{
    // 启动前只接受半压档输入范围和 48~52 Hz 的输入频率，防止异常输入直接开闸。
    return ((meas.ui_rms >= UI_START_MIN) &&
            (meas.ui_rms <= UI_START_MAX) &&
            (pll_ui.omega >= TWO_PI_F * 48.0f) &&
            (pll_ui.omega <= TWO_PI_F * 52.0f)) ? 1U : 0U;
}

Uint16 input_run_ok(void)
{
    return ((meas.ui_rms >= UI_RUN_MIN) &&
            (meas.ui_rms <= UI_RUN_MAX)) ? 1U : 0U;
}

Uint16 predicted_zero(void)
{
    // SOGI alpha 接近零且 PLL 相位接近 0 或 π 时认为到达过零点；并联时还要
    // 检查输出 PLL 同时过零，避免在活母线非同步位置接入。
    float phase = wrap_pi(pll_ui.theta);
    float pa = fabsf(phase);
    float pb = fabsf(pa - PI_F);
    float distance = (pa < pb) ? pa : pb;
    Uint16 ui_zero = ((fabsf(pll_ui.sogi.alpha) < 1.5f) &&
                      (distance < 2.0f * PI_F / 180.0f)) ? 1U : 0U;
    if (ui_zero == 0U) return 0U;
#if UNIT_ROLE == UNIT_1
    if (is_share_role() != 0U)
    {
        float out_phase = wrap_pi(pll_uo.theta);
        float oa = fabsf(out_phase);
        float ob = fabsf(oa - PI_F);
        float out_distance = (oa < ob) ? oa : ob;
        return ((fabsf(pll_uo.sogi.alpha) < 1.5f) &&
                (out_distance < 2.0f * PI_F / 180.0f) &&
                (pll_uo.locked != 0U)) ? 1U : 0U;
    }
#endif
    return ui_zero;
}

float effective_u_ref(void)
{
    // 将按键设定值叠加低端/高端 ±25 mV 微调；普通电压档保持 0.5 V 步进。
    if (DIRECTION_DIAG_8V) return 1.0f;
#if UNIT_ROLE == UNIT_1
    if (model_cmd == MODEL_VOLTAGE || model_active == MODEL_VOLTAGE)
    {
        if (fabsf(u_ref_cmd - 1.0f) < 0.001f)
            return u_ref_cmd + trim_low;
        if (fabsf(u_ref_cmd - UO_SINGLE_MAX) < 0.001f)
            return u_ref_cmd + trim_high;
    }
#endif
    return u_ref_cmd;
}

float active_u_max(void)
{
#if UNIT_ROLE == UNIT_1
    if ((model_cmd == MODEL_VOLTAGE) || (model_active == MODEL_VOLTAGE))
        return UO_SINGLE_MAX;
#endif
    return UO_PARALLEL_MAX;
}

//********** 数学工具函数 **********//
float clampf_local(float x, float lo, float hi)
{
    // 通用限幅：保护参考、占空比、PI 输出和校准阈值不越过安全边界。
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float slew(float x, float target, float step)
{
    // 每次只允许有限步长，限制软启动、停机和并联系数变化率。
    if (x < target - step) return x + step;
    if (x > target + step) return x - step;
    return target;
}

float wrap_pi(float x)
{
    // 将相位折返到 [-π, π]，用于比较输入/输出 PLL 的相对相位。
    while (x > PI_F) x -= TWO_PI_F;
    while (x < -PI_F) x += TWO_PI_F;
    return x;
}

void sogi_update(SogiState *s, float input, float omega)
{
    // 二阶广义积分器产生 alpha/beta 正交量；dalpha 是后续电容电流前馈所需的
    // 电压导数近似。omega 由对应 PLL 提供，频率变化时滤波中心同步变化。
    // Implicit trapezoidal (Tustin) update of
    // x1'=-k*w*x1-w*x2+k*w*v, x2'=w*x1.
    float a = 0.5f * CONTROL_TS;
    float c = a * PLL_SOGI_K * omega + a * a * omega * omega;
    float x1_old = s->alpha;
    float x2_old = s->beta;
    float x1_new = ((1.0f - c) * x1_old - 2.0f * a * omega * x2_old +
                    a * PLL_SOGI_K * omega * (s->v_prev + input)) /
                   (1.0f + c);
    float x2_new = x2_old + a * omega * (x1_old + x1_new);
    s->dalpha = -PLL_SOGI_K * omega * x1_new - omega * x2_new +
                PLL_SOGI_K * omega * input;
    s->alpha = x1_new;
    s->beta = x2_new;
    s->v_prev = input;
}

void pll_update_fast(SinglePhasePll *pll, float input, float amp_min)
{
    // 20 kHz 快速路径：SOGI 更新、相位误差、频率积分、正交递推。
    // 不调用 sinf/cosf/sqrtf；sin/cos 状态由正交递推推进，每步仅 4 次乘加。
    // 幅值使用 1 kHz 发布的 amplitude_published 带下限归一化。
    float sn = pll->sin_theta;
    float cs = pll->cos_theta;
    float error;
    float sn_new;
    float cs_new;

    sogi_update(&pll->sogi, input, pll->omega);
    {
        float den = pll->amplitude_published;
        if (den < amp_min) den = amp_min;
        error = (pll->sogi.alpha * cs + pll->sogi.beta * sn) / den;
    }
    error = clampf_local(error, -1.0f, 1.0f);
    pll->q_norm = error;
    pll->integrator += PLL_KI * CONTROL_TS * error;
    pll->integrator = clampf_local(pll->integrator,
                                   PLL_W_MIN - TWO_PI_F * 50.0f,
                                   PLL_W_MAX - TWO_PI_F * 50.0f);
    pll->omega = clampf_local(TWO_PI_F * 50.0f + pll->integrator +
                              PLL_KP * error, PLL_W_MIN, PLL_W_MAX);
    pll->theta += pll->omega * CONTROL_TS;
    if (pll->theta >= TWO_PI_F) pll->theta -= TWO_PI_F;
    if (pll->theta < 0.0f) pll->theta += TWO_PI_F;

    // 正交递推：sin(θ+Δθ) = sinθ·cosΔθ + cosθ·sinΔθ
    //            cos(θ+Δθ) = cosθ·cosΔθ - sinθ·sinΔθ
    sn_new = sn * pll->cos_dtheta + cs * pll->sin_dtheta;
    cs_new = cs * pll->cos_dtheta - sn * pll->sin_dtheta;
    pll->sin_theta = sn_new;
    pll->cos_theta = cs_new;
}

void pll_slow_amplitude_update(SinglePhasePll *pll, float amp_min)
{
    // 每毫秒只在本槽执行一次幅值开方，并在同一处更新锁定资格。
    float amplitude = sqrtf(pll->sogi.alpha * pll->sogi.alpha +
                            pll->sogi.beta * pll->sogi.beta);
    pll->amplitude = amplitude;
    pll->amplitude_published = amplitude;

    if ((amplitude >= amp_min) && (fabsf(pll->q_norm) < 0.05f) &&
        (pll->omega >= TWO_PI_F * 48.0f) &&
        (pll->omega <= TWO_PI_F * 52.0f))
    {
        if (pll->lock_samples < 100UL) pll->lock_samples++;
    }
    else pll->lock_samples = 0UL;
    pll->locked = (pll->lock_samples >= 100UL) ? 1U : 0U;
}

void pll_slow_normalize_update(SinglePhasePll *pll)
{
    // 正交状态归一化单独占用一个载波槽，避免与其他数学库调用叠加。
    float mag = sqrtf(pll->sin_theta * pll->sin_theta +
                      pll->cos_theta * pll->cos_theta);
    if (mag > 1e-9f)
    {
        pll->sin_theta /= mag;
        pll->cos_theta /= mag;
    }
}

void pll_slow_theta_update(SinglePhasePll *pll)
{
    // atan2f 单独占用一个载波槽，同步递推正交状态与显式 theta。
    pll->theta = atan2f(pll->sin_theta, pll->cos_theta);
    if (pll->theta < 0.0f) pll->theta += TWO_PI_F;
}

void pll_slow_sin_dtheta_update(SinglePhasePll *pll)
{
    // sin/cos 共用同一份相位步长快照，但分别放在不同载波槽。
    pll->dtheta_pending = pll->omega * CONTROL_TS;
    pll->sin_dtheta_pending = sinf(pll->dtheta_pending);
}

void pll_slow_cos_dtheta_update(SinglePhasePll *pll)
{
    pll->cos_dtheta_pending = cosf(pll->dtheta_pending);
}

void pll_slow_commit_dtheta_update(SinglePhasePll *pll)
{
    // sin/cos均完成后一次性切换，避免快速路径短暂使用不配对的旋转系数。
    pll->sin_dtheta = pll->sin_dtheta_pending;
    pll->cos_dtheta = pll->cos_dtheta_pending;
}

void rms_update_fast(SlidingRms *r, float x)
{
    // 20 kHz 快速路径：只更新滑动窗的平方和与直流和，不调用 sqrtf。
    // sqrtf 在 1 kHz 慢速路径统一发布，节省约 30 ticks/通道 × 4 通道。
    float old = r->data[r->index];
    r->data[r->index] = x;
    r->index++;
    if (r->index >= RMS_WINDOW) r->index = 0U;
    if (r->filled < RMS_WINDOW)
    {
        r->filled++;
        old = 0.0f;
    }
    r->sum_sq += x * x - old * old;
    r->sum += x - old;
    if (r->sum_sq < 0.0f) r->sum_sq = 0.0f;
}

void rms_slow_update(SlidingRms *r)
{
    // 1 kHz 慢速路径：从已累积的平方和/直流和发布 RMS 和均值。
    if (r->filled > 0U)
    {
        r->rms = sqrtf(r->sum_sq / (float)r->filled);
        r->mean = r->sum / (float)r->filled;
    }
}

void mean_update_fast(SlidingRms *r, float x)
{
    // 轻量版滑动窗：仅累加直流和，不累加平方和。
    // 用于 Uo 通道——Uo RMS 由 SOGI 观察器提供，此通道只服务直流偏置保护。
    float old = r->data[r->index];
    r->data[r->index] = x;
    r->index++;
    if (r->index >= RMS_WINDOW) r->index = 0U;
    if (r->filled < RMS_WINDOW)
    {
        r->filled++;
        old = 0.0f;
    }
    r->sum += x - old;
}

void mean_slow_update(SlidingRms *r)
{
    // 仅发布均值，不计算 sqrtf。
    if (r->filled > 0U)
        r->mean = r->sum / (float)r->filled;
}

float pi_update(PiController *pi, float error, Uint16 integrate)
{
    // PI 输出先计算并限幅；只有 integrate 且输出未饱和，或误差方向有利于脱离
    // 饱和时才更新积分，形成基本的抗积分饱和。
    float unsat = pi->kp * error + pi->integral;
    float out = clampf_local(unsat, pi->out_min, pi->out_max);
    if (integrate && ((unsat == out) ||
        ((unsat > pi->out_max) && (error < 0.0f)) ||
        ((unsat < pi->out_min) && (error > 0.0f))))
    {
        pi->integral += pi->ki * CONTROL_TS * error;
        pi->integral = clampf_local(pi->integral, pi->out_min, pi->out_max);
    }
    return out;
}

// PI 控制器版本，允许调用者指定实际控制周期 dt。
// 外环以 1 kHz 运行（dt = 0.001f），不能用 CONTROL_TS（50 µs）。
float pi_update_dt(PiController *pi, float error, Uint16 integrate, float dt)
{
    float unsat = pi->kp * error + pi->integral;
    float out = clampf_local(unsat, pi->out_min, pi->out_max);
    if (integrate && ((unsat == out) ||
        ((unsat > pi->out_max) && (error < 0.0f)) ||
        ((unsat < pi->out_min) && (error > 0.0f))))
    {
        pi->integral += pi->ki * dt * error;
        pi->integral = clampf_local(pi->integral, pi->out_min, pi->out_max);
    }
    return out;
}

void pi_reset(PiController *pi)
{
    pi->integral = 0.0f;
}

void dq_lpf_update(DqLowPass *f, float x, float sn, float cs)
{
    // 将单相电流投影到 PLL 的 d/q 轴，再用一阶低通提取基波分量。
    f->d = FUND_LPF_A * f->d + (1.0f - FUND_LPF_A) * (2.0f * x * sn);
    f->q = FUND_LPF_A * f->q + (1.0f - FUND_LPF_A) * (2.0f * x * cs);
}

//********** 文本格式化 **********//
void clear_line(char *line)
{
    Uint16 i;
    for (i = 0U; i < 16U; i++) line[i] = ' ';
    line[16] = '\0';
}

void put_text(char *line, Uint16 pos, const char *s)
{
    while ((*s != '\0') && (pos < 16U)) line[pos++] = *s++;
}

void put_u2(char *line, Uint16 pos, Uint16 v)
{
    v %= 100U;
    if (pos < 16U) line[pos] = (char)('0' + v / 10U);
    if (pos + 1U < 16U) line[pos + 1U] = (char)('0' + v % 10U);
}

void put_fixed(char *line, Uint16 pos, float x, Uint16 decimals,
                      Uint16 width)
{
    Uint32 scale = 1UL;
    Uint32 value;
    Uint16 i;
    Uint16 negative = (x < 0.0f) ? 1U : 0U;
    for (i = 0U; i < decimals; i++) scale *= 10UL;
    value = (Uint32)(fabsf(x) * (float)scale + 0.5f);
    for (i = 0U; i < width; i++)
    {
        Uint16 at = pos + width - 1U - i;
        if (at >= 16U) continue;
        if ((decimals != 0U) && (i == decimals))
        {
            line[at] = '.';
        }
        else
        {
            line[at] = (char)('0' + (value % 10UL));
            value /= 10UL;
        }
    }
    if (negative && (pos < 16U)) line[pos] = '-';
}

const char *state_text(RunState state)
{
    switch (state)
    {
        case ST_SAFE: return "SAFE";
        case ST_PRECHECK: return "PRE ";
        case ST_INPUT_PLL: return "IPLL";
        case ST_WAIT_UO: return "WAIT";
        case ST_OUTPUT_PLL: return "OPLL";
        case ST_SS5_RAMP: return "SS5 ";
        case ST_SS5_HOLD: return "HOLD";
        case ST_SHARE_RAMP: return "SHAR";
        case ST_VOLT_RAMP: return "RAMP";
        case ST_RUN: return "RUN ";
        case ST_STOP_RAMP: return "STOP";
        case ST_WAIT_MASTER_OFF: return "WT0 ";
        case ST_DISCHARGE: return "DISC";
        case ST_GATE_TEST: return "GATE";
        case ST_FAULT: return "FLT ";
        default: return "BOOT";
    }
}

const char *profile_text(void)
{
#if DIRECTION_DIAG_8V
    return "8VDg";
#elif POWER_PROFILE == PROFILE_FULL
    return "FULL";
#elif AUX_TOPOLOGY == AUX_SHARED_LAB_DEBUG
#if HALF_CURRENT_STAGE == HALF_IHI
    return "HIDbg";
#else
    return "HSDbg";
#endif
#else
#if HALF_CURRENT_STAGE == HALF_IHI
    return "HIHI";
#else
    return "HSAFE";
#endif
#endif
}

//********** 硬件初始化 **********//
// 先把四个门极脚保持为普通 GPIO 低电平，完成 ePWM、CMPSS、TZ 后才切到 PWM 复用。
void hardware_init(void)
{
    InitGpio();
    GPIO_unlockPortConfig(GPIO_PORT_A, 0xFFFFFFFFUL);
    GPIO_unlockPortConfig(GPIO_PORT_B, 0xFFFFFFFFUL);
    GPIO_unlockPortConfig(GPIO_PORT_H, 0xFFFFFFFFUL);

    DINT;
    InitPieCtrl();
    IER = 0x0000U;
    IFR = 0x0000U;
    InitPieVectTable();

    EALLOW;
    // 采用 ADCB EOC2 作为中断入口，保证 ADCA SOC0-1 和 ADCB SOC0-2 均已转换完成。
    PieVectTable.ADCB1_INT = &adcB1ISR;
    AnalogSubsysRegs.DCDCCTL.bit.DCDCEN = 0U;
    EDIS;

    gpio_init_local();
    keys_init();
    OLED_Init();
    OLED_Clean();
    adc_init_local();
    pwm_init_local();
    cmpss_trip_init();
    // GPIO0..3 are connected to ePWM only after AQ and both asynchronous
    // one-shot paths are configured and already forced low.
    gpio_enable_pwm_mux();
    adc_soc_init();
}

void gpio_init_local(void)
{
    EALLOW;
    // Keep every possible gate pin as an ordinary low output until ePWM AQ/TZ
    // is fully initialized.  This prevents a reset-default PWM level from
    // reaching either UCC21520 during boot.
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0U;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0U;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0U;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0U;
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1U;
    GpioDataRegs.GPACLEAR.bit.GPIO0 = 1U;
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1U;
    GpioDataRegs.GPACLEAR.bit.GPIO2 = 1U;
    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1U;
    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1U;
    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1U;

    // ISR 脉宽测量用：GPIO6 (EPWM4A) 初始化为普通输出低电平。
    // 示波器接 GPIO6，ISR 入口置高、出口置低，脉宽即 ISR 执行时间。
    GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0U;
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1U;
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1U;
    EDIS;
}

void gpio_enable_pwm_mux(void)
{
    EALLOW;
    // Final mapping: GPIO0=Q1a, GPIO1=Q2a, GPIO2=Q1b, GPIO3=Q2b.
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1U;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1U;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1U;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1U;
    EDIS;
}

void keys_init(void)
{
    const Uint16 pins[6] = {27U, 25U, 17U, 26U, 16U, 39U};
    Uint16 i;
    for (i = 0U; i < 6U; i++)
    {
        GPIO_setPadConfig(pins[i], GPIO_PIN_TYPE_PULLUP);
        GPIO_setDirectionMode(pins[i], GPIO_DIR_MODE_IN);
        GPIO_setQualificationMode(pins[i], GPIO_QUAL_6SAMPLE);
        GPIO_setQualificationPeriod(pins[i], 20U);
    }
}

void adc_init_local(void)
{
    /*
     * 仿真器异常断开后，CPU复位不一定能完全复位ADC模拟部分。
     * 因此先把ADCA、ADCB、ADCC明确掉电，再重新配置参考电压和上电。
     */

    EALLOW;

    // 先关闭三个ADC，强制模拟部分重新上电
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 0U;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 0U;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 0U;

    EDIS;

    DELAY_US(100U);

    /*================ ADCA =================*/

    ADC_setVREF(ADCA_BASE,
                ADC_REFERENCE_INTERNAL,
                ADC_REFERENCE_3_3V);

    EALLOW;

    AdcaRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1U;

    // 清除可能残留的ADC状态
    AdcaRegs.ADCINTFLGCLR.all = 0x000FU;
    AdcaRegs.ADCINTOVFCLR.all = 0x000FU;
    AdcaRegs.ADCSOCOVFCLR1.all = 0xFFFFU;

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1U;

    EDIS;

    DELAY_US(1000U);

    /*================ ADCB =================*/

    ADC_setVREF(ADCB_BASE,
                ADC_REFERENCE_INTERNAL,
                ADC_REFERENCE_3_3V);

    EALLOW;

    AdcbRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1U;

    AdcbRegs.ADCINTFLGCLR.all = 0x000FU;
    AdcbRegs.ADCINTOVFCLR.all = 0x000FU;
    AdcbRegs.ADCSOCOVFCLR1.all = 0xFFFFU;

    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1U;

    EDIS;

    DELAY_US(1000U);

    /*================ ADCC =================*/

    /*
     * 虽然本程序暂时不用ADCC，但07110915初始化了ADCC后能恢复，
     * 所以先保留这一步，用于完整恢复内部模拟参考系统。
     */
    ADC_setVREF(ADCC_BASE,
                ADC_REFERENCE_INTERNAL,
                ADC_REFERENCE_3_3V);

    EALLOW;

    AdccRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1U;

    AdccRegs.ADCINTFLGCLR.all = 0x000FU;
    AdccRegs.ADCINTOVFCLR.all = 0x000FU;
    AdccRegs.ADCSOCOVFCLR1.all = 0xFFFFU;

    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1U;

    EDIS;

    DELAY_US(1000U);
}

void adc_soc_init(void)
{
    EALLOW;
    // ADCA：SOC0=Ui(A0)，SOC1=Uo(A1)。
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0U;
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 9U;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 9U;    // ePWM3 SOCA
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1U;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 9U;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 9U;

    // ADCB：SOC0=iL(B0)，SOC1=本机输出电流 Io(B1)。
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0U;
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 9U;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 9U;
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1U;
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 9U;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 9U;
#if UNIT_ROLE == UNIT_1
    // Unit1 的 SOC2=Itotal(B2)；Unit2 没有第三个传感器，重复 B1 只为保持
    // ADCB 的采样时序一致，不能把 Unit2 RESULT2 当作独立总电流。
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2U;
#else
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 1U;
#endif
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 9U;
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 9U;

    // ADCB EOC2 触发中断：确保 ADCA SOC0-1 + ADCB SOC0-2 全部完成后再进 ISR。
    // 先禁用 ADCA INT1，避免误触发。
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 0U;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
    // ADCB EOC2 → ADCINT1
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 2U;
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1U;
    AdcbRegs.ADCINTSEL1N2.bit.INT1CONT = 0U;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
    EDIS;
}

void pwm_module_init(volatile struct EPWM_REGS *pwm, Uint16 master)
{
    // ePWM1 为同步主模块，ePWM2 跟随 ePWM1 相位；CMPA/CMPB 和 AQ 表均在
    // ZERO 影子装载，保证控制计算不会在一个载波周期中途改变门极真值。
    pwm->TBCTL.bit.CTRMODE = TB_FREEZE;
    pwm->TBCTL.bit.PHSEN = (master != 0U) ? TB_DISABLE : TB_ENABLE;
    pwm->TBCTL.bit.SYNCOSEL = (master != 0U) ? TB_CTR_ZERO : TB_SYNC_IN;
    pwm->TBCTL.bit.HSPCLKDIV = TB_DIV1;
    pwm->TBCTL.bit.CLKDIV = TB_DIV1;
    pwm->TBPHS.all = 0UL;
    pwm->TBCTR = 0U;
    pwm->TBPRD = PWM_TBPRD;
    pwm->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    pwm->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    pwm->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    pwm->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // AQ truth tables change between POS/NEG and the two ZC stages.  Keep
    // AQCTLA/B in shadow mode so those changes take effect at the same ZERO
    // as CMPA/B and continuous-force changes, never halfway through a cycle.
    pwm->AQCTL.all = 0U;
    pwm->AQCTL.bit.SHDWAQAMODE = 1U;
    pwm->AQCTL.bit.SHDWAQBMODE = 1U;
    pwm->AQCTL.bit.LDAQAMODE = 0U;       // load AQCTLA on ZERO
    pwm->AQCTL.bit.LDAQBMODE = 0U;       // load AQCTLB on ZERO
    pwm->CMPA.bit.CMPA = (Uint16)(PWM_TBPRD * 0.10f);
    pwm->CMPB.bit.CMPB = (Uint16)(PWM_TBPRD * 0.10f) + PWM_DEAD_TICKS;
    pwm->AQCTLA.all = 0U;
    pwm->AQCTLB.all = 0U;
    pwm->AQSFRC.bit.RLDCSF = 0U;             // continuous force loads at ZERO
    pwm->AQCSFRC.bit.CSFA = 1U;              // force low
    pwm->AQCSFRC.bit.CSFB = 1U;
    pwm->DBCTL.bit.OUT_MODE = 0U;             // dead-band permanently bypassed
    pwm->TZCTL.bit.TZA = TZ_FORCE_LO;
    pwm->TZCTL.bit.TZB = TZ_FORCE_LO;
    pwm->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
}

void pwm_init_local(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0U;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1U;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1U;
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1U;
    pwm_module_init(&EPwm1Regs, 1U);
    pwm_module_init(&EPwm2Regs, 0U);

    // ePWM3 is synchronized but never muxed to a gate pin.
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_FREEZE;
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm3Regs.TBPHS.all = 0UL;
    EPwm3Regs.TBCTR = 0U;
    EPwm3Regs.TBPRD = PWM_TBPRD;
    EPwm3Regs.CMPA.bit.CMPA = 0U;
    EPwm3Regs.ETSEL.bit.SOCAEN = 1U;
    // 与已验证 main.c 一致：CTR=CMPA 上数，CMPA=0，即 ZERO 点采样。
    EPwm3Regs.ETSEL.bit.SOCASEL = 2U;
    EPwm3Regs.ETPS.bit.SOCAPRD = ET_1ST;
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EDIS;

    pwm_force_off();           // 确保所有门极驱动信号为低
}

void cmpss_trip_init(void)
{
    // CMPSS7 同时接收 ADCINB0 的高、低电流比较结果。低阈值输出反相，高低
    // 比较器经短多数表决滤波后 OR，再由 XBAR TRIP7→DCAEVT1→ePWM OST
    // 关断四路门极；滤波不改变 ±IL_CMPSS_TRIP 阈值。
    Uint16 high_code;
    Uint16 low_code;
    float high_f = IL_ADC_OFFSET + IL_ADC_GAIN * IL_CMPSS_TRIP;
    float low_f = IL_ADC_OFFSET - IL_ADC_GAIN * IL_CMPSS_TRIP;

    high_code = (Uint16)clampf_local(high_f, 1.0f, 4094.0f);
    low_code = (Uint16)clampf_local(low_f, 1.0f, 4094.0f);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS7);
    EALLOW;
    AnalogSubsysRegs.CMPHPMXSEL.bit.CMP7HPMXSEL = 3U; // CMPSS7 HP <- ADCINB0
    AnalogSubsysRegs.CMPLPMXSEL.bit.CMP7LPMXSEL = 3U; // CMPSS7 LP <- ADCINB0
    EDIS;
    CMPSS_disableModule(CMPSS7_BASE);
    CMPSS_configHighComparator(CMPSS7_BASE, CMPSS_INSRC_DAC);
    CMPSS_configLowComparator(CMPSS7_BASE, CMPSS_INSRC_DAC | CMPSS_INV_INVERTED);
    CMPSS_configDAC(CMPSS7_BASE,
                    CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW);
    CMPSS_setDACValueHigh(CMPSS7_BASE, high_code);
    CMPSS_setDACValueLow(CMPSS7_BASE, low_code);
    CMPSS_setHysteresis(CMPSS7_BASE, 1U);
    CMPSS_enableModule(CMPSS7_BASE);
    CMPSS_configFilterHigh(CMPSS7_BASE, IL_CMPSS_FILTER_PRESCALE,
                           IL_CMPSS_FILTER_WINDOW, IL_CMPSS_FILTER_THRESHOLD);
    CMPSS_configFilterLow(CMPSS7_BASE, IL_CMPSS_FILTER_PRESCALE,
                          IL_CMPSS_FILTER_WINDOW, IL_CMPSS_FILTER_THRESHOLD);
    CMPSS_initFilterHigh(CMPSS7_BASE);
    CMPSS_initFilterLow(CMPSS7_BASE);
    CMPSS_clearFilterLatchHigh(CMPSS7_BASE);
    CMPSS_clearFilterLatchLow(CMPSS7_BASE);
    CMPSS_configOutputsHigh(CMPSS7_BASE,
                            CMPSS_TRIP_FILTER | CMPSS_TRIPOUT_FILTER);
    CMPSS_configOutputsLow(CMPSS7_BASE,
                           CMPSS_TRIP_FILTER | CMPSS_TRIPOUT_FILTER);

    XBAR_disableEPWMMux(XBAR_TRIP7, 0xFFFFFFFFUL);
    XBAR_setEPWMMuxConfig(XBAR_TRIP7, XBAR_EPWM_MUX12_CMPSS7_CTRIPH_OR_L);
    XBAR_invertEPWMSignal(XBAR_TRIP7, false);
#if DIAG_DISABLE_CMPSS_TRIP == 0U
    XBAR_enableEPWMMux(XBAR_TRIP7, XBAR_MUX12);
#else
    /*
     * 诊断模式：CMPSS 仍然工作并可读取状态，
     * 但不连接到 ePWM Trip7。
     */
    XBAR_disableEPWMMux(XBAR_TRIP7, XBAR_MUX12);
#endif

    EPWM_selectDigitalCompareTripInput(EPWM1_BASE, EPWM_DC_TRIP_TRIPIN7,
                                       EPWM_DC_TYPE_DCAH);
    EPWM_selectDigitalCompareTripInput(EPWM2_BASE, EPWM_DC_TRIP_TRIPIN7,
                                       EPWM_DC_TYPE_DCAH);
    EPWM_setTripZoneDigitalCompareEventCondition(EPWM1_BASE,
                 EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DCXH_HIGH);
    EPWM_setTripZoneDigitalCompareEventCondition(EPWM2_BASE,
                 EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DCXH_HIGH);
    EPWM_setDigitalCompareEventSource(EPWM1_BASE, EPWM_DC_MODULE_A,
                 EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
    EPWM_setDigitalCompareEventSource(EPWM2_BASE, EPWM_DC_MODULE_A,
                 EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
    EPWM_setDigitalCompareEventSyncMode(EPWM1_BASE, EPWM_DC_MODULE_A,
                 EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_NOT_SYNCED);
    EPWM_setDigitalCompareEventSyncMode(EPWM2_BASE, EPWM_DC_MODULE_A,
                 EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_NOT_SYNCED);
    EPWM_enableTripZoneSignals(EPWM1_BASE, EPWM_TZ_SIGNAL_DCAEVT1);
    EPWM_enableTripZoneSignals(EPWM2_BASE, EPWM_TZ_SIGNAL_DCAEVT1);
    EPWM_setTripZoneAction(EPWM1_BASE, EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM1_BASE, EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM2_BASE, EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM2_BASE, EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);
    pwm_force_off();           // 确保所有门极驱动信号为低
}

void controllers_init(void)
{
    memset(&pll_ui, 0, sizeof(pll_ui));
    memset(&pll_uo, 0, sizeof(pll_uo));
    memset(&uo_observer, 0, sizeof(uo_observer));
    memset(&rms_il, 0, sizeof(rms_il));
    memset(&rms_io, 0, sizeof(rms_io));
    memset(&rms_it, 0, sizeof(rms_it));
    memset(&rms_uo, 0, sizeof(rms_uo));
    memset(keys_state, 0, sizeof(keys_state));

    pll_ui.omega = TWO_PI_F * 50.0f;
    pll_ui.cos_theta = 1.0f;
    pll_ui.sin_theta = 0.0f;
    pll_ui.sin_dtheta = 0.0157073f;
    pll_ui.cos_dtheta = 0.9998766f;
    pll_ui.sin_dtheta_pending = pll_ui.sin_dtheta;
    pll_ui.cos_dtheta_pending = pll_ui.cos_dtheta;
    pll_ui.amplitude_published = 5.0f;
    pll_uo.omega = TWO_PI_F * 50.0f;
    pll_uo.cos_theta = 1.0f;
    pll_uo.sin_theta = 0.0f;
    pll_uo.sin_dtheta = 0.0157073f;
    pll_uo.cos_dtheta = 0.9998766f;
    pll_uo.sin_dtheta_pending = pll_uo.sin_dtheta;
    pll_uo.cos_dtheta_pending = pll_uo.cos_dtheta;
    pll_uo.amplitude_published = 5.0f;
    pi_id.kp = CURRENT_KP;
    pi_id.ki = CURRENT_LOOP_INTEGRAL_ENABLE ? CURRENT_KI : 0.0f;
    pi_id.out_min = -7.0f; pi_id.out_max = 7.0f;
    pi_vd.kp = VOLTAGE_KP;
    pi_vd.ki = VOLTAGE_KI;
    pi_vd.integral = 0.0f;
    pi_vd.out_min = -0.50f;
    pi_vd.out_max =  0.90f;

    pi_vq.kp = 5.0e-4f;
    pi_vq.ki = 5.0e-3f;
    pi_vq.integral = 0.0f;
    pi_vq.out_min = -0.05f;
    pi_vq.out_max =  0.05f;
    pi_sd.kp = SHARE_KP; pi_sd.ki = SHARE_KI;
    pi_sd.out_min = -I_BRANCH_CMD_MAX; pi_sd.out_max = I_BRANCH_CMD_MAX;
    pi_sq = pi_sd;

    pi_u_duty.kp = 0.004f;
    pi_u_duty.ki = 0.050f;
    pi_u_duty.integral = 0.0f;
    pi_u_duty.out_min = -0.10f;
    pi_u_duty.out_max =  0.10f;
    /* 增益在 control_update_slow 中按 u_ref_active 分段覆盖 */
}

//********** PWM 门极时序 **********//
// ePWM1A/B 对应 Q1a/Q2a，ePWM2A/B 对应 Q1b/Q2b。
// AQ 表采用 ZERO 影子装载；过零换向分为 ZC_A、ZC_B 两个载波周期。
void pwm_force_off(void)
{
    // OST 是最终硬件关断；同时把 AQCSFRC 立即置低，避免清 OST 时恢复旧的
    // PWM 图样。只有状态机满足安全条件时才允许后续重新装载 AQ。
    EPWM_forceTripZoneEvent(EPWM1_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(EPWM2_BASE, EPWM_TZ_FORCE_EVENT_OST);
    // Keep the underlying AQ state low as well.  RLDCSF=3 is the immediate
    // continuous-force load; it prevents an old active pattern from becoming
    // visible if the OST is ever cleared before the next carrier ZERO.
    EPwm1Regs.AQSFRC.bit.RLDCSF = 3U;
    EPwm2Regs.AQSFRC.bit.RLDCSF = 3U;
    EPwm1Regs.AQCSFRC.bit.CSFA = 1U;
    EPwm1Regs.AQCSFRC.bit.CSFB = 1U;
    EPwm2Regs.AQCSFRC.bit.CSFA = 1U;
    EPwm2Regs.AQCSFRC.bit.CSFB = 1U;
    gate_state = GATE_BLOCKED;
    gate_start_pending = 0U;
}

// 每 20 kHz ISR 调用：连续采样安全检查，累计 trip_clear_safe_count。
// 满足 2000 次（100 ms）连续安全后，pwm_clear_ost() 才允许清除 OST。
Uint16 trip_clear_qualify_fast(void)
{
#if POWER_STAGE_ENABLE == 1
    Uint16 raw_ok = ((meas.raw_ui >= ADC_RAW_RAIL_MIN) &&
                     (meas.raw_ui <= ADC_RAW_RAIL_MAX) &&
                     (meas.raw_uo >= ADC_RAW_RAIL_MIN) &&
                     (meas.raw_uo <= ADC_RAW_RAIL_MAX) &&
                     (meas.raw_il >= ADC_RAW_RAIL_MIN) &&
                     (meas.raw_il <= ADC_RAW_RAIL_MAX) &&
                     (meas.raw_io >= ADC_RAW_RAIL_MIN) &&
                     (meas.raw_io <= ADC_RAW_RAIL_MAX));
    Uint16 cmp_ok = (CMPSS_getStatus(CMPSS7_BASE) &
                     (CMPSS_STS_HI_FILTOUT | CMPSS_STS_LO_FILTOUT)) == 0U;
    if (!raw_ok || !cmp_ok || (fabsf(meas.il) > 0.8f * IL_CMPSS_TRIP) ||
        (pll_ui.locked == 0U))
    {
        trip_clear_safe_count = 0U;
        return 0U;
    }
    if (trip_clear_safe_count < 2000U) trip_clear_safe_count++;
    return (trip_clear_safe_count >= 2000U) ? 1U : 0U;
#else
    return 0U;
#endif
}

Uint16 pwm_clear_ost(void)
{
#if POWER_STAGE_ENABLE == 1
    if (trip_clear_safe_count < 2000U) return 0U;

    // The new AQ/CSF image was written with shadow load-on-ZERO.  Keep the
    // immediate low force in place until that image has had a carrier ZERO to
    // load, then release OST; the following cycle is the first possible gate.
    EPWM_clearOneShotTripZoneFlag(EPWM1_BASE, EPWM_TZ_OST_FLAG_DCAEVT1);
    EPWM_clearOneShotTripZoneFlag(EPWM2_BASE, EPWM_TZ_OST_FLAG_DCAEVT1);
    EPWM_clearTripZoneFlag(EPWM1_BASE, EPWM_TZ_FLAG_OST |
                           EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_INTERRUPT);
    EPWM_clearTripZoneFlag(EPWM2_BASE, EPWM_TZ_FLAG_OST |
                           EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_INTERRUPT);
    trip_clear_safe_count = 0U;
    return 1U;
#else
    // Deliberate commissioning lock: no executable path clears OST.
    pwm_force_off();           // 确保所有门极驱动信号为低
    return 0U;
#endif
}

void pwm_force_pair(volatile struct EPWM_REGS *pwm, Uint16 a_force,
                           Uint16 b_force)
{
    pwm->AQSFRC.bit.RLDCSF = 0U; // shadow-to-active at ZERO
    pwm->AQCSFRC.bit.CSFA = a_force;
    pwm->AQCSFRC.bit.CSFB = b_force;
}

void pwm_set_aq_active(volatile struct EPWM_REGS *pwm)
{
    // p: high from ZERO to CMPA(up), high again at CMPA(down).
    // pbar: low from ZERO to CMPB(up), low again at CMPB(down).
    pwm->AQCTLA.all = 0U;
    pwm->AQCTLA.bit.ZRO = AQ_SET;
    pwm->AQCTLA.bit.CAU = AQ_CLEAR;
    pwm->AQCTLA.bit.CAD = AQ_SET;
    pwm->AQCTLB.all = 0U;
    pwm->AQCTLB.bit.ZRO = AQ_CLEAR;
    pwm->AQCTLB.bit.CBU = AQ_SET;
    pwm->AQCTLB.bit.CBD = AQ_CLEAR;
    pwm_force_pair(pwm, 0U, 0U);
}

void pwm_set_aq_delayed_high(volatile struct EPWM_REGS *pwm,
                                     Uint16 output_a, Uint16 ticks)
{
    pwm->CMPA.bit.CMPA = ticks;
    pwm->CMPB.bit.CMPB = ticks;
    if (output_a != 0U)
    {
        pwm->AQCTLA.all = 0U;
        pwm->AQCTLA.bit.ZRO = AQ_CLEAR;
        pwm->AQCTLA.bit.CAU = AQ_SET;
    }
    else
    {
        pwm->AQCTLB.all = 0U;
        pwm->AQCTLB.bit.ZRO = AQ_CLEAR;
        pwm->AQCTLB.bit.CBU = AQ_SET;
    }
}

void pwm_set_normal(HalfPolarity half, float duty)
{
    // 正常运行时，一组半桥输出 PWM 互补，另一组保持导通；half 决定当前
    // 输入电压正、负半周所使用的开关组。
    Uint16 cmpa;
    Uint16 cmpb;
    duty = clampf_local(duty, PWM_D_MIN, PWM_D_MAX);
    cmpa = (Uint16)(duty * (float)PWM_TBPRD);
    cmpb = cmpa + PWM_DEAD_TICKS;
    if (cmpb >= PWM_TBPRD) cmpb = PWM_TBPRD - 1U;

    EPwm1Regs.CMPA.bit.CMPA = cmpa;
    EPwm1Regs.CMPB.bit.CMPB = cmpb;
    EPwm2Regs.CMPA.bit.CMPA = cmpa;
    EPwm2Regs.CMPB.bit.CMPB = cmpb;

    if (half == HALF_POS)
    {
        pwm_set_aq_active(&EPwm1Regs);       // Q1a=p, Q2a=pbar
        pwm_force_pair(&EPwm2Regs, 2U, 2U); // Q1b/Q2b continuously on
    }
    else
    {
        pwm_force_pair(&EPwm1Regs, 2U, 2U); // Q1a/Q2a continuously on
        pwm_set_aq_active(&EPwm2Regs);       // Q1b=p, Q2b=pbar
    }
}

void pwm_set_zc_a(HalfPolarity next_half)
{
    // ZC_A：先保持必要的回路导通，再把新半周的第一只 MOS 延迟 25 TBCLK 打开。
    // First transition period keeps both output-side MOSFETs on.  The old
    // constant input MOSFET is low at ZERO; the new one rises at Nzc.
    if (next_half == HALF_NEG)
    {
        pwm_force_pair(&EPwm1Regs, 0U, 2U);  // Q2a high; Q1a via AQ
        pwm_set_aq_delayed_high(&EPwm1Regs, 1U, PWM_ZC_TICKS);
        pwm_force_pair(&EPwm2Regs, 1U, 2U);  // Q1b low, Q2b high
    }
    else
    {
        pwm_force_pair(&EPwm1Regs, 1U, 2U);  // Q1a low, Q2a high
        pwm_force_pair(&EPwm2Regs, 0U, 2U);  // Q2b high; Q1b via AQ
        pwm_set_aq_delayed_high(&EPwm2Regs, 1U, PWM_ZC_TICKS);
    }
}

void pwm_set_zc_b(HalfPolarity next_half)
{
    // ZC_B：关断旧半周的恒导通器件，并以同样的死区切换到新半周。
    // Second transition period turns off the newly active Q2 at ZERO and
    // opens its Q1 only after the independent AQ dead interval.
    if (next_half == HALF_NEG)
    {
        pwm_force_pair(&EPwm1Regs, 2U, 2U);  // positive pair both on
        pwm_force_pair(&EPwm2Regs, 0U, 1U);  // Q2b low; Q1b via AQ
        pwm_set_aq_delayed_high(&EPwm2Regs, 1U, PWM_DEAD_TICKS);
    }
    else
    {
        pwm_force_pair(&EPwm1Regs, 0U, 1U);  // Q2a low; Q1a via AQ
        pwm_set_aq_delayed_high(&EPwm1Regs, 1U, PWM_DEAD_TICKS);
        pwm_force_pair(&EPwm2Regs, 2U, 2U);  // negative pair both on
    }
}

void request_half_change(HalfPolarity desired_half)
{
    // 只有检测到目标半周与当前半周不同才启动换向；换向期间冻结电流 PI 积分。
    if ((gate_state == GATE_ACTIVE) && (desired_half != half))
    {
        next_half = desired_half;
        gate_state = GATE_ZC_A;
        zc_stage_periods = 1U;     // 防止同一次 ISR 立即推进到 ZC_B
        pwm_set_zc_a(desired_half);
        debug_zc_a_count++;
    }
}

void gate_sequence_update(void)
{
    // ZC_A/ZC_B 各保持至少一个 ISR 周期，确保 AQ 影子寄存器在下一次
    // PWM ZERO 时生效，再提交下一阶段。
    if (((gate_state == GATE_ZC_A) ||
         (gate_state == GATE_ZC_B)) &&
        (zc_stage_periods != 0U))
    {
        zc_stage_periods--;
        return;
    }

    if (gate_state == GATE_ZC_A)
    {
        pwm_set_zc_b(next_half);
        gate_state = GATE_ZC_B;
        debug_zc_b_count++;
    }
    else if (gate_state == GATE_ZC_B)
    {
        half = next_half;
        pwm_set_normal(half, duty);
        gate_state = GATE_ACTIVE;
        debug_zc_done_count++;
    }
    else if (gate_state == GATE_ACTIVE)
    {
        pwm_set_normal(half, duty);
    }
}

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//********** ADC 采样与信号处理 **********//
// ADCA0/1：Ui/Uo；ADCB0/1/2：iL/Io/Itotal（Unit2 的 SOC2 重复 Io）。
// 所有 SOC 由 ePWM3 SOCA 同步触发，ADCA SOC1 完成后进入 20 kHz ISR。
void sample_and_calibrate(void)
{
    // 直接写入 meas 字段，避免每次 ISR 复制整个 Measurements 结构体。
    // RMS/均值字段由 signal_processing_slow 在 1 kHz 更新，此处不覆盖。
    meas.raw_ui = AdcaResultRegs.ADCRESULT0;
    meas.raw_uo = AdcaResultRegs.ADCRESULT1;
    meas.raw_il = AdcbResultRegs.ADCRESULT0;
    meas.raw_io = AdcbResultRegs.ADCRESULT1;
    meas.raw_it = AdcbResultRegs.ADCRESULT2;

    meas.ui = UI_ADC_POLARITY * (((float)meas.raw_ui - UI_ADC_OFFSET) / UI_ADC_GAIN);
    meas.uo = UO_ADC_POLARITY * (((float)meas.raw_uo - UO_ADC_OFFSET) / UO_ADC_GAIN);
    meas.il = IL_ADC_POLARITY * (((float)meas.raw_il - IL_ADC_OFFSET) / IL_ADC_GAIN);
    meas.io = IO_ADC_POLARITY * (((float)meas.raw_io - IO_ADC_OFFSET) / IO_ADC_GAIN);
#if UNIT_ROLE == UNIT_1
    meas.it = IT_ADC_POLARITY * (((float)meas.raw_it - IT_ADC_OFFSET) / IT_ADC_GAIN);
#else
    meas.it = 0.0f;
#endif
}

void signal_processing_fast(void)
{
    // 20 kHz 快速路径：SOGI 更新、PLL 相位/频率递推、dq 投影、RMS 滑动窗累加。
    // 不使用 sinf/cosf/sqrtf；dq 变换复用 PLL 缓存的正交状态。
    float ui_amp_min = DIRECTION_DIAG_8V ? 2.0f : 5.0f;
    Uint16 share_active = share_signal_processing_active();

    pll_update_fast(&pll_ui, meas.ui, ui_amp_min);
    sogi_update(&uo_observer, meas.uo, pll_ui.omega);

    if (share_active != 0U)
    {
        pll_update_fast(&pll_uo, meas.uo, 2.0f);
        dq_lpf_update(&it_dq, meas.it, pll_uo.sin_theta, pll_uo.cos_theta);
        dq_lpf_update(&io_dq, meas.io, pll_uo.sin_theta, pll_uo.cos_theta);
    }

    rms_update_fast(&rms_il, meas.il);
    rms_update_fast(&rms_io, meas.io);
    if (share_active != 0U)
        rms_update_fast(&rms_it, meas.it);
    mean_update_fast(&rms_uo, meas.uo);
}

void signal_processing_it_dq_publish_slow(void)
{
    if (share_signal_processing_active() != 0U)
        it_fund_rms = sqrtf(it_dq.d * it_dq.d +
                            it_dq.q * it_dq.q) * INV_SQRT2_F;
    else
        it_fund_rms = 0.0f;
}

void signal_processing_io_dq_publish_slow(void)
{
    if (share_signal_processing_active() != 0U)
        io_fund_rms = sqrtf(io_dq.d * io_dq.d +
                            io_dq.q * io_dq.q) * INV_SQRT2_F;
    else
        io_fund_rms = 0.0f;
}

void signal_processing_io2_dq_publish_slow(void)
{
    if (share_signal_processing_active() != 0U)
    {
        float io2_d = it_dq.d - io_dq.d;
        float io2_q = it_dq.q - io_dq.q;
        io2_fund_rms = sqrtf(io2_d * io2_d + io2_q * io2_q) *
                       INV_SQRT2_F;
    }
    else
        io2_fund_rms = 0.0f;
}

void signal_processing_il_publish_slow(void)
{
    rms_slow_update(&rms_il);
    meas.il_rms = rms_il.rms;
    meas.il_mean = rms_il.mean;
}

void signal_processing_io_publish_slow(void)
{
    rms_slow_update(&rms_io);
    meas.io_rms = rms_io.rms;
}

void signal_processing_it_publish_slow(void)
{
    if (share_signal_processing_active() != 0U)
    {
        rms_slow_update(&rms_it);
        meas.it_rms = rms_it.rms;
    }
    else
        meas.it_rms = 0.0f;
}

void signal_processing_voltage_publish_slow(void)
{
    // 本槽只有 Uo 幅值一次开方；均值发布不调用数学库。
    mean_slow_update(&rms_uo);
    meas.ui_rms = pll_ui.amplitude * INV_SQRT2_F;
    meas.uo_rms = sqrtf(uo_observer.alpha * uo_observer.alpha +
                        uo_observer.beta * uo_observer.beta) * INV_SQRT2_F;
    uo_amp_held = SQRT2_F * meas.uo_rms;
    meas.uo_mean = rms_uo.mean;
}

void software_protection_fast(void)
{
    // 20 kHz 快速路径：ADC 越界、CMPSS 状态以及 iL/Io/It/Uo 瞬时峰值。
    Uint16 raw_bad;
    Uint16 protection_active;

    raw_bad = ((meas.raw_ui < ADC_RAW_RAIL_MIN) ||
               (meas.raw_ui > ADC_RAW_RAIL_MAX) ||
               (meas.raw_uo < ADC_RAW_RAIL_MIN) ||
               (meas.raw_uo > ADC_RAW_RAIL_MAX) ||
               (meas.raw_il < ADC_RAW_RAIL_MIN) ||
               (meas.raw_il > ADC_RAW_RAIL_MAX) ||
               (meas.raw_io < ADC_RAW_RAIL_MIN) ||
               (meas.raw_io > ADC_RAW_RAIL_MAX)) ? 1U : 0U;
#if UNIT_ROLE == UNIT_1
    if ((meas.raw_it < ADC_RAW_RAIL_MIN) ||
        (meas.raw_it > ADC_RAW_RAIL_MAX)) raw_bad = 1U;
#endif
    if (raw_bad != 0U)
    {
        if (++adc_bad_count >= ADC_RAIL_BAD_LIMIT) latch_fault(FAULT_ADC);
    }
    else adc_bad_count = 0U;

#if DIAG_DISABLE_CMPSS_TRIP == 0U
    if ((EPwm1Regs.TZFLG.bit.DCAEVT1 != 0U) ||
        (EPwm2Regs.TZFLG.bit.DCAEVT1 != 0U))
    {
        debug_il_fault_source =
            ((EPwm1Regs.TZFLG.bit.DCAEVT1 != 0U) ? 1U : 0U) |
            ((EPwm2Regs.TZFLG.bit.DCAEVT1 != 0U) ? 2U : 0U);
        debug_cmpss_status_at_fault = Cmpss7Regs.COMPSTS.all;
        debug_epwm1_tz_at_fault = EPwm1Regs.TZFLG.all;
        debug_epwm2_tz_at_fault = EPwm2Regs.TZFLG.all;
        latch_fault(FAULT_IL_PK);
    }
#endif

    protection_active = ((run_state == ST_SS5_RAMP) ||
                         (run_state == ST_SS5_HOLD) ||
                         (run_state == ST_VOLT_RAMP) ||
                         (run_state == ST_SHARE_RAMP) ||
                         (run_state == ST_RUN) ||
                         (run_state == ST_STOP_RAMP)) ? 1U : 0U;
    if (protection_active == 0U)
    {
        il_fast_count = 0U;
        io_pk_count = 0U;
        it_pk_count = 0U;
        uo_pk_count = 0U;
        return;
    }

#if DIAG_DISABLE_IL_SW_TRIP == 0U
    // iL 瞬时峰值单采样锁存；其余支路峰值按设计连续确认以抑制毛刺。
    if (fabsf(meas.il) >= IL_SW_FAST_LIMIT)
    {
        debug_il_fault_source = 4U;
        if (++il_fast_count >= 1U) latch_fault(FAULT_IL_PK);
    }
    else il_fast_count = 0U;
#endif

    if (fabsf(meas.io) >= IO_PK_TRIP)
    {
        if (++io_pk_count >= 5U) latch_fault(FAULT_IO);
    }
    else io_pk_count = 0U;

#if UNIT_ROLE == UNIT_1
    if (fabsf(meas.it) >= IT_PK_TRIP)
    {
        if (++it_pk_count >= 5U) latch_fault(FAULT_IT);
    }
    else it_pk_count = 0U;
#endif

    if (fabsf(meas.uo) >= UO_ABS_PK_TRIP)
    {
        if (++uo_pk_count >= 3U) latch_fault(FAULT_UO_OV);
    }
    else uo_pk_count = 0U;
}

void software_protection_slow(void)
{
    // 1 kHz 慢速路径：RMS 过流、动态过压、输入窗口、PLL 失锁、母线丢失、
    // 直流偏置。这些保护依赖已发布的 RMS/均值，1 kHz 更新率足够。
    float dynamic_ov;

    if (!((run_state == ST_SS5_RAMP) ||
          (run_state == ST_SS5_HOLD) ||
          (run_state == ST_VOLT_RAMP) ||
          (run_state == ST_SHARE_RAMP) ||
          (run_state == ST_RUN) ||
          (run_state == ST_STOP_RAMP))) return;

    if (meas.il_rms >= IL_RMS_TRIP)
    {
        if (++il_rms_count >= 40U) latch_fault(FAULT_IL_RMS);
    }
    else il_rms_count = 0U;
    if (meas.io_rms >= IO_RMS_TRIP)
    {
        if (++io_rms_count >= 40U) latch_fault(FAULT_IO);
    }
    else io_rms_count = 0U;
#if UNIT_ROLE == UNIT_1
    if (meas.it_rms >= IT_RMS_TRIP)
    {
        if (++it_rms_count >= 40U) latch_fault(FAULT_IT);
    }
    else it_rms_count = 0U;
#endif

    dynamic_ov = u_ref_active +
                 fmaxf(1.5f, 0.20f * u_ref_active);

    dynamic_ov = fminf(dynamic_ov, UO_RMS_HARD_MAX);

    if (is_voltage_role() == 0U)
    {
        dynamic_ov = UO_RMS_HARD_MAX;
    }

    /*
     * 启动爬升阶段不能让保护线跟着尚未爬升完成的
     * u_ref_active降得过低，应参考最终目标值设置下限。
     */
    if (run_state == ST_VOLT_RAMP)
    {
        float ramp_ov_min = 1.30f * effective_u_ref();

        dynamic_ov = fmaxf(dynamic_ov, ramp_ov_min);
    }

    if ((run_state == ST_SS5_RAMP) ||
        (run_state == ST_SS5_HOLD))
    {
        dynamic_ov = 7.0f;
    }

    // 状态机只可抬高动态资格线，最终保护门限绝不能超过RMS硬上限。
    dynamic_ov = fminf(dynamic_ov, UO_RMS_HARD_MAX);

    debug_dynamic_ov = dynamic_ov;
    debug_u_ref_active = u_ref_active;

    /* 连续超过10 ms才确认 */
    if (meas.uo_rms > dynamic_ov)
    {
        if (++uo_rms_ov_count >= 10U)
        {
            latch_fault(FAULT_UO_OV);
        }
    }
    else
    {
        uo_rms_ov_count = 0U;
    }

    if (input_run_ok() == 0U)
    {
        if (++ui_bad_count >= 60U) latch_fault(FAULT_UI);
    }
    else ui_bad_count = 0U;

    if ((pll_ui.omega < TWO_PI_F * 46.0f) ||
        (pll_ui.omega > TWO_PI_F * 54.0f) || (pll_ui.locked == 0U))
    {
        if (++pll_bad_count >= 60U) latch_fault(FAULT_PLL);
    }
    else pll_bad_count = 0U;

    // 母线丢失保护：并联模式下母线电压低于 3.0 V 连续 5 ms 即锁故障。
    if (is_share_role() && bus_valid &&
        (run_state == ST_SHARE_RAMP || run_state == ST_RUN))
    {
        if (meas.uo_rms < 3.0f)
        {
            if (++bus_lost_count >= 5U) latch_fault(FAULT_BUS_LOST);
        }
        else bus_lost_count = 0U;
    }
    else bus_lost_count = 0U;

    // 直流偏置保护：iL/uo 均值偏离零超过阈值连续 100 ms 即锁故障。
    if ((fabsf(meas.il_mean) > 0.30f) || (fabsf(meas.uo_mean) > 0.50f))
    {
        if (++dc_bad_count >= 100U) latch_fault(FAULT_DC_OFFSET);
    }
    else dc_bad_count = 0U;
}

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

// ADCA INT1：SOC1 完成后读取两路电压和三路 B 组电流结果。
// 双速率架构：20 kHz 快速路径每周期执行；1 kHz 慢速路径每 20 周期执行。
// 快速路径仅含 ADC 标定、SOGI/PLL 相位递推、dq 投影、RMS 累加、
// 瞬时峰值保护、电流内环和门极提交——不调用 sinf/cosf/sqrtf。
__interrupt void adcB1ISR(void)
{
    GpioDataRegs.GPASET.bit.GPIO6 = 1U;  // ISR 脉宽测量：入口置高

    // ---- 20 kHz 快速路径 ----
    sample_and_calibrate();
    signal_processing_fast();
    software_protection_fast();
    trip_clear_qualify_fast();

    // ---- 1 kHz 慢速路径：每个载波槽最多安排一个重型数学调用 ----
    isr_heartbeat++;
    debug_isr_slot = sample_div;
    switch (debug_isr_slot)
    {
        case 0U:
            pll_slow_amplitude_update(&pll_ui,
                                      DIRECTION_DIAG_8V ? 2.0f : 5.0f);
            if (tick_1ms < 100U) tick_1ms++;
            if (++tick_div_5ms >= 5U) {
                tick_div_5ms = 0U;
                if (tick_5ms < 20U) tick_5ms++;
            }
            if (++tick_div_100ms >= 100U) {
                tick_div_100ms = 0U;
                if (tick_100ms < 4U) tick_100ms++;
            }
            break;
        case 1U:
            if (share_signal_processing_active() != 0U)
                pll_slow_amplitude_update(&pll_uo, 2.0f);
            break;
        case 2U:
            pll_slow_normalize_update(&pll_ui);
            break;
        case 3U:
            if (share_signal_processing_active() != 0U)
                pll_slow_normalize_update(&pll_uo);
            break;
        case 4U:
            pll_slow_theta_update(&pll_ui);
            break;
        case 5U:
            if (share_signal_processing_active() != 0U)
                pll_slow_theta_update(&pll_uo);
            break;
        case 6U:
            software_protection_slow();
            break;
        case 7U:
            pll_slow_sin_dtheta_update(&pll_ui);
            break;
        case 8U:
            if (share_signal_processing_active() != 0U)
                pll_slow_sin_dtheta_update(&pll_uo);
            break;
        case 9U:
            pll_slow_cos_dtheta_update(&pll_ui);
            break;
        case 10U:
            if (share_signal_processing_active() != 0U)
                pll_slow_cos_dtheta_update(&pll_uo);
            break;
        case 11U:
            signal_processing_it_dq_publish_slow();
            break;
        case 12U:
            signal_processing_io_dq_publish_slow();
            break;
        case 13U:
            signal_processing_io2_dq_publish_slow();
            break;
        case 14U:
            control_update_slow();
            break;
        case 15U:
            signal_processing_il_publish_slow();
            break;
        case 16U:
            signal_processing_io_publish_slow();
            break;
        case 17U:
            signal_processing_it_publish_slow();
            break;
        case 18U:
            signal_processing_voltage_publish_slow();
            break;
        case 19U:
            // 无数学库调用：一次性提交两套PLL的新旋转系数。
            pll_slow_commit_dtheta_update(&pll_ui);
            if (share_signal_processing_active() != 0U)
                pll_slow_commit_dtheta_update(&pll_uo);
            break;
        default:
            break;
    }
    if (++sample_div >= 20U) sample_div = 0U;

    // ---- 20 kHz 快速路径（续）：电流内环 + 门极 shadow 提交 ----
    if (run_state != ST_FAULT)
    {
        control_update_fast();
        gate_sequence_update();
    }

    // ADCINT 溢出检查：ADCB EOC2 触发，检查 ADCB 的溢出标志。
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    if (AdcbRegs.ADCINTOVF.bit.ADCINT1 != 0U)
    {
        debug_adc_ovf_count++;
        debug_adc_ovf_slot = debug_isr_slot;
        AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
        latch_fault(FAULT_CPU);
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1U;  // ISR 脉宽测量：出口置低
}

// 外环：电压或分流；前馈：Co 电流前馈 — 1 kHz 慢速路径。
//********** 外环控制 (1kHz) **********//
void control_update_slow(void)
{
#if GATE_OPEN_LOOP_TEST
    if (run_state == ST_GATE_TEST) return;
#endif
    // 1 kHz 慢速路径：dq 变换、电压/分流外环 PI、Co 电流前馈。
    // 结果 iconv_ref_held / dff_held 保持 20 个快速周期供电流内环使用。
    // sin/cos 复用 PLL 缓存的正交状态，不调用 sinf/cosf。
#if !DIRECT_VOLTAGE_DUTY_TEST
    float sn = pll_ui.sin_theta;
    float cs = pll_ui.cos_theta;
#endif
    float ui_amp = fmaxf(pll_ui.amplitude, 1.0f);
    float uo_amp = uo_amp_held;
    float dff;
    float iconv_ref = 0.0f;
    Uint16 ctrl_active;

    // 稳压角色使用目标电压计算前馈占空比，避免实际 Uo 超调后前馈增大
    // 形成正反馈维持过高输出。
    if (is_voltage_role() != 0U)
    {
        float target_amp = SQRT2_F * u_ref_active;
        dff = target_amp / fmaxf(ui_amp + target_amp, 1.0f);
    }
    else
    {
        dff = uo_amp / fmaxf(ui_amp + uo_amp, 1.0f);
    }
    dff = clampf_local(dff, PWM_D_MIN, PWM_D_MAX);
    dff_target = dff;

    ctrl_active = ((gate_state == GATE_ACTIVE) &&
                   (run_state == ST_VOLT_RAMP ||
                    (run_state == ST_SS5_RAMP) ||
                    (run_state == ST_SS5_HOLD) ||
                    (run_state == ST_SHARE_RAMP) ||
                    (run_state == ST_RUN) ||
                    (run_state == ST_STOP_RAMP))) ? 1U : 0U;

    // 稳压角色
    if (is_voltage_role() != 0U)
    {
#if !DIRECT_VOLTAGE_DUTY_TEST
        float target_u = SQRT2_F * u_ref_active;
#endif

#if DIRECT_VOLTAGE_DUTY_TEST

        {
            float voltage_error;
            float duty_correction;
            float duty_io_ff;
            float io_rms_used;
            Uint16 integrate_enable;

            /* 根据参考电压分段设置 PI 增益 */
            if (u_ref_active < 8.0f)
            {
                pi_u_duty.kp = 0.0030f;
                pi_u_duty.ki = 0.040f;
            }
            else if (u_ref_active < 20.0f)
            {
                pi_u_duty.kp = 0.0030f;
                pi_u_duty.ki = 0.035f;
            }
            else
            {
                pi_u_duty.kp = 0.0030f;
                pi_u_duty.ki = 0.025f;
            }

            /* RMS 误差：输出低于参考时误差为正 */
            voltage_error = u_ref_active - meas.uo_rms;

            /* 过压或限流时暂停积分，防止 windup */
            integrate_enable =
                ctrl_active &&
                (current_limit_active == 0U) &&
                (meas.io_rms < 2.05f);

            duty_correction =
                pi_update_dt(&pi_u_duty,
                             voltage_error,
                             integrate_enable,
                             0.001f);

            /* 负载电流前馈：负载增大时提前增加占空比 */
            io_rms_used = clampf_local(meas.io_rms, 0.0f, 2.10f);
            duty_io_ff = DUTY_IO_FF_GAIN * io_rms_used;

            direct_duty_target =
                clampf_local(dff_target +
                             duty_correction +
                             duty_io_ff,
                             PWM_D_MIN,
                             PWM_D_MAX);

            /* 旁路电流指令 */
            iconv_ref = 0.0f;
        }

#else

        {
            float v_alpha = uo_observer.alpha;
            float v_beta  = uo_observer.beta;
            float vd = v_alpha * sn - v_beta * cs;
            float vq = v_alpha * cs + v_beta * sn;
            float id_corr;
            float iq_corr;
            float duo_ref;

            id_corr = pi_update_dt(&pi_vd, target_u - vd, ctrl_active, 0.001f);
            iq_corr = pi_update_dt(&pi_vq, -vq, ctrl_active, 0.001f);
            debug_uo_d = vd;
            debug_uo_q = vq;
            debug_vd_error = target_u - vd;
            debug_vq_error = -vq;
            duo_ref = -target_u * pll_ui.omega * cs;
            iconv_ref = io_ff + CO_FARAD * duo_ref -
                        (id_corr * sn + iq_corr * cs);
        }

#endif
    }
    // 分流角色：Unit1 测量总电流和本机电流，在 Uo PLL dq 坐标系下调节 Io。
    else if (is_share_role() != 0U)
    {
        float so = pll_uo.sin_theta;
        float co = pll_uo.cos_theta;
        float it_rms = it_fund_rms;
        float kmin_safe;
        float kmax_safe;
        float alpha;
        float target_d;
        float target_q;
        float corr_d;
        float corr_q;
        float k_meas;

        kmin_safe = fmaxf(0.5f, it_rms / I_BRANCH_CMD_MAX - 1.0f);
        if (it_rms > I_BRANCH_CMD_MAX)
            kmax_safe = fminf(2.0f, I_BRANCH_CMD_MAX /
                              fmaxf(it_rms - I_BRANCH_CMD_MAX, 0.01f));
        else
            kmax_safe = 2.0f;

        if (kmin_safe > kmax_safe)
        {
            stop_request = 1U;
            k_limited = 1U;
            k_active = 1.0f;
        }
        else
        {
            float requested = clampf_local(k_cmd, kmin_safe, kmax_safe);
            k_limited = (fabsf(requested - k_cmd) > 0.001f) ? 1U : 0U;
            k_active = slew(k_active, requested, 0.5f * 0.001f);
        }

        alpha = (k_active / (1.0f + k_active)) * share_alpha_ramp;
        target_d = alpha * it_dq.d;
        target_q = alpha * it_dq.q;
        corr_d = pi_update_dt(&pi_sd, target_d - io_dq.d, ctrl_active, 0.001f);
        corr_q = pi_update_dt(&pi_sq, target_q - io_dq.q, ctrl_active, 0.001f);
        iconv_ref = target_d * so + target_q * co +
                    CO_FARAD * uo_observer.dalpha +
                    corr_d * so + corr_q * co;

        k_meas = io_fund_rms / fmaxf(io2_fund_rms, 0.02f);
        share_error_pct = 100.0f * fabsf(k_active - k_meas) /
                            fmaxf(k_active, 0.01f);
    }
    else
    {
        pi_reset(&pi_vd); pi_reset(&pi_vq);
        pi_reset(&pi_sd); pi_reset(&pi_sq);
        pi_reset(&pi_u_duty);
        direct_duty_target = PWM_D_MIN;
    }

    iconv_ref = clampf_local(iconv_ref, -SQRT2_F * I_BRANCH_CMD_MAX,
                             SQRT2_F * I_BRANCH_CMD_MAX);
    iconv_ref_target = iconv_ref;
}

// 内环：电感电流 PI — 20 kHz 快速路径。
//********** 电流内环 (20kHz) **********//
void control_update_fast(void)
{
    // 20 kHz 快速路径：将保持的 iconv_ref 映射为电感电流参考，经斜率限制后
    // 通过电流内环 PI 生成占空比，同时处理过零换向和门极启动。
    // 不使用 sinf/cosf/sqrtf；dff 和 iconv_ref 来自 1 kHz 慢速路径。
    /*
     * 将1 kHz外环输出平滑插值到20 kHz，
     * 避免每1 ms出现一次明显阶跃。
     */
    dff_held = slew(dff_held, dff_target, 0.0030f);
    iconv_ref_held = slew(iconv_ref_held, iconv_ref_target, 0.050f);

    float il_ref_target;
    float il_ref_dot;
    float error;
    float v_pi;
    float denom;
    float qsign;
    float d_unsat;
    float dff = dff_held;
    float iconv_ref = iconv_ref_held;
    Uint16 ctrl_active;

    io_ff = IO_FF_A * io_ff + (1.0f - IO_FF_A) * meas.io;

    ctrl_active = ((gate_state == GATE_ACTIVE) &&
                   (run_state == ST_VOLT_RAMP ||
                    (run_state == ST_SS5_RAMP) ||
                    (run_state == ST_SS5_HOLD) ||
                    (run_state == ST_SHARE_RAMP) ||
                    (run_state == ST_RUN) ||
                    (run_state == ST_STOP_RAMP))) ? 1U : 0U;

    {
        float div = 1.0f - dff;
        if (div < 0.15f) div = 0.15f;
        il_ref_target = IL_TO_OUTPUT_SIGN * iconv_ref / div;
    }
    il_ref_target = clampf_local(il_ref_target, -IL_REF_PK_MAX,
                                 IL_REF_PK_MAX);
#if DIRECT_VOLTAGE_DUTY_TEST

    if (is_voltage_role() != 0U)
    {
        /*
         * 直接电压控制只用于稳压角色调试。Unit1 Model2 是分流角色，
         * 必须继续执行下方的电流内环，不能被该调试开关旁路。
         */
        duty = slew(duty, direct_duty_target, 0.0002f);
        duty = clampf_local(duty, PWM_D_MIN, PWM_D_MAX);

        /* 稳压角色调试时电流参考和电流PI退出控制 */
        il_ref = 0.0f;
        il_ref_prev = 0.0f;
        il_ref_target = 0.0f;
        pi_reset(&pi_id);
    }
    else
#endif
    {
        il_ref = slew(il_ref_prev, il_ref_target, 0.125f);
        il_ref_dot = (il_ref - il_ref_prev) / CONTROL_TS;
        il_ref_prev = il_ref;

        if (ctrl_active != 0U)
        {
            error = il_ref - meas.il;
            v_pi = CURRENT_KP * error + pi_id.integral;
            denom = fabsf(meas.ui) + fabsf(meas.uo);
            if (denom < 10.0f) denom = 10.0f;
            qsign = (half == HALF_POS) ? 1.0f : -1.0f;
            d_unsat = dff + qsign * (L_HENRY * il_ref_dot + v_pi) / denom;
            duty = clampf_local(d_unsat, PWM_D_MIN, PWM_D_MAX);

            if (((d_unsat >= PWM_D_MIN) && (d_unsat <= PWM_D_MAX)) ||
                ((d_unsat > PWM_D_MAX) && (qsign * error < 0.0f)) ||
                ((d_unsat < PWM_D_MIN) && (qsign * error > 0.0f)))
            {
                pi_id.integral += pi_id.ki * CONTROL_TS * error;
                pi_id.integral = clampf_local(pi_id.integral, -5.0f, 5.0f);
            }
        }
        else
        {
#if GATE_OPEN_LOOP_TEST
            duty = (run_state == ST_GATE_TEST) ? OPEN_LOOP_DUTY : dff;
#else
            duty = dff;
#endif
            // ZC_A/ZC_B期间功率开关由换向序列接管，只冻结积分；真正
            // 关闸或退出控制时才清零，避免每半周丢失稳态电流补偿。
            if ((gate_state != GATE_ZC_A) &&
                (gate_state != GATE_ZC_B))
                pi_reset(&pi_id);
        }
    }

    // ---- 半周换向：带进入/退出迟滞 + 预测补偿 + 过零电流收缩 ----
    // 进入零区不立即换向，明确离开零区时才切换。
    // 使用 SOGI dalpha 预测 75µs 后的过零点，补偿换向延迟。
    // 在过零附近收缩电流参考，减轻换向期间输出电压折点。
    {
        float alpha_now;
        float alpha_zc;
        float zc_abs;
        float zc_scale;
         Uint16 request_pos = 0U;
        Uint16 request_neg = 0U;

        alpha_now = pll_ui.sogi.alpha;
        alpha_zc = alpha_now +
                   ZC_ADVANCE_SEC * pll_ui.sogi.dalpha;

        if (zc_block_count > 0U) zc_block_count--;

        if (zc_region == ZC_REGION_POS)
        {
            if (alpha_zc < ZC_ENTER_V)
                zc_region = ZC_REGION_ZERO;
        }
        else if (zc_region == ZC_REGION_NEG)
        {
            if (alpha_zc > -ZC_ENTER_V)
                zc_region = ZC_REGION_ZERO;
        }
        else
        {
            if (alpha_zc > ZC_EXIT_V)
            {
                if ((half != HALF_POS) && (zc_block_count == 0U))
                    request_pos = 1U;
                zc_region = ZC_REGION_POS;
            }
            else if (alpha_zc < -ZC_EXIT_V)
            {
                if ((half != HALF_NEG) && (zc_block_count == 0U))
                    request_neg = 1U;
                zc_region = ZC_REGION_NEG;
            }
        }

        if ((gate_state == GATE_ACTIVE) && (pll_ui.locked != 0U))
        {
            if (request_pos != 0U)
            {
                request_half_change(HALF_POS);
                zc_block_count = ZC_BLOCK_SAMPLES;
                debug_half_change_count++;
            }
            else if (request_neg != 0U)
            {
                request_half_change(HALF_NEG);
                zc_block_count = ZC_BLOCK_SAMPLES;
                debug_half_change_count++;
            }
        }

        if ((gate_start_pending != 0U) && (fault_code == FAULT_OK) &&
            ((request_pos != 0U) || (request_neg != 0U)))
        {
            half = (alpha_zc >= 0.0f) ? HALF_POS : HALF_NEG;
            if (is_share_role() != 0U)
            {
                next_half = half;
                pwm_set_zc_a(half);
                if (pwm_clear_ost() == 0U) return;
#if POWER_STAGE_ENABLE == 1
                gate_state = GATE_ZC_A;
#endif
            }
            else
            {
                pwm_set_normal(half, duty);
                if (pwm_clear_ost() == 0U) return;
#if POWER_STAGE_ENABLE == 1
                gate_state = GATE_ACTIVE;
#endif
            }
            gate_start_pending = 0U;
            zc_block_count = ZC_BLOCK_SAMPLES;
        }

        // 过零电流参考收缩：减轻换向期间输出电压折点
        zc_abs = fabsf(alpha_zc);
        if (zc_abs < ZC_CURRENT_FADE_V)
        {
            zc_scale = zc_abs / ZC_CURRENT_FADE_V;
            zc_scale = clampf_local(zc_scale, 0.0f, 1.0f);
            il_ref_target *= zc_scale;
        }

        ui_alpha_prev = alpha_now;
    }
}

//********** 运行状态机 **********//
// SAFE→PLL→软启动→电压/分流爬升→RUN；任意故障进入 FAULT 并保持 OST。
Uint16 voltage_current_limit_1ms(void)
{
    // Io RMS 达到支路上限即进入限流；降到 92% 以下并保持 40 ms 才释放，
    // 连续 500 ms 仍不能脱离限流则锁定启动/运行故障。
    if (meas.io_rms >= I_BRANCH_CMD_MAX)
    {
        current_limit_active = 1U;
        current_release_ms = 0U;
    }
    else if (current_limit_active != 0U)
    {
        if (meas.io_rms <= 0.92f * I_BRANCH_CMD_MAX)
        {
            if (current_release_ms < 40U) current_release_ms++;
            if (current_release_ms >= 40U)
            {
                current_limit_active = 0U;
                current_limit_ms = 0U;
            }
        }
        else
        {
            current_release_ms = 0U;
        }
    }

    if (current_limit_active != 0U)
    {
        if (current_limit_ms < 501U) current_limit_ms++;
        if (current_limit_ms >= 500U) latch_fault(FAULT_START);
    }
    else
    {
        current_limit_ms = 0U;
    }
    return current_limit_active;
}

void slow_state_machine_1ms(void)
{
    float target;
    state_ms++;

    // 故障态优先级最高：持续 OST，只有人工清除请求且安全条件连续满足才回 SAFE。
    if (fault_code != FAULT_OK)
    {
        run_state = ST_FAULT;
        pwm_force_off();           // 确保所有门极驱动信号为低
        if ((clear_request != 0U) && fault_clear_conditions())
        {
            fault_code = FAULT_OK;
            warning_code = FAULT_OK;
            clear_request = 0U;
            fault_clear_safe_ms = 0U;
            run_state = ST_SAFE;       // 初始化完成，进入安全待机
            model_cmd = MODEL_SAFE;
            model_active = MODEL_SAFE;
            state_ms = 0UL;
        }
        return;
    }

    if ((stop_request != 0U) &&
        (run_state != ST_SAFE) &&
        (run_state != ST_FAULT))
    {
        stop_request = 0U;
        start_request = 0U;
        gate_start_pending = 0U;

        u_ref_active = 0.0f;
        iconv_ref_held = 0.0f;
        iconv_ref_target = 0.0f;
        il_ref = 0.0f;
        il_ref_prev = 0.0f;

        pi_reset(&pi_id);
        pi_reset(&pi_vd);
        pi_reset(&pi_vq);
        pi_reset(&pi_sd);
        pi_reset(&pi_sq);
        pi_reset(&pi_u_duty);

        pwm_force_off();

        run_state = ST_DISCHARGE;
        state_ms = 0UL;
    }

#if POWER_STAGE_ENABLE == 0
    // First build is instrumentation-only.  Model selection remains visible,
    // but a start gesture cannot leave SAFE and cannot clear the OST latch.
    pwm_force_off();           // 确保所有门极驱动信号为低
    run_state = ST_SAFE;       // 初始化完成，进入安全待机
    model_active = MODEL_SAFE;
    start_request = 0U;
    stop_request = 0U;
    state_ms = 0UL;
    return;
#endif

    switch (run_state)
    {
        case ST_SAFE:
            // SAFE：门极关闭、参考值清零；按键选择的模式只有在这里才锁存为活动模式。
            pwm_force_off();           // 确保所有门极驱动信号为低
            model_active = MODEL_SAFE;
            u_ref_active = 0.0f;
            share_alpha_ramp = 0.0f;
            bus_valid = 0U;
            ss5_stable_ms = 0U;
            output_pll_stable_ms = 0U;
            voltage_stable_ms = 0U;
            voltage_error_ms = 0U;
            open_loop_test_ms = 0U;
            current_limit_ms = 0U;
            current_release_ms = 0U;
            current_limit_active = 0U;
            if (start_request != 0U)
            {
                start_request = 0U;
                if (model_is_legal(model_cmd) == 0U)
                {
                    warning_code = FAULT_MODE;
                    model_cmd = MODEL_SAFE;
                }
                else if (model_cmd != MODEL_SAFE)
                {
                    model_active = model_cmd;
                    run_state = ST_PRECHECK;
                    state_ms = 0UL;
                }
            }
            break;

        case ST_PRECHECK:
            // PRECHECK：确认输入范围和死母线，再清 PI 状态并进入输入 PLL 锁定。
            if (!dead_bus() || !input_start_ok())
            {
                if (state_ms > 2000UL) latch_fault(FAULT_START);
                break;
            }
            pi_reset(&pi_id); pi_reset(&pi_vd); pi_reset(&pi_vq);
            pi_reset(&pi_sd); pi_reset(&pi_sq);
            pi_reset(&pi_u_duty);
            direct_duty_target = PWM_D_MIN;
#if GATE_OPEN_LOOP_TEST
            run_state = ST_GATE_TEST;
#else
            run_state = ST_INPUT_PLL;
#endif
            state_ms = 0UL;
            break;

        case ST_INPUT_PLL:
            // INPUT_PLL：等待输入 PLL 锁定；Unit1 分流模式等待活母线，Unit2 稳压
            // 模式先执行 5 V 软启动，单机电压模式直接进入电压爬升。
            if (pll_ui.locked != 0U)
            {
                if (is_share_role() != 0U)
                {
                    run_state = ST_WAIT_UO;
                    reset_share_signal_processing();
                    state_ms = 0UL;
                }
#if UNIT_ROLE == UNIT_2
                else if (model_active == MODEL_PARALLEL)
                    run_state = ST_SS5_RAMP;
#endif
                else
                {
                    pi_reset(&pi_vd);
                    pi_reset(&pi_vq);
                    pi_reset(&pi_u_duty);
                    pi_vd.integral = 0.20f;
                    pi_vq.integral = 0.0f;
                    direct_duty_target = PWM_D_MIN;
                    run_state = ST_VOLT_RAMP;
                    state_ms = 0UL;
                }
            }
            else if (state_ms > 2000UL) latch_fault(FAULT_PLL);
            break;

        case ST_WAIT_UO:
            // WAIT_UO：分流机保持关闸，Uo达到1 V即提前启动输出PLL。
            // 这里只开始观测和锁相；并机接入仍要求4.5~5.5 V平台资格。
            if (meas.uo_rms >= 1.0f)
            {
                output_pll_stable_ms = 0U;
                run_state = ST_OUTPUT_PLL;
                state_ms = 0UL;
            }
            break;

        case ST_OUTPUT_PLL:
        {
            // OUTPUT_PLL：保持关闸等待宽松接入资格。电压上限跟随当前目标，
            // 避免Unit2离开5 V平台后Unit1因固定窗口反复清零或触发F10。
            float qualify_uo_max = effective_u_ref() + 2.0f;

            if (meas.uo_rms < 0.5f)
            {
                output_pll_stable_ms = 0U;
                run_state = ST_WAIT_UO;
                reset_share_signal_processing();
                state_ms = 0UL;
            }
            else if ((meas.uo_rms >= 3.0f) && (meas.uo_rms <= qualify_uo_max) &&
                (pll_uo.omega >= TWO_PI_F * 48.0f) &&
                (pll_uo.omega <= TWO_PI_F * 52.0f) &&
                (pll_uo.locked != 0U) &&
                (fabsf(wrap_pi(pll_uo.theta - pll_ui.theta)) <
                 (20.0f * PI_F / 180.0f)))
            {
                if (output_pll_stable_ms < 50U) output_pll_stable_ms++;
                if (output_pll_stable_ms >= 50U)
                {
                    bus_valid = 1U;
                    share_alpha_ramp = 0.0f;
                    gate_start_pending = 1U;
                    run_state = ST_SHARE_RAMP;
                    state_ms = 0UL;
                }
            }
            else output_pll_stable_ms = 0U;
            break;
        }

        case ST_SS5_RAMP:
            // SS5_RAMP：Unit2 先把参考值从 0 缓慢升到 5 V；达到约 1 V 后才申请开闸，
            // 防止零参考下直接施加不确定的占空比。
            if (voltage_current_limit_1ms() != 0U)
                u_ref_active = slew(u_ref_active, 0.0f, 0.005f);
            else
                u_ref_active = slew(u_ref_active, 5.0f, 0.050f);
            if ((u_ref_active >= 1.0f) && (gate_state == GATE_BLOCKED))
                gate_start_pending = 1U;
            if ((u_ref_active >= 4.99f) && (meas.uo_rms >= 4.5f) &&
                (meas.uo_rms <= 5.5f) && (current_limit_active == 0U))
            {
                if (ss5_stable_ms < 100U) ss5_stable_ms++;
            }
            else ss5_stable_ms = 0U;
            if (ss5_stable_ms >= 100U)
            {
                run_state = ST_SS5_HOLD;
                state_ms = 0UL;
                ss5_stable_ms = 0U;
            }
            else if (state_ms > 1500UL) latch_fault(FAULT_START);
            break;

        case ST_SS5_HOLD:
            // SS5_HOLD：5 V 实际输出需连续稳定 500 ms，确认主机已经建立可接入母线。
            if (voltage_current_limit_1ms() != 0U)
                u_ref_active = slew(u_ref_active, 0.0f, 0.005f);
            else
                u_ref_active = slew(u_ref_active, 5.0f, 0.005f);
            if ((meas.uo_rms >= 4.5f) && (meas.uo_rms <= 5.5f) &&
                (current_limit_active == 0U))
            {
                if (ss5_stable_ms < 500U) ss5_stable_ms++;
            }
            else ss5_stable_ms = 0U;
            if (ss5_stable_ms >= 500U)
            {
                pi_reset(&pi_vd);
                pi_reset(&pi_vq);
                pi_reset(&pi_u_duty);
                pi_vd.integral = 0.20f;
                pi_vq.integral = 0.0f;
                direct_duty_target = PWM_D_MIN;
                run_state = ST_VOLT_RAMP;
                state_ms = 0UL;
                voltage_stable_ms = 0U;
            }
            else if (state_ms > 1500UL) latch_fault(FAULT_START);
            break;

        case ST_VOLT_RAMP:
            // VOLT_RAMP：从 1 V 起按约 1 s 斜坡到目标值；低端/高端微调也在此阶段生效。
            target = effective_u_ref();
            if (voltage_current_limit_1ms() != 0U)
            {
                u_ref_active = slew(u_ref_active, 0.0f, 0.005f);
            }
            else
            {
                float ramp_target = ((target < 1.0f) &&
                                     (gate_state == GATE_BLOCKED)) ? 1.0f : target;
#if UNIT_ROLE == UNIT_2
                float ramp_span = (model_active == MODEL_PARALLEL) ?
                                  fmaxf(target - 5.0f, 1.0f) : fmaxf(target, 1.0f);
#else
                float ramp_span = fmaxf(target, 1.0f);
#endif
                u_ref_active = slew(u_ref_active, ramp_target,
                                      ramp_span / 125.0f);
            }
            if ((u_ref_active >= 1.0f) && (gate_state == GATE_BLOCKED))
                gate_start_pending = 1U;
            if ((fabsf(u_ref_active - target) < 0.01f) &&
                (fabsf(meas.uo_rms - target) <=
                 fmaxf(0.5f, 0.02f * target)))
            {
                if (voltage_stable_ms < 100U) voltage_stable_ms++;
            }
            else voltage_stable_ms = 0U;
            if ((state_ms >= 1000UL) && (voltage_stable_ms >= 100U))
            {
                run_state = ST_RUN;
                state_ms = 0UL;
                voltage_error_ms = 0U;
            }
            else if (state_ms > 2000UL) latch_fault(FAULT_START);
            break;

        case ST_SHARE_RAMP:
            // SHARE_RAMP：并联机逐步增加分流权重 alpha，避免一次性抢占主机电流。
            share_alpha_ramp = slew(share_alpha_ramp, 1.0f, 0.005f);
            if (share_alpha_ramp >= 0.999f)
            {
                run_state = ST_RUN;
                state_ms = 0UL;
            }
            break;

        case ST_RUN:
            // RUN：持续监测输入、PLL、输出误差和支路电流；稳压角色执行电流优先降压。
            if (is_voltage_role())
            {
                target = effective_u_ref();
                if (voltage_current_limit_1ms() != 0U)
                    u_ref_active = slew(u_ref_active, 0.0f, 0.005f);
                else
                    u_ref_active = slew(u_ref_active, target, 0.010f);

                if (fabsf(meas.uo_rms - u_ref_active) >
                    fmaxf(0.5f, 0.05f * u_ref_active))
                {
                    if (voltage_error_ms < 1000U) voltage_error_ms++;
                    if (voltage_error_ms >= 1000U) latch_fault(FAULT_START);
                }
                else voltage_error_ms = 0U;
            }
            break;

        case ST_STOP_RAMP:
            // STOP_RAMP：先撤销分流或降低电压参考，电流降到安全值后再回到等待/放电。
            if (is_share_role())
            {
                share_alpha_ramp = slew(share_alpha_ramp, 0.0f, 0.005f);
                if (share_alpha_ramp <= 0.001f)
                {
                    pwm_force_off();           // 确保所有门极驱动信号为低
                    run_state = ST_WAIT_MASTER_OFF;
                    state_ms = 0UL;
                }
            }
            else
            {
                u_ref_active = slew(u_ref_active, 0.0f, 0.005f);
                if ((u_ref_active < 1.0f) &&
                    (meas.il_rms < 0.10f) && (meas.io_rms < 0.10f))
                {
                    pwm_force_off();           // 确保所有门极驱动信号为低
                    run_state = ST_DISCHARGE;
                    state_ms = 0UL;
                }
            }
            break;

        case ST_WAIT_MASTER_OFF:
            if (dead_bus())
            {
                run_state = ST_SAFE;       // 初始化完成，进入安全待机
                model_cmd = MODEL_SAFE;
                stop_request = 0U;
                state_ms = 0UL;
            }
            break;

        case ST_DISCHARGE:
            if (dead_bus())
            {
                run_state = ST_SAFE;       // 初始化完成，进入安全待机
                model_cmd = MODEL_SAFE;
                stop_request = 0U;
                state_ms = 0UL;
            }
            else if (state_ms > 3000UL) latch_fault(FAULT_DISCHARGE);
            break;

        case ST_GATE_TEST:
#if GATE_OPEN_LOOP_TEST
            // 开环门极波形测试：固定 10% 占空比，仅做半周换向，不运行电压/电流控制环。
            // CMPSS、ADC 异常、ADCINT 溢出、KEY4 停机、OST 保护全部保留。
            ui_alpha_prev = pll_ui.sogi.alpha;
            if (gate_state == GATE_BLOCKED)
            {
                gate_start_pending = 1U;
            }
            // KEY4 短按立即停机
            if (stop_request != 0U)
            {
                stop_request = 0U;
                pwm_force_off();
                run_state = ST_SAFE;
                model_cmd = MODEL_SAFE;
                state_ms = 0UL;
                open_loop_test_ms = 0U;
                break;
            }
            // 5 秒自动关断
            if (open_loop_test_ms < 5000U) open_loop_test_ms++;
            if (open_loop_test_ms >= 5000U)
            {
                pwm_force_off();
                run_state = ST_SAFE;
                model_cmd = MODEL_SAFE;
                state_ms = 0UL;
                open_loop_test_ms = 0U;
            }
            // 不要求 Uo 建立，不触发 FAULT_START
#else
            latch_fault(FAULT_CPU);
#endif
            break;

        default:
            latch_fault(FAULT_CPU);
            break;
    }
}

void keys_update_5ms(void)
{
    // 每 5 ms 扫描一次按键；先做 6 次采样消抖，再生成按下、释放和长按重复事件。
    const Uint16 raw[6] = {KEY1_IN, KEY2_IN, KEY3_IN,
                           KEY4_IN, KEY5_IN, KEY6_IN};
    Uint16 i;
    for (i = 0U; i < 6U; i++)
    {
        KeyState *k = &keys_state[i];
        Uint16 long_event = 0U;
        k->press_edge = 0U;
        k->release_edge = 0U;
        k->repeat_edge = 0U;

        if (raw[i] == k->stable)
        {
            k->count = 0U;
        }
        else if (++k->count >= 5U)
        {
            k->stable = raw[i];
            k->count = 0U;
            if (k->stable != 0U)
            {
                k->press_edge = 1U;
                k->held_ms = 0U;
                k->repeat_ms = 0U;
                k->long_fired = 0U;
            }
            else
            {
                k->release_edge = 1U;
            }
        }

        if (k->stable != 0U)
        {
            if (k->held_ms < 65000U) k->held_ms += 5U;
            if (k->repeat_ms < 65000U) k->repeat_ms += 5U;
            if ((i == 3U) && (run_state == ST_FAULT) &&
                (k->held_ms >= 200U) && (k->long_fired == 0U))
            {
                long_event = 1U; k->long_fired = 1U;
            }
            else if ((i == 3U) && (run_state == ST_SAFE) &&
                     (k->held_ms >= 100U) && (k->long_fired == 0U))
            {
                long_event = 1U; k->long_fired = 1U;
            }
            if ((i != 0U) && (i != 1U) && (i != 2U) && (i != 3U) &&
                (k->repeat_ms >= 150U))
            {
                k->repeat_ms = 0U; k->repeat_edge = 1U;
            }
            if (((i == 0U) || (i == 1U)) &&
                (model_cmd == MODEL_PARALLEL) && (k->repeat_ms >= 150U))
            {
                k->repeat_ms = 0U; k->repeat_edge = 1U;
            }
        }
        key_action(i + 1U, k->press_edge | k->repeat_edge,
                   k->release_edge, long_event);
    }
}

void key_action(Uint16 key, Uint16 repeat_event, Uint16 release_event,
                       Uint16 long_event)
{
    // 按键只修改命令值，不直接改 PWM。模式、启停和故障清除均交给 1 ms 状态机执行。
    // KEY5/KEY6 以 UO_SET_STEP 步进，范围 UO_SET_MIN ~ UO_SET_MAX。
    float direction = ((key == 1U) || (key == 5U)) ? -1.0f : 1.0f;

    if ((key == 3U) && (release_event != 0U) && (run_state == ST_SAFE))
    {
#if UNIT_ROLE == UNIT_1
        if (model_cmd == MODEL_SAFE) model_cmd = MODEL_VOLTAGE;
        else if (model_cmd == MODEL_VOLTAGE) model_cmd = MODEL_PARALLEL;
        else model_cmd = MODEL_SAFE;
#else
        model_cmd = (model_cmd == MODEL_SAFE) ? MODEL_PARALLEL : MODEL_SAFE;
#endif
        warning_code = FAULT_OK;
        return;
    }

    if (key == 4U)
    {
        if (long_event != 0U)
        {
            if (run_state == ST_SAFE) start_request = 1U;
            else if (run_state == ST_FAULT) clear_request = 1U;
        }
        else if ((release_event != 0U) && (keys_state[3].long_fired == 0U) &&
                 (run_state != ST_SAFE) && (run_state != ST_FAULT))
        {
            stop_request = 1U;
        }
        return;
    }

    if (run_state == ST_FAULT) return;
#if DIRECTION_DIAG_8V
    return;
#endif

#if UNIT_ROLE == UNIT_1
    if (((key == 1U) || (key == 2U)) &&
        (model_cmd == MODEL_PARALLEL) && (repeat_event != 0U))
    {
        k_cmd = clampf_local(k_cmd + direction * 0.05f, 0.5f, 2.0f);
        return;
    }
    if (((key == 1U) || (key == 2U)) &&
        (model_cmd == MODEL_VOLTAGE) && (release_event != 0U))
    {
        if (fabsf(u_ref_cmd - 1.0f) < 0.001f)
            trim_low = clampf_local(trim_low + direction * 0.025f,
                                      -0.100f, 0.100f);
        else if (fabsf(u_ref_cmd - UO_SET_MAX) < 0.001f)
            trim_high = clampf_local(trim_high + direction * 0.025f,
                                       -0.100f, 0.100f);
        return;
    }
#endif

    if (((key == 5U) || (key == 6U)) && (repeat_event != 0U))
    {
        // KEY5 decrements and KEY6 increments on this hardware assignment.
        u_ref_cmd = clampf_local(u_ref_cmd + direction * UO_SET_STEP,
                                 UO_SET_MIN, UO_SET_MAX);
        u_ref_cmd = floorf(u_ref_cmd * 2.0f + 0.5f) * UO_SET_STEP;
    }
}
