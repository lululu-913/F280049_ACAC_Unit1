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

//========================== Build-time safety contract =======================
#define UNIT_1                         1
#define UNIT_2                         2
#define PROFILE_HALF                   1
#define PROFILE_FULL                   2
#define HALF_SAFE                      1
#define HALF_IHI                       2
#define AUX_SHARED_FORMAL              1
#define AUX_SHARED_LAB_DEBUG           2

#define UNIT_ROLE UNIT_1
#define POWER_STAGE_ENABLE          0U
#define POWER_PROFILE               PROFILE_HALF
#define HALF_CURRENT_STAGE          HALF_SAFE
#define HALF_IHI_TEST_PASSED_ACK       0
#define FORMAL_HW_RELEASE_ACK           0
#define DIRECTION_DIAG_6V              0
#define AUX_TOPOLOGY                   AUX_SHARED_FORMAL
#define CURRENT_LOOP_INTEGRAL_ENABLE   0U

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
#if DIRECTION_DIAG_6V != 0 && DIRECTION_DIAG_6V != 1
#error invalid DIRECTION_DIAG_6V
#endif
#if CURRENT_LOOP_INTEGRAL_ENABLE != 0 && CURRENT_LOOP_INTEGRAL_ENABLE != 1
#error invalid CURRENT_LOOP_INTEGRAL_ENABLE
#endif
#if DIRECTION_DIAG_6V && \
   (POWER_PROFILE != PROFILE_HALF || HALF_CURRENT_STAGE != HALF_SAFE || \
    AUX_TOPOLOGY != AUX_SHARED_LAB_DEBUG || HALF_IHI_TEST_PASSED_ACK != 0)
#error 6V direction diagnostic requires HALF SAFE and lab auxiliary
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

//=============================== Fixed hardware ==============================
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
#define IL_TO_OUTPUT_SIGN              (-1.0f)

// ADC = offset + gain * physical_value.  Voltage values are user calibration.
// Current gains for the 10 A sensors are commissioning-only initial estimates;
// replace every channel with its own positive/negative multi-point calibration
// before POWER_STAGE_ENABLE may be changed to one.
#define UI_ADC_OFFSET                    2066.2f
#define UI_ADC_GAIN                      36.443f
#define UO_ADC_OFFSET                    2067.0f
#define UO_ADC_GAIN                      36.378f
#define IL_ADC_OFFSET                    2091.9f
#define IL_ADC_GAIN                      334.04f
#define IO_ADC_OFFSET                    2097.6f
#define IO_ADC_GAIN                      333.10f
#define IT_ADC_OFFSET                    2094.0f
#define IT_ADC_GAIN                      330.25f
#define UI_ADC_POLARITY                  1.0f
#define UO_ADC_POLARITY                  1.0f
#define IL_ADC_POLARITY                  1.0f
#define IO_ADC_POLARITY                  1.0f
#define IT_ADC_POLARITY                  1.0f

#if POWER_PROFILE == PROFILE_HALF
#define UI_START_MIN                     16.5f
#define UI_START_MAX                     19.0f
#define UI_RUN_MIN                       15.0f
#define UI_RUN_MAX                       19.0f
#define UO_SINGLE_MAX                    17.5f
#define UO_PARALLEL_MAX                  15.0f
#define UO_DEFAULT                       15.0f
#define UO_RMS_HARD_MAX                  18.5f
#define UO_ABS_PK_TRIP                   27.0f
#define UOV_MARGIN                       1.0f
#else
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

#if DIRECTION_DIAG_6V
#undef UI_START_MIN
#undef UI_START_MAX
#undef UI_RUN_MIN
#undef UI_RUN_MAX
#undef UO_DEFAULT
#undef IL_REF_PK_MAX
#undef IL_SW_FAST_LIMIT
#undef IL_CMPSS_TRIP
#define UI_START_MIN                     4.0f
#define UI_START_MAX                     8.0f
#define UI_RUN_MIN                       4.0f
#define UI_RUN_MAX                       8.0f
#define UO_DEFAULT                       1.0f
#define IL_REF_PK_MAX                    0.45f
#define IL_SW_FAST_LIMIT                 0.60f
#define IL_CMPSS_TRIP                    0.80f
#endif

// Controller starting values.  They are commissioning points, not final gains.
#define PLL_KP                           88.9f
#define PLL_KI                           3948.0f
#define PLL_W_MIN                        (TWO_PI_F * 45.0f)
#define PLL_W_MAX                        (TWO_PI_F * 55.0f)
#define PLL_SOGI_K                       1.41421356237f
#define CURRENT_KP                       5.03f
#define CURRENT_KI                       632.0f
#define VOLTAGE_KP                       1.49e-3f
#define VOLTAGE_KI                       5.31e-2f
#define SHARE_KP                         0.25f
#define SHARE_KI                         78.5f

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
    ST_DISCHARGE, ST_FAULT
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
    float q_norm;
    Uint16 locked;
    Uint32 lock_samples;
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

//=============================== Shared state ================================
static volatile Measurements g_meas;
static SinglePhasePll g_pll_ui;
static SinglePhasePll g_pll_uo;
static SogiState g_uo_observer;
static SlidingRms g_rms_il;
static SlidingRms g_rms_io;
static SlidingRms g_rms_it;
static SlidingRms g_rms_uo;
static PiController g_pi_id;
static PiController g_pi_vd;
static PiController g_pi_vq;
static PiController g_pi_sd;
static PiController g_pi_sq;
static DqLowPass g_it_dq;
static DqLowPass g_io_dq;

static volatile RunState g_run_state = ST_BOOT;
static volatile GateState g_gate_state = GATE_BLOCKED;
static volatile HalfPolarity g_half = HALF_POS;
static volatile HalfPolarity g_next_half = HALF_POS;
static volatile FaultCode g_fault = FAULT_OK;
static volatile FaultCode g_warning = FAULT_OK;
static volatile Measurements g_fault_snapshot;
static volatile RunState g_fault_state = ST_BOOT;
static volatile float g_fault_duty = PWM_D_MIN;
static volatile Uint16 g_model_cmd = MODEL_SAFE;
static volatile Uint16 g_model_active = MODEL_SAFE;
static volatile Uint16 g_start_request = 0U;
static volatile Uint16 g_stop_request = 0U;
static volatile Uint16 g_clear_request = 0U;
static volatile Uint16 g_1ms_due = 0U;
static volatile Uint16 g_5ms_due = 0U;
static volatile Uint16 g_100ms_due = 0U;
static volatile Uint32 g_isr_heartbeat = 0UL;
static volatile Uint32 g_main_heartbeat = 0UL;
static volatile Uint32 g_state_ms = 0UL;
static volatile Uint32 g_isr_overrun_count = 0UL;

static float g_u_ref_cmd = UO_DEFAULT;
static float g_u_ref_active = 0.0f;
static float g_trim_low = 0.0f;
static float g_trim_high = 0.0f;
static float g_k_cmd = 1.0f;
static float g_k_active = 1.0f;
static float g_share_alpha_ramp = 0.0f;
static float g_io_ff = 0.0f;
static float g_duty = PWM_D_MIN;
static float g_il_ref = 0.0f;
static float g_il_ref_prev = 0.0f;
static float g_it_fund_rms = 0.0f;
static float g_io_fund_rms = 0.0f;
static float g_share_error_pct = 0.0f;
static Uint16 g_k_limited = 0U;
static Uint16 g_bus_valid = 0U;
static Uint16 g_gate_start_pending = 0U;
static Uint16 g_adc_bad_count = 0U;
static Uint16 g_il_fast_count = 0U;
static Uint16 g_io_pk_count = 0U;
static Uint16 g_it_pk_count = 0U;
static Uint16 g_uo_pk_count = 0U;
static Uint16 g_il_rms_count = 0U;
static Uint16 g_io_rms_count = 0U;
static Uint16 g_it_rms_count = 0U;
static Uint16 g_ui_bad_count = 0U;
static Uint16 g_pll_bad_count = 0U;
static Uint16 g_bus_lost_count = 0U;
static Uint16 g_dc_bad_count = 0U;
static Uint16 g_zc_stage_periods = 0U;
static Uint16 g_sample_div = 0U;
static Uint16 g_oled_line = 0U;
static Uint16 g_ss5_stable_ms = 0U;
static Uint16 g_voltage_stable_ms = 0U;
static Uint16 g_voltage_error_ms = 0U;
static Uint16 g_current_limit_ms = 0U;
static Uint16 g_current_release_ms = 0U;
static Uint16 g_current_limit_active = 0U;
static Uint16 g_fault_clear_safe_ms = 0U;
static Uint16 g_trip_clear_safe_count = 0U;
static KeyState g_keys[6];

#ifdef _FLASH
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;
#endif

//============================= Function declarations =========================
__interrupt void adcB1ISR(void);
static void hardware_init(void);
static void gpio_init_local(void);
static void gpio_enable_pwm_mux(void);
static void keys_init(void);
static void adc_init_local(void);
static void adc_soc_init(void);
static void pwm_init_local(void);
static void pwm_module_init(volatile struct EPWM_REGS *pwm, Uint16 master);
static void cmpss_trip_init(void);
static void controllers_init(void);
static void pwm_force_off(void);
static Uint16 pwm_clear_ost(void);
static void pwm_set_normal(HalfPolarity half, float duty);
static void pwm_set_zc_a(HalfPolarity next_half);
static void pwm_set_zc_b(HalfPolarity next_half);
static void pwm_set_aq_active(volatile struct EPWM_REGS *pwm);
static void pwm_set_aq_delayed_high(volatile struct EPWM_REGS *pwm, Uint16 output_a,
                                    Uint16 ticks);
static void pwm_force_pair(volatile struct EPWM_REGS *pwm, Uint16 a_force,
                           Uint16 b_force);
static void gate_sequence_update(void);
static void request_half_change(HalfPolarity next_half);
static void sample_and_calibrate(void);
static void signal_processing_update(void);
static void software_protection_update(void);
static void control_update(void);
static void slow_state_machine_1ms(void);
static Uint16 voltage_current_limit_1ms(void);
static void keys_update_5ms(void);
static void key_action(Uint16 key, Uint16 repeat_event, Uint16 release_event,
                       Uint16 long_event);
static void oled_update_one_line(void);
static void latch_fault(FaultCode fault);
static Uint16 fault_clear_conditions(void);
static Uint16 is_voltage_role(void);
static Uint16 is_share_role(void);
static Uint16 model_is_legal(Uint16 model);
static Uint16 dead_bus(void);
static Uint16 input_start_ok(void);
static Uint16 input_run_ok(void);
static Uint16 predicted_zero(void);
static float effective_u_ref(void);
static float active_u_max(void);
static float clampf_local(float x, float lo, float hi);
static float slew(float x, float target, float step);
static float wrap_pi(float x);
static void sogi_update(SogiState *s, float input, float omega);
static void pll_update(SinglePhasePll *pll, float input, float amp_min);
static void rms_update(SlidingRms *r, float x);
static float pi_update(PiController *pi, float error, Uint16 integrate);
static void pi_reset(PiController *pi);
static void dq_lpf_update(DqLowPass *f, float x, float sn, float cs);
static void clear_line(char *line);
static void put_text(char *line, Uint16 pos, const char *s);
static void put_u2(char *line, Uint16 pos, Uint16 v);
static void put_fixed(char *line, Uint16 pos, float x, Uint16 decimals,
                      Uint16 width);
static const char *state_text(RunState state);
static const char *profile_text(void);

//================================== Main ======================================
void main(void)
{
    Uint32 reset_cause;

#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif

    InitSysCtrl();
    reset_cause = SysCtl_getResetCause();
    hardware_init();
    controllers_init();

    g_run_state = ST_SAFE;
    pwm_force_off();
    if ((reset_cause & (SYSCTL_CAUSE_WDRS | SYSCTL_CAUSE_NMIWDRS)) != 0UL)
    {
        latch_fault(FAULT_CPU);
    }
    SysCtl_clearResetCause(reset_cause);

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1U;
    EDIS;

    IER |= M_INT1;
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1U;
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

    for (;;)
    {
        Uint16 do_1ms = 0U;
        Uint16 do_5ms = 0U;
        Uint16 do_100ms = 0U;
        Uint32 isr_now;
        static Uint32 isr_seen = 0UL;
        static Uint32 main_seen = 0UL;

        g_main_heartbeat++;
        DINT;
        if (g_1ms_due != 0U) { g_1ms_due--; do_1ms = 1U; }
        if (g_5ms_due != 0U) { g_5ms_due--; do_5ms = 1U; }
        if (g_100ms_due != 0U) { g_100ms_due--; do_100ms = 1U; }
        isr_now = g_isr_heartbeat;
        EINT;

        if (do_1ms != 0U)
        {
            slow_state_machine_1ms();
            if ((isr_now != isr_seen) && (g_main_heartbeat != main_seen))
            {
                isr_seen = isr_now;
                main_seen = g_main_heartbeat;
                SysCtl_serviceWatchdog();
            }
        }
        if (do_5ms != 0U)
        {
            keys_update_5ms();
        }
        if (do_100ms != 0U)
        {
            oled_update_one_line();
        }
    }
}

//================================== OLED ======================================
static void oled_update_one_line(void)
{
    char line[17];
    Uint16 model = (g_model_active != MODEL_SAFE) ? g_model_active : g_model_cmd;

    clear_line(line);
    if (g_oled_line == 0U)
    {
        put_text(line, 0U, profile_text());
#if UNIT_ROLE == UNIT_1
        put_text(line, 6U, (model == MODEL_PARALLEL) ? "M2SH" :
                            ((model == MODEL_VOLTAGE) ? "M1VC" : "M0--"));
#else
        put_text(line, 6U, (model == MODEL_PARALLEL) ? "M2VC" : "M0--");
#endif
        put_text(line, 11U, state_text(g_run_state));
    }
    else if (g_oled_line == 1U)
    {
        if (g_run_state == ST_FAULT)
        {
            put_text(line, 0U, "FAULT F:");
            put_u2(line, 8U, (Uint16)g_fault);
            put_text(line, 11U, "OST");
        }
        else
        {
            put_text(line, 0U, "UO:");
            put_fixed(line, 3U, (g_run_state == ST_FAULT) ?
                      g_fault_snapshot.uo_rms : g_meas.uo_rms, 1U, 4U);
            put_text(line, 8U, "UR:");
            put_fixed(line, 11U, effective_u_ref(), 1U, 5U);
        }
    }
    else if (g_oled_line == 2U)
    {
        put_text(line, 0U, "IL:");
        put_fixed(line, 3U, (g_run_state == ST_FAULT) ?
                  g_fault_snapshot.il_rms : g_meas.il_rms, 2U, 5U);
        put_text(line, 9U, "IO:");
        put_fixed(line, 12U, (g_run_state == ST_FAULT) ?
                  g_fault_snapshot.io_rms : g_meas.io_rms, 2U, 4U);
    }
    else
    {
#if UNIT_ROLE == UNIT_1
        if (model == MODEL_PARALLEL)
        {
            put_text(line, 0U, "K:");
            put_fixed(line, 2U, g_k_cmd, 2U, 4U);
            if (g_k_limited) put_text(line, 7U, "LIM");
        }
        else if ((fabsf(g_u_ref_cmd - 1.0f) < 0.001f) ||
                 (fabsf(g_u_ref_cmd - active_u_max()) < 0.001f))
        {
            float trim = (g_u_ref_cmd < 1.1f) ? g_trim_low : g_trim_high;
            put_text(line, 0U, "TRM:");
            put_fixed(line, 4U, trim, 3U, 6U);
        }
        else put_text(line, 0U, "STEP:0.500");
#else
        put_text(line, 0U, "PLL:");
        put_text(line, 4U, g_pll_ui.locked ? "Y" : "N");
#endif
        put_text(line, 11U, "F:");
        put_u2(line, 13U, (Uint16)((g_fault != FAULT_OK) ? g_fault : g_warning));
    }

    OLED_ShowString(0U, (Uchar)g_oled_line, line);
    g_oled_line = (g_oled_line + 1U) & 3U;
}

//============================= Safety predicates =============================
static void latch_fault(FaultCode fault)
{
    if ((fault != FAULT_OK) && (g_fault == FAULT_OK))
    {
        // Freeze the first fault context before changing the state or outputs.
        // Subsequent protection sources must not overwrite the root event.
        g_fault_snapshot = g_meas;
        g_fault_state = g_run_state;
        g_fault_duty = g_duty;
        g_fault_clear_safe_ms = 0U;
        g_fault = fault;
        g_run_state = ST_FAULT;
        g_model_cmd = MODEL_SAFE;
        g_start_request = 0U;
        g_stop_request = 0U;
        pwm_force_off();
    }
}

static Uint16 fault_clear_conditions(void)
{
    Uint16 raw_ok = ((g_meas.raw_ui >= 64U) && (g_meas.raw_ui <= 4031U) &&
                     (g_meas.raw_uo >= 64U) && (g_meas.raw_uo <= 4031U) &&
                     (g_meas.raw_il >= 64U) && (g_meas.raw_il <= 4031U) &&
                     (g_meas.raw_io >= 64U) && (g_meas.raw_io <= 4031U));
#if UNIT_ROLE == UNIT_1
    raw_ok = raw_ok && (g_meas.raw_it >= 64U) && (g_meas.raw_it <= 4031U);
#endif
    Uint16 comparator_active = CMPSS_getStatus(CMPSS7_BASE) &
                               (CMPSS_STS_HI_FILTOUT | CMPSS_STS_LO_FILTOUT);
    if (!raw_ok || !dead_bus() || (comparator_active != 0U) ||
        (fabsf(g_meas.il) > 0.8f * IL_CMPSS_TRIP))
    {
        g_fault_clear_safe_ms = 0U;
        return 0U;
    }
    if ((g_fault == FAULT_IL_RMS) && (g_meas.il_rms > 0.8f * IL_RMS_TRIP))
    {
        g_fault_clear_safe_ms = 0U; return 0U;
    }
    if ((g_fault == FAULT_IO) && (g_meas.io_rms > 0.8f * IO_RMS_TRIP))
    {
        g_fault_clear_safe_ms = 0U; return 0U;
    }
#if UNIT_ROLE == UNIT_1
    if ((g_fault == FAULT_IT) && (g_meas.it_rms > 0.8f * IT_RMS_TRIP))
    {
        g_fault_clear_safe_ms = 0U; return 0U;
    }
#endif
    if (g_fault_clear_safe_ms < 100U) g_fault_clear_safe_ms++;
    return (g_fault_clear_safe_ms >= 100U) ? 1U : 0U;
}

static Uint16 is_voltage_role(void)
{
#if UNIT_ROLE == UNIT_1
    return (g_model_active == MODEL_VOLTAGE) ? 1U : 0U;
#else
    return (g_model_active == MODEL_PARALLEL) ? 1U : 0U;
#endif
}

static Uint16 is_share_role(void)
{
#if UNIT_ROLE == UNIT_1
    return (g_model_active == MODEL_PARALLEL) ? 1U : 0U;
#else
    return 0U;
#endif
}

static Uint16 model_is_legal(Uint16 model)
{
#if UNIT_ROLE == UNIT_1
    return ((model == MODEL_SAFE) || (model == MODEL_VOLTAGE) ||
            (model == MODEL_PARALLEL)) ? 1U : 0U;
#else
    return ((model == MODEL_SAFE) || (model == MODEL_PARALLEL)) ? 1U : 0U;
#endif
}

static Uint16 dead_bus(void)
{
    static Uint16 count = 0U;
    Uint16 now = ((g_meas.uo_rms < 2.0f) && (g_meas.il_rms < 0.10f) &&
                  (g_meas.io_rms < 0.10f));
    if (now)
    {
        if (count < 100U) count++;
    }
    else count = 0U;
    return (count >= 100U) ? 1U : 0U;
}

static Uint16 input_start_ok(void)
{
    return ((g_meas.ui_rms >= UI_START_MIN) &&
            (g_meas.ui_rms <= UI_START_MAX) &&
            (g_pll_ui.omega >= TWO_PI_F * 48.0f) &&
            (g_pll_ui.omega <= TWO_PI_F * 52.0f)) ? 1U : 0U;
}

static Uint16 input_run_ok(void)
{
    return ((g_meas.ui_rms >= UI_RUN_MIN) &&
            (g_meas.ui_rms <= UI_RUN_MAX)) ? 1U : 0U;
}

static Uint16 predicted_zero(void)
{
    float phase = wrap_pi(g_pll_ui.theta);
    float distance = fminf(fabsf(phase), fabsf(fabsf(phase) - PI_F));
    Uint16 ui_zero = ((fabsf(g_pll_ui.sogi.alpha) < 1.5f) &&
                      (distance < 2.0f * PI_F / 180.0f)) ? 1U : 0U;
    if (ui_zero == 0U) return 0U;
#if UNIT_ROLE == UNIT_1
    if (is_share_role() != 0U)
    {
        float out_phase = wrap_pi(g_pll_uo.theta);
        float out_distance = fminf(fabsf(out_phase),
                                   fabsf(fabsf(out_phase) - PI_F));
        return ((fabsf(g_pll_uo.sogi.alpha) < 1.5f) &&
                (out_distance < 2.0f * PI_F / 180.0f) &&
                (g_pll_uo.locked != 0U)) ? 1U : 0U;
    }
#endif
    return ui_zero;
}

static float effective_u_ref(void)
{
    if (DIRECTION_DIAG_6V) return 1.0f;
#if UNIT_ROLE == UNIT_1
    if (g_model_cmd == MODEL_VOLTAGE || g_model_active == MODEL_VOLTAGE)
    {
        if (fabsf(g_u_ref_cmd - 1.0f) < 0.001f)
            return g_u_ref_cmd + g_trim_low;
        if (fabsf(g_u_ref_cmd - UO_SINGLE_MAX) < 0.001f)
            return g_u_ref_cmd + g_trim_high;
    }
#endif
    return g_u_ref_cmd;
}

static float active_u_max(void)
{
#if UNIT_ROLE == UNIT_1
    if ((g_model_cmd == MODEL_VOLTAGE) || (g_model_active == MODEL_VOLTAGE))
        return UO_SINGLE_MAX;
#endif
    return UO_PARALLEL_MAX;
}

//=============================== Math helpers =================================
static float clampf_local(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float slew(float x, float target, float step)
{
    if (x < target - step) return x + step;
    if (x > target + step) return x - step;
    return target;
}

static float wrap_pi(float x)
{
    while (x > PI_F) x -= TWO_PI_F;
    while (x < -PI_F) x += TWO_PI_F;
    return x;
}

static void sogi_update(SogiState *s, float input, float omega)
{
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

static void pll_update(SinglePhasePll *pll, float input, float amp_min)
{
    float sn;
    float cs;
    float error;
    sogi_update(&pll->sogi, input, pll->omega);
    pll->amplitude = sqrtf(pll->sogi.alpha * pll->sogi.alpha +
                           pll->sogi.beta * pll->sogi.beta);
    sn = sinf(pll->theta);
    cs = cosf(pll->theta);
    error = (pll->sogi.alpha * cs + pll->sogi.beta * sn) /
            fmaxf(pll->amplitude, amp_min);
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

    if ((pll->amplitude >= amp_min) && (fabsf(error) < 0.05f) &&
        (pll->omega >= TWO_PI_F * 48.0f) &&
        (pll->omega <= TWO_PI_F * 52.0f))
    {
        if (pll->lock_samples < 2000UL) pll->lock_samples++;
    }
    else pll->lock_samples = 0UL;
    pll->locked = (pll->lock_samples >= 2000UL) ? 1U : 0U;
}

static void rms_update(SlidingRms *r, float x)
{
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
    r->rms = sqrtf(r->sum_sq / (float)r->filled);
    r->mean = r->sum / (float)r->filled;
}

static float pi_update(PiController *pi, float error, Uint16 integrate)
{
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

static void pi_reset(PiController *pi)
{
    pi->integral = 0.0f;
}

static void dq_lpf_update(DqLowPass *f, float x, float sn, float cs)
{
    f->d = FUND_LPF_A * f->d + (1.0f - FUND_LPF_A) * (2.0f * x * sn);
    f->q = FUND_LPF_A * f->q + (1.0f - FUND_LPF_A) * (2.0f * x * cs);
}

//============================== Text formatting ==============================
static void clear_line(char *line)
{
    Uint16 i;
    for (i = 0U; i < 16U; i++) line[i] = ' ';
    line[16] = '\0';
}

static void put_text(char *line, Uint16 pos, const char *s)
{
    while ((*s != '\0') && (pos < 16U)) line[pos++] = *s++;
}

static void put_u2(char *line, Uint16 pos, Uint16 v)
{
    v %= 100U;
    if (pos < 16U) line[pos] = (char)('0' + v / 10U);
    if (pos + 1U < 16U) line[pos + 1U] = (char)('0' + v % 10U);
}

static void put_fixed(char *line, Uint16 pos, float x, Uint16 decimals,
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

static const char *state_text(RunState state)
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
        case ST_FAULT: return "FLT ";
        default: return "BOOT";
    }
}

static const char *profile_text(void)
{
#if DIRECTION_DIAG_6V
    return "6VDg";
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


//============================ Hardware initialization ========================
static void hardware_init(void)
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

static void gpio_init_local(void)
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
    EDIS;
}

static void gpio_enable_pwm_mux(void)
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

static void keys_init(void)
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

static void adc_init_local(void)
{
    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(ADCB_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    EALLOW;
    // PRESCALE=6 gives 25 MHz ADCCLK from this project's 100 MHz SYSCLK.
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1U;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1U;
    EDIS;
    DELAY_US(1000U);
}

static void adc_soc_init(void)
{
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0U;      // Ui, ADCINA0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 9U;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 9U;    // ePWM3 SOCA
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1U;      // Uo, ADCINA1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 9U;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 9U;

    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0U;      // iL, ADCINB0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 9U;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 9U;
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1U;      // Io-local, ADCINB1
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 9U;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 9U;
#if UNIT_ROLE == UNIT_1
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2U;      // Itotal, ADCINB2
#else
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 1U;      // duplicate Io2 for EOC timing
#endif
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 9U;
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 9U;

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 2U;  // ISR only after ADCB EOC2
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1U;
    AdcbRegs.ADCINTSEL1N2.bit.INT1CONT = 0U;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    EDIS;
}

static void pwm_module_init(volatile struct EPWM_REGS *pwm, Uint16 master)
{
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

static void pwm_init_local(void)
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
    EPwm3Regs.ETSEL.bit.SOCAEN = 1U;
    EPwm3Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;
    EPwm3Regs.ETPS.bit.SOCAPRD = ET_1ST;
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EDIS;

    pwm_force_off();
}

static void cmpss_trip_init(void)
{
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
    CMPSS_configOutputsHigh(CMPSS7_BASE,
                            CMPSS_TRIP_ASYNC_COMP | CMPSS_TRIPOUT_ASYNC_COMP);
    CMPSS_configOutputsLow(CMPSS7_BASE,
                           CMPSS_TRIP_ASYNC_COMP | CMPSS_TRIPOUT_ASYNC_COMP);
    CMPSS_configDAC(CMPSS7_BASE,
                    CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW);
    CMPSS_setDACValueHigh(CMPSS7_BASE, high_code);
    CMPSS_setDACValueLow(CMPSS7_BASE, low_code);
    CMPSS_setHysteresis(CMPSS7_BASE, 1U);
    CMPSS_clearFilterLatchHigh(CMPSS7_BASE);
    CMPSS_clearFilterLatchLow(CMPSS7_BASE);
    CMPSS_enableModule(CMPSS7_BASE);

    XBAR_disableEPWMMux(XBAR_TRIP7, 0xFFFFFFFFUL);
    XBAR_setEPWMMuxConfig(XBAR_TRIP7, XBAR_EPWM_MUX12_CMPSS7_CTRIPH_OR_L);
    XBAR_invertEPWMSignal(XBAR_TRIP7, false);
    XBAR_enableEPWMMux(XBAR_TRIP7, XBAR_MUX12);

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
    pwm_force_off();
}

static void controllers_init(void)
{
    memset(&g_pll_ui, 0, sizeof(g_pll_ui));
    memset(&g_pll_uo, 0, sizeof(g_pll_uo));
    memset(&g_uo_observer, 0, sizeof(g_uo_observer));
    memset(&g_rms_il, 0, sizeof(g_rms_il));
    memset(&g_rms_io, 0, sizeof(g_rms_io));
    memset(&g_rms_it, 0, sizeof(g_rms_it));
    memset(&g_rms_uo, 0, sizeof(g_rms_uo));
    memset(g_keys, 0, sizeof(g_keys));

    g_pll_ui.omega = TWO_PI_F * 50.0f;
    g_pll_uo.omega = TWO_PI_F * 50.0f;
    g_pi_id.kp = CURRENT_KP;
    g_pi_id.ki = CURRENT_LOOP_INTEGRAL_ENABLE ? CURRENT_KI : 0.0f;
    g_pi_id.out_min = -30.0f; g_pi_id.out_max = 30.0f;
    g_pi_vd.kp = VOLTAGE_KP;  g_pi_vd.ki = VOLTAGE_KI;
    g_pi_vd.out_min = -I_BRANCH_CMD_MAX; g_pi_vd.out_max = I_BRANCH_CMD_MAX;
    g_pi_vq = g_pi_vd;
    g_pi_sd.kp = SHARE_KP; g_pi_sd.ki = SHARE_KI;
    g_pi_sd.out_min = -I_BRANCH_CMD_MAX; g_pi_sd.out_max = I_BRANCH_CMD_MAX;
    g_pi_sq = g_pi_sd;
}

//============================ PWM gate sequencer =============================
static void pwm_force_off(void)
{
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
    g_gate_state = GATE_BLOCKED;
    g_gate_start_pending = 0U;
}

static Uint16 pwm_clear_ost(void)
{
#if POWER_STAGE_ENABLE == 1
    Uint16 raw_ok = ((g_meas.raw_ui >= 64U) && (g_meas.raw_ui <= 4031U) &&
                     (g_meas.raw_uo >= 64U) && (g_meas.raw_uo <= 4031U) &&
                     (g_meas.raw_il >= 64U) && (g_meas.raw_il <= 4031U) &&
                     (g_meas.raw_io >= 64U) && (g_meas.raw_io <= 4031U));
    Uint16 cmp_ok = (CMPSS_getStatus(CMPSS7_BASE) &
                     (CMPSS_STS_HI_FILTOUT | CMPSS_STS_LO_FILTOUT)) == 0U;
    Uint16 phase_ok = 1U;
#if UNIT_ROLE == UNIT_1
    if (g_model_active == MODEL_PARALLEL)
    {
        phase_ok = (g_bus_valid && (g_meas.uo_rms >= 4.0f) &&
                    (g_meas.uo_rms <= 6.5f) &&
                    (g_pll_uo.locked != 0U) &&
                    (fabsf(wrap_pi(g_pll_uo.theta - g_pll_ui.theta - PI_F)) <
                     (10.0f * PI_F / 180.0f))) ? 1U : 0U;
    }
#endif
    if (!raw_ok || !cmp_ok || (fabsf(g_meas.il) > 0.8f * IL_CMPSS_TRIP) ||
        (g_pll_ui.locked == 0U) || !phase_ok)
    {
        g_trip_clear_safe_count = 0U;
        return 0U;
    }
    if (g_trip_clear_safe_count < 2000U) g_trip_clear_safe_count++;
    if (g_trip_clear_safe_count < 2000U) return 0U;

    // The new AQ/CSF image was written with shadow load-on-ZERO.  Keep the
    // immediate low force in place until that image has had a carrier ZERO to
    // load, then release OST; the following cycle is the first possible gate.
    EPWM_clearOneShotTripZoneFlag(EPWM1_BASE, EPWM_TZ_OST_FLAG_DCAEVT1);
    EPWM_clearOneShotTripZoneFlag(EPWM2_BASE, EPWM_TZ_OST_FLAG_DCAEVT1);
    EPWM_clearTripZoneFlag(EPWM1_BASE, EPWM_TZ_FLAG_OST |
                           EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_INT);
    EPWM_clearTripZoneFlag(EPWM2_BASE, EPWM_TZ_FLAG_OST |
                           EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_INT);
    g_trip_clear_safe_count = 0U;
    return 1U;
#else
    // Deliberate commissioning lock: no executable path clears OST.
    pwm_force_off();
    return 0U;
#endif
}

static void pwm_force_pair(volatile struct EPWM_REGS *pwm, Uint16 a_force,
                           Uint16 b_force)
{
    pwm->AQSFRC.bit.RLDCSF = 0U; // shadow-to-active at ZERO
    pwm->AQCSFRC.bit.CSFA = a_force;
    pwm->AQCSFRC.bit.CSFB = b_force;
}

static void pwm_set_aq_active(volatile struct EPWM_REGS *pwm)
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

static void pwm_set_aq_delayed_high(volatile struct EPWM_REGS *pwm,
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

static void pwm_set_normal(HalfPolarity half, float duty)
{
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

static void pwm_set_zc_a(HalfPolarity next_half)
{
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

static void pwm_set_zc_b(HalfPolarity next_half)
{
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

static void request_half_change(HalfPolarity next_half)
{
    if ((g_gate_state == GATE_ACTIVE) && (next_half != g_half))
    {
        g_next_half = next_half;
        g_gate_state = GATE_ZC_A;
        g_zc_stage_periods = 0U;
        pwm_set_zc_a(next_half);
        pi_reset(&g_pi_id);
    }
}

static void gate_sequence_update(void)
{
    if (g_gate_state == GATE_ZC_A)
    {
        pwm_set_zc_b(g_next_half);
        g_gate_state = GATE_ZC_B;
    }
    else if (g_gate_state == GATE_ZC_B)
    {
        g_half = g_next_half;
        pwm_set_normal(g_half, g_duty);
        g_gate_state = GATE_ACTIVE;
    }
    else if (g_gate_state == GATE_ACTIVE)
    {
        pwm_set_normal(g_half, g_duty);
    }
}

//========================== Sampling and signal processing ===================
static void sample_and_calibrate(void)
{
    Measurements m;
    m.raw_ui = AdcaResultRegs.ADCRESULT0;
    m.raw_uo = AdcaResultRegs.ADCRESULT1;
    m.raw_il = AdcbResultRegs.ADCRESULT0;
    m.raw_io = AdcbResultRegs.ADCRESULT1;
    m.raw_it = AdcbResultRegs.ADCRESULT2;

    m.ui = UI_ADC_POLARITY * (((float)m.raw_ui - UI_ADC_OFFSET) / UI_ADC_GAIN);
    m.uo = UO_ADC_POLARITY * (((float)m.raw_uo - UO_ADC_OFFSET) / UO_ADC_GAIN);
    m.il = IL_ADC_POLARITY * (((float)m.raw_il - IL_ADC_OFFSET) / IL_ADC_GAIN);
    m.io = IO_ADC_POLARITY * (((float)m.raw_io - IO_ADC_OFFSET) / IO_ADC_GAIN);
#if UNIT_ROLE == UNIT_1
    m.it = IT_ADC_POLARITY * (((float)m.raw_it - IT_ADC_OFFSET) / IT_ADC_GAIN);
#else
    // ADCB SOC2 repeats ADCINB1 only to make ADCB EOC2 the common ISR point.
    // Unit 2 has no total-current signal and never consumes RESULT2 as Itotal.
    m.it = 0.0f;
#endif

    m.ui_rms = g_meas.ui_rms;
    m.uo_rms = g_meas.uo_rms;
    m.il_rms = g_meas.il_rms;
    m.io_rms = g_meas.io_rms;
    m.it_rms = g_meas.it_rms;
    m.il_mean = g_meas.il_mean;
    m.uo_mean = g_meas.uo_mean;
    g_meas = m;
}

static void signal_processing_update(void)
{
    float ui_amp_min;
    float uo_omega;

    ui_amp_min = DIRECTION_DIAG_6V ? 2.0f : 5.0f;
    pll_update(&g_pll_ui, g_meas.ui, ui_amp_min);
    sogi_update(&g_uo_observer, g_meas.uo, g_pll_ui.omega);

    if (is_share_role() != 0U)
    {
        pll_update(&g_pll_uo, g_meas.uo, 2.0f);
        uo_omega = g_pll_uo.omega;
        dq_lpf_update(&g_it_dq, g_meas.it, sinf(g_pll_uo.theta),
                      cosf(g_pll_uo.theta));
        dq_lpf_update(&g_io_dq, g_meas.io, sinf(g_pll_uo.theta),
                      cosf(g_pll_uo.theta));
        g_it_fund_rms = sqrtf(g_it_dq.d * g_it_dq.d +
                              g_it_dq.q * g_it_dq.q) * INV_SQRT2_F;
        g_io_fund_rms = sqrtf(g_io_dq.d * g_io_dq.d +
                              g_io_dq.q * g_io_dq.q) * INV_SQRT2_F;
    }
    else
    {
        uo_omega = g_pll_ui.omega;
    }
    (void)uo_omega;

    rms_update(&g_rms_il, g_meas.il);
    rms_update(&g_rms_io, g_meas.io);
    rms_update(&g_rms_it, g_meas.it);
    rms_update(&g_rms_uo, g_meas.uo);

    g_meas.ui_rms = g_pll_ui.amplitude * INV_SQRT2_F;
    g_meas.uo_rms = sqrtf(g_uo_observer.alpha * g_uo_observer.alpha +
                          g_uo_observer.beta * g_uo_observer.beta) * INV_SQRT2_F;
    g_meas.il_rms = g_rms_il.rms;
    g_meas.io_rms = g_rms_io.rms;
#if UNIT_ROLE == UNIT_1
    g_meas.it_rms = g_rms_it.rms;
#else
    g_meas.it_rms = 0.0f;
#endif
    g_meas.il_mean = g_rms_il.mean;
    g_meas.uo_mean = g_rms_uo.mean;
}

static void software_protection_update(void)
{
    Uint16 raw_bad;
    Uint16 active;
    float dynamic_ov;

    // PRECHECK/PLL/WAIT states run with OST set.  Their own state timeouts
    // validate acquisition and lock; applying the three-cycle run-time PLL
    // timer there would fault before the required five-cycle initial lock.
    active = ((g_run_state == ST_SS5_RAMP) ||
              (g_run_state == ST_SS5_HOLD) ||
              (g_run_state == ST_VOLT_RAMP) ||
              (g_run_state == ST_SHARE_RAMP) ||
              (g_run_state == ST_RUN) ||
              (g_run_state == ST_STOP_RAMP)) ? 1U : 0U;
    raw_bad = ((g_meas.raw_ui < 32U) || (g_meas.raw_ui > 4063U) ||
               (g_meas.raw_uo < 32U) || (g_meas.raw_uo > 4063U) ||
               (g_meas.raw_il < 32U) || (g_meas.raw_il > 4063U) ||
               (g_meas.raw_io < 32U) || (g_meas.raw_io > 4063U)) ? 1U : 0U;
#if UNIT_ROLE == UNIT_1
    if ((g_meas.raw_it < 32U) || (g_meas.raw_it > 4063U)) raw_bad = 1U;
#endif
    if (raw_bad != 0U)
    {
        if (++g_adc_bad_count >= 3U) latch_fault(FAULT_ADC);
    }
    else g_adc_bad_count = 0U;

    if ((EPwm1Regs.TZFLG.bit.DCAEVT1 != 0U) ||
        (EPwm2Regs.TZFLG.bit.DCAEVT1 != 0U))
    {
        latch_fault(FAULT_IL_PK);
    }

    if (active == 0U) return;

    if (fabsf(g_meas.il) >= IL_SW_FAST_LIMIT)
    {
        if (++g_il_fast_count >= 1U) latch_fault(FAULT_IL_PK);
    }
    else g_il_fast_count = 0U;

    if (fabsf(g_meas.io) >= IO_PK_TRIP)
    {
        if (++g_io_pk_count >= 5U) latch_fault(FAULT_IO);
    }
    else g_io_pk_count = 0U;

#if UNIT_ROLE == UNIT_1
    if (fabsf(g_meas.it) >= IT_PK_TRIP)
    {
        if (++g_it_pk_count >= 5U) latch_fault(FAULT_IT);
    }
    else g_it_pk_count = 0U;
#endif

    if (fabsf(g_meas.uo) >= UO_ABS_PK_TRIP)
    {
        if (++g_uo_pk_count >= 3U) latch_fault(FAULT_UO_OV);
    }
    else g_uo_pk_count = 0U;

    if (g_meas.il_rms >= IL_RMS_TRIP)
    {
        if (++g_il_rms_count >= 800U) latch_fault(FAULT_IL_RMS);
    }
    else g_il_rms_count = 0U;
    if (g_meas.io_rms >= IO_RMS_TRIP)
    {
        if (++g_io_rms_count >= 800U) latch_fault(FAULT_IO);
    }
    else g_io_rms_count = 0U;
#if UNIT_ROLE == UNIT_1
    if (g_meas.it_rms >= IT_RMS_TRIP)
    {
        if (++g_it_rms_count >= 800U) latch_fault(FAULT_IT);
    }
    else g_it_rms_count = 0U;
#endif

    dynamic_ov = g_u_ref_active + fmaxf(UOV_MARGIN, 0.10f * g_u_ref_active);
    dynamic_ov = fminf(dynamic_ov, UO_RMS_HARD_MAX);
    if (is_voltage_role() == 0U) dynamic_ov = UO_RMS_HARD_MAX;
    if ((g_run_state == ST_SS5_RAMP) || (g_run_state == ST_SS5_HOLD))
        dynamic_ov = 7.0f;
    if (g_meas.uo_rms > dynamic_ov) latch_fault(FAULT_UO_OV);

    if (input_run_ok() == 0U)
    {
        if (++g_ui_bad_count >= 1200U) latch_fault(FAULT_UI);
    }
    else g_ui_bad_count = 0U;

    if ((g_pll_ui.omega < TWO_PI_F * 46.0f) ||
        (g_pll_ui.omega > TWO_PI_F * 54.0f) || (g_pll_ui.locked == 0U))
    {
        if (++g_pll_bad_count >= 1200U) latch_fault(FAULT_PLL);
    }
    else g_pll_bad_count = 0U;

    if (is_share_role() && g_bus_valid &&
        (g_run_state == ST_SHARE_RAMP || g_run_state == ST_RUN))
    {
        if (g_meas.uo_rms < 3.0f)
        {
            if (++g_bus_lost_count >= 100U) latch_fault(FAULT_BUS_LOST);
        }
        else g_bus_lost_count = 0U;
    }
    else g_bus_lost_count = 0U;

    if ((fabsf(g_meas.il_mean) > 0.30f) || (fabsf(g_meas.uo_mean) > 0.50f))
    {
        if (++g_dc_bad_count >= 2000U) latch_fault(FAULT_DC_OFFSET);
    }
    else g_dc_bad_count = 0U;
}

// ADCB INT1 is selected by ADCB EOC2, after A0/A1 and B0/B1/B2 complete.
__interrupt void adcB1ISR(void)
{
    Uint16 entry_ctr = EPwm3Regs.TBCTR;
    Uint16 exit_ctr;
    static Uint16 overrun_consecutive = 0U;

    sample_and_calibrate();
    signal_processing_update();
    software_protection_update();
    control_update();
    // control_update() writes the next shadow image first; the sequencer then
    // submits that image before the next carrier ZERO.
    gate_sequence_update();

    g_isr_heartbeat++;
    if (++g_sample_div >= 20U)
    {
        g_sample_div = 0U;
        if (g_1ms_due < 100U) g_1ms_due++;
        if ((g_isr_heartbeat % 100UL) == 0UL && g_5ms_due < 20U) g_5ms_due++;
        if ((g_isr_heartbeat % 2000UL) == 0UL && g_100ms_due < 4U) g_100ms_due++;
    }

    exit_ctr = EPwm3Regs.TBCTR;
    if ((EPwm3Regs.TBSTS.bit.CTRDIR != 0U) ||
        ((entry_ctr >= exit_ctr) && ((entry_ctr - exit_ctr) > 2500U)))
    {
        g_isr_overrun_count++;
        if (++overrun_consecutive >= 2U) latch_fault(FAULT_CPU);
    }
    else overrun_consecutive = 0U;

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    if (AdcbRegs.ADCINTOVF.bit.ADCINT1 != 0U)
    {
        AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
        latch_fault(FAULT_CPU);
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//================================ Control =====================================
static void control_update(void)
{
    float sn = sinf(g_pll_ui.theta);
    float cs = cosf(g_pll_ui.theta);
    float ui_amp = fmaxf(g_pll_ui.amplitude, 1.0f);
    float uo_amp = sqrtf(g_uo_observer.alpha * g_uo_observer.alpha +
                         g_uo_observer.beta * g_uo_observer.beta);
    float dff = uo_amp / fmaxf(ui_amp + uo_amp, 1.0f);
    float iconv_ref = 0.0f;
    float il_ref_target;
    float il_ref;
    float il_ref_dot;
    float error;
    float v_pi;
    float denom;
    float qsign;
    float d_unsat;
    float target_u;
    Uint16 controller_active;

    dff = clampf_local(dff, PWM_D_MIN, PWM_D_MAX);
    g_io_ff = IO_FF_A * g_io_ff + (1.0f - IO_FF_A) * g_meas.io;
    controller_active = ((g_gate_state == GATE_ACTIVE) &&
                         (g_run_state == ST_VOLT_RAMP ||
                         (g_run_state == ST_SS5_RAMP) ||
                         (g_run_state == ST_SS5_HOLD) ||
                         (g_run_state == ST_SHARE_RAMP) ||
                         (g_run_state == ST_RUN) ||
                         (g_run_state == ST_STOP_RAMP))) ? 1U : 0U;

    if (is_voltage_role() != 0U)
    {
        float v_alpha = -g_uo_observer.alpha;
        float v_beta = -g_uo_observer.beta;
        float vd = v_alpha * sn - v_beta * cs;
        float vq = v_alpha * cs + v_beta * sn;
        float id_corr;
        float iq_corr;
        float duo_ref;

        target_u = SQRT2_F * g_u_ref_active;
        id_corr = pi_update(&g_pi_vd, target_u - vd, controller_active);
        iq_corr = pi_update(&g_pi_vq, -vq, controller_active);
        duo_ref = -target_u * g_pll_ui.omega * cs;
        iconv_ref = g_io_ff + CO_FARAD * duo_ref -
                    (id_corr * sn + iq_corr * cs);
    }
    else if (is_share_role() != 0U)
    {
        float so = sinf(g_pll_uo.theta);
        float co = cosf(g_pll_uo.theta);
        float it_rms = g_it_fund_rms;
        float kmin_safe;
        float kmax_safe;
        float alpha;
        float target_d;
        float target_q;
        float corr_d;
        float corr_q;
        float io2_d;
        float io2_q;
        float io2_rms;
        float k_meas;

        kmin_safe = fmaxf(0.5f, it_rms / I_BRANCH_CMD_MAX - 1.0f);
        if (it_rms > I_BRANCH_CMD_MAX)
            kmax_safe = fminf(2.0f, I_BRANCH_CMD_MAX /
                              fmaxf(it_rms - I_BRANCH_CMD_MAX, 0.01f));
        else
            kmax_safe = 2.0f;

        if (kmin_safe > kmax_safe)
        {
            g_stop_request = 1U;
            g_k_limited = 1U;
            g_k_active = 1.0f;
        }
        else
        {
            float requested = clampf_local(g_k_cmd, kmin_safe, kmax_safe);
            g_k_limited = (fabsf(requested - g_k_cmd) > 0.001f) ? 1U : 0U;
            g_k_active = slew(g_k_active, requested, 0.5f * CONTROL_TS);
        }

        alpha = (g_k_active / (1.0f + g_k_active)) * g_share_alpha_ramp;
        target_d = alpha * g_it_dq.d;
        target_q = alpha * g_it_dq.q;
        corr_d = pi_update(&g_pi_sd, target_d - g_io_dq.d, controller_active);
        corr_q = pi_update(&g_pi_sq, target_q - g_io_dq.q, controller_active);
        iconv_ref = target_d * so + target_q * co +
                    CO_FARAD * g_uo_observer.dalpha +
                    corr_d * so + corr_q * co;

        io2_d = g_it_dq.d - g_io_dq.d;
        io2_q = g_it_dq.q - g_io_dq.q;
        io2_rms = sqrtf(io2_d * io2_d + io2_q * io2_q) * INV_SQRT2_F;
        k_meas = g_io_fund_rms / fmaxf(io2_rms, 0.02f);
        g_share_error_pct = 100.0f * fabsf(g_k_active - k_meas) /
                            fmaxf(g_k_active, 0.01f);
    }
    else
    {
        pi_reset(&g_pi_vd); pi_reset(&g_pi_vq);
        pi_reset(&g_pi_sd); pi_reset(&g_pi_sq);
    }

    // Limit the complete sinusoidal converter-current command before mapping
    // it through the boost relation.  This is the fast current-priority guard
    // used by both roles; the 1 kHz task then reduces Uref and times out.
    iconv_ref = clampf_local(iconv_ref, -SQRT2_F * I_BRANCH_CMD_MAX,
                             SQRT2_F * I_BRANCH_CMD_MAX);
    il_ref_target = IL_TO_OUTPUT_SIGN * iconv_ref /
                    fmaxf(1.0f - dff, 0.15f);
    il_ref_target = clampf_local(il_ref_target, -IL_REF_PK_MAX,
                                 IL_REF_PK_MAX);
    // Bound reference derivative before applying L*di/dt feed-forward.  The
    // 0.125 A/sample limit is 2500 A/s, above the required 50 Hz rated slope
    // but prevents an outer-loop or limit transition from commanding a one-
    // sample voltage impulse.
    il_ref = slew(g_il_ref_prev, il_ref_target, 0.125f);
    il_ref_dot = (il_ref - g_il_ref_prev) / CONTROL_TS;
    g_il_ref_prev = il_ref;
    g_il_ref = il_ref;

    if (controller_active != 0U)
    {
        error = il_ref - g_meas.il;
        v_pi = CURRENT_KP * error + g_pi_id.integral;
        denom = fmaxf(fabsf(g_meas.ui) + fabsf(g_meas.uo), 10.0f);
        qsign = (g_half == HALF_POS) ? 1.0f : -1.0f;
        d_unsat = dff + qsign * (L_HENRY * il_ref_dot + v_pi) / denom;
        g_duty = clampf_local(d_unsat, PWM_D_MIN, PWM_D_MAX);

        if (((d_unsat >= PWM_D_MIN) && (d_unsat <= PWM_D_MAX)) ||
            ((d_unsat > PWM_D_MAX) && (qsign * error < 0.0f)) ||
            ((d_unsat < PWM_D_MIN) && (qsign * error > 0.0f)))
        {
            g_pi_id.integral += g_pi_id.ki * CONTROL_TS * error;
            g_pi_id.integral = clampf_local(g_pi_id.integral, -30.0f, 30.0f);
        }
    }
    else
    {
        g_duty = dff;
        pi_reset(&g_pi_id);
    }

    if ((g_gate_state == GATE_ACTIVE) && predicted_zero())
    {
        HalfPolarity desired = (g_pll_ui.sogi.alpha >= 0.0f) ? HALF_POS : HALF_NEG;
        if (desired != g_half) request_half_change(desired);
    }

    if ((g_gate_start_pending != 0U) && predicted_zero() &&
        (g_fault == FAULT_OK))
    {
        g_half = (g_pll_ui.sogi.alpha >= 0.0f) ? HALF_POS : HALF_NEG;
        if (is_share_role() != 0U)
        {
            g_next_half = g_half;
            pwm_set_zc_a(g_half);
            if (pwm_clear_ost() == 0U) return;
#if POWER_STAGE_ENABLE == 1
            g_gate_state = GATE_ZC_A;
#endif
        }
        else
        {
            pwm_set_normal(g_half, g_duty);
            if (pwm_clear_ost() == 0U) return;
#if POWER_STAGE_ENABLE == 1
            g_gate_state = GATE_ACTIVE;
#endif
        }
        g_gate_start_pending = 0U;
    }
}

//=============================== Run state ====================================
static Uint16 voltage_current_limit_1ms(void)
{
    if (g_meas.io_rms >= I_BRANCH_CMD_MAX)
    {
        g_current_limit_active = 1U;
        g_current_release_ms = 0U;
    }
    else if (g_current_limit_active != 0U)
    {
        if (g_meas.io_rms <= 0.92f * I_BRANCH_CMD_MAX)
        {
            if (g_current_release_ms < 40U) g_current_release_ms++;
            if (g_current_release_ms >= 40U)
            {
                g_current_limit_active = 0U;
                g_current_limit_ms = 0U;
            }
        }
        else
        {
            g_current_release_ms = 0U;
        }
    }

    if (g_current_limit_active != 0U)
    {
        if (g_current_limit_ms < 501U) g_current_limit_ms++;
        if (g_current_limit_ms >= 500U) latch_fault(FAULT_START);
    }
    else
    {
        g_current_limit_ms = 0U;
    }
    return g_current_limit_active;
}

static void slow_state_machine_1ms(void)
{
    float target;
    g_state_ms++;

    if (g_fault != FAULT_OK)
    {
        g_run_state = ST_FAULT;
        pwm_force_off();
        if ((g_clear_request != 0U) && fault_clear_conditions())
        {
            g_fault = FAULT_OK;
            g_warning = FAULT_OK;
            g_clear_request = 0U;
            g_fault_clear_safe_ms = 0U;
            g_run_state = ST_SAFE;
            g_model_cmd = MODEL_SAFE;
            g_model_active = MODEL_SAFE;
            g_state_ms = 0UL;
        }
        return;
    }

    if ((g_stop_request != 0U) && (g_run_state != ST_SAFE) &&
        (g_run_state != ST_STOP_RAMP) && (g_run_state != ST_DISCHARGE) &&
        (g_run_state != ST_WAIT_MASTER_OFF))
    {
        g_run_state = ST_STOP_RAMP;
        g_state_ms = 0UL;
    }

#if POWER_STAGE_ENABLE == 0
    // First build is instrumentation-only.  Model selection remains visible,
    // but a start gesture cannot leave SAFE and cannot clear the OST latch.
    pwm_force_off();
    g_run_state = ST_SAFE;
    g_model_active = MODEL_SAFE;
    g_start_request = 0U;
    g_stop_request = 0U;
    g_state_ms = 0UL;
    return;
#endif

    switch (g_run_state)
    {
        case ST_SAFE:
            pwm_force_off();
            g_model_active = MODEL_SAFE;
            g_u_ref_active = 0.0f;
            g_share_alpha_ramp = 0.0f;
            g_bus_valid = 0U;
            g_ss5_stable_ms = 0U;
            g_voltage_stable_ms = 0U;
            g_voltage_error_ms = 0U;
            g_current_limit_ms = 0U;
            g_current_release_ms = 0U;
            g_current_limit_active = 0U;
            if (g_start_request != 0U)
            {
                g_start_request = 0U;
                if (model_is_legal(g_model_cmd) == 0U)
                {
                    g_warning = FAULT_MODE;
                    g_model_cmd = MODEL_SAFE;
                }
                else if (g_model_cmd != MODEL_SAFE)
                {
                    g_model_active = g_model_cmd;
                    g_run_state = ST_PRECHECK;
                    g_state_ms = 0UL;
                }
            }
            break;

        case ST_PRECHECK:
            if (!dead_bus() || !input_start_ok())
            {
                if (g_state_ms > 2000UL) latch_fault(FAULT_START);
                break;
            }
            pi_reset(&g_pi_id); pi_reset(&g_pi_vd); pi_reset(&g_pi_vq);
            pi_reset(&g_pi_sd); pi_reset(&g_pi_sq);
            g_run_state = ST_INPUT_PLL;
            g_state_ms = 0UL;
            break;

        case ST_INPUT_PLL:
            if (g_pll_ui.locked != 0U)
            {
                if (is_share_role() != 0U)
                    g_run_state = ST_WAIT_UO;
#if UNIT_ROLE == UNIT_2
                else if (g_model_active == MODEL_PARALLEL)
                    g_run_state = ST_SS5_RAMP;
#endif
                else g_run_state = ST_VOLT_RAMP;
                g_state_ms = 0UL;
            }
            else if (g_state_ms > 2000UL) latch_fault(FAULT_PLL);
            break;

        case ST_WAIT_UO:
            if (g_meas.uo_rms > 3.5f)
            {
                g_run_state = ST_OUTPUT_PLL;
                g_state_ms = 0UL;
            }
            break;

        case ST_OUTPUT_PLL:
            if (g_meas.uo_rms < 2.5f)
            {
                g_run_state = ST_WAIT_UO;
                g_state_ms = 0UL;
            }
            else if ((g_meas.uo_rms >= 4.0f) && (g_meas.uo_rms <= 6.5f) &&
                (g_pll_uo.omega >= TWO_PI_F * 49.0f) &&
                (g_pll_uo.omega <= TWO_PI_F * 51.0f) &&
                (g_pll_uo.locked != 0U) &&
                (fabsf(wrap_pi(g_pll_uo.theta - g_pll_ui.theta - PI_F)) <
                 (10.0f * PI_F / 180.0f)))
            {
                g_bus_valid = 1U;
                g_share_alpha_ramp = 0.0f;
                g_gate_start_pending = 1U;
                g_run_state = ST_SHARE_RAMP;
                g_state_ms = 0UL;
            }
            else if (g_state_ms > 1000UL)
            {
                if ((g_pll_uo.locked != 0U) &&
                    (fabsf(wrap_pi(g_pll_uo.theta - g_pll_ui.theta - PI_F)) >=
                     (10.0f * PI_F / 180.0f)))
                    latch_fault(FAULT_PHASE);
                else
                    latch_fault(FAULT_START);
            }
            break;

        case ST_SS5_RAMP:
            if (voltage_current_limit_1ms() != 0U)
                g_u_ref_active = slew(g_u_ref_active, 0.0f, 0.005f);
            else
                g_u_ref_active = slew(g_u_ref_active, 5.0f, 0.025f);
            if ((g_u_ref_active >= 1.0f) && (g_gate_state == GATE_BLOCKED))
                g_gate_start_pending = 1U;
            if ((g_u_ref_active >= 4.99f) && (g_meas.uo_rms >= 4.5f) &&
                (g_meas.uo_rms <= 5.5f) && (g_current_limit_active == 0U))
            {
                if (g_ss5_stable_ms < 100U) g_ss5_stable_ms++;
            }
            else g_ss5_stable_ms = 0U;
            if (g_ss5_stable_ms >= 100U)
            {
                g_run_state = ST_SS5_HOLD;
                g_state_ms = 0UL;
                g_ss5_stable_ms = 0U;
            }
            else if (g_state_ms > 1500UL) latch_fault(FAULT_START);
            break;

        case ST_SS5_HOLD:
            if (voltage_current_limit_1ms() != 0U)
                g_u_ref_active = slew(g_u_ref_active, 0.0f, 0.005f);
            else
                g_u_ref_active = slew(g_u_ref_active, 5.0f, 0.005f);
            if ((g_meas.uo_rms >= 4.5f) && (g_meas.uo_rms <= 5.5f) &&
                (g_current_limit_active == 0U))
            {
                if (g_ss5_stable_ms < 500U) g_ss5_stable_ms++;
            }
            else g_ss5_stable_ms = 0U;
            if (g_ss5_stable_ms >= 500U)
            {
                g_run_state = ST_VOLT_RAMP;
                g_state_ms = 0UL;
                g_voltage_stable_ms = 0U;
            }
            else if (g_state_ms > 1500UL) latch_fault(FAULT_START);
            break;

        case ST_VOLT_RAMP:
            target = effective_u_ref();
            if (voltage_current_limit_1ms() != 0U)
            {
                g_u_ref_active = slew(g_u_ref_active, 0.0f, 0.005f);
            }
            else
            {
                float ramp_target = ((target < 1.0f) &&
                                     (g_gate_state == GATE_BLOCKED)) ? 1.0f : target;
#if UNIT_ROLE == UNIT_2
                float ramp_span = (g_model_active == MODEL_PARALLEL) ?
                                  fmaxf(target - 5.0f, 1.0f) : fmaxf(target, 1.0f);
#else
                float ramp_span = fmaxf(target, 1.0f);
#endif
                g_u_ref_active = slew(g_u_ref_active, ramp_target,
                                      ramp_span / 1000.0f);
            }
            if ((g_u_ref_active >= 1.0f) && (g_gate_state == GATE_BLOCKED))
                g_gate_start_pending = 1U;
            if ((fabsf(g_u_ref_active - target) < 0.01f) &&
                (fabsf(g_meas.uo_rms - target) <=
                 fmaxf(0.5f, 0.02f * target)))
            {
                if (g_voltage_stable_ms < 100U) g_voltage_stable_ms++;
            }
            else g_voltage_stable_ms = 0U;
            if ((g_state_ms >= 1000UL) && (g_voltage_stable_ms >= 100U))
            {
                g_run_state = ST_RUN;
                g_state_ms = 0UL;
                g_voltage_error_ms = 0U;
            }
            else if (g_state_ms > 2000UL) latch_fault(FAULT_START);
            break;

        case ST_SHARE_RAMP:
            g_share_alpha_ramp = slew(g_share_alpha_ramp, 1.0f, 0.005f);
            if (g_share_alpha_ramp >= 0.999f)
            {
                g_run_state = ST_RUN;
                g_state_ms = 0UL;
            }
            break;

        case ST_RUN:
            if (is_voltage_role())
            {
                target = effective_u_ref();
                if (voltage_current_limit_1ms() != 0U)
                    g_u_ref_active = slew(g_u_ref_active, 0.0f, 0.005f);
                else
                    g_u_ref_active = slew(g_u_ref_active, target, 0.005f);

                if (fabsf(g_meas.uo_rms - g_u_ref_active) >
                    fmaxf(0.5f, 0.05f * g_u_ref_active))
                {
                    if (g_voltage_error_ms < 1000U) g_voltage_error_ms++;
                    if (g_voltage_error_ms >= 1000U) latch_fault(FAULT_START);
                }
                else g_voltage_error_ms = 0U;
            }
            break;

        case ST_STOP_RAMP:
            if (is_share_role())
            {
                g_share_alpha_ramp = slew(g_share_alpha_ramp, 0.0f, 0.005f);
                if (g_share_alpha_ramp <= 0.001f)
                {
                    pwm_force_off();
                    g_run_state = ST_WAIT_MASTER_OFF;
                    g_state_ms = 0UL;
                }
            }
            else
            {
                g_u_ref_active = slew(g_u_ref_active, 0.0f, 0.005f);
                if ((g_u_ref_active < 1.0f) &&
                    (g_meas.il_rms < 0.10f) && (g_meas.io_rms < 0.10f))
                {
                    pwm_force_off();
                    g_run_state = ST_DISCHARGE;
                    g_state_ms = 0UL;
                }
            }
            break;

        case ST_WAIT_MASTER_OFF:
            if (dead_bus())
            {
                g_run_state = ST_SAFE;
                g_model_cmd = MODEL_SAFE;
                g_stop_request = 0U;
                g_state_ms = 0UL;
            }
            break;

        case ST_DISCHARGE:
            if (dead_bus())
            {
                g_run_state = ST_SAFE;
                g_model_cmd = MODEL_SAFE;
                g_stop_request = 0U;
                g_state_ms = 0UL;
            }
            else if (g_state_ms > 3000UL) latch_fault(FAULT_DISCHARGE);
            break;

        default:
            latch_fault(FAULT_CPU);
            break;
    }
}

static void keys_update_5ms(void)
{
    const Uint16 raw[6] = {KEY1_IN, KEY2_IN, KEY3_IN,
                           KEY4_IN, KEY5_IN, KEY6_IN};
    Uint16 i;
    for (i = 0U; i < 6U; i++)
    {
        KeyState *k = &g_keys[i];
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
            if ((i == 3U) && (g_run_state == ST_FAULT) &&
                (k->held_ms >= 2000U) && (k->long_fired == 0U))
            {
                long_event = 1U; k->long_fired = 1U;
            }
            else if ((i == 3U) && (g_run_state == ST_SAFE) &&
                     (k->held_ms >= 1000U) && (k->long_fired == 0U))
            {
                long_event = 1U; k->long_fired = 1U;
            }
            if ((i != 0U) && (i != 1U) && (i != 2U) && (i != 3U) &&
                (k->repeat_ms >= 150U))
            {
                k->repeat_ms = 0U; k->repeat_edge = 1U;
            }
            if (((i == 0U) || (i == 1U)) &&
                (g_model_cmd == MODEL_PARALLEL) && (k->repeat_ms >= 150U))
            {
                k->repeat_ms = 0U; k->repeat_edge = 1U;
            }
        }
        key_action(i + 1U, k->press_edge | k->repeat_edge,
                   k->release_edge, long_event);
    }
}

static void key_action(Uint16 key, Uint16 repeat_event, Uint16 release_event,
                       Uint16 long_event)
{
    float hi = active_u_max();
    float direction = ((key == 1U) || (key == 5U)) ? -1.0f : 1.0f;

    if ((key == 3U) && (release_event != 0U) && (g_run_state == ST_SAFE))
    {
#if UNIT_ROLE == UNIT_1
        if (g_model_cmd == MODEL_SAFE) g_model_cmd = MODEL_VOLTAGE;
        else if (g_model_cmd == MODEL_VOLTAGE) g_model_cmd = MODEL_PARALLEL;
        else g_model_cmd = MODEL_SAFE;
#else
        g_model_cmd = (g_model_cmd == MODEL_SAFE) ? MODEL_PARALLEL : MODEL_SAFE;
#endif
        g_warning = FAULT_OK;
        return;
    }

    if (key == 4U)
    {
        if (long_event != 0U)
        {
            if (g_run_state == ST_SAFE) g_start_request = 1U;
            else if (g_run_state == ST_FAULT) g_clear_request = 1U;
        }
        else if ((release_event != 0U) && (g_keys[3].long_fired == 0U) &&
                 (g_run_state != ST_SAFE) && (g_run_state != ST_FAULT))
        {
            g_stop_request = 1U;
        }
        return;
    }

    if (g_run_state == ST_FAULT) return;
#if DIRECTION_DIAG_6V
    return;
#endif

#if UNIT_ROLE == UNIT_1
    if (((key == 1U) || (key == 2U)) &&
        (g_model_cmd == MODEL_PARALLEL) && (repeat_event != 0U))
    {
        g_k_cmd = clampf_local(g_k_cmd + direction * 0.05f, 0.5f, 2.0f);
        return;
    }
    if (((key == 1U) || (key == 2U)) &&
        (g_model_cmd == MODEL_VOLTAGE) && (release_event != 0U))
    {
        if (fabsf(g_u_ref_cmd - 1.0f) < 0.001f)
            g_trim_low = clampf_local(g_trim_low + direction * 0.025f,
                                      -0.100f, 0.100f);
        else if (fabsf(g_u_ref_cmd - hi) < 0.001f)
            g_trim_high = clampf_local(g_trim_high + direction * 0.025f,
                                       -0.100f, 0.100f);
        return;
    }
#endif

    if (((key == 5U) || (key == 6U)) && (repeat_event != 0U))
    {
        float lo = ((UNIT_ROLE == UNIT_2) ||
                    (g_model_cmd == MODEL_PARALLEL)) ? 5.0f : 1.0f;
        // KEY5 decrements and KEY6 increments on this hardware assignment.
        g_u_ref_cmd = clampf_local(g_u_ref_cmd + direction * 0.5f, lo, hi);
        g_u_ref_cmd = floorf(g_u_ref_cmd * 2.0f + 0.5f) * 0.5f;
    }
}
