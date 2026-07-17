//#############################################################################
//
//  F280049 一号机：从机均流算法移植版（无保护）
//
//  保留 converter1 的硬件与显示：
//    ADCA0 = 输入电压 Ui
//    ADCA1 = 输出电压 Uo
//    ADCB0 = 电感电流 iL
//    ADCB1 = 一号机支路输出电流 Io1
//    ADCB2 = 两机总输出电流 Itotal
//    OLED 采用 converter1 的四行轮流刷新格式。
//
//  控制算法直接采用“从机”文件的实际运行算法：
//    IZ_REF1 = Itotal
//    middle_i = IZ_REF1 * POL
//    IZ_REF2 = middle_i / (1 + k_ref)
//    err_i = IZ_REF2 - Ibranch * POL
//    result_i = k_i * err_i
//    duty = (result_i + PID_U_REF) / Vin
//
//  PWM 半周调制采用 converter1 的方式：
//    正半周：EPWM1 调制，EPWM2 固定；
//    负半周：EPWM2 调制，EPWM1 固定；
//    过零区：进入安全换向状态。
//
//  EPWM1/2 的时基、同步、AQ、影子装载与 Dead-Band 配置来自“从机”文件。
//  EPWM3 保留为同步的 20 kHz ADC 触发源，以匹配 converter1 的硬件工程。
//
//  已删除：全部软件过流/过压/ADC/输入电压保护、故障代码、故障状态机、
//          故障清除条件、CMPSS、Trip-Zone、OST 和保护快照逻辑。
//
//#############################################################################

#include "F28x_Project.h"
#include "driverlib.h"
#include "math.h"
#include "string.h"
#include "oled.h"

//==============================================================================
// 固定参数
//==============================================================================
#define EPWM_TIMER_TBPRD              2500U
#define PWM_DEAD_TICKS                15U
#define CONTROL_HZ                    20000.0f
#define RMS_WINDOW                    400U
#define RMS_PUBLISH_DIV               20U
#define OLED_REFRESH_ISR_DIV          2000U

#define MODE_OFF                      0U
#define MODE_SHARE                    1U

#define PWM_D_MIN                     0.05f
#define PWM_D_MAX                     0.95f
#define ZERO_CROSS_V                  0.80f
#define START_ZERO_SAMPLES            3U

#define K_REF_MIN                     0.30f
#define K_REF_MAX                     2.50f
#define K_REF_STEP                    0.10f
#define U_REF_MIN                     1.0f
#define U_REF_MAX                     35.0f
#define U_REF_STEP                    0.5f
#define VIN_NOMINAL                   36.0f

//==============================================================================
// 一号机 ADC 标定：physical = (raw - offset) / gain
//==============================================================================
#define UI_ADC_OFFSET                 2066.2f
#define UI_ADC_GAIN                   36.443f
#define UO_ADC_OFFSET                 2067.0f
#define UO_ADC_GAIN                   36.378f
#define IL_ADC_OFFSET                 2077.3f
#define IL_ADC_GAIN                   164.61f
#define IO_ADC_OFFSET                 2097.6f
#define IO_ADC_GAIN                   333.10f
#define IT_ADC_OFFSET                 2081.8f
#define IT_ADC_GAIN                   162.97f

#define UI_ADC_POLARITY               1.0f
#define UO_ADC_POLARITY               1.0f
#define IL_ADC_POLARITY               1.0f
#define IO_ADC_POLARITY               1.0f
#define IT_ADC_POLARITY               1.0f

//==============================================================================
// 按键
//==============================================================================
#define KEY_H1  (GpioDataRegs.GPADAT.bit.GPIO27)
#define KEY_H2  (GpioDataRegs.GPADAT.bit.GPIO25)
#define KEY_H3  (GpioDataRegs.GPADAT.bit.GPIO17)
#define KEY_H4  (GpioDataRegs.GPADAT.bit.GPIO26)
#define KEY_H5  (GpioDataRegs.GPADAT.bit.GPIO16)
#define KEY_H6  (GpioDataRegs.GPBDAT.bit.GPIO39)

#define KEY1_PRESS                    1
#define KEY2_PRESS                    2
#define KEY3_PRESS                    3
#define KEY4_PRESS                    4
#define KEY5_PRESS                    5
#define KEY6_PRESS                    6
#define KEY_UNPRESS                   0

//==============================================================================
// 函数声明
//==============================================================================
__interrupt void adcB1ISR(void);

void Init_KEY(void);
char KEY_Scan(char key_mode);
void KEY_Control(int key_value);

void OLED_Update_OneLine(void);
void clear_line(char *line);
void put_text(char *line, Uint16 pos, const char *text);
void put_fixed(char *line, Uint16 pos, float value, Uint16 decimals,
               Uint16 width);

void initADC(void);
void initADCSOC(void);
void initEPWM(void);
void EPWM1_Init(void);
void EPWM2_Init(void);
void EPWM3_Init(void);
void gate_gpio_hold_low(void);
void gate_gpio_enable_pwm(void);

float PID_firstOrderFilter(float data);
void PWM_ForceOff(void);
void PWM_Apply(float command_duty);
void Reset_Control_State(void);
void Start_Run(void);
void Stop_Run(void);
void Update_Rms_Window(void);
float clampf_local(float value, float low, float high);

//==============================================================================
// 运行状态与从机控制变量
//==============================================================================
volatile Uint16 mode = MODE_SHARE;
volatile Uint16 run_enable = 0U;
volatile Uint16 gate_armed = 0U;

float PID_U_REF = 30.0f;
float PID_IZ_REF = 3.0f;     // 保留从机原变量，当前均流算法未使用
float IZ_REF1 = 0.0f;
float IZ_REF2 = 0.0f;
float middle_i = 0.0f;
float err_i = 0.0f;
float result_i = 0.0f;
float k_i = 5.0f;
float k_ref = 1.0f;
float Vin = VIN_NOMINAL;

float Uin = 0.0f;
float Uout = 0.0f;
float IL = 0.0f;
float Ibranch = 0.0f;
float Itotal = 0.0f;
float U_lb = 0.0f;
float duty = PWM_D_MIN;
int POL = 0;
Uint16 start_zero_count = 0U;

volatile float UI_RMS = 0.0f;
volatile float UO_RMS = 0.0f;
volatile float IL_RMS = 0.0f;
volatile float IO_RMS = 0.0f;
volatile float IT_RMS = 0.0f;

volatile Uint16 raw_ui = 0U;
volatile Uint16 raw_uo = 0U;
volatile Uint16 raw_il = 0U;
volatile Uint16 raw_io = 0U;
volatile Uint16 raw_it = 0U;

float ui_buffer[RMS_WINDOW] = {0.0f};
float uo_buffer[RMS_WINDOW] = {0.0f};
float il_buffer[RMS_WINDOW] = {0.0f};
float io_buffer[RMS_WINDOW] = {0.0f};
float it_buffer[RMS_WINDOW] = {0.0f};

float ui_sq_sum = 0.0f;
float uo_sq_sum = 0.0f;
float il_sq_sum = 0.0f;
float io_sq_sum = 0.0f;
float it_sq_sum = 0.0f;
Uint16 rms_index = 0U;
Uint16 rms_fill = 0U;
Uint16 rms_publish_div = 0U;

float filter_a = 0.8f;
float filter_last = 0.0f;

int key = 0;
volatile Uint16 oled_tick_100ms = 0U;
Uint16 oled_isr_div = 0U;
Uint16 oled_line = 0U;

#ifdef _FLASH
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;
#endif

//==============================================================================
// 主函数
//==============================================================================
void main(void)
{
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart,
           (size_t)&RamfuncsLoadSize);
#endif

    InitSysCtrl();
    InitGpio();

    GPIO_unlockPortConfig(GPIO_PORT_A, 0xFFFFFFFFUL);
    GPIO_unlockPortConfig(GPIO_PORT_B, 0xFFFFFFFFUL);
    GPIO_unlockPortConfig(GPIO_PORT_H, 0xFFFFFFFFUL);

    gate_gpio_hold_low();

    DINT;
    InitPieCtrl();
    IER = 0x0000U;
    IFR = 0x0000U;
    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCB1_INT = &adcB1ISR;
    AnalogSubsysRegs.DCDCCTL.bit.DCDCEN = 0U;
    EDIS;

    Init_KEY();
    OLED_Init();
    OLED_Clean();
    initADC();
    initEPWM();
    gate_gpio_enable_pwm();
    initADCSOC();

    Reset_Control_State();
    PWM_ForceOff();

    IER |= M_INT1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 0U;
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1U;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1U;
    EDIS;

    EINT;
    ERTM;

    for (;;)
    {
        Uint16 refresh_oled = 0U;

        key = KEY_Scan(0);
        KEY_Control(key);

        DINT;
        if (oled_tick_100ms != 0U)
        {
            oled_tick_100ms--;
            refresh_oled = 1U;
        }
        EINT;

        if (refresh_oled != 0U)
        {
            OLED_Update_OneLine();
        }

        if (ui_sq_sum < 0.0f) ui_sq_sum = 0.0f;
        if (uo_sq_sum < 0.0f) uo_sq_sum = 0.0f;
        if (il_sq_sum < 0.0f) il_sq_sum = 0.0f;
        if (io_sq_sum < 0.0f) io_sq_sum = 0.0f;
        if (it_sq_sum < 0.0f) it_sq_sum = 0.0f;
    }
}

//==============================================================================
// ADCB1 中断：20 kHz，从机均流控制核心
//==============================================================================
__interrupt void adcB1ISR(void)
{
    raw_ui = AdcaResultRegs.ADCRESULT0;
    raw_uo = AdcaResultRegs.ADCRESULT1;
    raw_il = AdcbResultRegs.ADCRESULT0;
    raw_io = AdcbResultRegs.ADCRESULT1;
    raw_it = AdcbResultRegs.ADCRESULT2;

    Uin = UI_ADC_POLARITY * (((float)raw_ui - UI_ADC_OFFSET) / UI_ADC_GAIN);
    Uout = UO_ADC_POLARITY * (((float)raw_uo - UO_ADC_OFFSET) / UO_ADC_GAIN);
    IL = IL_ADC_POLARITY * (((float)raw_il - IL_ADC_OFFSET) / IL_ADC_GAIN);
    Ibranch = IO_ADC_POLARITY * (((float)raw_io - IO_ADC_OFFSET) / IO_ADC_GAIN);
    Itotal = IT_ADC_POLARITY * (((float)raw_it - IT_ADC_OFFSET) / IT_ADC_GAIN);

    Update_Rms_Window();

    U_lb = PID_firstOrderFilter(Uin);
    if (U_lb > ZERO_CROSS_V) POL = 1;
    else if (U_lb < -ZERO_CROSS_V) POL = -1;
    else POL = 0;

    if ((mode == MODE_OFF) || (run_enable == 0U))
    {
        PWM_ForceOff();
    }
    else if (mode == MODE_SHARE)
    {
        // 从机原算法：启动前等待输入过零区连续3个采样点。
        if (gate_armed == 0U)
        {
            if ((U_lb < ZERO_CROSS_V) && (U_lb > -ZERO_CROSS_V))
            {
                if (start_zero_count < START_ZERO_SAMPLES)
                {
                    start_zero_count++;
                }

                if (start_zero_count >= START_ZERO_SAMPLES)
                {
                    gate_armed = 1U;
                }
            }
            else
            {
                start_zero_count = 0U;
            }

            PWM_ForceOff();
        }
        else
        {
            //=================== 从机均流算法（原式） ===================
            IZ_REF1 = Itotal;
            middle_i = IZ_REF1 * (float)POL;
            IZ_REF2 = middle_i / (1.0f + k_ref);
            err_i = IZ_REF2 - Ibranch * (float)POL;
            result_i = k_i * err_i;

            duty = (result_i + PID_U_REF) / Vin;
            duty = clampf_local(duty, PWM_D_MIN, PWM_D_MAX);

            PWM_Apply(duty);
        }
    }
    else
    {
        // 非法模式不锁存故障，直接退回M0并关闭全部门极。
        mode = MODE_OFF;
        run_enable = 0U;
        PWM_ForceOff();
        Reset_Control_State();
    }

    oled_isr_div++;
    if (oled_isr_div >= OLED_REFRESH_ISR_DIV)
    {
        oled_isr_div = 0U;
        if (oled_tick_100ms < 10U) oled_tick_100ms++;
    }

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    if (AdcbRegs.ADCINTOVF.bit.ADCINT1 == 1U)
    {
        AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//==============================================================================
// PWM 调制：保持 converter1 的半周切换方式
//==============================================================================
void PWM_Apply(float command_duty)
{
    Uint16 cmp = (Uint16)((float)EPWM_TIMER_TBPRD * command_duty);

    EPwm1Regs.AQCSFRC.bit.CSFA = 0U;
    EPwm1Regs.AQCSFRC.bit.CSFB = 0U;
    EPwm2Regs.AQCSFRC.bit.CSFA = 0U;
    EPwm2Regs.AQCSFRC.bit.CSFB = 0U;

    if (U_lb >= ZERO_CROSS_V)
    {
        EALLOW;
        EPwm1Regs.DBCTL.bit.POLSEL = 2U;
        EPwm2Regs.DBCTL.bit.POLSEL = 0U;
        EDIS;

        EPwm1Regs.CMPA.bit.CMPA = cmp;
        EPwm2Regs.CMPA.bit.CMPA = EPWM_TIMER_TBPRD;
    }
    else if (U_lb <= -ZERO_CROSS_V)
    {
        EALLOW;
        EPwm1Regs.DBCTL.bit.POLSEL = 0U;
        EPwm2Regs.DBCTL.bit.POLSEL = 2U;
        EDIS;

        EPwm1Regs.CMPA.bit.CMPA = EPWM_TIMER_TBPRD;
        EPwm2Regs.CMPA.bit.CMPA = cmp;
    }
    else
    {
        EALLOW;
        EPwm1Regs.DBCTL.bit.POLSEL = 2U;
        EPwm2Regs.DBCTL.bit.POLSEL = 2U;
        EDIS;

        EPwm1Regs.CMPA.bit.CMPA = EPWM_TIMER_TBPRD;
        EPwm2Regs.CMPA.bit.CMPA = EPWM_TIMER_TBPRD;
    }
}

void PWM_ForceOff(void)
{
    EALLOW;
    EPwm1Regs.DBCTL.bit.POLSEL = 0U;
    EPwm2Regs.DBCTL.bit.POLSEL = 0U;
    EDIS;

    EPwm1Regs.CMPA.bit.CMPA = 0U;
    EPwm2Regs.CMPA.bit.CMPA = 0U;

    EPwm1Regs.AQCSFRC.bit.CSFA = 1U;
    EPwm1Regs.AQCSFRC.bit.CSFB = 1U;
    EPwm2Regs.AQCSFRC.bit.CSFA = 1U;
    EPwm2Regs.AQCSFRC.bit.CSFB = 1U;
}

//==============================================================================
// 启停与状态清零（无故障状态机）
//==============================================================================
void Reset_Control_State(void)
{
    IZ_REF1 = 0.0f;
    IZ_REF2 = 0.0f;
    middle_i = 0.0f;
    err_i = 0.0f;
    result_i = 0.0f;
    duty = PWM_D_MIN;
    gate_armed = 0U;
    start_zero_count = 0U;
}

void Start_Run(void)
{
    if (mode != MODE_SHARE)
    {
        PWM_ForceOff();
        return;
    }

    Reset_Control_State();
    PWM_ForceOff();
    run_enable = 1U;
}

void Stop_Run(void)
{
    run_enable = 0U;
    PWM_ForceOff();
    Reset_Control_State();
}

//==============================================================================
// RMS递推：仅用于OLED显示，不参与保护
//==============================================================================
void Update_Rms_Window(void)
{
    float old_ui = ui_buffer[rms_index];
    float old_uo = uo_buffer[rms_index];
    float old_il = il_buffer[rms_index];
    float old_io = io_buffer[rms_index];
    float old_it = it_buffer[rms_index];

    ui_sq_sum += Uin * Uin - old_ui * old_ui;
    uo_sq_sum += Uout * Uout - old_uo * old_uo;
    il_sq_sum += IL * IL - old_il * old_il;
    io_sq_sum += Ibranch * Ibranch - old_io * old_io;
    it_sq_sum += Itotal * Itotal - old_it * old_it;

    ui_buffer[rms_index] = Uin;
    uo_buffer[rms_index] = Uout;
    il_buffer[rms_index] = IL;
    io_buffer[rms_index] = Ibranch;
    it_buffer[rms_index] = Itotal;

    rms_index++;
    if (rms_index >= RMS_WINDOW) rms_index = 0U;
    if (rms_fill < RMS_WINDOW) rms_fill++;

    rms_publish_div++;
    if (rms_publish_div >= RMS_PUBLISH_DIV)
    {
        float divisor = (rms_fill < RMS_WINDOW) ?
                        (float)rms_fill : (float)RMS_WINDOW;
        rms_publish_div = 0U;

        if (divisor > 0.0f)
        {
            UI_RMS = sqrtf(fmaxf(ui_sq_sum, 0.0f) / divisor);
            UO_RMS = sqrtf(fmaxf(uo_sq_sum, 0.0f) / divisor);
            IL_RMS = sqrtf(fmaxf(il_sq_sum, 0.0f) / divisor);
            IO_RMS = sqrtf(fmaxf(io_sq_sum, 0.0f) / divisor);
            IT_RMS = sqrtf(fmaxf(it_sq_sum, 0.0f) / divisor);
        }
    }
}

float clampf_local(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

float PID_firstOrderFilter(float data)
{
    filter_last = filter_a * data + (1.0f - filter_a) * filter_last;
    return filter_last;
}

//==============================================================================
// OLED：参考 converter1 的四行显示
//==============================================================================
void OLED_Update_OneLine(void)
{
    char line[17];

    clear_line(line);

    if (oled_line == 0U)
    {
        put_text(line, 0U, "U1");
        put_text(line, 3U, (mode == MODE_SHARE) ? "M1SH" : "M0--");

        if (run_enable == 0U)
            put_text(line, 8U, "STOP");
        else if (gate_armed == 0U)
            put_text(line, 8U, "WAIT");
        else
            put_text(line, 8U, "RUN ");
    }
    else if (oled_line == 1U)
    {
        put_text(line, 0U, "U");
        put_fixed(line, 1U, UO_RMS, 1U, 5U);
        put_text(line, 7U, "r");
        put_fixed(line, 8U, PID_U_REF, 1U, 5U);
    }
    else if (oled_line == 2U)
    {
        put_text(line, 0U, "L");
        put_fixed(line, 1U, IL_RMS, 2U, 5U);
        put_text(line, 7U, "o");
        put_fixed(line, 8U, IO_RMS, 2U, 5U);
    }
    else
    {
        put_text(line, 0U, "K");
        put_fixed(line, 1U, k_ref, 2U, 4U);
        put_text(line, 6U, "t");
        put_fixed(line, 7U, IT_RMS, 2U, 5U);
        put_text(line, 13U, "D");
        put_fixed(line, 14U, duty * 100.0f, 0U, 2U);
    }

    OLED_ShowString(0U, (Uchar)oled_line, line);
    oled_line = (oled_line + 1U) & 3U;
}

void clear_line(char *line)
{
    Uint16 i;
    for (i = 0U; i < 16U; i++) line[i] = ' ';
    line[16] = '\0';
}

void put_text(char *line, Uint16 pos, const char *text)
{
    while ((*text != '\0') && (pos < 16U))
    {
        line[pos++] = *text++;
    }
}

void put_fixed(char *line, Uint16 pos, float value, Uint16 decimals,
               Uint16 width)
{
    Uint32 scale = 1UL;
    Uint32 integer_value;
    Uint16 i;
    Uint16 negative = (value < 0.0f) ? 1U : 0U;

    for (i = 0U; i < decimals; i++) scale *= 10UL;
    integer_value = (Uint32)(fabsf(value) * (float)scale + 0.5f);

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
            line[at] = (char)('0' + (integer_value % 10UL));
            integer_value /= 10UL;
        }
    }

    if ((negative != 0U) && (pos < 16U)) line[pos] = '-';
}

//==============================================================================
// 按键：保留 converter1 的使用方式
//==============================================================================
void KEY_Control(int key_value)
{
    switch (key_value)
    {
        case KEY1_PRESS:
            k_ref += K_REF_STEP;
            if (k_ref > K_REF_MAX) k_ref = K_REF_MAX;
            break;

        case KEY2_PRESS:
            k_ref -= K_REF_STEP;
            if (k_ref < K_REF_MIN) k_ref = K_REF_MIN;
            break;

        case KEY3_PRESS:
            if (run_enable == 0U)
            {
                if (mode == MODE_SHARE) mode = MODE_OFF;
                else mode = MODE_SHARE;

                PWM_ForceOff();
                Reset_Control_State();
            }
            break;

        case KEY4_PRESS:
            if (run_enable == 0U) Start_Run();
            else Stop_Run();
            break;

        case KEY5_PRESS:
            PID_U_REF -= U_REF_STEP;
            if (PID_U_REF < U_REF_MIN) PID_U_REF = U_REF_MIN;
            break;

        case KEY6_PRESS:
            PID_U_REF += U_REF_STEP;
            if (PID_U_REF > U_REF_MAX) PID_U_REF = U_REF_MAX;
            break;

        default:
            break;
    }
}

char KEY_Scan(char key_mode)
{
    static char key_lock = 1;

    if (key_mode != 0) return KEY_UNPRESS;

    if ((key_lock == 1) &&
        ((KEY_H1 == 0) || (KEY_H2 == 0) || (KEY_H3 == 0) ||
         (KEY_H4 == 0) || (KEY_H5 == 0) || (KEY_H6 == 0)))
    {
        DELAY_US(5000U);
        key_lock = 0;

        if (KEY_H1 == 0) return KEY1_PRESS;
        if (KEY_H2 == 0) return KEY2_PRESS;
        if (KEY_H3 == 0) return KEY3_PRESS;
        if (KEY_H4 == 0) return KEY4_PRESS;
        if (KEY_H5 == 0) return KEY5_PRESS;
        if (KEY_H6 == 0) return KEY6_PRESS;
    }
    else if ((KEY_H1 == 1) && (KEY_H2 == 1) && (KEY_H3 == 1) &&
             (KEY_H4 == 1) && (KEY_H5 == 1) && (KEY_H6 == 1))
    {
        key_lock = 1;
    }

    return KEY_UNPRESS;
}

void Init_KEY(void)
{
    const Uint16 pins[6] = {27U, 25U, 17U, 26U, 16U, 39U};
    Uint16 idx;

    for (idx = 0U; idx < 6U; idx++)
    {
        GPIO_setAnalogMode(pins[idx], GPIO_ANALOG_DISABLED);
        GPIO_setPadConfig(pins[idx], GPIO_PIN_TYPE_PULLUP);
        GPIO_setDirectionMode(pins[idx], GPIO_DIR_MODE_IN);
        GPIO_setQualificationMode(pins[idx], GPIO_QUAL_6SAMPLE);
        GPIO_setQualificationPeriod(pins[idx], 20U);
    }
}

//==============================================================================
// ADC初始化：保留 converter1 的采样映射与掉电重启恢复流程
//==============================================================================
void initADC(void)
{
    EALLOW;
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 0U;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 0U;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 0U;
    EDIS;
    DELAY_US(100U);

    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdcaRegs.ADCINTFLGCLR.all = 0x000FU;
    AdcaRegs.ADCINTOVFCLR.all = 0x000FU;
    AdcaRegs.ADCSOCOVFCLR1.all = 0xFFFFU;
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1U;
    EDIS;
    DELAY_US(1000U);

    ADC_setVREF(ADCB_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    EALLOW;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6U;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1U;
    AdcbRegs.ADCINTFLGCLR.all = 0x000FU;
    AdcbRegs.ADCINTOVFCLR.all = 0x000FU;
    AdcbRegs.ADCSOCOVFCLR1.all = 0xFFFFU;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1U;
    EDIS;
    DELAY_US(1000U);

    ADC_setVREF(ADCC_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
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

void initADCSOC(void)
{
    EALLOW;

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0U;
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 9U;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 9U;

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1U;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 9U;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 9U;

    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0U;
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 9U;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 9U;

    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1U;
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 9U;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 9U;

    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2U;
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 9U;
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 9U;

    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 0U;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 2U;
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1U;
    AdcbRegs.ADCINTSEL1N2.bit.INT1CONT = 0U;
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1U;
    AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1U;

    EDIS;
}

//==============================================================================
// GPIO与EPWM
//==============================================================================
void gate_gpio_hold_low(void)
{
    EALLOW;

    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0U;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0U;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0U;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0U;

    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1U;
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1U;

    GpioDataRegs.GPACLEAR.bit.GPIO0 = 1U;
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1U;
    GpioDataRegs.GPACLEAR.bit.GPIO2 = 1U;
    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1U;

    EDIS;
}

void gate_gpio_enable_pwm(void)
{
    EALLOW;

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

void initEPWM(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0U;
    EDIS;

    EPWM1_Init();
    EPWM2_Init();
    EPWM3_Init();

    PWM_ForceOff();
}

// EPWM1/2配置直接采用从机文件的时基、AQ、影子和Dead-Band设置。
void EPWM1_Init(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1U;
    EDIS;

    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm1Regs.TBPHS.all = 0UL;
    EPwm1Regs.TBCTR = 0U;
    EPwm1Regs.TBPRD = EPWM_TIMER_TBPRD;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPA.bit.CMPA = 0U;
    EPwm1Regs.CMPB.bit.CMPB = 0U;

    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;

    EPwm1Regs.DBCTL.bit.IN_MODE = 0U;
    EPwm1Regs.DBCTL.bit.POLSEL = 2U;
    EPwm1Regs.DBCTL.bit.OUT_MODE = 3U;
    EPwm1Regs.DBRED.bit.DBRED = PWM_DEAD_TICKS;
    EPwm1Regs.DBFED.bit.DBFED = PWM_DEAD_TICKS;

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
    EPwm1Regs.ETSEL.bit.INTEN = 0U;
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;
}

void EPWM2_Init(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1U;
    EDIS;

    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm2Regs.TBPHS.all = 0UL;
    EPwm2Regs.TBCTR = 0U;
    EPwm2Regs.TBPRD = EPWM_TIMER_TBPRD;
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPA.bit.CMPA = 0U;
    EPwm2Regs.CMPB.bit.CMPB = 0U;

    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;
    EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;

    EPwm2Regs.DBCTL.bit.IN_MODE = 0U;
    EPwm2Regs.DBCTL.bit.POLSEL = 2U;
    EPwm2Regs.DBCTL.bit.OUT_MODE = 3U;
    EPwm2Regs.DBRED.bit.DBRED = PWM_DEAD_TICKS;
    EPwm2Regs.DBFED.bit.DBFED = PWM_DEAD_TICKS;

    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
    EPwm2Regs.ETSEL.bit.INTEN = 0U;
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;
}

void EPWM3_Init(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1U;
    EDIS;

    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm3Regs.TBPHS.all = 0UL;
    EPwm3Regs.TBCTR = 0U;
    EPwm3Regs.TBPRD = EPWM_TIMER_TBPRD;
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPA.bit.CMPA = 0U;
    EPwm3Regs.CMPB.bit.CMPB = 0U;

    EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;
    EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;

    EPwm3Regs.DBCTL.bit.IN_MODE = 0U;
    EPwm3Regs.DBCTL.bit.POLSEL = 2U;
    EPwm3Regs.DBCTL.bit.OUT_MODE = 3U;
    EPwm3Regs.DBRED.bit.DBRED = PWM_DEAD_TICKS;
    EPwm3Regs.DBFED.bit.DBFED = PWM_DEAD_TICKS;

    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
    EPwm3Regs.ETSEL.bit.INTEN = 0U;
    EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;

    // converter1使用EPWM3 SOCA，保持20 kHz波谷固定采样。
    EPwm3Regs.ETSEL.bit.SOCAEN = 1U;
    EPwm3Regs.ETSEL.bit.SOCASEL = 1U;
    EPwm3Regs.ETPS.bit.SOCAPRD = ET_1ST;
}
