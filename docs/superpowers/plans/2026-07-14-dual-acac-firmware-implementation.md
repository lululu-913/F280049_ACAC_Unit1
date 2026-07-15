# 双机并联单相 AC-AC 固件 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 从现有F280049工程复制出固定角色的1号机与2号机固件，实现20 kHz直接AC-AC Buck-Boost的采样、保护、门极、PLL、稳压/分流控制、按键/OLED和安全状态机，首版不实现AUX_OK。

**Architecture:** 两份CCS工程共用同一单文件架构，功能代码仅在各自`main.c`中；两份源代码除`UNIT_ROLE`外保持一致。20 kHz ADCB EOC2 ISR负责采样、快速保护、SOGI/PLL、控制计算和PWM影子更新，1 kHz任务负责状态机，主循环负责按键、OLED和看门狗健康握手。所有启动都先锁存ePWM One-Shot Trip；默认构建的`POWER_STAGE_ENABLE=0`只允许无功率联调。

**Tech Stack:** TI TMS320F280049、C2000Ware位域头文件与DriverLib、TI C2000 Compiler 22.6.1.LTS、CCS 12 Debug/Flash/EABI、PowerShell静态策略测试。

---

## 实施约束

- 设计依据：`docs/superpowers/specs/2026-07-14-dual-acac-parallel-control-design.md`。
- 原始工程`D:\CCS\workspace_v12\F280049_Xiaosai`只作为模板；保留用户未提交的`main.c`变化，不覆盖、不回退、不纳入本次固件提交。
- 新目录固定为`D:\CCS\workspace_v12\F280049_ACAC_Unit1`与`D:\CCS\workspace_v12\F280049_ACAC_Unit2`。
- 仅更新新工程的`.project`名称、launch文件名/项目引用以及`main.c`；不重配库、linker或外设支持文件。
- 首版没有`AUX_OK`、`AUX_UV_TRIP`、辅助ADC/GPIO、F17生成路径；`ADCINB3`不配置。`AUX_TOPOLOGY`只标识人工验证过的实际供电拓扑。
- 只构建Debug；Release仍是历史COFF配置，不用于下载或验收。

## Task 1：建立先失败的固件策略测试

**Files:**

- Create: `tests/acac_firmware_policy.ps1`
- Read: `docs/superpowers/specs/2026-07-14-dual-acac-parallel-control-design.md`

**Step 1: 写入失败测试**

测试必须读取两份目标`main.c`，逐项断言：

```powershell
$ErrorActionPreference = 'Stop'
$units = @(
    @{ Path = 'D:\CCS\workspace_v12\F280049_ACAC_Unit1\main.c'; Role = 'UNIT_1' },
    @{ Path = 'D:\CCS\workspace_v12\F280049_ACAC_Unit2\main.c'; Role = 'UNIT_2' }
)

foreach ($unit in $units) {
    if (-not (Test-Path -LiteralPath $unit.Path)) { throw "Missing $($unit.Path)" }
    $text = Get-Content -Raw -LiteralPath $unit.Path
    foreach ($required in @(
        "#define UNIT_ROLE $($unit.Role)",
        '#define POWER_STAGE_ENABLE 0U',
        '#define POWER_PROFILE PROFILE_HALF',
        '#define HALF_CURRENT_STAGE HALF_SAFE',
        'ADCBRESULT2', 'ADCINB1', 'ADCB EOC2',
        'IL_TO_OUTPUT_SIGN', 'PWM_BLOCKED', 'OUTPUT_CONNECTED_ZC',
        'CMPSS7', 'TZFRC', 'model_cmd', 'Utrim_lo', 'Utrim_hi'
    )) {
        if ($text -notmatch [regex]::Escape($required)) { throw "$($unit.Role): missing $required" }
    }
    foreach ($forbidden in @('AUX_OK', 'AUX_UV', 'ADCINB3', 'FAULT_AUX', 'F17')) {
        if ($text -match $forbidden) { throw "$($unit.Role): forbidden token $forbidden" }
    }
}

$u1 = Get-Content -Raw -LiteralPath $units[0].Path
$u2 = Get-Content -Raw -LiteralPath $units[1].Path
$normalized1 = $u1 -replace '#define UNIT_ROLE UNIT_1', '#define UNIT_ROLE UNIT_X'
$normalized2 = $u2 -replace '#define UNIT_ROLE UNIT_2', '#define UNIT_ROLE UNIT_X'
if ($normalized1 -cne $normalized2) { throw 'Unit main.c files differ beyond UNIT_ROLE' }

'AC-AC firmware policy: PASS'
```

**Step 2: 运行并确认失败**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\acac_firmware_policy.ps1
```

Expected: `Missing D:\CCS\workspace_v12\F280049_ACAC_Unit1\main.c`。

**Step 3: 提交测试**

```powershell
git add tests/acac_firmware_policy.ps1 docs/superpowers/plans/2026-07-14-dual-acac-firmware-implementation.md
git commit -m "test: define dual ac-ac firmware policy"
```

## Task 2：安全复制并改名两个CCS工程

**Files:**

- Create: `D:\CCS\workspace_v12\F280049_ACAC_Unit1\...`
- Create: `D:\CCS\workspace_v12\F280049_ACAC_Unit2\...`
- Modify: each `.project`
- Rename/Modify: each `.launches\F280049_Xiaosai.launch`

**Step 1: 验证目标不存在**

```powershell
Resolve-Path 'D:\CCS\workspace_v12\F280049_Xiaosai'
Test-Path 'D:\CCS\workspace_v12\F280049_ACAC_Unit1'
Test-Path 'D:\CCS\workspace_v12\F280049_ACAC_Unit2'
```

Expected:源目录解析成功，两个目标均为`False`。若任一目标存在，停止而不是覆盖。

**Step 2: 复制允许清单**

对每个目标只复制根工程文件、`.settings`、`.launches`、`cmd`、`include`、`lib`、`src`、`targetConfigs`及两个linker文件；排除`.git`、`Debug`、`Release`、`.agents`、`.claude`、`.superpowers`、`.vscode`和所有构建产物。使用PowerShell原生`Copy-Item`，复制后用`Resolve-Path`确认两个目标仍在`D:\CCS\workspace_v12`内。

**Step 3: 更新最小工程标识**

- `.project`只替换`<name>F280049_Xiaosai</name>`。
- launch文件改为对应工程名，并只替换文件内六处项目名。
- `.cproject`和`.ccsproject`保持字节不变。

**Step 4: 校验副本**

```powershell
rg -n "F280049_Xiaosai" D:\CCS\workspace_v12\F280049_ACAC_Unit1\.project D:\CCS\workspace_v12\F280049_ACAC_Unit1\.launches
rg -n "F280049_Xiaosai" D:\CCS\workspace_v12\F280049_ACAC_Unit2\.project D:\CCS\workspace_v12\F280049_ACAC_Unit2\.launches
```

Expected:无输出。再比较两工程除`.project`、`.launches`和`main.c`外的文件哈希完全一致。

## Task 3：重建单文件固件骨架与编译期安全合同

**Files:**

- Replace: `D:\CCS\workspace_v12\F280049_ACAC_Unit1\main.c`
- Replace: `D:\CCS\workspace_v12\F280049_ACAC_Unit2\main.c`

**Step 1: 先写公共配置、类型和编译守卫**

两份文件使用同一内容，仅此行不同：

```c
#define UNIT_ROLE UNIT_1 /* Unit2 uses UNIT_2 */
```

公共安全配置固定为：

```c
#define POWER_STAGE_ENABLE          0U
#define POWER_PROFILE               PROFILE_HALF
#define HALF_CURRENT_STAGE          HALF_SAFE
#define DIRECTION_DIAG_6V           0U
#define AUX_TOPOLOGY                AUX_SHARED_FORMAL
#define HALF_IHI_TEST_PASSED_ACK    0U
#define FORMAL_HW_RELEASE_ACK       0U

#if (POWER_PROFILE == PROFILE_FULL) && \
    ((!HALF_IHI_TEST_PASSED_ACK) || (!FORMAL_HW_RELEASE_ACK))
#error PROFILE_FULL requires HALF IHI and formal hardware release records
#endif
#if (POWER_PROFILE == PROFILE_FULL) && (AUX_TOPOLOGY != AUX_SHARED_FORMAL)
#error PROFILE_FULL requires AUX_SHARED_FORMAL
#endif
#if DIRECTION_DIAG_6V && (AUX_TOPOLOGY != AUX_SHARED_LAB_DEBUG)
#error 6 V direction diagnostic requires isolated laboratory auxiliary supply
#endif
```

定义`UnitRole`、`PowerProfile`、`HalfCurrentStage`、`Model`、`RunState`、`GateState`、`HalfCycle`和`FaultCode`；`FaultCode`只到F16，不定义F17。建立`Measurements`、`SogiPll`、`PiController`、`RmsWindow`、`ControlState`、`ProtectionState`、`KeyState`和只读`OledSnapshot`。

**Step 2: 建立所有静态职责函数**

```c
static void board_init(void);
static void adc_init(void);
static void adc_soc_init(void);
static void pwm_init_blocked(void);
static void cmpss7_trip_init(void);
static void trip_force_all(void);
static bool trip_clear_if_safe(void);
static void measurements_update(void);
static void signal_processing_step(void);
static void control_step(void);
static void gate_step(void);
static void protection_fast_step(void);
static void state_machine_1khz(void);
static void keys_5ms(void);
static void oled_10hz(void);
static void watchdog_handshake(void);
__interrupt void adcb1_isr(void);
```

**Step 3: 建立最小main和永远封锁的ISR**

`main()`按原工程完成Flash搬运、系统/PIE/OLED/按键/ADC/PWM初始化，第一条功率动作是`trip_force_all()`；ADCB1 ISR只读ADC、更新显示快照并再次强制OST。此时不得有清OST调用。

**Step 4: 运行策略测试**

Run Task 1脚本。Expected:除尚未实现的所需符号外逐项推进，最终到下一Task前通过角色一致性和禁用AUX检查。

## Task 4：实现固定时序ADC、标定和统计量

**Files:** modify both `main.c`

**Step 1: 配置采样时基**

- EPWM3中心对齐、`TBPRD=2500`，不输出GPIO，PRD处SOCA；控制代码永不修改其CMP。
- ADCA SOC0=A0/Ui、SOC1=A1/Uo。
- ADCB SOC0=B0/iL、SOC1=B1/Io。
- Unit1 SOC2=B2/Itotal；Unit2 SOC2再次选择B1/Io2，物理B2/B3不参与控制。
- ADCB INT1由EOC2触发，PIE向量使用`adcb1_isr`。

**Step 2: 写入独立标定常量**

```c
Ui       = ((float)raw_ui     - 2043.6f) / 36.499f;
Uo       = ((float)raw_uo     - 2047.3f) / 36.323f;
Io_local = ((float)raw_io     - 2032.1f) / 324.40f;
iL       = ((float)raw_il     - 2050.2f) / 152.02f;
Itotal   = ((float)raw_itotal - 2049.5f) / 163.70f;
```

后两式只标记为10AB无功率联调初值；`POWER_STAGE_ENABLE`解除前必须用各实物正负多点标定替换。每通道保留独立`offset/gain/polarity`常量，控制中不得用`fabsf()`掩盖测量极性。

**Step 3: 实现统计量**

- Ui/Uo基波RMS取SOGI幅值/√2。
- iL、Io、Itotal分别使用400样本滚动平方和与直流平均值。
- Io前馈使用400 Hz一阶低通，系数0.881911。
- 所有原始码连续3次落在`<32`或`>4063`锁存F09。

**Step 4: 无功率验证**

用直流注入ADC，确认零点、正负号、Unit2 RESULT2确实随B1而非B2变化；示波器测ISR测试GPIO，目标<25 µs、绝对<50 µs。

## Task 5：实现CMPSS7、Trip Zone和四管AQ门极引擎

**Files:** modify both `main.c`

**Step 1: CMPSS/TZ先于任何清Trip路径实现**

- CMPSS7正负阈值按B0的最终offset/gain换算为DAC码。
- 高/低比较器OR到ePWM X-BAR/Digital Compare异步路径。
- EPWM1A/B与EPWM2A/B的OST动作全部强制低。
- 所有软件故障统一写两模块`TZFRC.OST=1`。
- 清Trip只能由`trip_clear_if_safe()`在SAFE、死母线、CMPSS正常100 ms、ADC原始码64～4031且`POWER_STAGE_ENABLE=1`时执行。

**Step 2: 固定物理映射**

```text
EPWM1A/GPIO0=Q1a, EPWM1B/GPIO1=Q2a,
EPWM2A/GPIO2=Q1b, EPWM2B/GPIO3=Q2b
```

Dead-Band永久旁路。活动p/pbar由独立AQ生成：上数ZERO置p、CAU关p、CBU开pbar；下数CBD关pbar、CAD开p，`CMPB=CMPA+25`。常导通对和过零过渡只在ZERO同步改变AQ图样。

**Step 3: 实现门极状态**

`PWM_BLOCKED`、`OUTPUT_CONNECTED_ZC`、`ACTIVE`与`POS/ZC_PN/NEG/ZC_NP`互相独立。过零只置pending，ZC_A和ZC_B分别在后续两个ZERO应用；250 ns等待由AQ比较事件产生，代码中禁止`DELAY_US`、NOP或忙等。

**Step 4: 无功率门极测试**

保持MOS断开，逐半周检查四个MCU输出：无同对重叠、死区250 ns、Trip信号1～2 µs内四路低并锁存、复位和model0从不输出脉冲。通过前不修改`POWER_STAGE_ENABLE`。

## Task 6：实现SOGI-PLL和三级控制链

**Files:** modify both `main.c`

**Step 1: 单相SOGI-PLL**

用Tustin离散SOGI，`k=√2`，PLL初值`Kp=88.9`、`Ki=3948`、频率限45～55 Hz。PLL_UI始终运行；只有Unit1/model2在5 V平台建立后启用独立PLL_UO。锁定条件是幅值窗口、频率窗口和归一化q误差<0.05连续5周期。

**Step 2: 稳压角色外环**

Unit1/model1与Unit2/model2共用`Kpv=1.49e-3`、`Kiv=5.31e-2`，目标`uo_ref=-sqrt(2)*Uref_active*sin(theta_i)`；输出电流参考为`Io_ff + Co*duo_ref/dt + iv_corr`。Co固定21 µF，外环和支路电流均条件积分/限幅。

**Step 3: Unit1/model2分流环**

`alpha=K/(1+K)`，`io1_ref=alpha*Itotal_fund`，`Kps=0.25`、`Kis=78.5`；K命令0.5～2、步进0.05、变化率0.5/s，并按Itotal RMS和本档支路能力实时夹取安全K区间。禁止Unit1失去2号机时自动改成稳压。

**Step 4: 电流内环和前馈**

```c
iL_ref = -iconv_ref / fmaxf(1.0f - Dff, 0.15f);
Dff = Uo_amp / fmaxf(Ui_amp + Uo_amp, 10.0f);
D_unsat = Dff + q * (L * diL_ref_dt + 5.03f * e_i + x_i)
                   / fmaxf(fabsf(Ui) + fabsf(Uo), 10.0f);
D_cmd = clampf(D_unsat, 0.02f, 0.85f);
```

`L=1 mH`、`Kii=632`，但首次功率调试按代码开关先`Ki=0`；Trip、PLL未锁、ZC、限幅和模式切换时冻结或无扰预置积分器。

**Step 5: 数值回放**

用正弦ADC样本离线/CCS Graph观察：Ui和Uo反相、`IL_TO_OUTPUT_SIGN=-1`、Dff在HALF 18→15 V约0.455、所有占空比与电流参考均有界且无NaN。

## Task 7：实现模式、软启动、停机和故障状态机

**Files:** modify both `main.c`

**Step 1: 1 kHz统一状态机**

实现`BOOT, SAFE, PRECHECK, INPUT_PLL, WAIT_UO, OUTPUT_PLL, SS5_RAMP, SS5_HOLD, SHARE_RAMP, VOLT_RAMP, RUN, STOP_RAMP, WAIT_MASTER_OFF, DISCHARGE, FAULT`。PRECHECK不含任何辅电变量。

**Step 2: 固定角色合法性**

- Unit1：model0/1/2；model1稳压，model2分流。
- Unit2：model0/2；收到model1强制回0、保持SAFE并显示F12提示，不进入锁存FAULT。
- 模式切换只在死母线SAFE锁存；活动中改变model只软停，不自动以新角色启动。

**Step 3: 启停序列**

- 稳压角色：`PRECHECK→INPUT_PLL→VOLT_RAMP→RUN`，参考低于1 V保持OST；在预测过零清Trip并以5 V/s斜升。
- Unit1分流：`PRECHECK→WAIT_UO→OUTPUT_PLL→SHARE_RAMP→RUN`；只在公共Uo有效且相位差<10°的预测过零接入。
- Unit2/model2先建立5 V平台并保持500 ms，再等待Unit1投入后升到最终参考。
- 正常停机执行退流/降压/死母线确认；严重故障立即OST。

**Step 4: 保护和清故障**

实现F01～F16中规格定义且有传感器/逻辑来源的路径；F16保持预留但不自动生成。首故障冻结Ui/Uo/iL/Io/Itotal、状态、D和PLL。任何复位令`model_cmd=0`且保持OST，不自动恢复。清除只回SAFE。

## Task 8：实现按键、OLED和看门狗

**Files:** modify both `main.c`

**Step 1: 按键状态机**

GPIO27/25/17/26/16/39均为普通数字输入；5 ms扫描、25 ms消抖。KEY3切model，KEY4长按1 s启动/短按软停/FAULT长按2 s清除，KEY5/6按0.5 V粗调并150 ms连发。

**Step 2: 实现端点25 mV微调复用**

Unit1/model1且命令恰为1.0 V或当前高端17.5/35 V时，KEY1/2在释放沿调整对应trim ±0.025 V，范围±0.100 V且不连发；Unit1/model2时KEY1/2优先调整K。

**Step 3: OLED四行定宽页面**

10 Hz刷新且每次只写一行；首行组合`HSAFE/HIHI/FULL/HSDbg/HIDbg/6VDg + 角色 + 状态`，不显示板号。故障页覆盖普通页。自行写定点格式化，支持负号、三位数和trim；不在ISR调用OLED或字符串函数。

**Step 4: 看门狗双健康握手**

ISR和主循环各自递增心跳，只有监控任务看到二者均前进才喂狗；看门狗/非法复位锁存F15并保持OST。

## Task 9：角色复制一致性、静态策略和CCS构建

**Files:**

- Modify: Unit2 `main.c` role line only
- Verify: both Debug configurations

**Step 1: 生成Unit2**

从已完成的Unit1 `main.c`复制到Unit2，然后只把`#define UNIT_ROLE UNIT_1`改为`UNIT_2`。

**Step 2: 运行静态策略测试**

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\acac_firmware_policy.ps1
```

Expected: `AC-AC firmware policy: PASS`。

另运行：

```powershell
rg -n "AUX_OK|AUX_UV|ADCINB3|F17|DELAY_US.*(dead|zc)|EPWM[45678]" D:\CCS\workspace_v12\F280049_ACAC_Unit1\main.c D:\CCS\workspace_v12\F280049_ACAC_Unit2\main.c
```

Expected:无输出。

**Step 3: 导入两个项目**

关闭图形CCS后分别执行：

```powershell
D:\CCS\ccs\eclipse\eclipsec.exe -noSplash -data D:\CCS\workspace_v12 -application com.ti.ccstudio.apps.projectImport -ccs.location D:\CCS\workspace_v12\F280049_ACAC_Unit1
D:\CCS\ccs\eclipse\eclipsec.exe -noSplash -data D:\CCS\workspace_v12 -application com.ti.ccstudio.apps.projectImport -ccs.location D:\CCS\workspace_v12\F280049_ACAC_Unit2
```

**Step 4: 构建Debug/Flash/EABI**

```powershell
D:\CCS\ccs\eclipse\eclipsec.exe -noSplash -data D:\CCS\workspace_v12 -application com.ti.ccstudio.apps.projectBuild -ccs.projects F280049_ACAC_Unit1 -ccs.configuration Debug -ccs.buildType full -ccs.listProblems -ccs.autoOpen
D:\CCS\ccs\eclipse\eclipsec.exe -noSplash -data D:\CCS\workspace_v12 -application com.ti.ccstudio.apps.projectBuild -ccs.projects F280049_ACAC_Unit2 -ccs.configuration Debug -ccs.buildType full -ccs.listProblems -ccs.autoOpen
```

Expected:两工程0 error，生成各自`.out`；日志含`_FLASH`、`--abi=eabi`、`280049C_FLASH_lnk.cmd`、`cmd/f28004x_headers_nonbios.cmd`和`lib/driverlib_eabi.lib`，不使用RAM linker或Release。

**Step 5: 编译守卫负测**

在临时副本或预处理命令中验证：FULL缺任一ACK、FULL+LAB AUX、6VDg+FORMAL AUX都明确`#error`；恢复默认源后重新构建通过。

## Task 10：代码审查与无功率交付门槛

**Files:** review both `main.c`, test and build logs

**Step 1: 独立审查**

逐项核对：ADC EOC顺序、CMPSS模拟路由、四路TZ、AQ无重叠、状态转换合法性、所有积分器抗饱和、Unit2不采B2、Unit1不自动接管稳压、无AUX变量、无ISR内OLED/延时。

**Step 2: 明确首版锁止状态**

保持`POWER_STAGE_ENABLE=0`交付。OLED必须显示LOCK/SAFE；按键、PLL、ADC和状态可以观察，但任何路径都不能清OST。只有用户完成无功率门极/CMPSS测试、替换iL/Itotal实物标定并明确授权后，才在下一版本把它改为1。

**Step 3: 交付记录**

记录两个工程路径、构建产物、默认宏、尚待实测参数和下一人工放行点。不得把“编译通过”表述为“可以直接满功率运行”。
