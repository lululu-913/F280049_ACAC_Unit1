# 双机并联单相直接 AC-AC Buck-Boost 控制设计

- 日期：2026-07-14
- 目标器件：TMS320F280049，每台变换器一块独立控制板
- 设计状态：用户已逐节确认并明确取得共用辅助电源许可；已完成控制/安全/验证三方自审，低压放行为HALF→FULL两档，保留连续25 mV端点微调；用户已批准首版不接AUX_OK，进入软件实施
- 原始工程：`D:\CCS\workspace_v12\F280049_Xiaosai`

## 1. 目标与验收指标

系统由两台直接式单相反相 Buck-Boost AC-AC 变换器组成。两台F280049不能通信，也不能增加同步线或共享模拟控制信号；主办方已明确允许两块采样控制板和四片UCC输入侧共用由T供电的辅助DC母线及控制地，该共用供电不作为控制器通信。除公共输入、公共输出和获准的公共辅助供电外，不增加其他跨机连线。

固定工作指标如下：

| 项目 | 条件 | 指标 |
|---|---|---|
| 输入 | 公共隔离变压器次级 | 36 Vrms，50 Hz |
| 单机调压 | 1号机，`RL=20 Ω` | 1～35 Vrms，步进0.5 V；1 V/35 V端点另有25 mV/次、±0.100 V范围的连续修调 |
| 负载调整率 | 30 V，0.1～2 A | `S≤0.5%` |
| 单机效率 | 30 V，2 A | `η≥90%`，计入辅助电源功耗 |
| 并联输出 | 30 V，`Io≥4 A` | `0.9≤Io1/Io2≤1.1` |
| 输出电压THD | 30 V，4 A，K=1 | `THD≤5%` |
| 数字分流比 | 30 V，3 A，`K=0.5～2` | 相对误差绝对值`δ≤3%` |

本设计不使用AC-DC-AC或参与能量变换的主功率中间直流母线；获准的低功率辅助DC母线只给控制、采样和驱动供电，不构成AC-DC-AC功率级。本设计也不使用控制器通信、热并机或运行中自动交换主从角色。正式报告保存主办方允许共用辅助DC母线和控制地的书面或可追溯确认记录。

## 2. 已锁定的硬件基线

每台功率模块使用：

- `L=1 mH`，`Isat≥7 A`，`Irms≥4 A`；
- `Ci=11 µF`；
- `Co=21 µF`，两机并联后公共输出侧总电容量约42 µF；
- `fs=20 kHz`；
- 四只 Infineon `BSC093N15NS5`，150 V；
- 图纸中跨接每只MOS的`SS510B`仅为100 V器件，正式功率板必须DNP或换成经校核的`VRRM≥150 V`器件，推荐200 V；它不是TVS，也不得承担雪崩钳位。反相拓扑在`Ui=36 Vrms、Uo=35 Vrms`时的理想完整阻断峰值已达`√2·(36+35)=100.4 V`，背靠背器件不能按各分担一半电压选型；
- 两片 `UCC21520DWR`，每片驱动一个背靠背双向开关的两只MOS；
- 每片驱动器由独立12 V转12 V隔离电源供电；
- `UCC21520 DT`通过0 Ω接VCCI，`DISABLE`接GND，因此交越死区和故障关断必须由F280049完成；
- UCC21520的INA/INB开路时由芯片内部下拉为低，F280049复位高阻期间必须保持该默认关断路径；板级噪声验证不合格时再在四个输入补外部下拉。每次首次功率上电前必须示波验证VCCI/VDDA/VDDB建立、MCU复位、看门狗复位和掉电全过程四路VGS无误脉冲；
- 正常工况MOS的`VDS`尖峰设计目标不超过120 V，并保留RC吸收/TVS调整位置；
- 每个21 µF输出电容配置独立10～22 kΩ、0.5 W泄放电阻，推荐10 kΩ；
- 每个11 µF输入电容同样配置独立10～22 kΩ、0.5 W泄放电阻，推荐10 kΩ；任何输入空气开关操作前以实测Ui为准确认放电；
- 硬件快速Trip会令四管全低，因此每台必须安装双向TVS/RCD等确定的电感吸能通路：单次额定吸能不低于0.25 J，并在6.5 A故障电流下把任一MOS的VDS实测限制在135 V以下。该通路未安装和低压验证前，禁止额定电压过流/短路试验。

题目中的调压器和隔离变压器T位于两台变换器的公共输入侧。每台输出端不再增加独立工频变压器。

### 2.1 常供电辅助电源

辅助电源从T次级公共`H/L`端、QF1-IN和QF2-IN之前并联取电（HALF时该点为18 Vac，FULL时为36 Vac），因此主功率支路停机或切换时两块F280049、采样和OLED仍可运行。主办方已明确允许正式结构共用一个整流桥、一个非隔离DC母线和控制地：

```text
T次级公共H/L（HALF 18 Vac / FULL 36 Vac）
├─ AUX保险/限流/差模电感/BR/Cbulk/公共DC配电点
│   ├─ 本地支路保险/去耦 → 1号采样控制板 + 本机两片UCC输入侧
│   └─ 本地支路保险/去耦 → 2号采样控制板 + 本机两片UCC输入侧
├─ QF1-IN → 1号功率级
└─ QF2-IN → 2号功率级
```

公共DC采用星形配电，不用一块控制板串接给另一块供电；本地保险/可断0 Ω只允许放在各板正电源支路。两块板GND分别用固定低阻导线直接返回唯一星点，GND不得串保险、开关或可断0 Ω，防止断地后回流改走TL074、ADC保护或调试接口。两块板分别配置本地储能和高频去耦，控制/采样回流各自返回星点，避免一块板的PWM和OLED电流流经另一块板AGND。两块F280049之间仍不连接UART、CAN、SPI、同步GPIO、共享ADC输出或均流模拟线。四片UCC21520只在输入侧共用控制电源和GND；每个驱动器输出侧仍由对应B1212S隔离供电，所有SW1/SW2保持原设计互相隔离。

并联功率测试时仍拔除非隔离USB/JTAG，只使用差分探头或隔离调试设备，避免把公共控制地再接到保护地或其他仪器地形成功率回路。固件使用编译期`AUX_TOPOLOGY`：`AUX_SHARED_FORMAL`表示获准的T供电正式公共辅助母线，`AUX_SHARED_LAB_DEBUG`表示6 V方向检查或HALF档公共辅助无法稳压时使用的外部稳压公共实验室辅助电源；后者OLED用`HSDbg/HIDbg`前缀并禁止生成FULL档：

```c
#if POWER_PROFILE == PROFILE_FULL && AUX_TOPOLOGY != AUX_SHARED_FORMAL
#error FULL profile requires the approved transformer-fed auxiliary supply
#endif
#if POWER_PROFILE == PROFILE_FULL && FORMAL_HW_RELEASE_ACK != 1
#error FULL profile requires explicit formal hardware release
#endif
#if POWER_PROFILE == PROFILE_FULL && HALF_IHI_TEST_PASSED_ACK != 1
#error FULL profile requires completed HALF rated-current tests
#endif
```

辅助支路交流串联电感只允许串在公共小功率整流分支，不能串入36 Vac主功率干线。它可抑制浪涌斜率和高频干扰，但不能单独保证桥式整流加电容负载的电流THD改善；必须与保险、脉冲限流电阻或NTC及必要阻尼配合，并实测LC振铃、公共Ui畸变和峰值削顶。整流后理论峰值约50.9 V，首版采用`Cbulk≥100 V`和有足够裕量的整流桥及后级DC转换。公共辅助始终在线时的全部实功率计入效率测试。

HALF档把同一辅助取电点物理调到18 Vac，桥后空载理论峰值约`18·√2-1.4=24.1 V`；在15 V运行窗下限时仅约19.8 V。必须在两块OLED/控制板运行、四路MOS按20 kHz实际切换的最大辅助负载下，用外部仪器实测`Vbulk_min`、12 V、5 V、3.3 V、±5 V以及四路驱动侧电源。全部电源稳定至少500 ms且VGS高电平仍为11～13 V后才由操作员启动；出现DC/DC启动反复、UCC UVLO或MCU复位时禁止带功率。`Cbulk`由实测辅助电流和允许的100 Hz纹波按`ΔV≈I/(100·Cbulk)`反算，增大后必须重做36 V浪涌测试。6 V功率方向测试强制使用`AUX_SHARED_LAB_DEBUG`；HALF运行下限不能可靠稳压时HALF也改用该拓扑，不能用降低VGS勉强运行。

用户已明确批准首版不增加`AUX_OK`采样或比较器接口，也不编写辅助欠压判定和F17。`ADCINB3`保持未使用，不允许在软件中伪造“AUX正常”采样值。因此首版不能自动检出12 V母线掉电；运行前和每次升档前必须按上段用外部仪器完成人工辅电确认，测到UVLO、复位、VGS不足或电源反复启动即停止功率测试。将来增加硬件后，AUX在线联锁必须作为独立版本重新设计、实施和验证，不属于本次首版代码。

本设计术语严格区分：TVS/RCD/RC是ns～µs尺度的MOS尖峰钳位和电感能量吸收通路；Ci/Co/Cbulk泄放电阻是停机后0.1 s～数秒尺度的电容放电通路。二者均会耗散能量，但不能互相替代；图中SS510B也不能替代任一通路。

## 3. 功率侧坐标与传感器方向

交流端子的`H/L`仅表示测量参考方向，不表示直流正负极：

```text
Ui = VIN_H - VIN_L
Uo = VOUT_H - VOUT_L
```

输出电压按本机Co两端测量。Ui和Uo都采用`H`接电压前端正端、`L`接负端，只是统一了`VH-VL`测量定义，不会消除功率拓扑的反相。反相Buck-Boost正常运行时，原始`Ui`与原始`Uo`基波相差约180°，软件相位检查应以`180°±10°`为正常关系，不能通过交换某一台输出功率线消除反相。控制内部可以定义`v=-Uo`获得与Ui同相的正d轴量，但原始ADC物理量仍保留符号。

ACS722从`IP+`流向`IP-`定义为正电流：

- `iL`的实物顺序固定为：Q2/Q3中央开关节点→ACS的IP+→IP-→电感上端→电感下端/公共回线；传感器串在中央节点与电感上端之间；
- `Io1/Io2`的实物顺序固定为：本机Co/功率级输出→QF-OUT→ACS的IP+→IP-→公共汇流点；传感器位于输出空气开关之后；
- `Itotal`：IP+接两支路汇流点，IP-接负载侧。

在上述接线和正斜率标定下，`Ui>0`的稳态电阻负载半周必须测得`iL>0、Uo<0、Io1<0、Io2<0、Itotal<0`；即原始物理坐标中`sign(iL)=sign(Ui)`，而Io1、Io2、Itotal的50 Hz有功基波与Uo同相并与Ui反相，且`Itotal_fund≈Io1_fund+Io2_fund`。软启动、过零和限流瞬态允许瞬时电流短暂反向，不能把单个采样点异号当成接线故障。输出支路传感器位于本机Co之后，因此不会把本机电容电流计入比赛定义的支路电流；QF-OUT断开时，本机Uo仍可看到Co电压，而Io应接近零。

所有模拟前端均以1.65 V对应物理零点，正常线性范围限定在0.15～3.15 V；TL074输出不得直接产生负电压或超过3.3 V送入ADC。正式标定后，正常工作原始码应保持在100～3995之间。

## 4. 两套工程与固定角色

实施阶段直接从原始工程目录复制出两个独立CCS工程：

- `F280049_ACAC_Unit1`
- `F280049_ACAC_Unit2`

原始工程保留不动。功能代码修改限定在两个新工程各自的`main.c`；只允许为CCS区分工程而修改必要的工程名称元数据，不重配已有外设库和工程环境。两个新工程使用同一份角色参数化`main.c`设计，编译期常量固定板号；按键不能交换板号或把2号机改为1号机。

| model | 1号机 | 2号机 |
|---:|---|---|
| 0 | SAFE，PWM封锁 | SAFE，PWM封锁 |
| 1 | 单机稳压 | 非法，保持SAFE并报告F12 |
| 2 | 并联分流，数字设定K | 并联稳压 |

并联时只有2号机形成输出电压，1号机只根据公共输出和总电流注入规定的支路电流。任何故障都不能让1号机在带电状态自动从分流切换为稳压。

### 4.1 HALF和FULL编译档

输入电压必须由调压器物理设置为18或36 Vrms；软件不能“降低输入电压”，也不能通过篡改ADC比例制造低压显示。为保持工程配置不变，功率电压档只在各自`main.c`顶部用编译期宏选择，必须且只能选中一个：

```c
PROFILE_HALF
PROFILE_FULL
```

实施时必须把取值检查写成真正的编译互锁，不能只靠注释：

```c
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
#if DIRECTION_DIAG_6V && \
   (POWER_PROFILE != PROFILE_HALF || HALF_CURRENT_STAGE != HALF_SAFE || \
    AUX_TOPOLOGY != AUX_SHARED_LAB_DEBUG || HALF_IHI_TEST_PASSED_ACK != 0)
#error 6V direction diagnostic requires HALF SAFE and isolated lab auxiliary
#endif
```

`DIRECTION_DIAG_6V=1`只是首次功率接线检查的编译期测试门，不是第三个比赛电压PROFILE，也不能在运行中开启。它临时把Ui窗口改为4～8 V，固定1 V高阻输出、禁用并联和调压按键，两台只能分别测试；`iL_ref`峰值、软件快速限流和CMPSS初值分别为0.45/0.60/0.80 A，OLED前缀为`6VDg`。完成MOS方向、门极映射、过零和Trip后必须重新编译为`DIRECTION_DIAG_6V=0`，该测试不能置位任何放行ACK。

HALF再用`HALF_CURRENT_STAGE=HALF_SAFE/HALF_IHI`分成两个不可运行时切换的电流放行级：`HALF_SAFE`用于首次上电，`HALF_IHI`用于升FULL前验证额定电流和故障吸能。这不是第三个输入/输出电压档；两者的Ui/Uo窗口完全相同，只改变电流保护常量。进入`HALF_IHI`前必须完成HALF SAFE的门极、极性、过零、电流标定和3.8 A CMPSS测试，TVS/RCD已安装且有VDS记录。`HALF_IHI_TEST_PASSED_ACK`是另一个只能在HALF IHI全部测试完成后才改为1的结果确认，不能用“已进入IHI”代替“IHI已通过”。

档位、电流级和ACK不能由按键、串口或调试器在运行中解除。任何切换都必须重新编译/烧录、清除运行命令、锁存OST并从PRECHECK重新启动。FULL另要求`HALF_IHI_TEST_PASSED_ACK=1`、`FORMAL_HW_RELEASE_ACK=1`和`AUX_SHARED_FORMAL`，缺任一项即`#error`。

| 电压参数 | HALF | FULL |
|---|---:|---:|
| 物理标称Ui | 18 Vrms | 36 Vrms |
| 启动Ui窗口 | 16.5～19 V | 33～38 V |
| 运行Ui窗口 | 15～19 V | 30～38 V |
| 并联Uo命令范围/默认 | 5～15 V / 15 V | 5～30 V / 30 V |
| 1号机单机Uo命令范围/默认 | 1～17.5 V / 15 V | 1～35 V / 30 V |
| Uo RMS硬上限 | 18.5 V | 37 V |
| Uo绝对峰值Trip | 27 V | 53 V |
| 动态过压余量`UOV_MARGIN` | 1.0 V | 2.0 V |

| 电流参数 | `HALF_SAFE` | `HALF_IHI` | FULL |
|---|---:|---:|---:|
| `I_BRANCH_CMD_MAX` | 1.15 Arms | 2.05 Arms | 2.05 Arms |
| 本机Io峰值Trip | 2.0 A | 3.5 A | 3.5 A |
| 本机Io RMS Trip | 1.30 A | 2.25 A | 2.25 A |
| Itotal峰值Trip | 4.0 A | 7.0 A | 7.0 A |
| Itotal RMS Trip | 2.50 A | 4.50 A | 4.50 A |
| iL参考瞬时峰值上限 | 3.10 A | 5.80 A | 5.80 A |
| iL软件快速限流 | 3.40 A | 6.00 A | 6.00 A |
| CMPSS硬件Trip | 3.80 A | 6.50 A | 6.50 A |
| iL连续RMS上限 | 2.20 A | 4.00 A | 4.00 A |
| iL RMS故障阈值（2周期） | 2.40 A | 4.20 A | 4.20 A |

HALF SAFE单机使用`RL=20 Ω`，并联K=1才使用7.5 Ω，使每台支路约1 A；它只用于基本方向、稳压和K=1测试，不宣称覆盖额定电流或K端点。HALF IHI先让1号机model1和2号机model2分别单机在15 V/7.5 Ω验证2 A，再在15 V/3.75 Ω验证4 A并联电流，在15 V/5 Ω验证3 A及K=0.5～2，并逐步做6.5 A受控Trip；该工况1 mH电感储能约21 mJ，所以能在低压下先验证正式故障电流的吸能链。5 V并联启动平台、50 Hz频率窗口、D计算、死区、过零换权和200 ms分流斜坡在HALF/FULL保持相同；平台后分别升至15/30 V，约1 s完成。

动态RMS过压使用：

```text
Uov = min(UO_RMS_HARD_MAX,
          Uref_active + max(UOV_MARGIN, 0.10*Uref_active))
```

其中`UOV_MARGIN`由上表选择；SS5仍使用7 V专用阈值。K安全区中的支路上限使用当前电流放行级的`I_BRANCH_CMD_MAX`，不能固定写2.05 A。CMPSS阈值根据当前真实安培值及10AB实测offset/gain计算DAC码，不能简单缩放FULL的DAC码；6.5 A始终是绝对不可突破上限。

以下量始终保存并显示真实物理值，绝不乘2或改变标定：ADC原始码、各通道offset/gain/polarity、Ui/Uo/iL/Io/Itotal、L/Ci/Co/fs/Ts、PLL频率、PI物理单位增益、`Dff=|Uo|/(|Ui|+|Uo|)`、180°相位合同、ADC越界码和器件绝对耐压。OLED首行必须在16字符内同时合并档位、角色和状态：正式辅助的HALF SAFE/HALF IHI/FULL前缀分别为`HSAFE`、`HIHI`、`FULL`，实验室辅助的两个HALF级用`HSDbg`、`HIDbg`，6 V方向试验用`6VDg`；HALF固件检测到接36 V输入时保持OST并报F06，不能只告警。

## 5. 模拟采样接口

### 5.1 ADC映射

| F280049接口 | 1号机 | 2号机 | 传感器与带宽 |
|---|---|---|---|
| ADCINA0 | `Ui1` | `Ui2` | TL074电压调理，跨本机Ci，QF-IN之后 |
| ADCINA1 | `Uo1` | `Uo2` | TL074电压调理，跨本机Co，Io传感器之前 |
| ADCINB0 | `iL1` | `iL2` | ACS722-10AB，BW_SEL接GND，80 kHz；同时送CMPSS7 |
| ADCINB1 | `Io1` | `Io2` | ACS722-05AB，BW_SEL接3.3 V，20 kHz |
| ADCINB2 | `Itotal` | 不接 | 1号机ACS722-10AB，BW_SEL接3.3 V，20 kHz |

Itotal传感器的一次侧串在公共负载导线上，其VCC、AGND和VIOUT全部属于1号控制板，不得把VIOUT同时接到两块F280049。

首版不配置`AUX_OK`接口。两台的`ADCINB3`均保持未使用，不配置SOC、中断、滤波或软件变量，也不得用固定值伪造“辅电正常”。1号机的ADCB SOC2仍采`Itotal`；2号机的ADCB SOC2按下节重复采ADCINB1以对齐转换完成时序，物理ADCINB2与ADCINB3都不参与控制。

### 5.2 ADC调度

- ePWM3作为不输出门极的20 kHz采样时基，与ePWM1/2同步；
- ePWM3固定在`TBCTR=TBPRD`产生SOCA，初始化后控制代码不得修改该事件；不能使用随占空比移动的CMPA采样点；
- ADCA SOC0/1和ADCB SOC0/1/2由同一个SOCA触发；
- 1号机ADCB SOC2采`Itotal`；
- 2号机明确配置`ADCB SOC2.CHSEL=ADCINB1`，RESULT2为第二次Io2样本，仅用于保持两套固件相同的转换完成时序；物理ADCINB2不采样、不悬空参与控制；
- 控制ISR由ADCB EOC2触发，在进入ISR后一次性读取本周期全部结果；
- 1号机ADCB顺序为B0、B1、B2，2号机为B0、B1、B1；ADCA为A0、A1。在相同采集窗下EOC2晚于ADCA EOC1，若以后改变SOC数量或采集窗，必须重新验证不会读到上一周期数据；
- ePWM1/2的CMPA/CMPB使用影子寄存器并固定在`TBCTR=ZERO`装载。ADC在PRD采样，ISR在下一个ZERO前完成计算，新占空比在该ZERO同步生效；
- ADC时钟目标25 MHz，首版采集窗使用现有缓冲前端验证过的100 ns，并通过静态与动态采样试验确认；
- 控制ISR目标执行时间小于25 µs，50 µs为不可越过的硬上限。

### 5.3 标定合同

电压通道先沿用用户已测得的公式：

```c
Ui = ((float)adc_Ui - 2043.6f) / 36.499f;
Uo = ((float)adc_Uo - 2047.3f) / 36.323f;
```

05AB支路电流通道可用现有B1公式作为无功率软件联调初值：

```c
Io_local = ((float)adc_Io - 2032.1f) / 324.40f;
```

`iL`和`Itotal`使用10AB，禁止直接使用原05AB的304.04或327.40 count/A增益带功率运行。无功率联调初值分别采用约152.02和163.70 count/A，且只允许用于显示和信号注入；第一次清除功率PWM的OST前必须完成每个实物通道的正负多点标定。所有通道使用独立`offset`、`gain`和`polarity`常量，不在控制公式中用`abs()`掩盖极性错误。

## 6. PWM和四MOS映射

20 kHz使用中心对齐计数，100 MHz时基下首版`TBPRD=2500`。四路门极逻辑按交越开关对组织，使ePWM死区直接作用于真正互斥的MOS：

| GPIO/ePWM | 功率管 | 驱动连接 |
|---|---|---|
| GPIO0 / EPWM1A | Q1a | 输入侧UCC的INA |
| GPIO1 / EPWM1B | Q2a | 输出侧UCC的INA |
| GPIO2 / EPWM2A | Q1b | 输入侧UCC的INB |
| GPIO3 / EPWM2B | Q2b | 输出侧UCC的INB |

因此控制板到两片UCC的四根逻辑线必须交叉分组，不能简单把EPWM1A/B都接到同一片UCC。若线束标签仍使用H1/L1/H2/L2，以本表的Q1a/Q1b/Q2a/Q2b功能为最终依据。

令`p=1`表示电感接输入充电，`pbar=1`表示电感接输出放电：

| 半周 | Q1a | Q1b | Q2a | Q2b |
|---|---|---|---|---|
| Ui正半周 | p | 常导通 | pbar | 常导通 |
| Ui负半周 | 常导通 | p | 常导通 | pbar |
| model0/Trip | 低 | 低 | 低 | 低 |

根据TI [《TMS320F28004x Technical Reference Manual》](https://www.ti.com/lit/ug/sprui33h/sprui33h.pdf) ePWM章节的输出链路，F280049的输出路径为`Action-Qualifier→Dead-Band→Trip Zone→GPIO`。为避免运行中切换Dead-Band产生毛刺，首版中EPWM1和EPWM2的Dead-Band永久旁路，四路最终波形全部由独立AQ比较事件产生。

中心对齐活动对的固定真值如下，令`CMPA=D·TBPRD`、`CMPB=CMPA+Ndead`：

```text
上计数：ZERO置p=1/pbar=0；CAU关p；CBU开pbar
下计数：CBD关pbar；CAD开p
```

因此上、下计数两个换流边沿都具有`CMPB-CMPA`的非重叠区。100 MHz TBCLK下，首版`Ndead=25`，即250 ns；允许实测调整范围为10～60 TBCLK，但每次调整都必须重新验证VGS和VDS。CMPB必须保持小于TBPRD，D限幅同时预留死区计数。

非活动对通过带ZERO同步装载的AQ连续软件强制保持两路高。禁止在任意TBCLK直接切换AQ强制或DBCTL。过零换权由两个专用AQ过渡图样完成：

1. `ZC_A`：ISR只设置`zc_sequence_pending`；下一个ZERO关断旧输入侧常导通管并保持Q2a/Q2b为高，在`TBCTR=Nzc`打开新半周的常导通输入MOS；
2. `ZC_B`：再下一个ZERO关断新半周活动支路的Q2 MOS，在`TBCTR=Ndead`才打开对应Q1 MOS；随后进入正常中心对齐PWM图样。

首版`Nzc=Ndead=25`。这样常导通换权和第一次p/pbar换流都具有硬件break-before-make，总时长不超过两个PWM周期。不得用`DELAY_US`、NOP或CPU循环产生250～500 ns等待。

Trip Zone位于最终输出级。任何软件故障通过`TZFRC.OST`进入同一One-Shot关断链，四路输出均配置为低电平。

第一次功率上电前必须用万用表二极管档和6 V限流电源逐管确认MOS漏源方向，并验证：正/负半周承担阻断的MOS正确、`p=1`确实连接输入、`p=0`确实连接输出。PCB器件标签与本表不一致时，先修正线束/逻辑映射，不允许靠控制环符号补救。

## 7. 信号处理与控制链

已经确认的软件控制链为：

```text
Co电流前馈
    + 电压外环或分流外环
    ↓
变换器输出电流参考
    ↓ /(1-Dff)
电感电流参考
    ↓
20 kHz电感电流PI
    ↓
占空比与无断流半周换向
```

### 7.1 SOGI-PLL和基波提取

- 单相SOGI-SRF-PLL，`k=√2`；
- 归一化相位检测，避免电压幅值改变PLL增益；
- 固件确定初值为10 Hz、阻尼比0.707，`Kp=88.9`、`Ki=3948`；允许在8～10 Hz范围内按低压锁相测试整定；
- 频率限制45～55 Hz；
- SOGI和PLL PI按20 kHz采样使用双线性/Tustin离散，SOGI中心频率跟随受限PLL频率；
- `PLL_UI`在所有活动模式持续运行，用于输入幅值/频率、门极半周q、过零预测和Ui/Uo相差检查；
- 1号机model2在5 V平台建立后额外运行独立`PLL_UO`，用于Itotal、Io1基波提取和分流参考；`PLL_UI`不能停止；
- 2号机及1号机model1只使用PLL_UI，Uo由与PLL_UI同步的SOGI观察器处理，不再运行第二个自由PLL；
- PLL锁定定义为：幅值在该状态允许范围内、频率在允许范围内、归一化q轴误差绝对值小于0.05，并连续满足5个估计工频周期；
- 1号机相位检查使用`wrap(theta_o-theta_i-pi)`，绝对值必须小于10°；门极q只由PLL_UI和Ui实测滞环决定，分流正弦只由PLL_UO决定；
- Itotal和Io1通过Uo-SOGI同步坐标提取50 Hz基波，不把20 kHz毛刺直接送入分流公式；
- Co导数项由SOGI正交状态解析计算，禁止对ADC电压直接差分。

电压和电流统计量明确分开：Ui/Uo基波RMS由SOGI幅值除以`√2`得到；iL、Io和Itotal的保护RMS/平均值使用400点滑动窗；稳压前馈的Io使用一阶400 Hz低通，20 kHz下系数`a=exp(-2π·400·50 µs)≈0.8819`；分流K只使用PLL_UO同步的50 Hz基波。三种量不能共用同一个慢滤波结果。

### 7.2 占空比前馈

使用基波幅值而不是过零附近的瞬时值：

```text
Dff = Uo_amp / (Ui_amp + Uo_amp)
```

36 V输入、30 V输出时`Dff≈0.4545`。ACTIVE状态占空比限制为`0.02≤D≤0.85`，`1-D`换算分母不得小于0.15。

门极执行状态独立于系统run_state：

```text
PWM_BLOCKED：OST锁存，四路低
OUTPUT_CONNECTED_ZC：输入侧阻断，Q2a/Q2b全导通；只允许在预测Uo过零附近短暂保持1～2个PWM周期
ACTIVE：D限制在[Dmin,Dmax]
```

稳压软启动内部参考从0开始，但在参考低于1 V时保持PWM_BLOCKED；达到1 V、重新通过死母线联锁并到达预测过零后才清OST进入ACTIVE。1 V时`Dff≈0.027`，与Dmin兼容。若低端负修调使最终目标为0.9～0.975 V，启动仍先升到1.0 V并清OST，进入ACTIVE后再按5 V/s上限平滑降到`Uref_ctrl`，避免软停再启时因RAM中保留负trim而永远达不到1 V。0.9 V工况必须在HALF测试Dmin和稳定性，未通过时锁止负trim而不得静默改变Dmin。1号机分流加入带电母线时，只在预测Uo过零由PWM_BLOCKED短暂进入OUTPUT_CONNECTED_ZC，完成AQ换权后立即以`Dff≈|Uo|/(|Ui|+|Uo|)`进入ACTIVE；200 ms斜坡只作用于`alpha/iconv_ref`，不得把OUTPUT_CONNECTED_ZC延长到多个毫秒。Q2a/Q2b全导通会令`vL=Uo`，因此该状态不是可长期保持的零电压续流状态。

### 7.3 稳压角色

1号机model1和2号机model2使用同一稳压算法。参考链只有以下单向关系，不得在不同模块里各自重算一个泛称`Uref`：

```text
Uref_cmd → Uref_ctrl（只在端点加trim）
         → Uref_active（启动/按键斜率限制）
         → Uo_rms_ref（稳压PI实时目标）
```

稳压PI和正常RUN误差判断使用`Uo_rms_ref=Uref_active`；初始斜坡完成后的F10最终到达判断使用`Uref_ctrl`，因为此时`Uref_active`应已到达它；动态过压`Uov`使用`Uref_active`。由于反相拓扑，原始输出参考为：

```text
uo_ref = -sqrt(2) * Uo_rms_ref * sin(theta_i)
```

定义控制坐标电压`v=-Uo`，使正常目标与输入PLL同相并位于正d轴。电压同步PI的固定初值带宽为8 Hz，Co=21 µF时：

```text
Kpv = 1.49e-3 A/V
Kiv = 5.31e-2 A/(V·s)
```

PI输出在控制坐标中重构后必须乘回负号。按本文`sin/cos`约定：

```text
iv_corr_raw = -(Ivd*sin(theta_i) + Ivq*cos(theta_i))
```

变换器输出电流参考为：

```text
iconv_ref = Io_local_ff + Co * duo_ref/dt + iv_corr
```

其中`Io_local_ff`使用7.1节定义的400 Hz低通，`iv_corr`由电压PI产生。30 V、21 µF、50 Hz时，本机Co基波电流峰值约0.28 A，轻载时不能忽略。电压PI输出首先受本机电流能力限幅，限幅时使用条件积分，电流优先级高于稳压误差。`I_BRANCH_CMD_MAX`的单位是Arms，生成正弦支路参考时的基波瞬时峰值不得超过`√2·I_BRANCH_CMD_MAX`；再加入`Co·duo_ref/dt`等电容电流后，必须由`IL_REF_PK_MAX_ACTIVE`继续约束换算后的`iL_ref`，不得把2.05 Arms当成2.05 A瞬时峰值。

### 7.4 1号机并联分流角色

定义：

```text
alpha = K / (1 + K)
io1_ref_fund = alpha * Itotal_fund
```

分流校正环比较Io1基波与`io1_ref_fund`，固定初值为`Kps=0.25`、`Kis=78.5 s^-1`，对应约10 Hz，只修正传感器标定、器件损耗和模型误差：

```text
iconv1_ref = io1_ref_fund + Co * duo_fund/dt + ishare_corr
```

K命令范围为0.5～2，按键步进0.05，`K_active`变化率限制为0.5/s。在FULL档3 A总电流下，端点目标分别为：

| K | Io1 | Io2 |
|---:|---:|---:|
| 0.5 | 1.0 A | 2.0 A |
| 1.0 | 1.5 A | 1.5 A |
| 2.0 | 2.0 A | 1.0 A |

运行中根据正值`IT=Itotal_fund_rms`和当前档支路上限`IB=I_BRANCH_CMD_MAX`计算安全K区间；禁止代入瞬时有符号Itotal或其正负基波分量：

```text
IT = Itotal_fund_rms
IB = I_BRANCH_CMD_MAX
Kmin_safe = max(0.5, IT/IB - 1)
Kmax_safe = (IT > IB) ? min(2.0, IB/(IT-IB)) : 2.0
```

`K_active`被夹在安全区间内并显示`LIM`。若`Kmin_safe>Kmax_safe`，直接进入总负载超能力限流/软停，禁止继续夹取无效K区间。因此K端点不会在4 A总电流下生效，也不会命令单台承担约2.67 A。

### 7.5 电感电流内环

按已定义的传感器方向，输出侧电流和电感电流的固定极性为`IL_TO_OUTPUT_SIGN=-1`；不允许对参考取绝对值：

```text
iL_ref = -iconv_ref / (1-Dff)
```

6 V极性试验必须确认该固定符号；如果实物相反，应修正传感器线束并重新标定，而不是在两个半周使用不同符号。

每个半周的平均模型定义为：

```text
L*diL/dt = q*[D*|Ui|-(1-D)*|Uo|] - Rsum*iL
q=+1（POS），q=-1（NEG）
e_i=iL_ref-iL
v_pi=Kpi*e_i+x_i
D_unsat=Dff + q*(L*diL_ref/dt + Rsum*iL_ref + v_pi)
                    / max(|Ui|+|Uo|, 10 V)
D_cmd=clamp(D_unsat,0.02,0.85)
```

`diL_ref/dt`由基波/SOGI状态解析计算，禁止差分ADC电流。首版`Rsum=0`，只在测得电感DCR和导通压降后加入前馈；PI仍负责消除误差。过零ZC状态不执行此占空比公式。

积分器使用确定的条件积分：未饱和时积分；饱和后仅当误差会把`D_unsat`拉回允许区间时积分。每次在PRD采样后按`x_i += Kii*Ts*e_i`更新，结果在下一次ZERO装载。首版参数：

- 目标带宽800 Hz；
- 若PI输出为电感电压修正量，`Kpi≈5.03 V/A`；
- 积分零点先放20 Hz，`Kii≈632 V/(A·s)`；
- 先以`Ki=0`验证Kp，再加入积分和模型前馈；
- 电感平均电流参考的瞬时峰值取`IL_REF_PK_MAX_ACTIVE`，测得开关纹波峰值的软件快速限流取`IL_SW_FAST_LIMIT_ACTIVE`；它们在正常档取第4.1节值，6 V DIAG时分别取0.45/0.60 A；
- 所有外环和内环使用条件积分或反算抗饱和。

模式切换、PLL未锁、过零换向、参考限幅和Trip时，相关积分器必须冻结或复位；重新投入时使用无扰预置，不能从零积分状态产生占空比跳变。

## 8. 无断流过零换向

电压过零时Co电流接近峰值，不能等待iL为零或把四管全部关断。每台控制器使用：

```text
POS → ZC_PN → NEG → ZC_NP → POS
```

固件初值采用相位窗口±2°、瞬时Ui/Uo或SOGI重构值绝对值小于1.5 V，并检查Ui/Uo反相方向正确；允许在1～2 V范围低压整定。门极过零预测永远使用PLL_UI，PLL_UO只参与相位一致性和分流基波。

以POS转NEG为例：

1. ISR只置位`zc_sequence_pending`并冻结电感PI积分器；
2. 下一个TBCTR=ZERO进入ZC_A：令`p=0`、关Q1b，同时使Q2a/Q2b保持导通；
3. `TBCTR=Nzc=25`的硬件比较事件打开Q1a，不执行CPU延时；
4. 再下一个ZERO进入ZC_B：先关Q2b，在`TBCTR=Ndead=25`才开Q1b；
5. 完成后进入NEG正常PWM图样，清除pending并恢复积分器。

NEG转POS执行对称过程：ZERO关Q1a，Nzc比较事件开Q1b。换向最迟在两个PWM周期内完成。只有正常停机且电感能量释放后，或硬件故障Trip时，才允许四路门极全部低；硬Trip的剩余电感能量由第2节规定的外部钳位网络吸收。

## 9. 运行状态机

统一状态集合：

```text
BOOT, SAFE, PRECHECK, INPUT_PLL, WAIT_UO, OUTPUT_PLL,
SS5_RAMP, SS5_HOLD, SHARE_RAMP, VOLT_RAMP, RUN,
STOP_RAMP, WAIT_MASTER_OFF, DISCHARGE, FAULT
```

公共规则：

- 上电先软件强制One-Shot Trip，初始化完成后只进入SAFE；若复位原因是看门狗或非法复位，先锁存F15，人工确认后才能回SAFE；
- `model_cmd`是按键请求，`model_active`只能在死母线SAFE状态锁存；
- 合法model下长按KEY4一秒，进入PRECHECK；
- PRECHECK只检查本机角色/model合法性、死母线、ADC原始码、当前档Ui窗口、电流比较器/TZ状态以及该角色所需PLL条件；首版没有`aux_ok`输入，辅电轨、驱动侧电源和VGS由操作员在进入软件启动流程前用外部仪器完成硬件放行；
- 运行中短按KEY4或请求model0，进入软停；
- 运行中写入1↔2，只软停并回SAFE，不自动以新模式启动；
- 死母线只能用完整交流量判定：本机`Uo_fund_rms<2 V`，400点窗口的`iL_rms`和本机`Io_rms<0.1 A`，连续5个估计工频周期；瞬时过零不能用于锁存model或操作空气开关；
- 稳压角色正常停机路径为`STOP_RAMP→DISCHARGE→SAFE`；稳压机PWM关闭后3 s仍不能完成本机Co放电，进入F14；
- 1号机model2停机路径为`STOP_RAMP→WAIT_MASTER_OFF→SAFE`：分流电流降到0后PWM关闭，无限等待2号机把公共Uo降至死母线，不启用F14计时；
- 严重故障立即进入FAULT并锁存首个故障。

清除上电OST和实际允许门极变化只有两个合法转换点：

1. 1号机model1或2号机model2：PRECHECK确认死母线后进入INPUT_PLL；在VOLT_RAMP内部参考达到1 V、再次确认死母线未被外部建立并到达预测过零时，清OST直接进入ACTIVE。检查后Uo重新升高则取消启动、保持OST并报F10。
2. 1号机model2：PRECHECK必须先在死母线完成，随后带OST进入WAIT_UO；只有OUTPUT_PLL全部条件满足、到达预测Uo过零时，才清OST，短暂进入OUTPUT_CONNECTED_ZC完成AQ换权，并立即以Dff进入ACTIVE/SHARE_RAMP。

INPUT_PLL最长等待2 s，未锁定进入F08。任何其他状态都不能清OST。

### 9.1 1号机model1

```text
SAFE → PRECHECK → INPUT_PLL → VOLT_RAMP → RUN
```

Uo参考从0斜升到设定值，额定启动计划时间1 s；低端负trim按第7.2节先到1.0 V清OST再降到目标。斜坡结束后允许1 s稳定；若仍未连续5周期满足`|Uo_rms-Uref_ctrl|≤max(0.5 V,2%·Uref_ctrl)`，进入F10。单机输出RMS必须按当前档`I_BRANCH_CMD_MAX`限制；2 A只是HALF IHI/FULL的验证工况，不能绕过HALF SAFE的1.15 Arms限制。

### 9.2 1号机model2

```text
SAFE → PRECHECK → INPUT_PLL → WAIT_UO → OUTPUT_PLL → SHARE_RAMP → RUN
```

WAIT_UO期间OST保持、积分器清零。`Uo_rms<2.5 V`时允许无限等待；上升超过3.5 V后启动1 s的OUTPUT_PLL窗口；若重新低于2.5 V则复位窗口并返回WAIT_UO，2.5～3.5 V保持当前状态，避免噪声反复跳转。必须在该1 s窗口内满足：

- `4.0 V≤Uo≤6.5 V`；
- `49 Hz≤fo≤51 Hz`；
- Uo-PLL连续锁定5周期；
- Ui/Uo相位关系为`180°±10°`。

随后200 ms把分流系数从0斜升到目标alpha，并置`bus_valid_latched=true`。只有该锁存置位后的SHARE_RAMP/RUN才启用F07；WAIT_UO、OUTPUT_PLL、STOP和WAIT_MASTER_OFF不启用F07。

### 9.3 2号机model2

```text
SAFE → PRECHECK → INPUT_PLL → SS5_RAMP → SS5_HOLD → VOLT_RAMP → RUN
```

- 200 ms从内部参考0升到5 V；开始ACTIVE后500 ms内必须进入4.5～5.5 V，否则F10；
- Uo连续5周期稳定在4.5～5.5 V后，保持5 V至少500 ms，供1号机锁相和完成200 ms升流；SS5状态使用7 V专用过压阈值；
- 再用1.0 s从5 V升到最终设定值；
- 2号机model2的`Uref_cmd`下限固定5 V，HALF/FULL的上限和默认值分别为15/15 V和30/30 V；1号机model1的`Uref_cmd`下限1 V，HALF/FULL上限分别为17.5/35 V；若最终`Uref_cmd=5 V`，平台保持结束后直接进入RUN；
- `Io2_rms≥I_BRANCH_CMD_MAX`时进入电流优先限幅并冻结/降低电压参考，低于`0.92·I_BRANCH_CMD_MAX`连续2周期后释放；达到本档`IO_RMS_TRIP`连续2周期触发F03；
- 限流持续500 ms仍不能继续，判定1号机未投入或负载过重，进入F10；
- 最终1 s斜坡结束后允许1 s稳定，仍未连续5周期满足`|Uo_rms-Uref_ctrl|≤max(0.5 V,2%·Uref_ctrl)`则进入F10。

无通信条件下，轻载时2号机无法证明1号机已经参加并联。操作员必须看到1号机OLED首行显示当前档前缀加`M2SH RUN`后，才把系统视为并联就绪。

RUN中稳压误差连续1 s超过`max(0.5 V,5%·Uref_active)`进入F10。1号机异常退出时，2号机继续执行本档`I_BRANCH_CMD_MAX`电流优先限幅，不能无期限替代两台容量；持续500 ms后F10软停，达到本档`IO_RMS_TRIP`的F03条件则立即Trip。2号机异常或Uo崩溃时，1号机只允许F07停止注流，绝不自动切为稳压。两块板均不得自动恢复。

## 10. 空气开关操作顺序

负载始终接在公共输出端。四个三相空气开关分别作为QF1-IN、QF1-OUT、QF2-IN、QF2-OUT；每个只使用所需联动极，第三极悬空，禁止并联触点分流。单路空气开关作为公共输入隔离变压器T一次侧的运行总开关`QF0`。公共AUX位于T次级公共H/L、四个支路QF之前；正常模式切换保持QF0和公共辅助电源接通。单极QF0不能作为维护隔离：检修必须拔除一次侧电源或使用上游双极联动隔离并验电。

空气开关没有辅助触点，软件只能验证电压和电流，不能证明机械位置正确；OLED提示不能替代操作者检查。除紧急分断外：

- 所有正常QF操作要求两块PWM均被OST封锁、相关电流RMS小于0.1 A；紧急分断除外；
- QF-IN只能在QF0断开后等待至少2 s，并用外部万用表确认对应Ci低于2 Vrms时改变位置；QF0断开后辅助和OLED失电，不能把OLED作为唯一放电确认。禁止把11 µF Ci通过机械空气开关随机相位热接到36 Vac。若要保持QF0和辅助电源常通，QF1-IN与QF2-IN必须在第一次冷态上电前一并合上，此后模式切换期间保持不动；
- 合、分QF-OUT前，本机Co侧和公共负载侧都必须低于2 Vrms；由于每台只测本机Co，操作者必须同时检查两台OLED；
- 禁止公共输入带电时新增输入支路，当前硬件没有预充或输入过零合闸装置；辅助支路中的串联电感/NTC不能限制主功率Ci的合闸涌流；
- 冷态预合两只QF-IN只是把两块Ci改由QF0统一上电，并未消除T励磁、22 µF总Ci和公共Cbulk的合闸浪涌。QF0/T一次侧或最高36 Vac公共主干必须配置覆盖整机的浪涌限制，或者在最坏合闸相位反复实测证明QF0、T、Ci、Cbulk和线束均安全；
- QF1-IN、QF2-IN贴“仅QF0断开后操作”标签，正常运行机械保持合位。需要频繁改变输入支路时必须增加机械联锁或主功率预充/过零投入，不能只依赖操作说明。

### 10.1 全冷态单机启动

1. QF0断开且外部确认Ui/Uo死电；冷态先合QF1-IN和QF2-IN，使两台Ci都随下一次公共上电建立；此时控制器尚未供电，不能依赖model或软件OST；
2. 合QF1-OUT，保持QF2-OUT断开，负载保持连接；
3. 合QF0，公共AUX和两台输入同时建立；等待两块板BOOT完成并显示SAFE；
4. 1号机选择model1，2号机保持model0；
5. 长按1号机KEY4，执行PRECHECK、INPUT_PLL和VOLT_RAMP。

### 10.2 全冷态并联启动

1. QF0断开且外部确认两台Ui/Uo死电；冷态合QF1-IN、QF2-IN、QF1-OUT和QF2-OUT；此时控制器尚未供电，安全由UCC输入默认下拉、驱动UVLO和无误脉冲电源时序保证；
2. 合QF0，等待公共AUX和两块板BOOT完成并显示SAFE；
3. 两台选择model2；
4. 先启动1号机进入WAIT_UO；
5. 再启动2号机建立5 V平台；
6. 操作者确认1号机进入SHARE_RAMP/RUN，2号机再完成最终升压。

### 10.3 单机切换并联

1. 前提是QF1-IN和QF2-IN已在冷态上电前合好并始终保持；1号机model1软停并进入SAFE，等待公共Uo死电；
2. 两台选择model2，确认两块PWM均被OST封锁且QF2两侧低于2 Vrms；
3. 保持QF0和辅助电源常通，合QF2-OUT；禁止在此时操作QF2-IN；
4. 先启动1号机WAIT_UO，再启动2号机，执行5 V平台和升流流程。

如果QF2-IN此前没有在冷态合上，则本节流程无效：必须先断开QF0、等待Ci/Ui放电、合QF2-IN，再重新合QF0；或者另行增加经验证的主功率预充/过零投入硬件。

### 10.4 并联退回单机

1. 先让1号机退流、关PWM并进入WAIT_MASTER_OFF；
2. 再让2号机降压关PWM；两机确认公共Uo死电；
3. 保持QF0、公共AUX、QF1-IN和QF2-IN接通，断开QF2-OUT；
4. 1号机选择model1，2号机选择model0；
5. 启动1号机。QF2-IN保持接通但其PWM封锁，不产生下一次热接Ci涌流。

## 11. 保护与故障锁存

### 11.1 硬件过流链

```text
ADCINB0/iL模拟量
  → CMPSS7高、低比较器（±IL_CMPSS_TRIP_ACTIVE：6V DIAG 0.8 A，否则取HALF SAFE 3.8 A或HALF IHI/FULL 6.5 A）
  → ePWM X-BAR / Digital Compare OR
  → 异步One-Shot Trip Zone
  → EPWM1A、1B、2A、2B全部强制低
```

CMPSS正负阈值由最终10AB通道标定的offset和gain计算。故障ISR只保存CMPSS/TZ标志与冻结数据，不得自动清Trip。

### 11.2 分档阈值

| 故障 | 阈值 | 确认时间/动作 |
|---|---|---|
| iL硬件峰值 | `|iL|≥IL_CMPSS_TRIP_ACTIVE`：6V DIAG 0.8 A，HALF SAFE 3.8 A，HALF IHI/FULL 6.5 A | CMPSS异步One-Shot，F01 |
| iL RMS | `IL_RMS_CONT_PROFILE / IL_RMS_TRIP_PROFILE`：HALF SAFE 2.2/2.4 A，HALF IHI/FULL 4.0/4.2 A | 超Trip值2个工频周期，F02 |
| 本机Io峰值/RMS | `IO_PK_TRIP_PROFILE / IO_RMS_TRIP_PROFILE`：HALF SAFE 2.0/1.30 A，HALF IHI/FULL 3.5/2.25 A | 5采样 / 2周期，F03 |
| Itotal峰值/RMS | `IT_PK_TRIP_PROFILE / IT_RMS_TRIP_PROFILE`：HALF SAFE 4.0/2.50 A，HALF IHI/FULL 7.0/4.50 A | 5个连续采样 / 2周期，F04，仅1号机 |
| Uo绝对峰值 | `|Uo|>UO_ABS_PK_TRIP_PROFILE`：HALF 27 V，FULL 53 V | 3采样，F05，给ADC线性范围留余量 |
| 稳压角色Uo RMS过压 | `>UOV_PROFILE(Uref_active)`，公式与常量只取第4.1节 | 1周期，F05；SS5单独使用7 V |
| 1号机分流Uo RMS过压 | `>UO_RMS_HARD_MAX_PROFILE`：HALF 18.5 V，FULL 37 V | 1周期，F05 |
| 输入启动范围 | `UI_START_WINDOW_PROFILE`：HALF 16.5～19 V，FULL 33～38 V；均48～52 Hz | 连续5周期才允许启动 |
| 输入运行幅值 | `UI_RUN_WINDOW_PROFILE`：HALF 15～19 V，FULL 30～38 V | 超界3周期，F06 |
| 输入频率/PLL | 46～54 Hz或PLL失锁 | 3周期，F08 |
| 1号机公共Uo快速丢失 | `Uo_sogi_rms<3 V`，且bus_valid已锁存 | 5 ms，F07并立即停止注流 |
| 1号机Uo-PLL丢失 | SHARE_RAMP/RUN且失锁 | 2周期，F07 |
| ADC越界 | 原始值`<32`或`>4063` | 3采样，F09 |
| iL/Uo直流偏置 | `|mean(iL)|>0.3 A`或`|mean(Uo)|>0.5 V` | 5周期，F11 |
| 放电超时 | 3 s | F14 |
| ISR超时 | 连续2次 | F15 |

分流比误差`δ>3%`连续5周期只显示警告；`δ>10%`持续1 s显示严重警告。是否Trip最终仍由电流、电压和PLL安全条件决定。F07在WAIT_UO、OUTPUT_PLL、STOP_RAMP、WAIT_MASTER_OFF和DISCHARGE中明确禁用。

HALF使用第4.1节逐项确定的电压窗口和SAFE/IHI电流阈值，不能采用“Ui 4～26 V均放行”的宽窗口。低压档只能收窄工作范围，不能关闭ADC越界、相位检查、Trip Zone、状态机互锁、VDS记录或吸能验证。正式验收必须使用`PROFILE_FULL+AUX_SHARED_FORMAL`并同时具备`HALF_IHI_TEST_PASSED_ACK`和`FORMAL_HW_RELEASE_ACK`，Flash独立运行前人工核对OLED首行为`FULL ...`且不带`Dbg`。

### 11.3 故障码

| 代码 | OLED | 含义 |
|---|---|---|
| F00 | OK | 无故障 |
| F01 | ILPK | 电感峰值过流 |
| F02 | ILRM | 电感RMS过流 |
| F03 | IOOC | 本机输出过流 |
| F04 | ITOC | 总输出过流 |
| F05 | UOOV | 输出过压 |
| F06 | UIBD | 输入欠压/过压 |
| F07 | BUSL | 公共输出丢失 |
| F08 | PLL | PLL失锁 |
| F09 | ADC | ADC越界或时序无效 |
| F10 | STRT | 启动或限流等待超时 |
| F11 | DCOF | 直流偏置异常 |
| F12 | MODE | 本机model非法；非锁存配置错误 |
| F13 | PHAS | Ui/Uo相位错误 |
| F14 | DISC | Co放电超时 |
| F15 | CPU | ISR超时、看门狗或非法复位 |
| F16 | TEMP | 预留温度保护 |
| F17 | RSVD | 预留编号；首版无生成、锁存或显示路径 |

2号机收到model1时不进入FAULT：立即把`model_cmd`强制回0，保持SAFE并显示`F12 MODE/SET M0`。其余故障进入FAULT时自动把`model_cmd`置0。

清故障必须同时满足：PWM仍被One-Shot封锁、母线死电、CMPSS正常至少100 ms、ADC原始值在64～4031至少100 ms，并长按KEY4两秒。电流故障要求相关RMS低于阈值80%连续5周期；电压/频率/PLL故障要求回到启动允许范围连续5周期；F14要求Uo死电；F15要求本次复位后ISR和主循环健康计数均已前进。清除后只返回SAFE，不自动重启。

## 12. 按键与OLED

| 按键 | GPIO | 行为 |
|---|---:|---|
| KEY1/KEY2 | 27/25 | 1号机model2下K−/+0.05，范围0.5～2；1号机model1且`Uref_cmd`恰为1.0 V或当前档高端（HALF 17.5 V，FULL 35.0 V）时，复用为对应端点修调−/+0.025 V/次 |
| KEY3 | 17 | 只在SAFE切model；1号机0→1→2→0，2号机0→2→0 |
| KEY4 | 26 | SAFE长按1 s启动；活动状态短按软停；FAULT且死母线长按2 s清故障 |
| KEY5/KEY6 | 16/39 | 修改粗调`Uref_cmd`−/+0.5 V；上下限由当前PROFILE决定，2号机model2下限保持5 V |

按键每5 ms扫描，消抖25 ms，连续调节重复周期150 ms。KEY5/6在SAFE和RUN有效、FAULT忽略；RUN中`Uref_active`以不超过5 V/s跟随命令，不能把0.5 V按键变成瞬时阶跃。启动状态使用第9节规定的专用斜坡。GPIO16/39按普通数字输入配置，不调用无效的模拟模式API。

端点微调与0.5 V比赛命令分开保存：

```text
Uref_cmd  ∈ {1.0, 1.5, ..., UO_CMD_MAX} V
Utrim_lo  ∈ [-0.100, +0.100] V，步进0.025 V
Utrim_hi  ∈ [-0.100, +0.100] V，步进0.025 V
```

当`Uref_cmd=1.0 V`时`Uref_ctrl=Uref_cmd+Utrim_lo`，当`Uref_cmd=UO_CMD_MAX`（HALF 17.5 V，FULL 35.0 V）时使用`Utrim_hi`，其余点`Uref_ctrl=Uref_cmd`。KEY1每完成一次“按下→释放”向负方向移动25 mV，KEY2向正方向移动25 mV，到±0.100 V后饱和并显示`LIM`。长按不自动连发，必须稳定释放后才能接受下一步；一次误触最多改变25 mV。离开端点后trim保存在RAM但不参与控制，返回端点重新生效；上电两个trim均归零。HALF高端修调只用于低压功能验证，正式验收高端仍是35 V。

修调只用于补偿端点稳态误差，不能修改ADC标定公式，也不能掩盖传感器增益或极性错误。`Uref_active`仍以不超过5 V/s跟随`Uref_ctrl`。正式观察以外部真有效值表和多周期RMS为准，因为25 mV约等于现有电压通道一个ADC码；单步只验证方向，四步累计100 mV用于验证幅值。

KEY1/2的模式优先级固定为：1号机model2调整K；1号机model1端点调整修调；其他角色/电压点忽略。OLED在端点分别显示命令和修调，例如HALF的`CMD:17.500`或FULL的`CMD:35.000`，下一行`TRM:+0.050`，不能把修调合并显示成“命令+修调”后的比赛命令。非端点显示`STEP:0.500`，model2页面显示K而不显示trim。

OLED使用GPIO28/29软件I²C，128×64、16字符×4行；以10 Hz刷新且每次只更新一行，绝不放入20 kHz控制ISR。显示采用定宽定点格式，必须正确处理负号和三位数。页面不显示C1/C2板号，板号使用机箱标签。例如`HSAFE M2SH WAIT`、`HIHI M2VC SS5`、`FULL M2VC RUN`和`HSDbg M1VC RUN`都在16字符内，不再用另一个固定首行覆盖角色/状态。

典型页面：

```text
HSAFE M2SH WAIT     HIHI M2VC SS5
UO:00.0 PLL:N       UR:05.0 UO:04.9
I1:0.00 IT:0.00     I2:0.67 PLL:Y
K:1.00 F:00         T:0.5s F:00
```

故障页覆盖普通页面并显示首个故障和冻结值。

1号机退流后等待2号机关机时显示：

```text
HIHI M2SH WAIT0
UO:30.0 PWM:OFF
I1:0.00 IT:4.00
STOP UNIT2
```

## 13. 软件执行结构

遵守用户要求，实施主要修改两个复制工程中的`main.c`；现有OLED/I²C驱动继续复用。即使保留单文件，也要按以下独立静态函数和数据结构划分职责：

1. 编译期板角色、硬件参数和集中保护阈值；
2. GPIO、ADC、CMPSS、XBAR、ePWM和Trip初始化；
3. ADC标定、RMS、SOGI和PLL；
4. 电压外环、分流外环和电流内环；
5. 半周门极状态机；
6. 系统run_state与故障管理；
7. 按键与OLED低速任务；
8. 20 kHz ISR和主循环调度。

20 kHz ISR只执行采样、快速保护、信号处理、控制计算和本周期PWM更新。1 kHz慢任务执行run_state和超时；按键、OLED、字符串格式化、空气开关提示放在主循环。ISR入口/出口使用测试GPIO测时。

看门狗采用双向健康握手：ISR和主循环各自递增独立心跳；只有监控任务观察到两个心跳都比上次前进，才允许喂狗并更新快照。任何一侧停滞都让看门狗复位。启动时读取复位原因，看门狗/非法复位进入F15且保持OST。

### 13.1 固件确定初值与整定门槛

| 参数 | 固件初值 | 允许整定范围 | 最迟锁定门槛 |
|---|---:|---:|---|
| 活动PWM非重叠`Ndead` | 25 TBCLK/250 ns | 10～60 TBCLK | 6 V门极测试 |
| 过零break-before-make`Nzc` | 25 TBCLK/250 ns | 25～50 TBCLK | 6 V换向测试 |
| 过零电压阈值 | 1.5 V瞬时/重构值 | 1～2 V | HALF单机 |
| PLL带宽 | 10 Hz | 8～10 Hz | HALF单机 |
| 电流环带宽 | 800 Hz | 600～1000 Hz | HALF SAFE/IHI单机 |
| 电压环带宽 | 8 Hz | 6～10 Hz | HALF单机 |
| 分流环 | Kps=0.25，Kis=78.5/s | 10～15 Hz等效带宽 | HALF IHI并联 |
| Io稳压前馈LPF | 400 Hz | 300～500 Hz | HALF负载阶跃 |
| K_active变化率 | 0.5/s | 固定 | 首版软件 |
| SS5稳定保持 | 500 ms | 不得缩短 | 首版软件 |
| D范围 | 0.02～0.85 | 只允许收窄 | 首版软件 |

表中的“允许整定范围”不表示运行时自动搜索。固件总是使用一组明确数值；每次人工整定都记录到版本说明，并执行相关回归测试。代理不得在缺少示波器或功率测试结果时自行宣称最终值已经锁定。

## 14. 分阶段验证

### 14.1 放行顺序

```text
PSIM含非理想模型
→ 无功率ADC/PWM/CMPSS测试
→ 6 V限流电源确认MOS方向、门极映射和控制符号
→ PROFILE_HALF + HALF SAFE：两台分别以RL=20 Ω做18→15 V单机，并以同一高阻负载做18→17.5 V最大电压比
→ PROFILE_HALF + HALF SAFE：15 V/7.5 Ω并联K=1，验证掉机/失锁/限流/ADC/看门狗
→ PROFILE_HALF + HALF IHI：两台分别在15 V/7.5 Ω单机验证2 A，再用15 V/3.75 Ω验证4 A、15 V/5 Ω验证3 A和K=0.5～2，逐步验证6.5 A受控Trip吸能
→ PROFILE_FULL + AUX_SHARED_FORMAL：先保持主功率QF全断开，用外部仪器验证Vbulk/各路辅助/浪涌/掉电；HALF曾显示`HSDbg/HIDbg`时该回归为必做项
→ PROFILE_FULL + AUX_SHARED_FORMAL：再高阻轻载验证公共辅助噪声/ADC/启停/过零/VDS
→ PROFILE_FULL：再单机36→30/35 V，从低电流逐步加载
→ PROFILE_FULL：36→30 V并联
→ 额定验收和受控故障复核
```

任一级出现门极映射、死区、VDS、iL、Trip或状态机异常，都必须停止升级并返回该级修正。

### 14.2 分级硬门槛

第一次6 V前：

- 支路电流相对标定误差不超过0.5%，零点噪声目标小于0.03 A；
- 6 V功率输入与`AUX_SHARED_LAB_DEBUG`彼此独立；公共实验室辅助源限流正确且不通过保护地形成额外功率回路；
- `AUX_TOPOLOGY`与实物一致；使用AUX_SHARED_FORMAL时已断电检查公共控制GND不会经两块TL074电压前端形成H/L功率短路，公共DC采用星形配电且普通USB/JTAG已拔除；
- 图纸中四只SS510B已经DNP或更换为经校核的`VRRM≥150 V`器件，优先200 V；若保留外置二极管，还必须校核6.5 A换流脉冲、IFSM、反向恢复、结电容、温升和PCB寄生。任何100 V或只核对VRRM而未完成动态校核的外置二极管不得随正式功率板上电；
- 20 kHz ISR小于25 µs；
- 无功率门极逻辑正确，VGS高电平11～13 V、振铃在−2～15 V；
- VCCI、两路驱动侧12 V、MCU复位、看门狗复位和掉电时序全过程四路VGS保持低且无窄脉冲；
- 用外部仪器和示波器验证辅助电源上电、缓降、阶跃掉电和恢复全过程；记录UCC21520 UVLO、MCU复位及四路VGS，任何掉电恢复后MCU仍运行并重新输出PWM、窄脉冲或非预期门极的情况都必须停止放行并增加独立硬件联锁；
- QF0最坏相位重复合闸时，T励磁、两块Ci和公共辅助Cbulk的峰值电流不超过器件、开关和线束能力；没有整机浪涌限制或实测记录不得进入功率放行；
- CMPSS信号注入后1～2 µs内四路门极全部低并锁存；
- model0、复位和看门狗复位期间四路门极保持低；
- 从第一次功率上电起即使用BSC093N15NS5，CSD19533Q5A数据不能用于任何放行。

升FULL前：HALF SAFE和HALF IHI全部稳态、过零、负载阶跃和受控故障测试通过；在15 V输入运行窗下限记录当时实际辅助拓扑的`Vbulk_min`及12/5/3.3/±5 V和四路驱动电源，外部仪器确认无UVLO、复位或电源反复启动。对每只MOS和外置二极管，取HALF所有正常工况的实测最大值，至少覆盖18→17.5 V高阻、18→15 V单机2 A/并联4 A、负载阶跃、过零换向和启停，必须同时满足`2.1·VDS_pk_HALF_max≤120 V`和`2.1·VR_pk_HALF_max≤120 V`，即最大实测峰值不超过57.1 V。HALF IHI还必须在6.5 A受控Trip下实测钳位网络使任一MOS VDS小于135 V；故障电感能量由电流决定，不允许用电压2倍外推代替该实测。

若HALF使用`AUX_SHARED_LAB_DEBUG`，不要求在18 V下切回本已证明不能稳压的T供电辅助重测。改为进入FULL后先在36 V、`AUX_SHARED_FORMAL`下保持主功率QF全断，用外部仪器重做Vbulk、各辅助轨、浪涌和掉电；再于高阻轻载下重做公共辅助噪声、ADC、启停、过零和VDS回归。未全部通过前不得逐步增加FULL电流。FULL仍必须重做逐管正常`VDS≤120 V`和故障`VDS<135 V`测试。

### 14.3 正式验收矩阵

- Ui=36 V、RL=20 Ω：1～35 V每0.5 V，共69点；
- 单机1 V和35 V端点分别从0连续按键，验证trim以25 mV序列到达±0.100 V并饱和；长按不重复，`CMD`和ADC标定保持不变，外部Uo方向正确且四步累计变化可测；
- 按键复用验证：model1非端点时KEY1/2无效，model2时KEY1/2只改变K且不改变trim，KEY5/6只在0.5 V网格改变`Uref_cmd`，离开并返回端点时各自trim正确恢复；
- 30 V：在0.1 A测`Uo1`、2 A测`Uo2`，按`S=|(Uo2-Uo1)/Uo1|×100%`计算，正式`S≤0.5%`，内部目标`≤0.3%`；
- 主功率QF全部断开、公共AUX在线：先测辅助总实功率、公共Ui电流峰值和电压THD；
- 30 V/2 A：T本体损耗排除，但公共12/5/3.3/±5 V辅助电源的全部功耗必须计入输入有功功率，按`η=Po/Pin×100%`验证`η≥90%`；
- 30 V/4 A、K=1：外部测得`Kmeas=Io1_rms/Io2_rms`，验证0.9～1.1；
- 30 V/3 A：验证K=0.5、0.75、1、1.25、1.5、2；每点稳定至少10个工频周期，按`δ=|(Kset-Kmeas)/Kset|×100%`验证`δ≤3%`；
- THD在30 V/4 A、K=1下采集不少于20个完整50 Hz周期，统计2～50次工频谐波；20 kHz开关纹波单独报告，不计入该工频THD口径；持续观察至少60 s记录异步载波漂移中的最差值；
- 7.5 Ω负载始终连接：并联启动至少30次；
- 两块20 kHz载波相位按0°、90°、180°和持续漂移仿真；
- 单机30 V/2 A和并联30 V/4 A分别连续运行至少30 min，记录MOS、电感、驱动、隔离电源和电容温升；
- 每类故障关断都记录四路VGS、iL、Uo及每只MOS的VDS，正常≤120 V、硬故障≤135 V；
- 36 V输入、35 V输出时另记录每只外置二极管的最大反向电压和温升，正常`VR,pk≤120 V`、硬故障`VR,pk<135 V`，确认不存在100 V器件或重复雪崩；
- 输出频率用外部仪器验证为50 Hz，并记录PLL稳态误差；
- 正式数据全部在Flash独立运行、断开CCS/JTAG控制后采集；
- 故障测试按“仿真→无功率信号注入→低压功率→额定条件”的顺序执行。

OLED和软件值只用于运行观察；正式RMS、K、THD和效率以外部仪器为准。

## 15. 实施边界

后续实施计划必须遵循：

- 不修改或覆盖用户在原始`main.c`中的未提交改动；
- 从`D:\CCS\workspace_v12\F280049_Xiaosai`分别直接复制两个工程目录，不把`.git`、`Debug`、`.superpowers`和个人工具目录复制进去；
- 功能代码只修改两个新工程的`main.c`；工程改名只更新CCS正确区分项目所必需的`.project`名称及已有启动配置引用，不重新生成或改动其他外设/库配置；
- 首次生成启用`POWER_PROFILE=PROFILE_HALF`、`HALF_CURRENT_STAGE=HALF_SAFE`、`HALF_IHI_TEST_PASSED_ACK=0`、`DIRECTION_DIAG_6V=0`和`AUX_TOPOLOGY=AUX_SHARED_FORMAL`；6 V方向检查才按第4.1节临时切换DIAG与实验室辅助。18 V或15 V运行下限导致公共辅助不能稳压时，改用`AUX_SHARED_LAB_DEBUG`，OLED使用`HSDbg/HIDbg`前缀。`PROFILE_FULL`必须同时具源码`HALF_IHI_TEST_PASSED_ACK=1`和`FORMAL_HW_RELEASE_ACK=1`，否则编译失败；
- 首版不配置`AUX_OK`、`AUX_UV_TRIP`、F17或任何ADC/GPIO辅电检测路径；`ADCINB3`保持闲置。`FORMAL_HW_RELEASE_ACK`只代表操作者已按第2节和第14节完成外部仪器检查，不代表软件具备在线辅电监测；
- 第一次可运行版本保持PWM强制Trip，只验证ADC、按键、OLED和门极静态状态；
- 控制环按“固定门极映射→CMPSS/TZ→电流Kp→电流Ki→单机电压环→并联K=1→K端点”的顺序逐层开放；
- 所有首版PI、PLL、死区和保护数值都是已定义的安全整定起点，最终值必须由设计中的分阶段测试决定并记录。

实施计划必须拆成四个带人工放行点的工作包：

1. 复制/改名两工程，固定板角色，完成ADC、按键、OLED、PWM安全态和CMPSS/TZ；只允许无功率验证；
2. 完成AQ门极图样、无断流换向、电流环和稳压环；使用HALF SAFE完成两台18→15 V低压单机及18→17.5 V高阻放行；
3. 完成1号机双PLL、分流算法和双机状态机；先用HALF SAFE完成基本并联，再人工把`HALF_CURRENT_STAGE`改为`HALF_IHI`，依次完成两台单机2 A、并联4 A、K端点和6.5 A受控Trip；全部记录通过后才把`HALF_IHI_TEST_PASSED_ACK`改为1；
4. 开放FULL；若HALF使用实验室辅助，先在36 V高阻轻载下用获准的T供电公共辅助重做关键回归；再完成36 V并联、钳位/吸收整定、公共辅助噪声/效率、温升和正式验收。

每个工作包都以用户提供的示波器、标定和功率测试结果为继续条件。没有实测结果时，代理必须停在对应放行点，不能自行填写“最终”PI、死区、极性或吸收参数。

题目中的系统重量评分不属于控制固件实现：称重边界为不含T和RL，机械减重另行处理。最终设计报告也是独立交付物，可引用本规格和验收记录，但不包含在首轮固件编码工作包内。
