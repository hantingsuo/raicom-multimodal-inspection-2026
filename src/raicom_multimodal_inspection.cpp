// ============================================================
//  RAICOM 2026 multimodal-inspection controller
//
//  This public source is based on the national-final integrated controller.
//  It combines line following, obstacle avoidance, stair traversal, platform
//  alignment, warning-sign actions, and Go2-side serial coordination with an
//  external manipulator in one mission state machine.
//
//  Publication revision:
//    - warning actions 1/2/3 are selected only by warning-sign recognition;
//    - placement commands 5/6 are selected only by placement-sign recognition;
//    - those identifiers cannot be preselected on the command line;
//    - an inconclusive recognition result suppresses the corresponding action.
//
//  Important optional arguments (order-independent):
//    nogui              disable OpenCV windows and trackbars;
//    dogonly            disable manipulator and material-camera operations;
//    9 | 10             forward or reverse stair strategy;
//    lab | sun          visual environment profile;
//    cal*               stationary calibration modes;
//    startat=<stage>    controlled field-test restart point;
//    linecam=<device>   line-camera selection;
//    laserdev=<path>    front range-sensor serial device;
//    armdev=<path>      external manipulator serial device.
//
//  Run without arguments for the complete deployment and calibration help.
//  Review docs/HARDWARE_SETUP.md and docs/SAFETY.md before hardware use.
// ============================================================

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/ros2/PointStamped_.hpp>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <atomic>
#include <thread>
#include <vector>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <unistd.h>
#include <climits>
#include <algorithm>
#include <cstdint>
#include <sstream>
#include <string>
#include <deque>       // 台阶到顶判据的 rel_pitch 滚动窗
#include <utility>
#include <cstring>     // strerror (机械臂串口)
#include <cstdlib>     // realpath (串口 by-path/by-id 符号链接解析)
#include <cerrno>      // errno
#include <fcntl.h>     // open
#include <termios.h>   // 串口配置
#include <sys/ioctl.h> // DTR/RTS
#include <sys/select.h> // calmat 终端按键兜底

// ============================================================
//  共享全局状态 (三个模块统一)
// ============================================================
std::atomic<bool> g_running{true};

// rt/sportmodestate
std::atomic<bool>  g_state_received{false};
std::atomic<uint64_t> g_state_seq{0};
std::atomic<float> g_pos_x{0.0f}, g_pos_y{0.0f};
std::atomic<float> g_roll{0.0f}, g_pitch{0.0f};
std::atomic<float> g_yaw_rad{0.0f};   // rpy[2] 弧度  —— 台阶模块用
std::atomic<float> g_yaw_deg{0.0f};   // 四元数算出的 yaw (度,左转为正) —— 避障模块用

// 程序启动时的 yaw 基准 + 全局累计 yaw。
// 用每帧 dyaw wrap 后累加,避免 ±pi 跳变导致“转过一圈”丢失。
std::atomic<bool>  g_initial_yaw_ready{false};
std::atomic<float> g_initial_yaw_rad{0.0f};
std::atomic<float> g_initial_yaw_deg{0.0f};
std::atomic<float> g_global_yaw_prev_rad{0.0f};
std::atomic<float> g_global_yaw_cum_rad{0.0f};

// rt/lowstate  (真实四足触地力, 0=FR 1=FL 2=RR 3=RL)
std::atomic<bool> g_lowstate_received{false};
std::atomic<int>  g_foot_force[4];

// rt/utlidar/range_info  (官方处理好的三方向距离)
std::atomic<bool>  g_lidar_received{false};
std::atomic<float> g_front_dist{5.0f};
std::atomic<float> g_left_dist{5.0f};
std::atomic<float> g_right_dist{5.0f};

// 前方 GY-53 / VL53L0X 激光测距。避障只替换前距; 左右仍用 Go2 lidar。
// ★ 串口路径命令行 laserdev= 可覆盖; 当前默认前激光 /dev/ttyUSB0。
static std::string g_front_laser_dev = "/dev/ttyUSB0";
struct LaserSensorState {
    std::atomic<float> dist_m{8.0f};
    std::atomic<long>  ms{0};
};
LaserSensorState g_front_laser;
std::atomic<bool> g_front_laser_running{false};
const long  FRONT_LASER_STALE_MS = 500;

void signalHandler(int) {
    std::cout << "\n[!] 收到中断信号,准备停止..." << std::endl;
    g_running = false;
}

// ============================================================
//  共享工具函数
// ============================================================
// ---------- ★ GUI 开关 (nogui 比赛模式) ----------
//   g_gui_enabled=false 时: 不创建窗口/滑动条, 不 imshow, waitKey 直接返回 -1。
//   ESC 中断只在 GUI 模式可用; Ctrl+C (SIGINT) 任何模式都可用。
bool g_gui_enabled = true;
inline int guiWaitKey(int delay_ms = 1) {
    if (!g_gui_enabled) return -1;
    return cv::waitKey(delay_ms);
}

// ---------- ★ 摄像头打开 (2026-07-04): 索引或稳定路径 ----------
//   纯数字 = /dev/videoN 索引 —— 受 USB 枚举顺序影响, 插拔任何 USB 设备都可能漂移;
//   其他 = 设备路径, 推荐 /dev/v4l/by-id/usb-XXX-video-index0
//   (按相机型号+序列号命名, 和插拔顺序/接口无关, 现场强烈建议用这个)。
bool openCameraByIdOrIndex(cv::VideoCapture& cap, const std::string& id, const char* tag) {
    bool is_idx = !id.empty()
                  && id.find_first_not_of("0123456789") == std::string::npos;
    // ★ 必须显式指定 CAP_V4L2; 默认走 GStreamer 后端,
    //   set(CAP_PROP_AUTO_EXPOSURE/EXPOSURE/WB) 会全部失败 (unhandled property)
    if (is_idx) {
        int idx = std::stoi(id);
        cap.open(idx, cv::CAP_V4L2);
        if (!cap.isOpened()) {
            std::cerr << "[" << tag << "] V4L2 后端打开失败, 回退默认后端 (曝光将无法锁定)"
                      << std::endl;
            cap.open(idx);
        }
    } else {
        cap.open(id, cv::CAP_V4L2);
    }
    return cap.isOpened();
}

inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}
inline float rad2deg(float r) { return r * 180.0f / 3.14159265f; }
inline float deg2rad(float d) { return d * 3.14159265f / 180.0f; }
// 角度归一化到 [-180,180] (度) —— 避障模块用
inline float normalize_180(float a) {
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}
// 角度归一化到 [-pi,pi] (弧度) —— 台阶模块用
inline float normalize_180_rad(float a) {
    while (a >  3.14159265f) a -= 2 * 3.14159265f;
    while (a < -3.14159265f) a += 2 * 3.14159265f;
    return a;
}

void ResetInitialYawBaseline() {
    float yaw0 = g_yaw_rad.load();
    float yaw0_deg = g_yaw_deg.load();
    g_initial_yaw_rad = yaw0;
    g_initial_yaw_deg = yaw0_deg;
    g_global_yaw_prev_rad = yaw0;
    g_global_yaw_cum_rad = 0.0f;
    g_initial_yaw_ready = true;
    std::cout << "[初始Yaw] baseline=" << rad2deg(yaw0)
              << "° (deg通道=" << yaw0_deg << "°) 全局累计 yaw 清零"
              << std::endl;
}

// ============================================================
//  DDS 回调 (统一: 一次性填好两套 yaw 表示)
// ============================================================
void StateHandler(const void* msg) {
    auto* s = (const unitree_go::msg::dds_::SportModeState_*)msg;
    g_pos_x   = s->position()[0];
    g_pos_y   = s->position()[1];
    g_roll    = s->imu_state().rpy()[0];
    g_pitch   = s->imu_state().rpy()[1];
    g_yaw_rad = s->imu_state().rpy()[2];
    float yaw_rad_now = g_yaw_rad.load();

    if (g_initial_yaw_ready.load()) {
        float prev = g_global_yaw_prev_rad.load();
        float dyaw = normalize_180_rad(yaw_rad_now - prev);
        g_global_yaw_cum_rad = g_global_yaw_cum_rad.load() + dyaw;
        g_global_yaw_prev_rad = yaw_rad_now;
    }

    // 四元数 → yaw (度, 左转为正, 不取负号 —— 与项目约定一致)
    float w = s->imu_state().quaternion()[0];
    float x = s->imu_state().quaternion()[1];
    float y = s->imu_state().quaternion()[2];
    float z = s->imu_state().quaternion()[3];
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    g_yaw_deg = std::atan2(siny_cosp, cosy_cosp) * 180.0f / 3.14159265f;

    g_state_received = true;
    g_state_seq.fetch_add(1, std::memory_order_release); // 所有本帧字段写完后再发布序号
}

void LowStateHandler(const void* msg) {
    auto* low = (const unitree_go::msg::dds_::LowState_*)msg;
    for (int i = 0; i < 4; i++) g_foot_force[i] = (int)low->foot_force()[i];
    g_lowstate_received = true;
}

void RangeInfoHandler(const void* msg) {
    auto* p = (const geometry_msgs::msg::dds_::PointStamped_*)msg;
    g_front_dist = (float)p->point().x();
    g_left_dist  = (float)p->point().y();
    g_right_dist = (float)p->point().z();
    g_lidar_received = true;
}

static long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

static float frontLaserDistFresh() {
    long t = g_front_laser.ms.load();
    if (t == 0 || (nowMs() - t) > FRONT_LASER_STALE_MS) {
        // ★ 断流降级: 回退 Go2 自带雷达前距 (原 avoid END 的前距来源)。
        //   原来返回 8m "无限远" 是 fail-open: 避障中激光掉线会顶着挡板
        //   走满 90s 看门狗。Go2 雷达即使精度差些, 也比假装前方开阔安全。
        static long last_warn_ms = 0;
        long now = nowMs();
        if (now - last_warn_ms > 2000) {   // 2s 限流告警, 避免刷屏
            std::cout << "[前激光] 数据断流 (>" << FRONT_LASER_STALE_MS
                      << "ms), 回退 Go2 雷达前距 " << g_front_dist.load()
                      << "m (查串口 laserdev=" << g_front_laser_dev << ")" << std::endl;
            last_warn_ms = now;
        }
        return g_front_dist.load();
    }
    return g_front_laser.dist_m.load();
}

static int openFrontLaserSerial9600(const char* dev) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        std::cerr << "[前激光] open " << dev << " 失败: " << std::strerror(errno) << std::endl;
        return -1;
    }
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "[前激光] tcgetattr " << dev << ": " << std::strerror(errno) << std::endl;
        close(fd);
        return -1;
    }
    cfmakeraw(&tty);
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | HUPCL);
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "[前激光] tcsetattr " << dev << ": " << std::strerror(errno) << std::endl;
        close(fd);
        return -1;
    }
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl != -1) fcntl(fd, F_SETFL, fl & ~O_NONBLOCK);
    tcflush(fd, TCIFLUSH);
    return fd;
}

static void feedFrontLaserByte(std::vector<unsigned char>& f, unsigned char b) {
    if (f.empty()) {
        if (b == 0x5A) f.push_back(b);
        return;
    }
    if (f.size() == 1) {
        if (b == 0x5A) f.push_back(b);
        else { f.clear(); if (b == 0x5A) f.push_back(b); }
        return;
    }
    f.push_back(b);
    if (f.size() >= 4) {
        int len   = f[3];
        int total = 5 + len;
        if (total < 6 || total > 64) { f.clear(); return; }
        if ((int)f.size() >= total) {
            unsigned int sum = 0;
            for (int i = 0; i < total - 1; ++i) sum += f[i];
            bool chk_ok = ((sum & 0xFF) == f[total - 1]);
            if (chk_ok && f[2] == 0x15 && len >= 2) {
                int mm = (f[4] << 8) | f[5];
                if (mm > 0) {
                    g_front_laser.dist_m = (float)mm / 1000.0f;
                    g_front_laser.ms = nowMs();
                }
            }
            f.clear();
        }
    }
}

static void frontLaserThread() {
    std::vector<unsigned char> frame;
    while (g_running && g_front_laser_running.load()) {
        int fd = openFrontLaserSerial9600(g_front_laser_dev.c_str());
        if (fd < 0) {
            for (int i = 0; i < 20 && g_running && g_front_laser_running.load(); ++i) {
                usleep(50 * 1000);
            }
            continue;
        }
        std::cout << "[前激光] " << g_front_laser_dev << " 已打开 (9600 8N1)" << std::endl;
        frame.clear();
        unsigned char buf[256];
        while (g_running && g_front_laser_running.load()) {
            ssize_t n = read(fd, buf, sizeof(buf));
            if (n > 0) {
                for (ssize_t i = 0; i < n; ++i) feedFrontLaserByte(frame, buf[i]);
            } else if (n < 0) {
                if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                    usleep(2 * 1000);
                    continue;
                }
                std::cerr << "[前激光] read 错误: " << std::strerror(errno)
                          << ", 重连" << std::endl;
                break;
            }
        }
        close(fd);
    }
    std::cout << "[前激光] 线程退出" << std::endl;
}

// ============================================================
// ============================================================
//  模块 1: 巡线  (源自 line.cpp v3,逻辑原样保留)
//          + 当前整合流程:第一阶段巡线 10s + 盲走 2s 后进入避障
// ============================================================
// ============================================================

// ---------- 摄像头配置 ----------
// linecam= 可以是索引 ("0") 也可以是稳定路径 ("/dev/v4l/by-id/...-video-index0")
std::string g_line_cam_id = "0";
const std::string GO2_CAM_PIPELINE =
    "udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 "
    "! application/x-rtp, media=video, encoding-name=H264 "
    "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert "
    "! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1";
const bool USE_USB_CAMERA = true;

// ---------- 摄像头曝光锁定 (应对强直射光) ----------
//   原理: 自动曝光在直射区会把亮度压低 → 黑线灰化, mask 抓不到。
//         锁定手动曝光到一个"偏低"的值, 让直射区不过曝, 黑线保持深色。
//   V4L2 约定: CAP_PROP_AUTO_EXPOSURE = 1 表示手动, 3 表示自动 (反直觉)。
//   g_cam_exposure_val 范围因摄像头而异 (常见 0-2047 或 0-10000),
//   实测调到能在直射区清晰看到黑线即可, 起点 100-300, 太低就全黑。
const bool CAM_LOCK_EXPOSURE = true;     // sun 模式是否锁手动曝光
int  g_cam_exposure_val      = 156;      // sun 模式手动曝光值 (★ 可由 exp= 命令行覆盖)
const bool CAM_LOCK_WB       = false;    // ★ 关闭 WB 锁: 4600K 不匹配真实色温会绿偏,
                                          //   红圆识别不到。 让 WB 走自动, 不影响曝光锁。
const int  CAM_WB_TEMP       = 4600;     // (上面 false 时无效)
const bool CAM_LOCK_GAIN     = false;    // 是否锁增益 (有些摄像头需要)
const int  CAM_GAIN_VAL      = 0;        // 增益值 (0 = 最低)

// ---------- 视觉环境 profile ----------
// lab: 实验室/人造光。黑线发灰或 mask 断线时更宽松,并恢复自动曝光。
// sun: 强太阳光。沿用低曝光 + 保守阈值,防止白地过曝后吞掉黑线。
enum class VisionProfile { LAB, SUN };
VisionProfile g_vision_profile = VisionProfile::LAB;

const int SUN_MAX_BRIGHTNESS    = 120;
const int SUN_REAL_LINE_MIN     = 60;
const int SUN_USE_OTSU          = 1;
const int SUN_MIN_AREA          = 5000;
const int SUN_BLUR_SIZE         = 5;
const int SUN_CONNECT_THRESHOLD = 220;

const int LAB_MAX_BRIGHTNESS    = 130;
const int LAB_REAL_LINE_MIN     = 80;
const int LAB_USE_OTSU          = 1;
const int LAB_MIN_AREA          = 1800;
const int LAB_BLUR_SIZE         = 5;
const int LAB_CONNECT_THRESHOLD = 260;

// 阈值过宽时,地面纹理/平台边缘可能连成巨大色块。
// 黑色导引线通常不会占到整帧/整块 ROI 的这么大比例,直接丢掉更稳。
const double MAX_FRAME_CONTOUR_AREA_RATIO = 0.90;
const double MAX_ROI_CONTOUR_AREA_RATIO   = 0.90;

// ---------- 十字横杠识别 (穿越黑色十字时不被横杠带偏方向) ----------
//   选中轮廓"很宽" + "质心两侧都很长"(对称横杠) → 判为十字横杠,该 ROI 这一帧不用它导向。
//   直角拐弯的横臂只伸向一侧 → 质心一侧很短 → 不会被误判 (再叠加远端 ROI 兜底, 更稳)。
const double CROSS_BAR_MIN_W_RATIO = 0.55; // 外接框宽 / 画面宽 超过此值才可能是横杠
const double CROSS_BAR_SIDE_RATIO  = 0.20; // 质心到左、右两端的最小延伸 / 画面宽 (两侧都满足=对称横杠)

// ---------- 巡线阈值参数 (滑动条) ----------
int g_max_brightness    = LAB_MAX_BRIGHTNESS;
int g_real_line_min     = LAB_REAL_LINE_MIN;
int g_use_otsu          = LAB_USE_OTSU;
int g_min_area          = LAB_MIN_AREA;
int g_blur_size         = LAB_BLUR_SIZE;
int g_connect_threshold = LAB_CONNECT_THRESHOLD;   // 轨迹连续性窗口 (像素)

// ---------- ★ 自适应阈值 (对比度法, 抗光照/磨损) ----------
//   统计区域 = 近+远 ROI 行带 (不用全帧, 防止狗影子/远处黑物劫持最暗点):
//     floorL   = 行带 L 的 70 分位 (地板亮度)
//     minL     = 行带 L 最小值     (黑线亮度)
//     contrast = floorL - minL
//   contrast < g_min_contrast → 判无线 (替代绝对值真线保护)
//   阈值 thr = minL + g_contrast_frac% * contrast, 并夹在 [minL+15, floorL-20]。
//   光照/曝光变化时 floorL/minL 一起浮动, 阈值自动跟随;
//   磨损发灰的线 (minL=100, floor=200) 也能过 (旧逻辑 minL>80 直接拒识)。
//   g_use_adaptive=0 时回退旧逻辑 (固定 g_max_brightness + 绝对真线保护)。
int g_use_adaptive  = 1;    // 1=自适应阈值, 0=旧固定阈值 (现场可一键回退)
int g_min_contrast  = 50;   // 最小地板-黑线对比度
int g_contrast_frac = 60;   // 阈值位置 (%): thr = minL + frac% * contrast
// HUD 调试输出 (extractBlackLine 每帧回写, 标定时看这四个数)
int g_dbg_minL = 0, g_dbg_floorL = 0, g_dbg_contrast = 0, g_dbg_thr = 0;

// ---------- PD 控制参数 (滑动条 *1000) ----------
int g_kp_x1000 = 2;
int g_kd_x1000 = 3;

// ---------- 速度控制参数 ----------
const double MAX_SPEED         = 0.20;
const double MIN_SPEED_RATIO   = 0.30;
const double ROTATION_LIMIT    = 0.5;
const double SPEED_DECAY_PIVOT = 200.0;
// ★ 三连转 phase 2 (第二个 90° 触发后) 速度倍率, 让狗在最后一段慢下来
//   收尾更稳, 退出本阶段 (THREE_TURN_DONE) 自然恢复
const double TT_PHASE2_SPEED_MULT = 0.5;

// ---------- ROI 配置 ----------
// 近端 ROI 原来吃到画面底部 95%, 实验室视角下会看到狗自身雷达/机身而误检。
// 现在把近端检测窗口整体上移,并截掉底部区域;远端 ROI 保持原逻辑。
const double ROI_NEAR_TOP    = 0.45;
const double ROI_NEAR_BOTTOM = 0.7;
const double ROI_FAR_TOP     = 0.20;
const double ROI_FAR_BOTTOM  = 0.50;

// ---------- 丢线恢复 ----------
const int    LOST_SOFT_THRESHOLD = 8;
const int    LOST_HARD_THRESHOLD = 30;
const double SEARCH_YAW          = 0.3;
// ★ 检测模式 (PLATFORM_DETECT / THREE_TURN_DETECT / DUAL_PLATFORM_DETECT) 下,
//   HARD LOST 不硬停, 而是慢速盲走前进, 让 yaw/lidar 检测继续累积。
//   (否则硬停 → 不动 → 永远到不了触发条件 → 死循环)
const float  LOST_BLIND_VX       = 0.15f;

// ---------- 第一阶段硬时序参数 ----------
// 第一阶段:前跳后先固定速度冲击启动,再低速定时巡线,随后不再识别视觉,直接盲走 STAGE1_BLIND_TIME_SEC 秒。
const double STAGE1_LINE_KICK_TIME_SEC = 0.5;  // 前跳后的巡线启动段
const double STAGE1_LINE_KICK_SPEED    = 0.20;
const double STAGE1_LINE_SLOW_TIME_SEC = 6.0;  // 启动后低速巡线段 (★ 4.0→6.0: 进走廊前把头摆正的时间加长)
const double STAGE1_LINE_SLOW_SPEED    = 0.05;
const double STAGE1_LINE_ROT_MULT      = 4.0;   // 前跳后这段巡线的转向修正倍率 (★ 3.0→4.0: 摆正修正加快;
                                                //   限幅 ROTATION_LIMIT×倍率 自动跟随)
// ★ 2026-07-05 (治第一跳踉跄): 前跳落地后的巡线, 转向倍率不再一上来就 x3,
//   而是在此秒数内从 x1 线性升到 STAGE1_LINE_ROT_MULT。落地姿态若偏线,
//   旧逻辑第一帧就给到 1.5rad/s 大扭转 + 0.20 前进 → 刚落地的狗被拧踉跄。
const double STAGE1_ROT_RAMP_SEC       = 1.5;
const double STAGE1_BLIND_TIME_SEC = 2.0;
const double BLIND_FORWARD_SPEED   = 0.18;  // 盲走直行速度,现场可按狗状态微调到 0.15~0.20

// ---------- ★ 国赛: 第一跳落地后的固定左移 ----------
// 航向复位完成后不再读取黑线做左右居中；固定左移2s，再直接进入定时巡线。
// vy>0=左；横移期间仍锁定跳前 yaw，防止平移把刚调好的朝向带偏。
const float  POST_JUMP_FIXED_LSHIFT_VY       = +0.05f;
const double POST_JUMP_FIXED_LSHIFT_SEC      = 2.0;
const double POST_JUMP_LSHIFT_STATE_STALE_SEC = 0.30;
const float  POST_JUMP_LSHIFT_YAW_KP         = 0.02f;
const float  POST_JUMP_LSHIFT_OMEGA_MAX      = 0.30f;

// ---------- 前跳 + 红圆收尾序列参数 (现场可调) ----------
const double START_LINE_SEC     = 2.0;   // 程序开始先巡线多少秒再前跳 (★ 现为横杆检测的"名义时刻")
const double START_PREJUMP_LINE_SPEED = 0.20; // 第一个前跳前巡线固定速度
const int    FRONTJUMP_WAIT_SEC = 3;     // 每次前跳后等待动画完成秒数
// ★ 2026-07-05 前跳稳定性修复 (治"第一跳踉跄"):
//   ① 起跳前不再"0.20 m/s 急刹 0.5s 就跳": 先线性减速再停, 停稳后用 IMU 姿态门确认;
//   ② 落地后的站稳时间 0.6 → 1.5s (第一跳后紧接 0.20+转向x3 的 kick, 0.6s 不够狗回稳)。
const double PREJUMP_DECEL_SEC     = 0.6;  // 起跳前 vx 线性减速到 0 的时长
const double PREJUMP_MIN_STAND_SEC = 1.0;  // 软停后至少站稳这么久才允许起跳
const double PREJUMP_MAX_STAND_SEC = 2.5;  // 姿态迟迟不达标的兜底上限 (到时也跳)
const float  PREJUMP_ATT_TOL_DEG   = 2.5f; // 起跳姿态门: |roll| 和 |pitch| 都小于此值
// ★ 2026-07-06 第三轮 ("一触发就跳"): 横杆识别触发的前跳走快速通道 ——
//   接近速度已经是 0.10 蠕行, 动量极小, 不需要完整的 0.6s 减速 + 1.0~2.5s 站稳仪式;
//   快速通道: 0.2s 减速 → 零速流软停 → 0.3s 站稳 (姿态好立即跳, 最多 0.8s 兜底)。
//   触发→起跳延迟从 1.6~3.1s 压到 0.5~1.0s。收尾/异常路径仍走完整慢通道。
const double PREJUMP_QUICK_DECEL_SEC     = 0.2;
const double PREJUMP_QUICK_MIN_STAND_SEC = 0.3;
const double PREJUMP_QUICK_MAX_STAND_SEC = 0.8;
const double POST_JUMP_SETTLE_SEC = 1.5; // 前跳落地后原地站稳的秒数 (只等待, 不切步态/不发前进);
                                          //   给落地留缓冲=跳稳, 之后由 runLineFollowing 单次 StaticWalk 锁步态 (=xbts)
                                          //   ★ 原 0.6s, 实测第一跳落地后紧接大转向修正易踉跄, 加长到 1.5s
// ---- ★ 2026-07-06 前跳落地姿态看门狗 (治"落地踉跄~40%/向前拱→人工干预-30") ----
//   落地窗口内姿态健康时【一条指令都不发】(不破坏步态链); 姿态坏了才分级出手:
//   坏 0.3s → BalanceStand() 重踩;  再坏 0.8s → RecoveryStand() 完整自恢复。
const double POST_JUMP_GUARD_START_SEC    = 1.5;  // 窗口前段是跳跃动画 (pitch 大属正常), 不查
const float  POST_JUMP_GUARD_PITCH_DEG    = 12.0f;// |pitch| 超此值算姿态坏
const float  POST_JUMP_GUARD_ROLL_DEG     = 8.0f; // |roll| 超此值算姿态坏
const double POST_JUMP_GUARD_HOLD_SEC     = 0.3;  // 坏姿态持续这么久 → BalanceStand
const double POST_JUMP_GUARD_ESCALATE_SEC = 0.8;  // BalanceStand 后仍坏这么久 → RecoveryStand
const double POST_JUMP_GUARD_RECOVERY_WAIT_SEC = 2.5; // RecoveryStand 动作完成等待
// 前跳航向偏置标定: 连跑后把“起跳后 yaw 变化”的均值取反填入前馈。
const float  JUMP_YAW_FF_DEG    = 0.0f;
const float  JUMP_YAW_ALIGN_TOL = 1.5f;
float g_last_jump_yaw_delta_deg = 0.0f;
// ---- ★ 2026-07-07 八轮: 软急停 (模拟遥控器松杆, 治硬停冻结歪站姿) ----
const double SOFTSTOP_SETTLE_SEC = 1.2;  // 零速流保持时长 (步态自己减速+原地踏步踩方)
void softStop(unitree::robot::go2::SportClient& sport, double settle_sec);

// ---------- ★ 白色横杆障碍识别 (2026-07-05, 触发前跳; obdetect=0 可整体关闭) ----------
//   原理: 不直接找"白色物体" (横杆和白地对比度可能很低), 而是找"黑色导引线被一段
//   '非黑'横带截断"的图案 —— 沿线中心竖条带逐行统计黑线 mask:
//        线段(近/下方) ← 亮缺口(横杆) ← 线段(远/上方)
//   缺口行的平均亮度必须明显高于黑线阈值 (排除阴影/磨损假缺口),
//   缺口高度限制在 [OB_GAP_MIN_PX, OB_GAP_MAX_PX] (排除十字横杠/整段丢线)。
//   时序: 只在 [名义时刻-OB_WIN_HALF_SEC, 名义时刻+OB_WIN_HALF_SEC] 窗口内检测;
//   连续 OB_STABLE_FRAMES 帧看到图案、且缺口下沿走到画面 OB_TRIGGER_ROW_FRAC 行以下
//   才触发 → 起跳距离由"横杆在画面里的位置"决定, 不再吃行走速度误差。
//   窗口上限到了还没识别到 → 按原计时逻辑兜底照跳 (最坏情况 = 现在的纯计时方案)。
bool         g_ob_detect_enabled  = true;  // obdetect=0|1 命令行可关
// ★ 2026-07-06 (治"识别到了但跳得晚"): 障碍接近速度改为 "0.20 起步踢 0.5s → 0.10 蠕行"
//   (用户方案)。0.10 m/s 直接起步越不过 Go2 低速死区 (狗不动), 先 0.20 踢 0.5s 再降;
//   蠕行让检测多看好几帧、且刹停/减速距离减半 —— 起跳位置直接提前且更一致。
//   两个前跳落地之后各段巡线速度不变, 不受影响。
const double OB_APPROACH_KICK_SPEED = 0.20;  // 起步踢速度 (越过低速死区)
const double OB_APPROACH_KICK_SEC   = 0.5;   // 起步踢时长
const double OB_APPROACH_SLOW_SPEED = 0.10;  // 之后的慢速蠕行 (检测与起跳都在此速度)
// ★ 检测窗半宽改为按"距离"定义 (速度变了时间窗自动等效缩放): ±0.40m / 0.10 = ±4.0s
const double OB_WIN_HALF_DIST     = 0.40;    // 检测窗半宽 (米)
const double OB_WIN_HALF_SEC      = OB_WIN_HALF_DIST / OB_APPROACH_SLOW_SPEED;
const int    OB_STRIP_HALF_W_PX   = 70;    // 沿线中心取竖条带的半宽 (像素)
// ★ 2026-07-06 第四轮: 扫描条带顶到画面最上方 (0.30→0.02) —— 旧上界砍掉了画面上
//   30%, 横杆刚进视野时"缺口上方的远端线段"落在扫描区外, 图案要等横杆走近才成立,
//   白白浪费了确认时间。detectObstacleGap 只在 TO_OBSTACLE 窗口内被调用, 前跳
//   结束后自然"恢复"(其余模式根本不跑这段检测), 不需要任何还原逻辑。
const double OB_SCAN_TOP_FRAC     = 0.02;  // 条带扫描行范围 (画面高度比例)
const double OB_SCAN_BOTTOM_FRAC  = 0.97;
const int    OB_ROW_LINE_MIN_PX   = 25;    // 该行条带内黑线像素 ≥ 此值 = "该行有线"
const int    OB_GAP_MIN_PX        = 12;    // 缺口最小高度 (太窄=拼缝/噪声)
const int    OB_GAP_MAX_PX        = 260;   // 缺口最大高度 (太高=真丢线, 不是横杆)
const int    OB_SEG_MIN_PX        = 10;    // 缺口上/下两段线各自的最小行数
const int    OB_GAP_BRIGHT_MARGIN = 15;    // 缺口平均亮度须 ≥ 黑线阈值 + 此余量
// ★ 触发行 = 起跳距离的【唯一】旋钮: 值越小(行越靠上)横杆离得越远就起跳。
//   0.55→0.48 (第四轮, 治"跳太晚"): 480 高画面上触发行 264→230, 提前 ~34px。
//   现场再站定微调: 把狗摆在理想起跳点, 读 HUD 的 gap_bottom, 设为 gap_bottom/H。
const double OB_TRIGGER_ROW_FRAC  = 0.32;  // 缺口下沿走到画面此比例行以下才起跳
// ★ 九轮 (收尾前跳跳太晚): 触发行上移(值更小)→ 横杆离得更远就起跳。
//   0.40→0.32 (用户: 起步前跳也改成和收尾一样的位置, 两个前跳现在同触发行 0.32)。
//   覆盖机制保留 (g_ob_trigger_row_frac), 以后想给两个前跳设不同位置可再用。
const double OB_TRIGGER_ROW_FRAC_FINAL = 0.32;
double       g_ob_trigger_row_frac = OB_TRIGGER_ROW_FRAC;  // 默认起步值, 收尾前设 FINAL
                                           //   (★ 0.48→0.40 现场标定: 触发线再上移一点, 更早起跳)
const int    OB_STABLE_FRAMES     = 3;     // 图案连续帧数确认
// 把"原纯计时方案 (匀速 0.20) 走过的距离"换算成新速度曲线的等效名义时刻:
//   dist = 0.20*0.5 (踢) + 0.10*t (蠕行)  →  START 0.4m→3.5s, FINAL 1.0m→9.5s
inline double obNominalSecForDist(double dist_m) {
    double kick_dist = OB_APPROACH_KICK_SPEED * OB_APPROACH_KICK_SEC;
    if (dist_m <= kick_dist) return dist_m / OB_APPROACH_KICK_SPEED;
    return OB_APPROACH_KICK_SEC + (dist_m - kick_dist) / OB_APPROACH_SLOW_SPEED;
}
const double FINAL_LINE_SEC     = 5.0;   // 双侧触发后:最终前跳前的限时巡线
const double FINAL_PREJUMP_LINE_SPEED = 0.20; // 最后前跳前巡线固定速度
const float  FINAL_TURN_TOL     = 1.0f;  // 收尾转弯容差 (严格)
const float  FINAL_YAW_EXTRA_LEFT_DEG = 8.0f; // 收尾左转目标相对初始 yaw 的偏置
                                              //   (★ 2026-07-07 收尾左转改为转到初始 yaw+8°)
const float  FINAL_SHIFT_VY     = -0.10f;// 最终未看到黑线时的默认向右找线速度 (vy<0=右)
const int    FINAL_CENTER_TOL_PX = 40;    // 最终停车:黑线中心距画面中心小于此值 (原 25, 放宽防振荡)
const int    FINAL_CENTER_FLUSH_FRAMES = 5;  // 最终居中前丢弃摄像头缓存帧 (原 15, 缩短防 gait 掉回站立)
const int    FINAL_CENTER_STABLE_FRAMES = 3; // 连续居中多少帧后才确认停车
const int    FINAL_CENTER_MIN_AREA = 600;    // 最终居中允许边缘残线, 不用全局 1800 面积门槛
const int    FINAL_CENTER_LOST_HOLD_FRAMES = 15; // 识别抖掉后, 短时间沿用上一次纠偏方向
// ★ 横移: 比例 + 地板速度。vy = clamp(-Kp*err_px, ±MAX); 线在右(err>0)→向右(vy<0)。
//   |err|>tol 时 |vy| 至少 FLOOR, 越过 Go2 横移死区 (实测 0.05 不动、0.2 能动)。
const float  FINAL_CENTER_VY_KP    = 0.0015f; // 比例增益 (m/s per px)
const float  FINAL_CENTER_VY_FLOOR = 0.10f;   // 最小有效横移速度 (越过横移死区)
const float  FINAL_CENTER_VY_MAX   = 0.18f;   // 最大横移速度
const double FINAL_CENTER_TIMEOUT_SEC = 25.0; // 居不中的兜底超时, 到时停车结束 (防无限卡死)
// (★ 九轮改四: 前跳后"巡线找启停区(见蓝/黑线消失/底部蓝)"整套已删 —— 越改越乱,
//  回归最原始"跳完→左转→固定右移2s→三段对齐"。相关 FINAL_SEEK_* 常量随之删除。)
const int    FINAL_CX_JUMP_MAX_REJECT    = 5;     // cx 跳变连续丢弃上限, 超则重锁新线 (三段对齐用)
const float  FINAL_YAW_HOLD_KP   = 0.02f;
const float  FINAL_YAW_HOLD_OMEGA_MAX = 0.30f;
// ★ 收尾相机居中之前的固定动作 (前跳完 + 左转完之后, 居中之前): 先向右平移, 再向前
const float  FINAL_PRE_RSHIFT_VY  = -0.15f; // 居中前向右平移速度 (vy<0=右)
const float  FINAL_PRE_RSHIFT_FIXED_VY  = -0.27f; // ★ 用户: 固定右移速度 0.15→0.25→0.27 (仅固定右移用)
const double FINAL_PRE_RSHIFT_FIXED_SEC = 3.0;    // ★ 九轮改四 (用户: 回归原始): 固定右移时长 (2.0→3.0 +1s)
// ★ 2026-07-06 三轮重写 (0706 日志: 假线让"提前停"在 0s 就触发, 固定右移等于被删,
//   真线从未进视野): 改为"至少右移 MIN_SEC (几何上必然需要的量), 之后继续右移
//   直到【排蓝检测】连续 LOCK_FRAMES 帧看到真线且线已到目标x附近, 上限 MAX_SEC"。
// (★ 六轮删除: 原 FINAL_PRE_RSHIFT_MIN_SEC=2.0 无条件右移 —— 前跳落点偏右时会把狗
//  直接推出启停区; 预平移已改为条件式: 有线不移、无线才找)
const double FINAL_PRE_RSHIFT_MAX_SEC = 6.0;  // 右移找线上限 (到时交给居中阶段继续搜)
const int    FINAL_PRE_RSHIFT_LOCK_FRAMES = 3; // 连续几帧看到真线才算"找到"
// (★ 2026-07-06 删除: 原"居中前向前 0.10×0.5s"环节, 见收尾序列处说明)
const float  FINAL_FB_SEARCH_BACK_VX  = -0.15f; // 收尾看不到蓝区时向后探一步/后移找蓝区速度
const double FINAL_FB_SEARCH_BACK_SEC = 0.5;    // 收尾看不到蓝区时向后探一步时长
const double FINAL_FB_SEARCH_BACK_MAX_SEC = 6.0; // 后探后仍看不到蓝区时, 有界后移找蓝区上限

// ---------- ★ 2026-07-06 收尾前后闭环: 蓝色启停区边界对齐 (fbalign=0|1 可关) ----------
//   启停区是蓝色垫子, 白地上"蓝→白"交界就是天然的横向参照线。
//   程序启动时狗本来就摆在启停区里 → 起跑前先记住蓝区前边界在画面里的行号 (起点参照);
//   收尾相机居中阶段, 左右居中的同时用 vx 把边界行伺服回起点参照行 ——
//   前后位置直接"复现起点摆位", 不吃前跳落距散布/转弯位置漂移/里程漂移。
//   起点没采到参照 (蓝区不在视野/阈值不对) 则退回 FINAL_FB_TARGET_ROW_FRAC_DEFAULT。
bool         g_fb_align_enabled = true;      // fbalign=0|1 命令行可关
const int    FINAL_FB_H_MIN = 90;            // 蓝色 HSV 色相下限 (OpenCV H: 0-180)
const int    FINAL_FB_H_MAX = 135;           // 蓝色 HSV 色相上限
const int    FINAL_FB_S_MIN = 50;            // 饱和度下限 (白地 S 低, 进不来)
const int    FINAL_FB_V_MIN = 40;            // 亮度下限 (黑线 V 低, 进不来)
const double FINAL_FB_ROW_BLUE_FRAC = 0.40;  // 一行内蓝色占比超过此值 = "垫子行"
const int    FINAL_FB_MIN_BLUE_ROWS = 25;    // 全图垫子行少于此 = 没看到蓝区 (防噪声)
const float  FINAL_FB_KP      = 0.0012f;     // vx = KP × (参照行 - 当前边界行) [px]
const float  FINAL_FB_VX_MAX  = 0.08f;       // 前后微调速度上限 (温和)
const float  FINAL_FB_VX_FLOOR = 0.04f;      // 最小有效前后速度 (越过低速死区)
const int    FINAL_FB_TOL_ROWS = 18;         // 边界行进入参照行 ± 此值 = 前后到位 (旧对称判据, 仅日志用)
// ★ 九轮 (用户: 前后对位好几次离前边线很近/踩出去了, 后边空间还很大): 到位判据改
//   【非对称】—— fb_err_rows = target-cur, cur=蓝区远端边界行, 狗前进 cur 增大→err 变负。
//   偏后(err>0)放宽 (利用后边大空间尽量靠后停); 偏前(err<0, 逼近远端边缘)收严, 绝不踩出前边。
const int    FINAL_FB_TOL_BACK  = 40;        // 偏后容差 (松): err ≤ 此值即到位, 停得靠后
const int    FINAL_FB_TOL_FRONT = 10;        // 偏前容差 (严): err ≥ -此值, 远离前边缘
const double FINAL_FB_NOBLUE_GIVEUP_SEC = 3.0; // 居中开始后一直看不到蓝区 → 先向后探一步
const double FINAL_FB_TARGET_ROW_FRAC_DEFAULT = 0.55; // 起点没采到参照时的兜底目标行 (×画面高)
int          g_final_fb_target_row = -1;     // 起点采到的参照边界行 (-1 = 未采到)
// ---- ★ 2026-07-06 ③前后启动踢 (五连测: 细调速度推不动站立的狗, 实测启动阈值 ~0.2) ----
//   失速检测 (用户方案): 命令了运动但里程计不动 → 0.2 m/s 定向踢 0.35s 破起步死区,
//   踢完回细调速度 (步态已在迈步, 低速能维持); 再失速就再踢。
const float  FINAL_FB_KICK_VX      = 0.20f;  // 启动踢速度 (实测 Go2 前后起步阈值)
const double FINAL_FB_KICK_SEC     = 0.35;   // 踢时长 (~7cm, 与 ±18px 容差同量级)
const double FINAL_FB_STALL_SEC    = 0.6;    // 连续命令运动这么久
const float  FINAL_FB_STALL_MOVE_M = 0.01f;  //   位移仍小于此 = 失速 → 踢
// ---- ★ ①左右目标 x 改起点参照 (治"次次偏右却不再调": 线在画面居中 ≠ 身体在区里居中,
//        相机装偏/线不在启停区正中都是系统差; 人工摆正那一刻线的 x 就是正确答案) ----
int          g_final_lat_target_x = -1;      // 起点采到的线参照 x (-1 = 未采到, 退回画面中心)
const int    FINAL_PRE_RSHIFT_STOP_MARGIN_PX = 40; // 预右移时线进目标x+此裕量 → 提前停右移

// ---- ★ 2026-07-06 收尾改三段对齐: ①左右居中 → ②朝向对平 → ③前后对位 ----
//   ②用【视觉】把蓝白交界线转到与画面水平平行 —— 收尾"看着歪"的根因是
//   里程计 yaw 跑完全程有漂移 (对初始 yaw 已达标但画面明显斜), 必须以场地为基准。
//   对平完成后把当时的里程计 yaw 记为保持目标, ③阶段与最终校正都锁它。
const float  FINAL_YAW_VIS_KP          = 0.02f; // omega = KP × 边界倾角(°)
const float  FINAL_YAW_VIS_OMEGA_FLOOR = 0.05f; // 最小转速 (越过原地小角速死区)
const float  FINAL_YAW_VIS_TOL_DEG     = 1.2f;  // 倾角进此容差 = 已对平
const float  FINAL_YAW_VIS_TILT_MAX    = 25.0f; // 拟合倾角超此值视为误检, 不采用
const double FINAL_PHASE_LAT_MAX_SEC   = 6.0;   // ①左右相位兜底时长 (超时也往下走)
const double FINAL_PHASE_YAW_MAX_SEC   = 4.0;   // ②朝向相位兜底时长

// ---------- 阶段3 → 阶段4 之间的向右平移参数 ----------
//   定时巡线结束后, 进 runStairs 之前, 常规步态 (StaticWalk) 向右平移一段
const float  PRE_STAIR_SHIFT_VY  = 0.0f;    // 台阶前不再右移
const double PRE_STAIR_SHIFT_SEC = 0.5;

// ---------- 阶段4 (台阶) → 阶段5 (弧形) 之间的向右平移参数 ----------
//   下完台阶后停住, 常规步态向右平移一段, 再进弧形→平台
const float  POST_STAIR_SHIFT_VY  = -0.20f; // 向右速度 (vy<0 = 右)
const double POST_STAIR_SHIFT_SEC = 2.0;    // 平移持续时间
const double ARC_START_LINE_CHECK_SEC = 0.0; // 弧形启动后多久没锁上黑线就右移找线
                                             //   (★ 2026-07-06 2.0→0.0: 原来先罚站 2s 才开始右移,
                                             //    下台阶后本来就大概率看不到线 —— 现在进弧形巡线
                                             //    没锁线【立即】右移, 锁上线立刻恢复巡线, 与收尾
                                             //    右移同款 yaw 锁定; 上限仍 ARC_START_SHIFT_MAX_SEC)
// ★ 2026-07-05 弧形入口补救重写: 右移不再是"一次性 2s", 而是持续右移直到
//   near ROI 真正锁上导引线 (连续 ARC_START_LINE_LOCK_FRAMES 帧 valid),
//   上限 ARC_START_SHIFT_MAX_SEC 兜底。"看到线"不再用整幅画面里有黑块判断
//   (台阶体/平台侧面全是黑的, 旧判据必然第一帧误判"已看到线" → 补救永远不启动)。
const double ARC_START_SHIFT_MAX_SEC    = 6.0; // 持续右移找线的时长上限
const int    ARC_START_LINE_LOCK_FRAMES = 3;   // 连续多少帧锁上线才算"找到线"
const double POST_STAIR_LINE_REOPEN_WAIT_SEC = 1.0; // 台阶后重开巡线相机等待
const double POST_STAIR_LINE_FLUSH_SEC = 2.0;       // 台阶后按时间丢弃旧缓存帧
const int    POST_STAIR_LINE_CHECK_FRAMES = 8;      // 台阶后用多少帧新画面判定黑线
// ---------- no 模式: 不跑台阶的保底安全路径 ----------
//   阶段3定时巡线后: 后退 → 快速左移 → 左移找线 → 前进 → 原地左转90° → 进入弧形逻辑。
//   右移找线复用弧形入口自己的 POST_STAIR_SHIFT_VY / ARC_START_* 逻辑。
const float  NO_STAIR_PRE_BACK_VX        = -0.10f;
const double NO_STAIR_PRE_BACK_SEC       = 1.0;
const float  NO_STAIR_LEFT_FAST_VY        = +0.35f;
const double NO_STAIR_LEFT_FAST_SEC       = 4.0;
const float  NO_STAIR_LEFT_SEARCH_VY      = +0.25f;
const float  NO_STAIR_AFTER_LINE_FWD_VX   = +0.25f;
const double NO_STAIR_AFTER_LINE_FWD_SEC  = 1.0;
const float  NO_STAIR_TURN_LEFT_DEG       = 90.0f;
const float  NO_STAIR_AFTER_TURN_FWD_VX   = +0.20f;
const double NO_STAIR_AFTER_TURN_FWD_SEC  = 1.0;
// ★ 下台阶后、向右平移之前: 向机械臂发送 0, 然后原地等待
const double POST_STAIR_ARM0_WAIT_SEC = 10.0; // 发送机械臂 0 后原地等待时长

// ---------- 抓取平台停位后参数 ----------
//   到达抓取平台 + 左移后, 在此停留 PLATFORM_WAIT_SEC 秒, 期间运行机械臂抓取流程,
//   抓取完成 (或窗口超时) 再进入"三连转检测"巡线
const double PLATFORM_WAIT_SEC      = 80.0; // ★ 抓取窗口总时长: 串口启动+就位+检测+抓取+收尾等待
                                            //   (★ 十轮 70→80: 第一抓取补救改三段 退/右/退, 每段
                                            //    各占 5s 检测窗 + 移动时间; 且 D2 超时 24→32 盖住
                                            //    空夹重试 —— 最坏路径 ≈68s, 70s 会被恰好耗尽)
// ★ 抓取平台停下 + 5s 之后, 向左平移一段再进阶段 6
const float  PLATFORM_LSHIFT_VY  = +0.14f;  // 向左速度 (vy>0=左)
                                             // ★ 十三轮 (用户: 0.18 移过头): 0.18→0.14,
                                             //   3s 时长不变, 位移约 0.54→0.42m
const double PLATFORM_LSHIFT_SEC = 3.0;     // 平移持续时间
const float  PLATFORM_AFTER_GRAB_RSHIFT_VY  = -0.20f; // 第一平台抓取结束后向右 (vy<0=右)
const double PLATFORM_AFTER_GRAB_RSHIFT_SEC = 2.0;    // 第一平台抓取结束后右移时间
const float  PLAT1_ALIGN_TOL_DEG = 0.5f;    // 平台一停位锚点对准容差
float        g_plat1_trim_deg    = 0.0f;    // 平台一锚点 = 初始 yaw + trim (左正)

// ---------- 阶段6 三连转检测参数 (THREE_TURN_DETECT 模式) ----------
//   ★ 不是原地转, 而是继续巡线; 程序检测 yaw 累计变化经过三个相位:
//     phase 0: 当前 yaw 相对三连转初始 yaw ≥ +PHASE0_THRESH
//     phase 1: 当前 yaw 相对 phase 0 触发时 yaw ≤ -PHASE1_THRESH
//     phase 2: 当前 yaw 相对三连转初始 yaw ≥ +FINAL_THRESH,且全局累计 yaw 超过启动阈值
//   角度一直在变化,不再要求连续多帧;命中一次即进下一相位。
//   当前按现场要求: phase 0 用 +46°, phase 1 用 -31°;
//   最终停车放宽到 POST_PLAT_FINAL_THRESH_DEG, 避免第三个 90° 偶发不触发。
const float POST_PLAT_PHASE0_THRESH_DEG = 46.0f; // phase 0 触发阈值
const float POST_PLAT_PHASE1_THRESH_DEG = 31.0f; // phase 1 触发阈值
const float POST_PLAT_FINAL_THRESH_DEG = 90.0f; // phase 2 最终停车阈值
                                        // ★ 九轮 (用户): 93→90, 三连转最后触发条件改 90°
const int   POST_PLAT_STABLE_FRAMES    = 1;     // 角度触发:命中一次即算
const double TT_LINE_LOST_BACK_SEC      = 5.0;   // 三连转中 near ROI 连续无有效线多久后退一步
const int    TT_LINE_LOST_MAX_NUDGES    = 2;     // 三连转找线后退最多次数
const float  TT_LINE_LOST_BACK_VX       = -0.15f;
const double TT_LINE_LOST_BACK_MOVE_SEC = 1.0;
const float  TT_PHASE2_NEAR_TARGET_DEG  = 3.0f;  // phase2: 只在最终角目标前 3° 内启用卡死后退
const float  TT_PHASE2_STALL_DELTA_DEG  = 1.5f;  // phase2: 角度变化小于此值视为卡住
// (★ 2026-07-06 删除: 原 THREE_TURN_WAIT_SEC=5.0 第二抓取后停留, 现直接左移接后续)

// 三连转完成 5s 后的向左平移 + 原地左转 + 恢复巡线参数
const float  POST_TT_SHIFT_VY  = +0.20f;  // 向左速度 (vy>0=左)
const double POST_TT_SHIFT_SEC = 3.0;     // 平移持续时间
const float  POST_TT_TURN_DEG  = 55.0f;   // 平移后原地左转角度 (★ 九轮: 60→55, 用户要求少转5°)
const float  POST_TT_TURN_TOL  = 1.0f;    // 转弯容差 (严格)
const float  POST_TT_FWD_VX    = 0.15f;   // ★ 九轮 (用户): 左移+左转后向前速度
const double POST_TT_FWD_SEC   = 1.0;     //   向前时长 (★ 用户: 2.0→1.0)

// ---------- 红圆动作后 → 双侧放置平台检测参数 (DUAL_PLATFORM_DETECT 模式) ----------
//   红圆警示动作执行完后, 继续巡线, 直到左右雷达同时检测到放置平台 (近距离)
//   触发后停 DUAL_PLAT_WAIT_SEC 秒, 然后再进 FINISH_LINE1 计时巡线
const float  DUAL_PLAT_LIDAR_THRESH_M = 0.40f; // 左右 lidar 触发阈值 (m)
// ★ 红圆动作 → 双侧检测 之间, 必须先巡线 >= DUAL_PLAT_MIN_LINE_SEC 秒, 防止动作刚结束
//   时狗附近残留 lidar 读数立刻误触发
const double DUAL_PLAT_MIN_LINE_SEC   = 1.0;
// ★ 双侧触发 + 停 5s 之后, 向前走一段再进 FINAL_LINE 巡线
const float  POST_DUAL_FWD_VX  = 0.15f; // 向前速度
const double POST_DUAL_FWD_SEC = 1.0;   // 持续时间
const int    DUAL_PLAT_STABLE_FRAMES  = 5;     // 连续满足帧数 (≈0.1s)
const double DUAL_PLAT_WAIT_SEC       = 5.0;   // 触发后停留时间
const double DUAL_PLAT_POST_BLIND_SEC = 1.0;   // 双侧触发后盲走时间 (逻辑同避障盲走)
const double DUAL_ARM_AFTER_SEND_WAIT_SEC = 18.0; // ★ 九轮: 双侧发 5/6 等 D5/D6 超时 (实测最长 13.06s +5)
const double DOG_ONLY_PLATFORM_WAIT_SEC = 5.0; // dogonly: 每个机械臂平台只停留确认 5s

// ---------- 初始 yaw 回环触发参数 ----------
//   平台停位: 全局累计 yaw 相对程序启动时超过 302°,且当前 yaw 回到初始 yaw 附近 (<1°)。
//   三连转最终完成: 全局累计 yaw 相对程序启动时超过 302°,再配合局部三相位条件退出。
const float INITIAL_YAW_CUM_TRIGGER_DEG = 302.0f;
const float INITIAL_YAW_MATCH_TOL_DEG   = 1.0f;

// ---------- 弧形 → 抓取平台停位参数 (PLATFORM_DETECT 模式) ----------
//   当前角度触发改为: 全局累计 yaw 超过 302° 且当前 yaw 回到初始 yaw 附近,
//   同时左侧 lidar 读到平台立面 (60cm 实测 + 余量)。
//   角度一直在变化,不再要求连续多帧;命中一次即退出。
const float ARC_YAW_TARGET_DELTA_DEG = INITIAL_YAW_CUM_TRIGGER_DEG;  // HUD 显示用
const float ARC_YAW_TOL_DEG          = INITIAL_YAW_MATCH_TOL_DEG;    // yaw 回初始容差
const float ARC_LIDAR_LEFT_THRESH_M  = 0.70f;    // 左侧 lidar 触发阈值 (实测 60cm)
const int   ARC_STABLE_FRAMES        = 1;        // 角度触发:命中一次即算

// 兼容保留旧分支变量:主流程当前不再使用“丢线触发避障”。
const int LINE_LOST_TRIGGER_CONSEC = 1;
bool g_line_lost_avoidance_triggered = false;

// ---------- 状态机偏置接口 ----------
int g_fork_bias_near = 0;
int g_fork_bias_far  = 0;
int g_last_cx_near   = -1;
int g_last_cx_far    = -1;

const char* visionProfileName(VisionProfile p) {
    return (p == VisionProfile::SUN) ? "sun" : "lab";
}

void ApplyVisionProfileDefaults() {
    if (g_vision_profile == VisionProfile::SUN) {
        g_max_brightness    = SUN_MAX_BRIGHTNESS;
        g_real_line_min     = SUN_REAL_LINE_MIN;
        g_use_otsu          = SUN_USE_OTSU;
        g_min_area          = SUN_MIN_AREA;
        g_blur_size         = SUN_BLUR_SIZE;
        g_connect_threshold = SUN_CONNECT_THRESHOLD;
    } else {
        g_max_brightness    = LAB_MAX_BRIGHTNESS;
        g_real_line_min     = LAB_REAL_LINE_MIN;
        g_use_otsu          = LAB_USE_OTSU;
        g_min_area          = LAB_MIN_AREA;
        g_blur_size         = LAB_BLUR_SIZE;
        g_connect_threshold = LAB_CONNECT_THRESHOLD;
    }
}

void ResetTrajectory() {
    g_last_cx_near = -1;
    g_last_cx_far  = -1;
}

// ============================================================
//  ★ 巡线相机打开 + 全套配置 (从 main 抽出成函数)
//    供两处使用: ① main 启动时; ② runLineFollowing 断流重连时。
//    配置内容与原 main 内联代码一致: V4L2 后端、BUFFERSIZE=1、
//    lab/sun 曝光策略、自动白平衡恢复、可选增益锁、丢弃过渡帧。
// ============================================================
bool openAndConfigLineCamera(cv::VideoCapture& cap) {
    if (USE_USB_CAMERA) {
        std::cout << "[INFO] USB 摄像头: " << g_line_cam_id
                  << " (强制 V4L2 后端, 否则曝光控制不可用)" << std::endl;
        if (openCameraByIdOrIndex(cap, g_line_cam_id, "寻线相机")) {
            std::cout << "[OK] 打开成功, backend="
                      << cap.getBackendName() << std::endl;
        }
    } else {
        cap.open(GO2_CAM_PIPELINE, cv::CAP_GSTREAMER);
    }
    if (!cap.isOpened()) {
        std::cerr << "[错误] 无法打开摄像头" << std::endl;
        return false;
    }
    {
        bool ok_buf = cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
        std::cout << "[摄像头] set BUFFERSIZE=1 ret=" << ok_buf
                  << " 回读=" << cap.get(cv::CAP_PROP_BUFFERSIZE) << std::endl;
    }

    // ---------- 摄像头曝光/白平衡锁定 ----------
    //   lab 模式主动恢复自动曝光; sun 模式沿用低曝光,应对强直射光。
    //   V4L2 设置可能持久化,所以 lab 也必须显式 set 回自动曝光。
    bool use_manual_exposure =
        (g_vision_profile == VisionProfile::SUN) && CAM_LOCK_EXPOSURE;
    if (use_manual_exposure) {
        std::cout << "[摄像头] sun 模式: 尝试锁定曝光 target="
                  << g_cam_exposure_val << std::endl;
        // 先关自动曝光 (V4L2: 1=手动 3=自动)
        bool ok_auto = cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        usleep(100 * 1000);
        // 再设手动曝光值
        bool ok_exp  = cap.set(cv::CAP_PROP_EXPOSURE, g_cam_exposure_val);
        usleep(100 * 1000);
        std::cout << "  set AUTO_EXPOSURE=1 ret=" << ok_auto
                  << "  set EXPOSURE=" << g_cam_exposure_val << " ret=" << ok_exp
                  << std::endl;
        std::cout << "  回读 AUTO_EXPOSURE=" << cap.get(cv::CAP_PROP_AUTO_EXPOSURE)
                  << "  EXPOSURE=" << cap.get(cv::CAP_PROP_EXPOSURE) << std::endl;

        // 若回读的 AUTO_EXPOSURE 不是 1, 试试 3 (不同摄像头约定不同)
        if ((int)cap.get(cv::CAP_PROP_AUTO_EXPOSURE) == 3) {
            std::cout << "  [警告] AUTO_EXPOSURE 还是 3 (自动), 试 set=3..." << std::endl;
            cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);
            usleep(100 * 1000);
            cap.set(cv::CAP_PROP_EXPOSURE, g_cam_exposure_val);
            std::cout << "  二次回读 AUTO_EXPOSURE="
                      << cap.get(cv::CAP_PROP_AUTO_EXPOSURE)
                      << "  EXPOSURE=" << cap.get(cv::CAP_PROP_EXPOSURE) << std::endl;
        }
    } else {
        std::cout << "[摄像头] lab 模式: 恢复自动曝光 (避免沿用 sun 低曝光)"
                  << std::endl;
        // V4L2 常见约定: 3=自动曝光,1=手动曝光。
        bool ok_auto = cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);
        usleep(200 * 1000);
        std::cout << "  set AUTO_EXPOSURE=3 ret=" << ok_auto
                  << "  回读 AUTO_EXPOSURE=" << cap.get(cv::CAP_PROP_AUTO_EXPOSURE)
                  << "  EXPOSURE=" << cap.get(cv::CAP_PROP_EXPOSURE) << std::endl;
    }

    if (CAM_LOCK_WB) {
        std::cout << "[摄像头] 尝试锁定白平衡: target=" << CAM_WB_TEMP << "K" << std::endl;
        bool ok_awb = cap.set(cv::CAP_PROP_AUTO_WB, 0);
        usleep(50 * 1000);
        bool ok_wb  = cap.set(cv::CAP_PROP_WB_TEMPERATURE, CAM_WB_TEMP);
        std::cout << "  set AUTO_WB=0 ret=" << ok_awb
                  << "  set WB_TEMPERATURE=" << CAM_WB_TEMP << " ret=" << ok_wb
                  << std::endl;
        std::cout << "  回读 AUTO_WB=" << cap.get(cv::CAP_PROP_AUTO_WB)
                  << "  WB_TEMPERATURE=" << cap.get(cv::CAP_PROP_WB_TEMPERATURE)
                  << std::endl;
    } else {
        // ★ V4L2 设置会持久化! 上次锁的 WB 会留在驱动里, 必须主动恢复自动 WB,
        //   否则画面会一直绿偏 (停留在 4600K 之类的旧手动值)
        std::cout << "[摄像头] 恢复自动白平衡 (避免持久化的旧 WB 锁定)" << std::endl;
        bool ok_awb = cap.set(cv::CAP_PROP_AUTO_WB, 1);
        usleep(50 * 1000);
        std::cout << "  set AUTO_WB=1 ret=" << ok_awb
                  << "  回读 AUTO_WB=" << cap.get(cv::CAP_PROP_AUTO_WB) << std::endl;
    }

    if (CAM_LOCK_GAIN) {
        bool ok_gain = cap.set(cv::CAP_PROP_GAIN, CAM_GAIN_VAL);
        std::cout << "[摄像头] set GAIN=" << CAM_GAIN_VAL << " ret=" << ok_gain
                  << "  回读=" << cap.get(cv::CAP_PROP_GAIN) << std::endl;
    }

    // 锁定后丢弃前几帧 (有些摄像头切换设置后头几帧是过渡帧)
    {
        cv::Mat tmp;
        for (int i = 0; i < 5; ++i) cap >> tmp;
    }
    return true;
}

// ---------- ★ 巡线相机断流保护 ----------
//   连续空帧超过此数: softStop 并尝试重连 (跳跃/台阶震动可能让 USB 松动)
const int CAM_EMPTY_FRAMES_REOPEN = 30;

// ============================================================
//  第一阶段触发避障说明
//
//  当前版本不再识别“入口”文字，也不再依赖 entrance_template.png。
//  主流程采用硬时序:先巡线 10s,再直行盲走 2s,然后直接进入避障。
//  旧的“丢线触发”分支仅保留为备用代码,主流程不再调用。
// ============================================================

// ============================================================
//  引导线提取  (原样保留)
// ============================================================
bool extractBlackLine(const cv::Mat& frame, cv::Mat& mask, cv::Mat& dbg_L,
                      int min_area_override = -1) {
    static cv::Mat morph_kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

    cv::Mat lab;
    cv::cvtColor(frame, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> lab_ch;
    cv::split(lab, lab_ch);
    cv::Mat L = lab_ch[0];

    int bs = g_blur_size;
    if (bs % 2 == 0) bs += 1;
    if (bs < 3) bs = 3;
    cv::Mat blurred;
    cv::GaussianBlur(L, blurred, cv::Size(bs, bs), 0);
    dbg_L = blurred.clone();

    // ---------- ★ 阈值确定: 自适应 (对比度法) 或 旧固定阈值 ----------
    int  thr_used = 0;
    bool no_line  = false;
    if (g_use_adaptive) {
        // 统计区域 = 近+远 ROI 行带 (防止全帧最暗点被狗影子/远处黑物劫持)
        int y0 = (int)(blurred.rows * ROI_FAR_TOP);
        int y1 = (int)(blurred.rows * ROI_NEAR_BOTTOM);
        y0 = std::max(0, std::min(y0, blurred.rows - 2));
        y1 = std::max(y0 + 1, std::min(y1, blurred.rows));
        cv::Mat band = blurred.rowRange(y0, y1);

        double minL_d, maxL_d;
        cv::minMaxLoc(band, &minL_d, &maxL_d);
        int minL = (int)minL_d;

        // floorL = 行带 70 分位亮度 (地板占大头, 取分位数代表地板)
        int histSize = 256;
        float range[] = {0.f, 256.f};
        const float* histRange = range;
        cv::Mat hist;
        cv::calcHist(&band, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);
        double total = (double)band.total();
        double accum = 0.0;
        int floorL = 255;
        for (int i = 0; i < histSize; ++i) {
            accum += (double)hist.at<float>(i);
            if (accum >= total * 0.70) { floorL = i; break; }
        }

        int contrast = floorL - minL;
        g_dbg_minL = minL; g_dbg_floorL = floorL; g_dbg_contrast = contrast;

        if (contrast < g_min_contrast) {
            // 对比度不足 → 视野里没有"比地板暗得多"的东西 → 判无线
            no_line = true; g_dbg_thr = -1;
        } else {
                 int thr = minL + g_contrast_frac * contrast / 100;
            int lo  = minL + 15;          // 至少高出线亮度一点, 防止线被自己切掉
            int hi  = floorL - 20;        // 至少低于地板一截, 防止地板整片进 mask
            if (hi < lo) { no_line = true; g_dbg_thr = -1; }
            else {
                thr_used = std::max(lo, std::min(thr, hi));
                g_dbg_thr = thr_used;
            }
        }
    } else {
        // 旧逻辑: 全帧最暗点绝对保护 + 固定阈值 (useAdaptive=0 一键回退)
        double minL, maxL;
        cv::minMaxLoc(blurred, &minL, &maxL);
        g_dbg_minL = (int)minL; g_dbg_floorL = -1; g_dbg_contrast = -1;
        if (minL > g_real_line_min) { no_line = true; g_dbg_thr = -1; }
        else { thr_used = g_max_brightness; g_dbg_thr = thr_used; }
    }
    if (no_line) {
        mask = cv::Mat::zeros(blurred.size(), CV_8UC1);
        return false;
    }

    // 硬阈值 + Otsu 取交集
    cv::Mat hard_mask;
    cv::threshold(blurred, hard_mask, thr_used, 255, cv::THRESH_BINARY_INV);

    if (g_use_otsu) {
        cv::Mat otsu_mask;
        cv::threshold(blurred, otsu_mask, 0, 255,
                      cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        cv::bitwise_and(hard_mask, otsu_mask, mask);
    } else {
        mask = hard_mask.clone();
    }

    // 形态学
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, morph_kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, morph_kernel);

    // 轮廓面积过滤
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours,
                     cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat filtered = cv::Mat::zeros(mask.size(), CV_8UC1);
    double max_frame_area = mask.cols * mask.rows * MAX_FRAME_CONTOUR_AREA_RATIO;
    int min_area_used = (min_area_override >= 0) ? min_area_override : g_min_area;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > min_area_used && area < max_frame_area) {
            cv::drawContours(filtered, contours, i, cv::Scalar(255), cv::FILLED);
        }
    }
    mask = filtered;
    return true;
}

// ============================================================
//  ROI 内的轨迹连续性轮廓选择  (原样保留)
// ============================================================
int computeRoiDirection(const cv::Mat& full_mask, const cv::Rect& roi,
                        cv::Mat& debug_frame, const cv::Scalar& draw_color,
                        int& last_cx, int fork_bias,
                        bool* is_wide_bar = nullptr) {
    if (is_wide_bar) *is_wide_bar = false;
    cv::Mat roi_mask = full_mask(roi);
    int frame_center = full_mask.cols / 2;
    int target_cx = (last_cx >= 0) ? last_cx : (frame_center + fork_bias);

    cv::rectangle(debug_frame, roi, draw_color, 1);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roi_mask, contours,
                     cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    struct Cand { int idx; double area; int cx; int cy; };
    std::vector<Cand> cands;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area < g_min_area) continue;
        if (area > roi.area() * MAX_ROI_CONTOUR_AREA_RATIO) continue;
        cv::Moments m = cv::moments(contours[i]);
        if (m.m00 == 0) continue;
        int cx = int(m.m10 / m.m00) + roi.x;
        int cy = int(m.m01 / m.m00) + roi.y;
        cands.push_back({ (int)i, area, cx, cy });
    }
    if (cands.empty()) {
        last_cx = -1;
        return INT_MIN;
    }

    Cand* best = nullptr;
    double best_area = 0;
    for (auto& c : cands) {
        if (std::abs(c.cx - target_cx) <= g_connect_threshold && c.area > best_area) {
            best_area = c.area;
            best = &c;
        }
    }
    bool used_fallback = false;
    if (!best) {
        used_fallback = true;
        best_area = 0;
        for (auto& c : cands) {
            if (c.area > best_area) { best_area = c.area; best = &c; }
        }
    }

    // ★ 十字横杠识别: 选中轮廓很宽 + 质心两侧都很长 (对称) → 横杠
    cv::Rect bb = cv::boundingRect(contours[best->idx]);   // roi 局部坐标
    int cx_local  = best->cx - roi.x;
    int left_ext  = cx_local - bb.x;
    int right_ext = (bb.x + bb.width) - cx_local;
    bool wide_bar = (bb.width  > CROSS_BAR_MIN_W_RATIO * full_mask.cols)
                 && (left_ext  > CROSS_BAR_SIDE_RATIO  * full_mask.cols)
                 && (right_ext > CROSS_BAR_SIDE_RATIO  * full_mask.cols);
    if (is_wide_bar) *is_wide_bar = wide_bar;

    // 横杠帧不更新连续性基准 (避免把质心跳到横杠中点, 污染下一帧追踪)
    if (!wide_bar) last_cx = best->cx;

    std::vector<std::vector<cv::Point>> shifted = { contours[best->idx] };
    for (auto& p : shifted[0]) { p.x += roi.x; p.y += roi.y; }
    cv::drawContours(debug_frame, shifted, 0, draw_color, 2);
    cv::circle(debug_frame, cv::Point(best->cx, best->cy), 8, draw_color, -1);

    cv::Scalar tgt_color = used_fallback ? cv::Scalar(0, 0, 255)
                                         : cv::Scalar(255, 0, 255);
    cv::line(debug_frame,
             cv::Point(target_cx, roi.y),
             cv::Point(target_cx, roi.y + roi.height),
             tgt_color, 1);

    for (auto& c : cands) {
        if (&c == best) continue;
        cv::circle(debug_frame, cv::Point(c.cx, c.cy), 5,
                   cv::Scalar(128, 128, 128), -1);
    }

    return best->cx - frame_center;
}

// ============================================================
//  ★ 红色圆形检测 → 看到过红圆 → 红圆离开视野 → 停 → 执行警示动作
//    红圆是地面线上的标记, USB 巡线相机可见; 检测集成在 runLineFollowing。
//    触发方式: 视野内先看到红圆; 此后红圆连续消失超过 RED_LOST_TRIGGER_FRAMES 帧即停车。
//    (原 ±90° 原地转已删除, turnInPlace 仅供 FINAL 收尾左转 90° 使用)
//    动作号由警示标志识别给定: 1=伸懒腰 2=打招呼 3=闪前灯三次, 整程只做一次。
// ============================================================
// ---------- 红圆检测参数 (现场可调) ----------
int          g_red_s_min     = 80;    // HSV 饱和度下限 (★ reds= 可覆盖; 强光褪饱和调低)
int          g_red_v_min     = 50;    // HSV 明度下限 (★ redv= 可覆盖; 低曝光/阴影调低)
const double RED_MIN_AREA    = 800;   // 红斑最小面积 (像素), 过滤小红点
const double RED_CIRCULARITY = 0.50;  // 圆度下限 (允许远处/斜视/局部遮挡的红圆)
const int    RED_SEEN_CONFIRM_FRAMES = 3; // ★ 看到红圆多少帧后进入"等待消失" (原 1, 防单帧误触发)
const int    RED_LOST_TRIGGER_FRAMES = 5; // 看到过红圆后, 连续消失超过此帧数即停车
// ★ 红圆位置先验: 地面红圆贴近时必然出现在画面下半部; 立板上的识别标志
//   (红色同心圆靶) 出现在画面上部 → 只接受圆心 y 在画面下 50% 的候选。
const float  RED_MIN_Y_FRAC  = 0.50f;
// ★ 红圈等待超时 (FOREVER 模式): 超时按"已到检测点"降级 —— 停车、做动作、
//   置完成标志、进收尾。防止红圈检不出时整段挂死到比赛结束。
//   注意要大于"三连转结束→红圆"这段路的正常耗时 (现场实测后用 redtimeout= 调)。
double       g_red_timeout_sec = 40.0;

// ---------- ★ 无界检测阶段看门狗 (超时降级, 防整段挂死到比赛结束) ----------
//   超时后的降级行为 (主流程实现):
//     弧形→平台: softStop, 跳过第一抓取 (位置不可信, 不浪费 70s 窗口), 直接进三连转
//     三连转:     跳过第二抓取和盲移/转向 (狗仍在线上), 直接进红圈等待巡线
//     双侧平台:   按"已触发"同路径处理 (盲走→放置→收尾)
const double ARC_WATCHDOG_SEC  = 180.0; // 弧形→抓取平台检测
const double TT_WATCHDOG_SEC   = 120.0; // 三连转检测
const double DUAL_WATCHDOG_SEC = 60.0;  // 双侧平台检测
const int    WARN_ACT_WAIT_SEC = 5;   // 伸懒腰/打招呼后等待动画完成的秒数
// ★ 2026-07-06 第五轮 (0706 第七次实验: 打招呼时重心不稳 → 人工干预-30):
//   Hello/Stretch 是全身动作, 之前是"巡线急停 + 固定睡 1s"就直接做 —— 与前跳
//   第一版"急刹就跳踉跄"是同一个病根。照搬前跳的治法: 动作前先 BalanceStand
//   重踩站姿, 再用 IMU 姿态门 (|roll|,|pitch| < PREJUMP_ATT_TOL_DEG) 确认站稳,
//   至少 WARN_ACT_MIN_STAND_SEC, 姿态迟迟不达标 WARN_ACT_MAX_STAND_SEC 兜底照做。
const double WARN_ACT_MIN_STAND_SEC = 1.0;  // 动作前至少站稳这么久
const double WARN_ACT_MAX_STAND_SEC = 3.0;  // 姿态不达标的兜底上限 (到时也做)
const float  POST_RED_ACTION_LSHIFT_VY  = +0.20f; // 红圆动作完成后向左平移速度
const double POST_RED_ACTION_LSHIFT_SEC = 1.0;    // 红圆动作完成后向左平移时长
const double POST_RED_ACTION_LSHIFT_SEC_STRETCH = 2.0; // ★ 九轮 (用户): 仅动作1(伸懒腰)左移 +1s
const float  POST_RED_STRETCH_LTURN_DEG = 30.0f;  // 动作1伸懒腰左移后额外左转角度

// ---------- 红圆命中后的"停-左转-停-右转回-动作"序列参数 ----------
// (★ 八轮: RED_STOP_SEC 已删 —— 红圆停车改 softStop 零速流软停, 见 execRedActionSequence)
// ★ 八轮改二 (用户: 盲走前压不一定盖得全): 视觉伺服盖圈 —— 前压期间边用黑线
//   纠偏 (圈就在线上) 边盯红圈在画面里下沉, 连续 RED_COVER_LOST_FRAMES 帧看不到
//   圈 (=已进到身下/被裁成非圆) 再多推 RED_COVER_EXTRA_SEC 收尾盖过身体中心;
//   RED_COVER_MAX_SEC 兜底。触发距离每次不同, 它自己适应, 不赌固定时长。
const float  RED_COVER_FWD_VX      = 0.10f; // 前压速度
const int    RED_COVER_LOST_FRAMES = 5;     // 连续丢圈帧数 = 圈已进身下
const double RED_COVER_EXTRA_SEC   = 0.35;  // 丢圈后的收尾推进时长
const double RED_COVER_MAX_SEC     = 2.5;   // 前压总时长兜底
const float  RED_COVER_YAW_KP      = 0.002f;// 前压黑线纠偏增益 (rad/s per px)
const float  RED_COVER_OMEGA_MAX   = 0.25f; // 纠偏角速度上限
const float  RED_TURN_DEG     = 90.0f; // 原地转角度 (先左转此值, 再右转回原朝向)
const float  RED_TURN_OMEGA   = 0.6f;  // 原地转角速度 (rad/s)
const float  RED_TURN_TOL     = 1.0f;  // 转到位容差 (度,代码使用 <1°)
const float  RED_TURN_MAX_SEC = 8.0f;  // 单次转向超时保护 (秒)
const float  RED_TURN_KP      = 0.04f; // 转向比例控速系数 (deg→rad/s), 接近目标减速
const float  RED_TURN_OMEGA_MIN = 0.12f; // 末端最小角速度 (防太慢转不动)

// ---------- 警示动作相关全局 ----------
int  g_action_id        = 0;      // 视觉识别出的警示动作 (1/2/3; 0=尚未识别)
int  g_dual_arm_cmd     = 0;      // 视觉识别出的放置指令 (5/6; 0=尚未识别)
bool g_dog_only_mode    = false;  // 只调机械狗: 跳过机械臂串口和抓取相机
bool g_no_stairs_mode   = false;  // no: 保底安全路径, 跳过台阶流程
bool g_warn_action_done = false;  // 整程只执行一次动作
unitree::robot::go2::VuiClient* g_vui = nullptr;  // 前灯控制, 启动时初始化以支持识别结果3

// ============================================================
//  红色圆形检测: frame 里找够大、够圆的红斑, 命中则画圈并返回 true
// ============================================================
bool detectRedCircle(cv::Mat& frame) {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat m1, m2, mask;
    cv::inRange(hsv, cv::Scalar(0,   g_red_s_min, g_red_v_min),
                     cv::Scalar(10,  255, 255), m1);   // 红色低色相段
    cv::inRange(hsv, cv::Scalar(170, g_red_s_min, g_red_v_min),
                     cv::Scalar(180, 255, 255), m2);   // 红色高色相段
    mask = m1 | m2;
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    // ★ 红圈调试窗口 (GUI 模式): 标定 reds=/redv= 时看这个 mask
    if (g_gui_enabled) cv::imshow("Red Mask", mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < RED_MIN_AREA) continue;
        double peri = cv::arcLength(c, true);
        if (peri <= 1.0) continue;
        double circ = 4.0 * 3.14159265 * area / (peri * peri);  // 圆度
        if (circ >= RED_CIRCULARITY) {
            cv::Point2f ctr; float r;
            cv::minEnclosingCircle(c, ctr, r);
            // ★ 位置先验: 只接受画面下半部的红圆 (拒掉立板上的红色同心圆标志)
            if (ctr.y < frame.rows * RED_MIN_Y_FRAC) continue;
            cv::circle(frame, ctr, (int)r, cv::Scalar(0, 0, 255), 3);
            cv::putText(frame, "RED CIRCLE",
                        cv::Point((int)ctr.x - 45, (int)ctr.y - (int)r - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
            return true;
        }
    }
    return false;
}

// ============================================================
//  抽签标志视觉识别入口
//    警示动作和放置指令只由视觉识别产生。
//    识别失败返回 0, 主流程安全跳过对应动作, 不使用预设编号兜底。
// ============================================================
namespace autoid {

int largestContourIndex(const std::vector<std::vector<cv::Point>>& contours,
                        double min_area) {
    int best = -1;
    double best_area = min_area;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > best_area) {
            best_area = area;
            best = (int)i;
        }
    }
    return best;
}

int classifyWarningAction(const cv::Mat& frame, cv::Mat* debug = nullptr) {
    if (frame.empty()) return 0;
    cv::Mat hsv, yellow;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(15, 70, 70), cv::Scalar(45, 255, 255), yellow);
    cv::morphologyEx(yellow, yellow, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(yellow.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    int idx = largestContourIndex(contours, 1200.0);
    if (idx < 0) return 0;

    cv::Rect box = cv::boundingRect(contours[idx]) & cv::Rect(0, 0, frame.cols, frame.rows);
    if (box.width < 40 || box.height < 40) return 0;

    cv::Rect inner(
        box.x + box.width / 4,
        box.y + box.height / 5,
        box.width / 2,
        box.height * 3 / 5);
    inner &= cv::Rect(0, 0, frame.cols, frame.rows);
    if (inner.width < 20 || inner.height < 20) return 0;

    cv::Mat gray, black;
    cv::cvtColor(frame(inner), gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, black, 85, 255, cv::THRESH_BINARY_INV);
    cv::morphologyEx(black, black, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    cv::Mat labels, stats, centers;
    int n = cv::connectedComponentsWithStats(black, labels, stats, centers, 8);
    int comp_count = 0;
    int largest_area = 0;
    for (int i = 1; i < n; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area >= 40) {
            comp_count++;
            largest_area = std::max(largest_area, area);
        }
    }

    int band_y0 = std::max(0, black.rows * 2 / 3);
    cv::Mat bottom_band = black.rowRange(band_y0, black.rows);
    double bottom_ratio = (double)cv::countNonZero(bottom_band)
                        / std::max(1, bottom_band.rows * bottom_band.cols);
    double black_ratio = (double)cv::countNonZero(black)
                       / std::max(1, black.rows * black.cols);

    int action = 0;
    if (comp_count >= 4) {
        action = 3; // radiation: multiple separated lobes/dot
    } else if (bottom_ratio > 0.16 && black_ratio > 0.08) {
        action = 2; // oxidizer: circle/base gives heavier lower band
    } else if (largest_area > 0) {
        action = 1; // electric: one dominant lightning-like component
    }

    std::cout << "[AUTOID] warning comp=" << comp_count
              << " largest=" << largest_area
              << " bottom=" << bottom_ratio
              << " black=" << black_ratio
              << " -> action=" << action << std::endl;

    if (debug) {
        frame.copyTo(*debug);
        cv::rectangle(*debug, box, cv::Scalar(0, 255, 255), 2);
        cv::rectangle(*debug, inner, cv::Scalar(255, 0, 255), 2);
        cv::putText(*debug, "AUTOID warn action=" + std::to_string(action),
                    cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 255), 2);
    }
    return action;
}

int classifyPlaceCommand(const cv::Mat& frame, cv::Mat* debug = nullptr) {
    if (frame.empty()) return 0;
    cv::Mat hsv, r1, r2, red;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 70, 70), cv::Scalar(12, 255, 255), r1);
    cv::inRange(hsv, cv::Scalar(168, 70, 70), cv::Scalar(180, 255, 255), r2);
    red = r1 | r2;
    cv::morphologyEx(red, red, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    int idx = largestContourIndex(contours, 1000.0);
    if (idx < 0) return 0;

    cv::Rect box = cv::boundingRect(contours[idx]) & cv::Rect(0, 0, frame.cols, frame.rows);
    if (box.width < 40 || box.height < 40) return 0;

    cv::Rect inner(
        box.x + box.width / 6,
        box.y + box.height / 6,
        box.width * 2 / 3,
        box.height * 2 / 3);
    inner &= cv::Rect(0, 0, frame.cols, frame.rows);
    if (inner.width < 20 || inner.height < 20) return 0;

    cv::Mat gray, black;
    cv::cvtColor(frame(inner), gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, black, 95, 255, cv::THRESH_BINARY_INV);

    int band_h = std::max(4, black.rows / 5);
    int y0 = std::max(0, black.rows / 2 - band_h / 2);
    cv::Mat mid = black.rowRange(y0, std::min(black.rows, y0 + band_h));
    cv::Mat left = mid.colRange(0, mid.cols / 2);
    cv::Mat right = mid.colRange(mid.cols / 2, mid.cols);
    int left_black = cv::countNonZero(left);
    int right_black = cv::countNonZero(right);
    int diff = left_black - right_black;
    int total = left_black + right_black;
    if (total < 80 || std::abs(diff) < total * 0.08) {
        std::cout << "[AUTOID] place ambiguous left=" << left_black
                  << " right=" << right_black << std::endl;
        return 0;
    }

    int cmd = (diff > 0) ? 5 : 6; // 1号标识左侧黑横杠更重 -> 一号平台命令5
    std::cout << "[AUTOID] place left=" << left_black
              << " right=" << right_black
              << " -> arm_cmd=" << cmd << std::endl;

    if (debug) {
        frame.copyTo(*debug);
        cv::rectangle(*debug, box, cv::Scalar(0, 0, 255), 2);
        cv::rectangle(*debug, inner, cv::Scalar(255, 0, 255), 2);
        cv::putText(*debug, "AUTOID place cmd=" + std::to_string(cmd),
                    cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 0, 255), 2);
    }
    return cmd;
}

int recognizeWarningFromCamera(cv::VideoCapture& cap, double sec = 1.5) {
    cv::Mat frame, dbg;
    int best = 0, stable = 0;
    auto t0 = std::chrono::steady_clock::now();
    while (g_running) {
        cap >> frame;
        if (frame.empty()) break;
        int cur = classifyWarningAction(frame, g_gui_enabled ? &dbg : nullptr);
        if (cur > 0 && cur == best) stable++;
        else { best = cur; stable = (cur > 0) ? 1 : 0; }
        if (g_gui_enabled && !dbg.empty()) cv::imshow("AutoID", dbg);
        guiWaitKey(1);
        if (stable >= 3) return best;
        double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count() / 1000.0;
        if (el >= sec) break;
        usleep(30 * 1000);
    }
    return 0;
}

int recognizePlaceFromCamera(cv::VideoCapture& cap, double sec = 1.5) {
    cv::Mat frame, dbg;
    int best = 0, stable = 0;
    auto t0 = std::chrono::steady_clock::now();
    while (g_running) {
        cap >> frame;
        if (frame.empty()) break;
        int cur = classifyPlaceCommand(frame, g_gui_enabled ? &dbg : nullptr);
        if (cur > 0 && cur == best) stable++;
        else { best = cur; stable = (cur > 0) ? 1 : 0; }
        if (g_gui_enabled && !dbg.empty()) cv::imshow("AutoID", dbg);
        guiWaitKey(1);
        if (stable >= 3) return best;
        double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count() / 1000.0;
        if (el >= sec) break;
        usleep(30 * 1000);
    }
    return 0;
}

} // namespace autoid

// ============================================================
//  原地转向: 相对当前朝向转 delta_deg (左+ / 右-), 用 g_yaw_deg 反馈
// ============================================================
void turnInPlace(unitree::robot::go2::SportClient& sport,
                 float delta_deg, float tol_deg = RED_TURN_TOL,
                 bool stop_at_end = true) {
    // ★ 重新断言常规步态 (softStop + 停顿后 SDK 可能已切回 AI 步态),
    //   用 StaticWalk 常规步态原地转
    //   stop_at_end=false 时, 末尾不软停, 让狗保持步态状态,
    //   方便外面紧接着发别的 Move 命令 (避免 SDK 切回 BalanceStand 后 vy 被 deadband 吃掉)
    sport.StaticWalk();
    usleep(500 * 1000);
    float target = g_yaw_deg + delta_deg;
    auto  t0 = std::chrono::steady_clock::now();
    while (g_running) {
        float err = normalize_180(target - g_yaw_deg);
        if (std::abs(err) < tol_deg) break;
        float el = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - t0).count() / 1000.0f;
        if (el > RED_TURN_MAX_SEC) {
            std::cout << "  [转向超时] 误差还有 " << err << "°" << std::endl;
            break;
        }
        // 比例控速: 误差大时全速, 接近目标按比例减速 → 停位更准
        float omega = clampf(err * RED_TURN_KP, -RED_TURN_OMEGA, RED_TURN_OMEGA);
        if (omega > 0 && omega <  RED_TURN_OMEGA_MIN) omega =  RED_TURN_OMEGA_MIN;
        if (omega < 0 && omega > -RED_TURN_OMEGA_MIN) omega = -RED_TURN_OMEGA_MIN;
        sport.Move(0.0f, 0.0f, omega);
        usleep(20 * 1000);
    }
    if (stop_at_end) softStop(sport, SOFTSTOP_SETTLE_SEC);
}

// ============================================================
//  ★ 续跑入口虚拟 yaw 基准 (startat= 通用版; 原 1free 专用函数
//    泛化而来, 1free 等价于 startat=arc, offset=180 / cum 预载 180)
//    virtual_initial = 当前 yaw + offset_deg; 全局累计 yaw 预载 cum_preload_deg。
//    offset_deg = 该入口点狗的朝向相对"启停区出发朝向"的地图夹角 (左转为正)。
//    ★ 除 arc 外的默认偏移是按场地图估的, 必须现场核一次 (方法见使用手册),
//      临时可用 yawoff=<度> 覆盖。
// ============================================================
float SetVirtualYawBaseline(float offset_deg, float cum_preload_deg,
                            const char* tag) {
    float yaw_now = g_yaw_rad.load();
    float yaw_now_deg = g_yaw_deg.load();
    float virtual_initial = normalize_180_rad(yaw_now + deg2rad(offset_deg));
    float virtual_initial_deg = normalize_180(yaw_now_deg + offset_deg);

    g_initial_yaw_rad = virtual_initial;
    g_initial_yaw_deg = virtual_initial_deg;
    g_global_yaw_prev_rad = yaw_now;
    g_global_yaw_cum_rad = deg2rad(cum_preload_deg);
    g_initial_yaw_ready = true;

    std::cout << "[" << tag << " yaw] current=" << rad2deg(yaw_now)
              << " deg (deg_channel=" << yaw_now_deg
              << " deg), virtual_initial=current+" << offset_deg << "="
              << rad2deg(virtual_initial)
              << " deg (deg_channel=" << virtual_initial_deg
              << " deg), preload_global_cum=" << cum_preload_deg << " deg"
              << std::endl;
    return virtual_initial;
}

// ============================================================
//  绝对转向: 转到指定 yaw_deg,用于最后回到程序初始 yaw
// ============================================================
bool turnToYawDeg(unitree::robot::go2::SportClient& sport,
                  float target_yaw_deg,
                  float tol_deg = FINAL_TURN_TOL,
                  bool stop_at_end = true) {
    int static_ret = sport.StaticWalk();
    if (static_ret != 0) {
        std::cout << "  [绝对转向取消] StaticWalk() ret=" << static_ret
                  << "，步态切换失败" << std::endl;
        sport.Move(0.0f, 0.0f, 0.0f);
        return false;
    }
    usleep(500 * 1000);
    auto t0 = std::chrono::steady_clock::now();
    auto last_fresh_yaw_time = t0;
    uint64_t last_yaw_seq = g_state_seq.load(std::memory_order_acquire);
    bool have_fresh_yaw = false;
    bool reached = false;
    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        uint64_t yaw_seq = g_state_seq.load(std::memory_order_acquire);
        if (yaw_seq != last_yaw_seq) {
            last_yaw_seq = yaw_seq;
            last_fresh_yaw_time = now;
            have_fresh_yaw = true;
        } else if (std::chrono::duration_cast<std::chrono::milliseconds>(
                       now - last_fresh_yaw_time).count() >= 500) {
            std::cout << "  [绝对转向取消] SportModeState 0.5s 无新帧，禁止用陈旧 yaw"
                      << std::endl;
            break;
        }
        if (!have_fresh_yaw) {
            usleep(20 * 1000);
            continue;
        }
        float err = normalize_180(target_yaw_deg - g_yaw_deg.load());
        if (std::abs(err) < tol_deg) {
            reached = true;
            break;
        }
        float el = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - t0).count() / 1000.0f;
        if (el > RED_TURN_MAX_SEC) {
            std::cout << "  [绝对转向超时] 目标 yaw=" << target_yaw_deg
                      << "° 误差还有 " << err << "°" << std::endl;
            break;
        }
        float omega = clampf(err * RED_TURN_KP, -RED_TURN_OMEGA, RED_TURN_OMEGA);
        if (omega > 0 && omega <  RED_TURN_OMEGA_MIN) omega =  RED_TURN_OMEGA_MIN;
        if (omega < 0 && omega > -RED_TURN_OMEGA_MIN) omega = -RED_TURN_OMEGA_MIN;
        sport.Move(0.0f, 0.0f, omega);
        usleep(20 * 1000);
    }
    // 无论成功、超时还是中断，都先覆盖最后一条可能非零的转向命令。
    sport.Move(0.0f, 0.0f, 0.0f);
    if (g_running && stop_at_end) {
        softStop(sport, SOFTSTOP_SETTLE_SEC);
    }
    return reached && g_running;
}

bool getWholeFrameLineCenterX(const cv::Mat& frame,
                              int& center_x,
                              cv::Mat& mask,
                              cv::Mat& dbg_L,
                              int min_area = g_min_area) {
    if (!extractBlackLine(frame, mask, dbg_L, min_area)) return false;
    if (mask.empty() || cv::countNonZero(mask) < min_area) return false;
    cv::Moments m = cv::moments(mask, true);
    if (m.m00 <= 0.0) return false;
    center_x = int(m.m10 / m.m00);
    return true;
}

// 第一跳落地专用固定左移：调用前已经完成绝对 yaw 复位并处于 StaticWalk。
// 不再读取相机或判断黑线位置；固定 vy=+0.05 左移2s，随后交给原定时巡线。
bool shiftLeftAfterFirstJump(unitree::robot::go2::SportClient& sport,
                             float target_yaw_deg) {
    std::cout << "[阶段1-跳后固定左移] vy=" << POST_JUMP_FIXED_LSHIFT_VY
              << "，持续 " << POST_JUMP_FIXED_LSHIFT_SEC
              << "s，yaw锁定 " << target_yaw_deg << "°" << std::endl;

    auto t0 = std::chrono::steady_clock::now();
    auto last_fresh_state_time = t0;
    uint64_t last_state_seq = g_state_seq.load(std::memory_order_acquire);

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now - t0).count() / 1000.0;
        if (elapsed >= POST_JUMP_FIXED_LSHIFT_SEC) break;

        uint64_t state_seq_now = g_state_seq.load(std::memory_order_acquire);
        if (state_seq_now != last_state_seq) {
            last_state_seq = state_seq_now;
            last_fresh_state_time = now;
        } else {
            double stale_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   now - last_fresh_state_time).count() / 1000.0;
            if (stale_sec >= POST_JUMP_LSHIFT_STATE_STALE_SEC) {
                std::cout << "[阶段1-跳后固定左移] SportModeState 已 "
                          << stale_sec << "s 无新帧，停止横移" << std::endl;
                sport.Move(0.0f, 0.0f, 0.0f);
                return false;
            }
        }

        float yaw_err = normalize_180(target_yaw_deg - g_yaw_deg.load());
        float omega = clampf(POST_JUMP_LSHIFT_YAW_KP * yaw_err,
                             -POST_JUMP_LSHIFT_OMEGA_MAX,
                              POST_JUMP_LSHIFT_OMEGA_MAX);
        sport.Move(0.0f, POST_JUMP_FIXED_LSHIFT_VY, omega);

        if (guiWaitKey(1) == 27) {
            g_running = false;
            break;
        }
        usleep(20 * 1000);
    }

    sport.Move(0.0f, 0.0f, 0.0f);
    if (!g_running) return false;
    usleep(200 * 1000);
    std::cout << "[阶段1-跳后固定左移] 完成，进入定时巡线" << std::endl;
    return true;
}


bool canSeeBlackLineNow(cv::VideoCapture& cap, const char* tag) {
    std::cout << "[" << tag << "] 重新打开巡线相机, 获取台阶后的新画面..." << std::endl;
    cap.release();
    usleep((int)(POST_STAIR_LINE_REOPEN_WAIT_SEC * 1000000));
    if (!openAndConfigLineCamera(cap)) {
        std::cout << "[" << tag << "] 巡线相机重开失败, 按未看到黑线处理" << std::endl;
        return false;
    }

    auto flush_start = std::chrono::steady_clock::now();
    int flush_frames = 0;
    while (g_running) {
        double flush_el = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - flush_start).count() / 1000.0;
        if (flush_el >= POST_STAIR_LINE_FLUSH_SEC) break;
        cv::Mat tmp;
        if (cap.read(tmp)) flush_frames++;
        guiWaitKey(1);
        usleep(5 * 1000);
    }
    std::cout << "[" << tag << "] 已按时间丢弃旧帧 " << POST_STAIR_LINE_FLUSH_SEC
              << "s, frames=" << flush_frames << std::endl;

    for (int i = 0; i < POST_STAIR_LINE_CHECK_FRAMES && g_running; ++i) {
        cv::Mat frame, mask, dbg_L;
        cap >> frame;
        if (frame.empty()) {
            usleep(20 * 1000);
            continue;
        }
        int center_x = 0;
        bool seen = getWholeFrameLineCenterX(frame, center_x, mask, dbg_L);
        std::cout << "[" << tag << "] 黑线检测帧 " << (i + 1)
                  << "/" << POST_STAIR_LINE_CHECK_FRAMES << ": "
                  << (seen ? "seen" : "not seen");
        if (seen) std::cout << " center_x=" << center_x;
        std::cout << std::endl;
        if (seen) return true;
        usleep(20 * 1000);
    }
    return false;
}

static bool noStairNearLineValid(const cv::Mat& frame, const char* tag, int& near_dir) {
    cv::Mat mask, dbg_L;
    cv::Mat vis = frame.clone();
    int W = vis.cols;
    int H = vis.rows;
    cv::Rect roi_near(0, int(H * ROI_NEAR_TOP), W,
                      int(H * (ROI_NEAR_BOTTOM - ROI_NEAR_TOP)));
    cv::Rect roi_far(0, int(H * ROI_FAR_TOP), W,
                     int(H * (ROI_FAR_BOTTOM - ROI_FAR_TOP)));

    bool have_real_line = extractBlackLine(frame, mask, dbg_L);
    int far_dir = INT_MIN;
    near_dir = INT_MIN;
    bool far_cross = false;
    bool near_cross = false;
    if (have_real_line) {
        far_dir = computeRoiDirection(mask, roi_far, vis,
                                      cv::Scalar(0, 200, 200),
                                      g_last_cx_far, g_fork_bias_far,
                                      &far_cross);
        near_dir = computeRoiDirection(mask, roi_near, vis,
                                       cv::Scalar(0, 255, 0),
                                       g_last_cx_near, g_fork_bias_near,
                                       &near_cross);
    }

    bool valid = have_real_line && (near_dir != INT_MIN);
    if (g_gui_enabled) {
        cv::line(vis, cv::Point(W / 2, 0), cv::Point(W / 2, H),
                 cv::Scalar(255, 255, 0), 1);
        char hud[256];
        std::snprintf(hud, sizeof(hud),
                      "NO-STAIR %s near=%d far=%d valid=%d",
                      tag, near_dir, far_dir, valid ? 1 : 0);
        cv::putText(vis, hud, cv::Point(18, 32),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65,
                    valid ? cv::Scalar(0, 180, 0) : cv::Scalar(0, 0, 255), 2);
        cv::imshow("Original", vis);
        if (!mask.empty()) {
            cv::Mat roi_mask = cv::Mat::zeros(mask.size(), mask.type());
            mask(roi_far).copyTo(roi_mask(roi_far));
            mask(roi_near).copyTo(roi_mask(roi_near));
            cv::Mat mask_view;
            cv::cvtColor(roi_mask, mask_view, cv::COLOR_GRAY2BGR);
            cv::rectangle(mask_view, roi_far, cv::Scalar(0, 200, 200), 2);
            cv::rectangle(mask_view, roi_near, cv::Scalar(0, 255, 0), 2);
            cv::imshow("Mask", mask_view);
        }
        if (!dbg_L.empty()) cv::imshow("L channel", dbg_L);
    }
    return valid;
}

static bool flushLineCameraFrames(cv::VideoCapture& cap, const char* tag) {
    std::cout << "[" << tag << "] 重开巡线相机并丢弃旧缓存帧..." << std::endl;
    cap.release();
    usleep((int)(POST_STAIR_LINE_REOPEN_WAIT_SEC * 1000000));
    if (!openAndConfigLineCamera(cap)) {
        std::cout << "[" << tag << "] 巡线相机重开失败" << std::endl;
        return false;
    }

    auto flush_start = std::chrono::steady_clock::now();
    int flush_frames = 0;
    while (g_running) {
        double flush_el = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - flush_start).count() / 1000.0;
        if (flush_el >= POST_STAIR_LINE_FLUSH_SEC) break;
        cv::Mat tmp;
        if (cap.read(tmp)) flush_frames++;
        guiWaitKey(1);
        usleep(5 * 1000);
    }
    std::cout << "[" << tag << "] 已按时间丢弃旧帧 "
              << POST_STAIR_LINE_FLUSH_SEC << "s, frames="
              << flush_frames << std::endl;
    return true;
}

static bool shiftUntilNearLine(unitree::robot::go2::SportClient& sport,
                               cv::VideoCapture& cap,
                               float vy,
                               double detect_after_sec,
                               const char* tag) {
    ResetTrajectory();
    int lock_cnt = 0;
    int empty_frames = 0;
    float yaw_hold = g_yaw_deg.load();
    auto t0 = std::chrono::steady_clock::now();
    auto t_log = t0;
    std::cout << "[" << tag << "] vy=" << vy
              << ", 先移动 " << detect_after_sec
              << "s 后开始用 near ROI 找线, yaw_hold="
              << yaw_hold << "°" << std::endl;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now - t0).count() / 1000.0;

        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            empty_frames++;
            if (empty_frames >= CAM_EMPTY_FRAMES_REOPEN) {
                std::cout << "[" << tag << "] 连续空帧, 重开巡线相机" << std::endl;
                if (!openAndConfigLineCamera(cap)) return false;
                empty_frames = 0;
            }
            usleep(20 * 1000);
            continue;
        }
        empty_frames = 0;

        if (elapsed >= detect_after_sec) {
            int near_dir = INT_MIN;
            bool valid = noStairNearLineValid(frame, tag, near_dir);
            if (valid) lock_cnt++;
            else       lock_cnt = 0;
            if (lock_cnt >= ARC_START_LINE_LOCK_FRAMES) {
                std::cout << "[" << tag << "] 已连续 "
                          << ARC_START_LINE_LOCK_FRAMES
                          << " 帧锁上 near ROI 黑线, near_dir="
                          << near_dir << ", elapsed=" << elapsed
                          << "s" << std::endl;
                softStop(sport, SOFTSTOP_SETTLE_SEC);
                return true;
            }
        }

        float yerr = normalize_180(yaw_hold - g_yaw_deg.load());
        float omega = clampf(0.02f * yerr, -0.30f, 0.30f);
        sport.Move(0.0f, vy, omega);

        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - t_log).count() >= 1000) {
            t_log = now;
            std::cout << "[" << tag << "] t=" << elapsed
                      << "s lock=" << lock_cnt << "/"
                      << ARC_START_LINE_LOCK_FRAMES
                      << " yaw_err=" << yerr << "°" << std::endl;
        }

        int key = guiWaitKey(1);
        if (key == 27) {
            sport.Move(0.0f, 0.0f, 0.0f);
            g_running = false;
            return false;
        }
        usleep(20 * 1000);
    }
    return false;
}

static bool noStairCheckNearLineNow(cv::VideoCapture& cap, const char* tag) {
    int lock_cnt = 0;
    int empty_frames = 0;
    for (int i = 0; g_running && i < POST_STAIR_LINE_CHECK_FRAMES; ++i) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            empty_frames++;
            if (empty_frames >= CAM_EMPTY_FRAMES_REOPEN) {
                std::cout << "[" << tag << "] 连续空帧, 重开巡线相机" << std::endl;
                if (!openAndConfigLineCamera(cap)) return false;
                empty_frames = 0;
            }
            usleep(20 * 1000);
            continue;
        }
        empty_frames = 0;

        int near_dir = INT_MIN;
        bool valid = noStairNearLineValid(frame, tag, near_dir);
        if (valid) lock_cnt++;
        else       lock_cnt = 0;

        std::cout << "[" << tag << "] 快速锁线检查 "
                  << (i + 1) << "/" << POST_STAIR_LINE_CHECK_FRAMES
                  << " lock=" << lock_cnt << "/"
                  << ARC_START_LINE_LOCK_FRAMES
                  << " near_dir=" << near_dir << std::endl;
        if (lock_cnt >= ARC_START_LINE_LOCK_FRAMES) return true;

        guiWaitKey(1);
        usleep(20 * 1000);
    }
    return false;
}

static void noStairMoveYawHold(unitree::robot::go2::SportClient& sport,
                               float vx, float vy, double sec,
                               const char* tag) {
    float yaw_hold = g_yaw_deg.load();
    auto t0 = std::chrono::steady_clock::now();
    std::cout << "[" << tag << "] vx=" << vx
              << ", vy=" << vy
              << ", sec=" << sec
              << ", yaw_hold=" << yaw_hold << "°" << std::endl;
    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now - t0).count() / 1000.0;
        if (elapsed >= sec) break;

        float yerr = normalize_180(yaw_hold - g_yaw_deg.load());
        float omega = clampf(0.02f * yerr, -0.30f, 0.30f);
        sport.Move(vx, vy, omega);
        usleep(20 * 1000);
    }
}

static bool runNoStairsSafetyBypass(unitree::robot::go2::SportClient& sport,
                                    cv::VideoCapture& cap) {
    std::cout << "\n############ no 模式: 跳过台阶保底找线 ############" << std::endl;
    std::cout << "[no] 流程: 后退 → 左移" << NO_STAIR_LEFT_FAST_VY
              << "/" << NO_STAIR_LEFT_FAST_SEC
              << "s → 必要时左移0.25找线 → 前进 → 原地左转 "
              << NO_STAIR_TURN_LEFT_DEG
              << "° → 前进 → 进入弧形入口右移找线" << std::endl;

    sport.StaticWalk();
    usleep(300 * 1000);
    if (!flushLineCameraFrames(cap, "no左移找线")) return false;

    noStairMoveYawHold(sport, NO_STAIR_PRE_BACK_VX, 0.0f,
                       NO_STAIR_PRE_BACK_SEC, "no左移前后退");

    noStairMoveYawHold(sport, 0.0f, NO_STAIR_LEFT_FAST_VY,
                       NO_STAIR_LEFT_FAST_SEC, "no固定快速左移");
    softStop(sport, SOFTSTOP_SETTLE_SEC);

    bool line_locked = noStairCheckNearLineNow(cap, "no快速左移后锁线");
    if (!line_locked) {
        std::cout << "[no] 固定左移后未锁上线, 改用 vy="
                  << NO_STAIR_LEFT_SEARCH_VY << " 继续左移找线" << std::endl;
        if (!shiftUntilNearLine(sport, cap, NO_STAIR_LEFT_SEARCH_VY,
                                0.0, "no左移找线")) {
            return false;
        }
    } else {
        std::cout << "[no] 固定左移后已锁上线, 跳过低速左移找线" << std::endl;
    }

    noStairMoveYawHold(sport, NO_STAIR_AFTER_LINE_FWD_VX, 0.0f,
                       NO_STAIR_AFTER_LINE_FWD_SEC, "no锁线后前进");
    softStop(sport, SOFTSTOP_SETTLE_SEC);

    float target_yaw = normalize_180(g_yaw_deg.load() + NO_STAIR_TURN_LEFT_DEG);
    std::cout << "[no] 原地左转 " << NO_STAIR_TURN_LEFT_DEG
              << "°: 当前 yaw=" << g_yaw_deg.load()
              << "° 目标 yaw=" << target_yaw << "°" << std::endl;
    turnToYawDeg(sport, target_yaw, FINAL_TURN_TOL, true);
    if (!g_running) return false;

    noStairMoveYawHold(sport, NO_STAIR_AFTER_TURN_FWD_VX, 0.0f,
                       NO_STAIR_AFTER_TURN_FWD_SEC, "no转后前进");
    softStop(sport, SOFTSTOP_SETTLE_SEC);

    std::cout << "[no] 保底预处理完成, 右移找线交给弧形入口 POST_STAIR/ARC_START 逻辑"
              << std::endl;
    return true;
}

// ============================================================
//  ★ 蓝色启停区边界检测 (收尾三段对齐用):
//    HSV 阈出蓝色垫子 → 闭运算填掉垫子上的白色大字"启停区", 然后两条路:
//    ① 行法 (存在性 + 兜底): 逐行蓝色占比, 自上而下首个稳定达标行;
//    ② 拟合 (倾角): 逐列找蓝区上边界点 → 最小二乘直线 (带一次离群剔除),
//       输出画面中心列处的边界行 + 倾角 tilt_deg。
//       tilt_deg > 0 = 边界右端更远(画面里更高) = 狗被顺时针带偏 → 需左转(+omega)。
//    返回 -1 = 画面里没有可信蓝区; tilt_valid 仅当拟合点数/横向跨度足够才置真。
// ============================================================
static int detectBlueZoneBoundary(const cv::Mat& frame_bgr,
                                  float* tilt_deg_out = nullptr,
                                  bool*  tilt_valid_out = nullptr) {
    if (tilt_deg_out)   *tilt_deg_out = 0.0f;
    if (tilt_valid_out) *tilt_valid_out = false;
    if (frame_bgr.empty()) return -1;
    cv::Mat hsv, blue;
    cv::cvtColor(frame_bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
                cv::Scalar(FINAL_FB_H_MIN, FINAL_FB_S_MIN, FINAL_FB_V_MIN),
                cv::Scalar(FINAL_FB_H_MAX, 255, 255), blue);
    cv::morphologyEx(blue, blue, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25)));
    int W = blue.cols, H = blue.rows;

    // ① 行法: 蓝区存在性判定 + 兜底边界行
    int need = (int)(W * FINAL_FB_ROW_BLUE_FRAC);
    std::vector<char> is_blue_row(H, 0);
    int blue_rows = 0;
    for (int y = 0; y < H; ++y) {
        if (cv::countNonZero(blue.row(y)) >= need) {
            is_blue_row[y] = 1;
            blue_rows++;
        }
    }
    if (blue_rows < FINAL_FB_MIN_BLUE_ROWS) return -1;
    int row_scan = -1;
    for (int y = 0; y < H && row_scan < 0; ++y) {
        if (!is_blue_row[y]) continue;
        int okr = 0;
        for (int k = 0; k < 8 && y + k < H; ++k) okr += is_blue_row[y + k];
        if (okr >= 5) row_scan = y;
    }
    if (row_scan < 0) return -1;

    // ② 逐列上边界点 (自上而下首个连续 5 像素蓝) → 直线拟合求倾角
    std::vector<cv::Point2f> pts;
    for (int x = 2; x < W - 2; x += 4) {
        int run = 0;
        for (int y = 0; y < H; ++y) {
            if (blue.at<uchar>(y, x)) {
                if (++run >= 5) { pts.push_back(cv::Point2f((float)x, (float)(y - 4))); break; }
            } else run = 0;
        }
    }
    float xmin = 1e9f, xmax = -1e9f;
    for (const auto& p : pts) { xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x); }
    if ((int)pts.size() >= 25 && (xmax - xmin) >= 0.35f * (float)W) {
        auto fitLS = [](const std::vector<cv::Point2f>& v, float& a, float& b) {
            double sx = 0, sy = 0, sxx = 0, sxy = 0;
            int n = (int)v.size();
            for (const auto& p : v) { sx += p.x; sy += p.y; sxx += p.x * p.x; sxy += p.x * p.y; }
            double d = n * sxx - sx * sx;
            if (std::abs(d) < 1e-6) return false;
            b = (float)((n * sxy - sx * sy) / d);
            a = (float)((sy - b * sx) / n);
            return true;
        };
        float a = 0.0f, b = 0.0f;
        if (fitLS(pts, a, b)) {
            std::vector<cv::Point2f> keep;   // 一次离群剔除 (残差>18px 丢弃) 后重拟合
            for (const auto& p : pts)
                if (std::abs(a + b * p.x - p.y) <= 18.0f) keep.push_back(p);
            if ((int)keep.size() >= 15) fitLS(keep, a, b);
            float tilt = rad2deg(std::atan(-b));   // 图像 y 向下, 取负: >0 = 右端更高/更远
            if (std::abs(tilt) <= FINAL_YAW_VIS_TILT_MAX) {
                if (tilt_deg_out)   *tilt_deg_out = tilt;
                if (tilt_valid_out) *tilt_valid_out = true;
                int rc = (int)(a + b * (W * 0.5f));   // 中心列处的边界行 (与行法同义但更稳)
                return std::max(0, std::min(H - 1, rc));
            }
        }
    }
    return row_scan;   // 拟合不可信: 退回行法, 倾角标记不可用
}

// ============================================================
//  ★ 收尾/起点语境的黑线中心检测 (排蓝版):
//    0706 日志实锤: 蓝色垫子(低曝光下 L 值也很暗)整块进了黑线 mask, 产生
//    假线 (cx=333) → 预右移 0s 就"提前结束"、①相位 0.24s 就在假线上"居中完成"。
//    这里把蓝色像素从黑线 mask 里抠掉 (膨胀一圈防边缘残留), 只认真正的黑线。
//    只用于起点采集/收尾对齐, 不动巡线主管线。
// ============================================================
// ★ 六轮追加"竖长验证": 导引线是纵向长条 (在画面里上下贯穿), 地面黑十字/杂块/
//   阴影不是。排蓝后再按连通域筛: 只认"高度 ≥ 画面高 45%"的最大黑块为线 ——
//   这才是"蓝区/黑色杂物冒充黑线"的根治 (0706 两晚的假线全都过不了这一关)。
const double FINAL_LINE_MIN_H_FRAC = 0.45;
// ★ 八轮迟滞跟踪 (严进宽出): 0706 五连日志实锤 —— 收尾几何下真线高度恰好骑在
//   45% 阈值上, 无记忆硬二值判据导致"闪断" (2234: 5.2s 内检测状态翻 5 次,
//   狗被默认右搜推过启停区右缘; 2226/2242: ①相位整 6s 拉锯超时 err=+94/−52)。
//   规则: 首次认线仍用严判据 (tall_ok||far_ok, 防伪不降级); 认上之后改宽判据
//   就地跟踪 (h≥25%H 且 cx 距上次 ≤60px 即认为还是那条线); 连续丢
//   LINE_TRACK_MISS_LIMIT 帧才降回"未锁", 需重新过严判据。
//   跟踪态跨调用间隔 >2s 自动清零 (起点采集与收尾对齐两个场景互不串扰)。
static int  s_line_track_cx   = -1;   // 跟踪中的线 cx (-1 = 未锁)
static int  s_line_track_miss = 0;    // 连续未匹配帧数
static std::chrono::steady_clock::time_point s_line_track_last_call;
static const int    LINE_TRACK_MISS_LIMIT = 10;
static const double LINE_TRACK_H_FRAC     = 0.25;  // 跟踪态最低高度
static const int    LINE_TRACK_DX_PX      = 60;    // 跟踪态 cx 连续性窗口
static bool getFinalLineCenterXNoBlue(const cv::Mat& frame, int& center_x,
                                      cv::Mat& mask, cv::Mat& dbg_L,
                                      int min_area) {
    auto now_t = std::chrono::steady_clock::now();
    if (s_line_track_cx >= 0
        && std::chrono::duration_cast<std::chrono::milliseconds>(
               now_t - s_line_track_last_call).count() > 2000) {
        s_line_track_cx = -1;   // 上下文切换 (两次调用隔太久), 清跟踪
        s_line_track_miss = 0;
    }
    s_line_track_last_call = now_t;
    auto reportMiss = []() {
        if (s_line_track_cx >= 0
            && ++s_line_track_miss > LINE_TRACK_MISS_LIMIT) {
            s_line_track_cx = -1;
            s_line_track_miss = 0;
        }
        return false;
    };
    if (!extractBlackLine(frame, mask, dbg_L, min_area)) return reportMiss();
    cv::Mat hsv, blue;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
                cv::Scalar(FINAL_FB_H_MIN, FINAL_FB_S_MIN, FINAL_FB_V_MIN),
                cv::Scalar(FINAL_FB_H_MAX, 255, 255), blue);
    cv::dilate(blue, blue,
               cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9)));
    mask.setTo(0, blue);
    if (cv::countNonZero(mask) < min_area) return reportMiss();
    cv::Mat labels, stats, cents;
    int n = cv::connectedComponentsWithStats(mask, labels, stats, cents, 8);
    int best = -1;  int best_area = 0;    // 严判据最佳 (tall/far)
    int track = -1; int track_area = 0;   // 宽判据跟踪最佳
    int min_h = (int)(mask.rows * FINAL_LINE_MIN_H_FRAC);
    for (int i = 1; i < n; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        int h    = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        int top  = stats.at<int>(i, cv::CC_STAT_TOP);
        int w    = stats.at<int>(i, cv::CC_STAT_WIDTH);
        if (area < min_area) continue;
        // ★ 七轮放宽 (2226: 狗停位靠后, 线只出现在画面上半段, 45% 高度不满足 →
        //   ①相位永远锁不上): 两条路认线 ——
        //   a) 高度 ≥45%H (原判据, 正常停位);
        //   b) 从画面顶部延伸下来 (top≤15%H) 且高度 ≥25%H 且宽 ≤45%W
        //      —— 真导引线在远处必然从画面上缘进入; 黑十字是孤立块, 顶不到上缘。
        bool tall_ok = (h >= min_h);
        bool far_ok  = (top <= (int)(mask.rows * 0.15))
                       && (h >= (int)(mask.rows * 0.25))
                       && (w <= (int)(mask.cols * 0.45));
        if ((tall_ok || far_ok) && area > best_area) {
            best_area = area; best = i;
        }
        // ★ 八轮: 宽判据跟踪 (严判据没过也接受, 只要还是"上次那条线")
        if (s_line_track_cx >= 0
            && h >= (int)(mask.rows * LINE_TRACK_H_FRAC)
            && std::abs((int)cents.at<double>(i, 0) - s_line_track_cx)
                   <= LINE_TRACK_DX_PX
            && area > track_area) {
            track_area = area; track = i;
        }
    }
    int chosen = (best >= 0) ? best : track;
    if (chosen < 0) return reportMiss();
    center_x = (int)cents.at<double>(chosen, 0);
    s_line_track_cx = center_x;
    s_line_track_miss = 0;
    return true;
}

// ★ 六轮: 蓝区质心 x (收尾找线时判断"启停区在画面哪一侧" → 决定平移方向)。
//   返回 -1 = 蓝区不在视野 (面积不足)。
static int blueZoneCentroidX(const cv::Mat& frame_bgr) {
    if (frame_bgr.empty()) return -1;
    cv::Mat hsv, blue;
    cv::cvtColor(frame_bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
                cv::Scalar(FINAL_FB_H_MIN, FINAL_FB_S_MIN, FINAL_FB_V_MIN),
                cv::Scalar(FINAL_FB_H_MAX, 255, 255), blue);
    cv::Moments m = cv::moments(blue, true);
    if (m.m00 < 0.03 * blue.cols * blue.rows) return -1;   // 蓝色太少 = 不在视野
    return (int)(m.m10 / m.m00);
}

// (★ 九轮改四已删: blueBottomFrac "底部蓝占比" —— 随"巡线找启停区"整套一起移除。)

// (★ 九轮二已撤销: 曾在此加"线倾角视觉对正"治平台一停位偏右 —— 错误方案,
//  平台一停位处的导引线是弧线, 底部条带拟合的是弧线切线角, 与起点直线参照
//  不可比。用户指出后回滚。)

// ============================================================
//  最终停车前相机居中: 黑线在左→左移,黑线在右→右移。
//  同时 yaw 始终闭环锁到程序初始 yaw。
//  ★ 2026-07-06 新增前后闭环: 蓝色启停区边界行伺服回起点参照行 (vx),
//    与左右居中 (vy)、yaw 锁定 (omega) 同时收敛, 三者都到位才确认停车。
// ============================================================
bool alignFinalLineCenter(unitree::robot::go2::SportClient& sport,
                          cv::VideoCapture& cap,
                          float target_yaw_deg) {
    std::cout << "[收尾] 相机居中微调: 目标 yaw=" << target_yaw_deg
              << "°, center_tol=" << FINAL_CENTER_TOL_PX << "px" << std::endl;

    // ★ 先断言常规步态, 防止后面的小幅横移被 BalanceStand 死区吃掉 (居中卡死的根因之一)
    sport.StaticWalk();
    usleep(200 * 1000);

    std::cout << "[收尾] 丢弃摄像头缓存帧 " << FINAL_CENTER_FLUSH_FRAMES
              << " 帧,避免使用旧画面" << std::endl;
    {
        cv::Mat discard;
        for (int i = 0; i < FINAL_CENTER_FLUSH_FRAMES && g_running; ++i) {
            if (!cap.grab()) {
                cap >> discard;
            } else {
                cap.retrieve(discard);
            }
            guiWaitKey(1);
            usleep(30 * 1000);
        }
    }

    cv::Mat frame, mask, dbg_L;
    int last_err_px = 0;
    int last_line_cx = -1;  // ★ 九轮: 上一有效帧线 cx (跳变误检保护)
    int jump_reject_cnt = 0;// ★ 九轮改三: 连续跳变丢弃计数 (超阈值则重锁新线)
    bool centered = false;
    int centered_stable = 0;
    int empty_frames = 0;   // ★ 断流保护
    bool seen_line_once = false;
    int lost_after_seen = 0;
    auto t_center_start = std::chrono::steady_clock::now();  // 兜底超时计时
    auto t_hb           = t_center_start;   // ★ 八轮: 1Hz 状态心跳打点计时
    // ★ 三段对齐状态: ①左右居中 → ②对平蓝区边界(朝向) → ③前后对位
    enum AlignPhase { PH_LATERAL = 0, PH_YAW = 1, PH_FB = 2 };
    int   phase        = PH_LATERAL;
    int   phase_stable = 0;                       // 当前相位判据连续满足帧数
    auto  t_phase      = std::chrono::steady_clock::now();
    float yaw_hold_deg = target_yaw_deg;          // 里程计保持目标; ②视觉对平后更新
    bool  fb_active     = g_fb_align_enabled;
    int   fb_target_row = -1;      // 首帧按画面高度确定 (起点参照优先)
    bool  fb_seen_blue  = false;   // 居中开始后是否见过蓝区
    bool  fb_no_blue_step_done = false; // 看不到蓝区时只后探一次, 避免无限向后
    bool  fb_no_blue_backsearch_done = false; // 后探后仍无蓝区时只后移找一次
    // ★ ③失速踢状态 (2026-07-06 用户方案: 命令了运动但里程计不动 → 0.2 定向踢)
    bool  fb_kicking    = false;
    auto  t_fb_kick     = std::chrono::steady_clock::now();
    auto  t_fb_stallref = std::chrono::steady_clock::now();
    float fb_ref_px     = g_pos_x.load();
    float fb_ref_py     = g_pos_y.load();
    auto vyFromErr = [](int err_px) {
        float vy_cmd = -FINAL_CENTER_VY_KP * (float)err_px;
        if (std::abs(vy_cmd) < FINAL_CENTER_VY_FLOOR) {
            vy_cmd = (vy_cmd >= 0.0f) ? FINAL_CENTER_VY_FLOOR : -FINAL_CENTER_VY_FLOOR;
        }
        return clampf(vy_cmd, -FINAL_CENTER_VY_MAX, FINAL_CENTER_VY_MAX);
    };
    auto stepBackForBlue = [&]() {
        std::cout << "[收尾] 未看到蓝区, 先向后探一步 vx="
                  << FINAL_FB_SEARCH_BACK_VX << " " << FINAL_FB_SEARCH_BACK_SEC
                  << "s, 然后继续识别蓝区" << std::endl;
        auto t_step = std::chrono::steady_clock::now();
        while (g_running) {
            double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - t_step).count() / 1000.0;
            if (el >= FINAL_FB_SEARCH_BACK_SEC) break;
            float step_yaw_err = normalize_180(yaw_hold_deg - g_yaw_deg.load());
            float step_omega = clampf(FINAL_YAW_HOLD_KP * step_yaw_err,
                                      -FINAL_YAW_HOLD_OMEGA_MAX,
                                       FINAL_YAW_HOLD_OMEGA_MAX);
            sport.Move(FINAL_FB_SEARCH_BACK_VX, 0.0f, step_omega);
            usleep(20 * 1000);
        }
        sport.Move(0.0f, 0.0f, 0.0f);
        usleep(100 * 1000);
        fb_no_blue_step_done = true;
        phase_stable = 0;
        t_phase = std::chrono::steady_clock::now();
    };

    while (g_running) {
        cap >> frame;
        if (frame.empty()) {
            // ★ 断流保护: 空帧时先停住 (否则上一条横移 Move 会一直生效),
            //   连续空帧过多则尝试重连; 外层 10s 兜底超时仍然有效。
            if (++empty_frames >= CAM_EMPTY_FRAMES_REOPEN) {
                std::cout << "[相机] 居中阶段连续 " << empty_frames
                          << " 空帧 → softStop + 尝试重连..." << std::endl;
                softStop(sport, SOFTSTOP_SETTLE_SEC);
                cap.release();
                if (!openAndConfigLineCamera(cap)) usleep(500 * 1000);
                empty_frames = 0;
            }
            usleep(20 * 1000);
            continue;
        }
        empty_frames = 0;

        int line_cx = -1;
        // ★ 排蓝检测: 蓝垫子不再冒充黑线 (0706 日志: 假线让①在 0.24s 假居中)
        bool have_line = getFinalLineCenterXNoBlue(
            frame, line_cx, mask, dbg_L, FINAL_CENTER_MIN_AREA);
        // ★ 九轮 (1816: 右移撞画面边缘黑块, line_cx 单帧 30→321 → 死追假线右移失控):
        //   相邻有效帧 cx 跳变 >150px 物理不可能 (20ms 走不了半屏), 判误检丢弃本帧。
        // ★ 九轮改三 (0707: 画面两条竖线 378/205 交替, 旧版永久锁死先出现的 378、
        //   真线 205 被丢几十帧→盯假线右移出区): 跳变连续超 MAX 帧 = 原锁线确已消失,
        //   接受新线重锁, 不再永久丢弃真线。
        if (have_line && last_line_cx >= 0
            && std::abs(line_cx - last_line_cx) > 150) {
            if (++jump_reject_cnt <= FINAL_CX_JUMP_MAX_REJECT) {
                std::cout << "[对齐] 线 cx 跳变 " << last_line_cx << "→" << line_cx
                          << " (>150px, " << jump_reject_cnt << "/"
                          << FINAL_CX_JUMP_MAX_REJECT << "), 判误检丢弃本帧" << std::endl;
                have_line = false;
            } else {
                std::cout << "[对齐] 线 cx 持续跳变 " << jump_reject_cnt
                          << " 帧 → 原锁线已消失, 重锁新线 cx=" << line_cx << std::endl;
                jump_reject_cnt = 0;
                last_line_cx = line_cx;   // 接受新线 (下面 have_line 分支再刷新)
            }
        } else {
            jump_reject_cnt = 0;
        }
        // ★ 左右目标 x: 优先起点参照 (人工摆正那一刻线的 x), 未采到退回画面中心
        int lat_target_x = (g_final_lat_target_x >= 0) ? g_final_lat_target_x
                                                       : (frame.cols / 2);
        int err_px = have_line ? (line_cx - lat_target_x) : last_err_px;
        if (have_line) {
            last_err_px = err_px;
            last_line_cx = line_cx;
            seen_line_once = true;
            lost_after_seen = 0;
        } else if (seen_line_once) {
            lost_after_seen++;
        }

        float vy = 0.0f;
        const char* center_mode = "STOP";
        if (have_line) {
            if (std::abs(err_px) > FINAL_CENTER_TOL_PX) {
                // 比例: 线在右(err>0)→向右移(vy<0); 线在左(err<0)→向左移(vy>0)
                vy = vyFromErr(err_px);
                center_mode = (vy > 0.0f) ? "LINE_LEFT_CMD_LEFT" : "LINE_RIGHT_CMD_RIGHT";
            } else {
                center_mode = "CENTER_WAIT_YAW";
            }
        } else if (seen_line_once
                   && lost_after_seen <= FINAL_CENTER_LOST_HOLD_FRAMES
                   && std::abs(last_err_px) > FINAL_CENTER_TOL_PX) {
            vy = vyFromErr(last_err_px);
            center_mode = (vy > 0.0f) ? "LOST_HOLD_LEFT" : "LOST_HOLD_RIGHT";
        } else {
            // ★ 八轮地理围栏: 无界默认右搜是"明明看得见线却一直向右蹭"的元凶之二
            //   (0706 五连: 检测一断 >15 帧就右拖, ①②③全相位生效且无任何约束)。
            //   蓝区质心在画面左侧 = 启停区已在身左, 右边不可能有线 → 左移回找;
            //   蓝区不可见 = 位置未知 → 不瞎横移, 停横移交给 vx 借位调整视野;
            //   蓝区在中/右侧才允许默认右移。
            int bzx = blueZoneCentroidX(frame);
            if (bzx >= 0 && bzx < (int)(frame.cols * 0.40)) {
                vy = +std::fabs(FINAL_SHIFT_VY);
                center_mode = "SEARCH_LEFT_GUARD";
            } else if (bzx < 0) {
                vy = 0.0f;
                center_mode = "SEARCH_HOLD_NOBLUE";
            } else {
                vy = FINAL_SHIFT_VY;
                center_mode = "SEARCH_RIGHT";
            }
        }

        // ---------- ★ 蓝区边界测量 (行号 + 倾角) ----------
        int   fb_boundary   = -2;      // -2=未启用, -1=本帧没看到蓝区
        int   fb_err_rows   = 0;
        float fb_tilt_deg   = 0.0f;    // 边界倾角 (>0 = 右端更远 → 需左转)
        bool  fb_tilt_valid = false;
        if (fb_active) {
            if (fb_target_row < 0) {
                fb_target_row = (g_final_fb_target_row >= 0)
                    ? g_final_fb_target_row
                    : (int)(frame.rows * FINAL_FB_TARGET_ROW_FRAC_DEFAULT);
                std::cout << "[收尾] 前后目标: 蓝区边界行 " << fb_target_row
                          << (g_final_fb_target_row >= 0 ? " (起点参照)" : " (默认兜底)")
                          << " 容差 偏后+" << FINAL_FB_TOL_BACK
                          << "/偏前-" << FINAL_FB_TOL_FRONT << "px" << std::endl;
            }
            fb_boundary = detectBlueZoneBoundary(frame, &fb_tilt_deg, &fb_tilt_valid);
            if (fb_boundary >= 0) fb_seen_blue = true;
            // HUD: 蓝区边界行 (紫) + 参照行 (绿)
            if (fb_boundary >= 0) {
                cv::line(frame, cv::Point(0, fb_boundary),
                         cv::Point(frame.cols, fb_boundary),
                         cv::Scalar(255, 0, 255), 2);
            }
            cv::line(frame, cv::Point(0, fb_target_row),
                     cv::Point(frame.cols, fb_target_row),
                     cv::Scalar(0, 255, 0), 1);
        }

        // ---------- ★ 三段对齐相位机: ①左右 → ②朝向 → ③前后 ----------
        double phase_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - t_phase).count() / 1000.0;
        float yaw_err = normalize_180(yaw_hold_deg - g_yaw_deg.load());
        float omega = clampf(FINAL_YAW_HOLD_KP * yaw_err,
                             -FINAL_YAW_HOLD_OMEGA_MAX,
                              FINAL_YAW_HOLD_OMEGA_MAX);
        float vx = 0.0f;
        bool  yaw_ok = std::abs(yaw_err) < FINAL_TURN_TOL;
        bool  fb_ok  = true;
        bool  lat_ok = have_line && std::abs(err_px) <= FINAL_CENTER_TOL_PX;
        bool  centered_now = false;
        const char* phase_name = (phase == PH_LATERAL) ? "1-LAT"
                               : (phase == PH_YAW)     ? "2-YAW" : "3-FB";

        if (phase == PH_LATERAL) {
            // ① 左右: vy 居中 (含右移找线), yaw 里程计锁初始
            // ★ 七轮 (2226: 停位靠后线太短, ①在原地横搜 6s 超时): 没看到线但蓝区
            //   边界可见且明显偏离参照行 → 边搜边前/后移, 把线"拉进"视野再居中
            if (!have_line && fb_active && fb_boundary >= 0
                && std::abs(fb_target_row - fb_boundary) > 40) {
                vx = (fb_target_row > fb_boundary) ? +0.08f : -0.08f;
            }
            if (lat_ok) phase_stable++; else phase_stable = 0;
            if (phase_stable >= FINAL_CENTER_STABLE_FRAMES
                || phase_sec >= FINAL_PHASE_LAT_MAX_SEC) {
                std::cout << "[收尾] ①左右"
                          << (phase_stable >= FINAL_CENTER_STABLE_FRAMES
                                  ? "居中完成" : "相位超时兜底")
                          << " (err_px=" << err_px << ", " << phase_sec
                          << "s) → ②对平蓝区边界" << std::endl;
                phase = PH_YAW; phase_stable = 0;
                t_phase = std::chrono::steady_clock::now();
            }
        } else if (phase == PH_YAW) {
            // ② 朝向: 视觉把蓝白交界线转到与画面水平平行 (vy 继续保持居中)
            bool advance = false;
            if (fb_active && fb_tilt_valid) {
                if (std::abs(fb_tilt_deg) <= FINAL_YAW_VIS_TOL_DEG) {
                    omega = 0.0f;
                    if (++phase_stable >= FINAL_CENTER_STABLE_FRAMES) {
                        yaw_hold_deg = g_yaw_deg.load();   // 记住"边界水平"时的里程计 yaw
                        std::cout << "[收尾] ②朝向对平完成 (倾角 " << fb_tilt_deg
                                  << "°), 锁定 yaw=" << yaw_hold_deg
                                  << "° → ③前后对位" << std::endl;
                        advance = true;
                    }
                } else {
                    phase_stable = 0;
                    omega = FINAL_YAW_VIS_KP * fb_tilt_deg;   // 倾角>0(右端远) → 左转(+)
                    if (std::abs(omega) < FINAL_YAW_VIS_OMEGA_FLOOR) {
                        omega = (omega >= 0.0f) ? FINAL_YAW_VIS_OMEGA_FLOOR
                                                : -FINAL_YAW_VIS_OMEGA_FLOOR;
                    }
                    omega = clampf(omega, -FINAL_YAW_HOLD_OMEGA_MAX,
                                           FINAL_YAW_HOLD_OMEGA_MAX);
                }
            } else {
                // 视觉不可用: 蓝区迟迟不出现 → 先向后探一步; 仍看不到再有界后移找蓝区
                if (fb_active && !fb_seen_blue
                    && phase_sec >= FINAL_FB_NOBLUE_GIVEUP_SEC) {
                    if (!fb_no_blue_step_done) {
                        stepBackForBlue();
                        continue;
                    } else {
                        // ★ 看不到蓝区时不再横向右搜; 改为沿前后方向后移找蓝区。
                        //   (最多 FINAL_FB_SEARCH_BACK_MAX_SEC 秒, 看到蓝区立刻停);
                        //   找到 → 回①重新左右居中;  还找不到 → 按旧逻辑降级。
                        if (!fb_no_blue_backsearch_done) {
                            fb_no_blue_backsearch_done = true;
                            std::cout << "[收尾] ② 后探后仍未看到蓝区 → 有界后移找蓝区"
                                      << " (vx=" << FINAL_FB_SEARCH_BACK_VX << ", 上限 "
                                      << FINAL_FB_SEARCH_BACK_MAX_SEC << "s)" << std::endl;
                            auto t_bs = std::chrono::steady_clock::now();
                            bool blue_found = false;
                            cv::Mat bs_f;
                            while (g_running) {
                                double el_bs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - t_bs).count() / 1000.0;
                                if (el_bs >= FINAL_FB_SEARCH_BACK_MAX_SEC) break;
                                cap >> bs_f;
                                if (!bs_f.empty() && detectBlueZoneBoundary(bs_f) >= 0) {
                                    blue_found = true;
                                    std::cout << "[收尾] ② 后移中找到蓝区 ("
                                              << el_bs << "s) → 回①重新左右居中" << std::endl;
                                    break;
                                }
                                float bs_yaw_err = normalize_180(yaw_hold_deg - g_yaw_deg.load());
                                float bs_omega = clampf(FINAL_YAW_HOLD_KP * bs_yaw_err,
                                                        -FINAL_YAW_HOLD_OMEGA_MAX,
                                                         FINAL_YAW_HOLD_OMEGA_MAX);
                                sport.Move(FINAL_FB_SEARCH_BACK_VX, 0.0f, bs_omega);
                                usleep(5 * 1000);
                            }
                            sport.Move(0.0f, 0.0f, 0.0f);
                            usleep(100 * 1000);
                            if (blue_found) {
                                phase = PH_LATERAL; phase_stable = 0;
                                t_phase = std::chrono::steady_clock::now();
                                continue;
                            }
                        }
                        fb_active = false;
                        std::cout << "[收尾] ② 后探/后移后仍未看到蓝区, 放弃视觉对平/前后闭环"
                                  << " (退回初始 yaw + 纯左右居中)" << std::endl;
                    }
                }
                if (!fb_active) {
                    if (yaw_ok) phase_stable++; else phase_stable = 0;
                    advance = (phase_stable >= FINAL_CENTER_STABLE_FRAMES);
                }
                // fb_active 且见过蓝但本帧倾角不可信: 保持 yaw 等下一帧
                // ★ 八轮: 行可见但倾角拟合不出 (拟合要 25 点/跨 35%W, 狗被拖偏后
                //   蓝白交界只剩一角凑不够) → vx 借位把边界拉回参照行附近凑跨度
                //   (2226/2242: ② 整 4s "不可用" → 带 6° 歪斜停车 = -10 的直接来源)
                if (fb_active && fb_seen_blue && fb_boundary >= 0
                    && std::abs(fb_target_row - fb_boundary) > 40) {
                    vx = (fb_target_row > fb_boundary) ? +0.08f : -0.08f;
                }
            }
            if (!advance && phase_sec >= FINAL_PHASE_YAW_MAX_SEC) {
                std::cout << "[收尾] ②朝向相位 " << FINAL_PHASE_YAW_MAX_SEC
                          << "s 超时兜底 (倾角" << (fb_tilt_valid ? "未收敛" : "不可用")
                          << ", 按当前朝向继续) → ③前后对位" << std::endl;
                yaw_hold_deg = g_yaw_deg.load();   // 别再让里程计把已部分转正的朝向拽回去
                advance = true;
            }
            if (advance) {
                phase = PH_FB; phase_stable = 0;
                t_phase = std::chrono::steady_clock::now();
                // ★ 进③重置失速基准
                fb_ref_px = g_pos_x.load(); fb_ref_py = g_pos_y.load();
                t_fb_stallref = std::chrono::steady_clock::now();
                fb_kicking = false;
            }
        } else {
            // ③ 前后: 蓝区边界行伺服回参照行; vy/omega 继续保持左右与朝向
            if (fb_active) {
                if (fb_seen_blue) {
                    // 见过蓝又整帧丢失 = 已冲过前边界 → 按最近处理, 后退找回
                    int cur = (fb_boundary >= 0) ? fb_boundary : frame.rows;
                    fb_err_rows = fb_target_row - cur;   // >0: 边界偏远(狗偏后) → 前进
                    // ★ 九轮 非对称到位: 偏后(err>0)放宽 TOL_BACK, 偏前(err<0)收严 TOL_FRONT
                    //   → 停得靠后、绝不踩出前边缘 (用户: 后边空间大, 前边紧)
                    fb_ok = (fb_boundary >= 0)
                            && (fb_err_rows <= FINAL_FB_TOL_BACK)
                            && (fb_err_rows >= -FINAL_FB_TOL_FRONT);
                    if (!fb_ok) {
                        vx = FINAL_FB_KP * (float)fb_err_rows;
                        if (std::abs(vx) < FINAL_FB_VX_FLOOR) {
                            vx = (vx >= 0.0f) ? FINAL_FB_VX_FLOOR : -FINAL_FB_VX_FLOOR;
                        }
                        vx = clampf(vx, -FINAL_FB_VX_MAX, FINAL_FB_VX_MAX);
                        // ★ 失速检测 + 启动踢 (五连测: 细调速度推不动站立的狗直到超时)
                        //   0.6s 内位移 <1cm = 失速 → 0.2 m/s 定向踢 0.35s 破起步死区,
                        //   踢完回细调速度; 再失速再踢。
                        auto  now_fb = std::chrono::steady_clock::now();
                        float mdx = g_pos_x.load() - fb_ref_px;
                        float mdy = g_pos_y.load() - fb_ref_py;
                        float moved = std::sqrt(mdx * mdx + mdy * mdy);
                        if (fb_kicking) {
                            double kick_el = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                 now_fb - t_fb_kick).count() / 1000.0;
                            if (kick_el < FINAL_FB_KICK_SEC) {
                                vx = (fb_err_rows >= 0) ? FINAL_FB_KICK_VX
                                                        : -FINAL_FB_KICK_VX;
                            } else {
                                fb_kicking = false;
                                fb_ref_px = g_pos_x.load(); fb_ref_py = g_pos_y.load();
                                t_fb_stallref = now_fb;
                            }
                        } else if (moved >= FINAL_FB_STALL_MOVE_M) {
                            fb_ref_px = g_pos_x.load(); fb_ref_py = g_pos_y.load();
                            t_fb_stallref = now_fb;
                        } else {
                            double stall_el = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                  now_fb - t_fb_stallref).count() / 1000.0;
                            if (stall_el >= FINAL_FB_STALL_SEC) {
                                fb_kicking = true;
                                t_fb_kick = now_fb;
                                vx = (fb_err_rows >= 0) ? FINAL_FB_KICK_VX
                                                        : -FINAL_FB_KICK_VX;
                                std::cout << "[收尾] ③失速 (" << stall_el
                                          << "s 位移<" << FINAL_FB_STALL_MOVE_M
                                          << "m) → 启动踢 vx=" << vx << " "
                                          << FINAL_FB_KICK_SEC << "s" << std::endl;
                            }
                        }
                    }
                } else if (phase_sec >= FINAL_FB_NOBLUE_GIVEUP_SEC) {
                    if (!fb_no_blue_step_done) {
                        stepBackForBlue();
                        continue;
                    } else {
                        fb_active = false;   // 保险 (正常在②就放弃了)
                        std::cout << "[收尾] ③后探后仍未见蓝区, 放弃前后闭环" << std::endl;
                    }
                } else {
                    fb_ok = false;       // 等蓝区出现
                }
            }
            centered_now = lat_ok && yaw_ok && fb_ok;
        }

        if (centered_now) centered_stable++;
        else              centered_stable = 0;
        centered = centered_stable >= FINAL_CENTER_STABLE_FRAMES;

        if (have_line) {
            cv::line(frame, cv::Point(lat_target_x, 0),
                     cv::Point(lat_target_x, frame.rows),
                     cv::Scalar(255, 255, 0), 1);
            cv::circle(frame, cv::Point(line_cx, frame.rows / 2), 8,
                       cv::Scalar(0, 255, 255), -1);
        }
        char info[300];
        snprintf(info, sizeof(info),
                 "FINAL[%s] have=%d err_px=%d vy=%.2f yawE=%.1f tilt=%.1f%s fb=%d/%d vx=%.2f st=%d/%d %s",
                 phase_name, have_line ? 1 : 0, err_px, vy, yaw_err,
                 fb_tilt_deg, fb_tilt_valid ? "" : "?",
                 fb_boundary, fb_target_row, vx,
                 centered_stable, FINAL_CENTER_STABLE_FRAMES, center_mode);
        // ★ 八轮: 1Hz 心跳进日志 —— 这行状态原先只画在 HUD 视频里, stdout 只打
        //   状态切换, 事后从日志拼不出①②③期间的拉锯过程 (本轮诊断全靠拼图)
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t_hb).count() >= 1000) {
            t_hb = std::chrono::steady_clock::now();
            std::cout << "[对齐] " << info << std::endl;
        }
        cv::putText(frame, info, cv::Point(20, frame.rows - 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(255, 255, 255), 2);
        if (g_gui_enabled) {
            cv::imshow("Original", frame);
            cv::imshow("Mask", mask);
            cv::imshow("L channel", dbg_L);
        }

        int key = guiWaitKey(1);
        if (key == 27) {
            g_running = false;
            softStop(sport, SOFTSTOP_SETTLE_SEC);
            return false;
        }

        if (centered) {
            std::cout << "[收尾] ③三段对齐全部到位: err_px=" << err_px
                      << " yaw_err=" << yaw_err << "° 边界倾角=" << fb_tilt_deg
                      << "° fb_err=" << fb_err_rows
                      << "px stable=" << centered_stable << "/"
                      << FINAL_CENTER_STABLE_FRAMES << std::endl;
            break;
        }

        // ★ 兜底超时: 居不中也不无限卡, 到时停车结束
        double center_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - t_center_start).count() / 1000.0;
        if (center_elapsed >= FINAL_CENTER_TIMEOUT_SEC) {
            std::cout << "[收尾] 居中超时 " << FINAL_CENTER_TIMEOUT_SEC
                      << "s, 放弃微调, 停车结束 (err_px=" << err_px
                      << ", have_line=" << (have_line ? 1 : 0) << ")" << std::endl;
            break;
        }

        // ★ 九轮 (用户: 看到线还一直右移出区): 全相位右移地理围栏 —— 不管有线无线、
        //   哪个相位, 只要蓝区质心 <0.40W (狗已到启停区右缘) 就禁止右移 (vy<0),
        //   宁可不居中也绝不蹭出区。正常停位蓝区质心≈0.5W 不受影响; 蓝区<3%不启用。
        if (fb_active && vy < 0.0f) {
            int fence_bzx = blueZoneCentroidX(frame);
            if (fence_bzx >= 0 && fence_bzx < (int)(frame.cols * 0.40)) {
                vy = 0.0f;
                center_mode = "RIGHT_FENCED";
            }
        }

        sport.Move(vx, vy, omega);
        usleep(20 * 1000);
    }

    // 居中后再做一次绝对 yaw 校正, 然后直接停车结束。
    //   ★ 目标用 yaw_hold_deg 而不是初始 yaw: ②视觉对平后它已更新为"边界水平"的
    //     里程计 yaw, 再转回初始 yaw 会把视觉对平毁掉 (里程计跑完全程有漂移)。
    if (g_running) {
        turnToYawDeg(sport, yaw_hold_deg, FINAL_TURN_TOL, /*stop_at_end=*/true);
    }
    softStop(sport, SOFTSTOP_SETTLE_SEC);
    usleep(200 * 1000);
    return centered;
}

// ============================================================
//  ★ 2026-07-06 前跳落地姿态看门狗 (治五连测"落地踉跄~40%/向前拱→人工干预"):
//    在等待窗口内持续监测 IMU; 姿态健康时【一条指令都不发】(不破坏步态链, 等待
//    行为与原 sleep 完全一致); 姿态坏了分级出手:
//      坏 POST_JUMP_GUARD_HOLD_SEC → BalanceStand() 让平衡控制器重新踩实;
//      再坏 POST_JUMP_GUARD_ESCALATE_SEC → RecoveryStand() 完整自恢复 (窗口自动延长)。
//    start_check_sec: 窗口前段是跳跃动画本身 (pitch 大属正常), 跳过不查。
// ============================================================
void postJumpAttitudeGuard(unitree::robot::go2::SportClient& sport,
                           double window_sec, double start_check_sec) {
    auto   t0 = std::chrono::steady_clock::now();
    double bad_since   = -1.0;   // 姿态首次变坏时刻 (相对 t0); <0 = 当前健康
    bool   did_balance = false, did_recovery = false;
    while (g_running) {
        double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count() / 1000.0;
        if (el >= window_sec) break;
        if (el >= start_check_sec && !did_recovery) {
            float p = std::fabs(rad2deg(g_pitch.load()));
            float r = std::fabs(rad2deg(g_roll.load()));
            bool  bad = (p > POST_JUMP_GUARD_PITCH_DEG) || (r > POST_JUMP_GUARD_ROLL_DEG);
            if (!bad) {
                bad_since = -1.0;
            } else {
                if (bad_since < 0) bad_since = el;
                double bad_dur = el - bad_since;
                if (!did_balance && bad_dur >= POST_JUMP_GUARD_HOLD_SEC) {
                    did_balance = true;
                    std::cout << "[前跳保险] 落地姿态异常 (|pitch|=" << p
                              << "° |roll|=" << r << "° 持续 " << bad_dur
                              << "s) → BalanceStand() 重踩" << std::endl;
                    sport.BalanceStand();
                } else if (did_balance
                           && bad_dur >= POST_JUMP_GUARD_HOLD_SEC
                                          + POST_JUMP_GUARD_ESCALATE_SEC) {
                    did_recovery = true;
                    std::cout << "[前跳保险] BalanceStand 未救回 (|pitch|=" << p
                              << "° |roll|=" << r
                              << "°) → RecoveryStand() 完整自恢复" << std::endl;
                    sport.RecoveryStand();
                    double need = el + POST_JUMP_GUARD_RECOVERY_WAIT_SEC;
                    if (need > window_sec) window_sec = need;   // 给恢复动作留够时间
                }
            }
        }
        usleep(20 * 1000);
    }
}

// ============================================================
//  ★ 2026-07-07 八轮: 软急停 softStop —— 模拟遥控器"松开摇杆"的停车方式。
//    用户遥控器实验实锤: 松杆(=手柄持续流零速度目标)后狗自己减速、原地踏步几下
//    把四脚踩方再静止, 之后连做 N 次前跳/打招呼全部安全; 而程序里的硬停
//    是硬停+切 BalanceStand 阻尼态, 四脚走到中步相哪儿就被冻在哪儿 —— 歪站姿
//    起步的全身动作才是 Hello 翻车/抬爪僵住的根因 (IMU 门拦不住: 躯干是平的,
//    歪的是脚)。做法: 50Hz 持续发 Move(0,0,0) settle_sec 秒 (步态控制器自己
//    收尾), 然后【直接停止发令】—— 不硬停、不 BalanceStand, 狗停留在
//    运动模式的自然站立, 与"松杆后按动作键"的状态链完全一致。
// ============================================================
void softStop(unitree::robot::go2::SportClient& sport, double settle_sec) {
    auto t0 = std::chrono::steady_clock::now();
    while (g_running) {
        double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count() / 1000.0;
        if (el >= settle_sec) break;
        sport.Move(0.0f, 0.0f, 0.0f);
        usleep(20 * 1000);
    }
}

// ============================================================
//  前跳 (★ 2026-07-05 重写, 治"带着 0.20m/s 惯性急刹 0.5s 就起跳 → 第一跳踉跄"):
//    ① 先把 vx 从巡线速度线性减到 0 (PREJUMP_DECEL_SEC), 让狗"走停"而不是"急刹";
//    ② 软停 (零速流) 后等 IMU 姿态稳住 (|roll|,|pitch| < PREJUMP_ATT_TOL_DEG),
//       至少站 PREJUMP_MIN_STAND_SEC, 最多 PREJUMP_MAX_STAND_SEC 兜底;
//    ③ 再 FrontJump, 跳完等动画结束 (带落地姿态看门狗)。收尾前跳共用本函数。
// ============================================================
bool doFrontJump(unitree::robot::go2::SportClient& sport,
                 float approach_vx = 0.20f,
                 bool quick = false,
                 float align_yaw_deg = NAN) {
    // quick=true: 横杆识别触发的快速通道 (0.1 蠕行动量极小, 触发后尽快起跳)
    const double decel_sec = quick ? PREJUMP_QUICK_DECEL_SEC     : PREJUMP_DECEL_SEC;
    const double min_stand = quick ? PREJUMP_QUICK_MIN_STAND_SEC : PREJUMP_MIN_STAND_SEC;
    const double max_stand = quick ? PREJUMP_QUICK_MAX_STAND_SEC : PREJUMP_MAX_STAND_SEC;
    if (quick) std::cout << "[前跳] 快速通道 (触发即跳): 减速 " << decel_sec
                         << "s, 站稳 " << min_stand << "~" << max_stand << "s" << std::endl;
    // ① 线性减速 (50Hz)
    {
        auto t0 = std::chrono::steady_clock::now();
        while (g_running) {
            double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - t0).count() / 1000.0;
            if (el >= decel_sec) break;
            float vx = approach_vx * (1.0f - (float)(el / decel_sec));
            if (vx < 0.0f) vx = 0.0f;
            sport.Move(vx, 0.0f, 0.0f);
            usleep(20 * 1000);
        }
    }
    // ★ 八轮: 硬停 → softStop 零速流软停 (遥控器松杆同款,
    //   步态自己收尾把四脚踩方, 起跳站姿不再是被冻结的中步相)
    softStop(sport, SOFTSTOP_SETTLE_SEC);
    // 起跳前回到指定场地航向；默认 NAN 时保持旧调用行为。
    if (!std::isnan(align_yaw_deg) && g_running) {
        float target_yaw = normalize_180(align_yaw_deg + JUMP_YAW_FF_DEG);
        float yaw_error = normalize_180(target_yaw - g_yaw_deg.load());
        std::cout << "[前跳] 起跳前航向对齐: 当前 yaw=" << g_yaw_deg.load()
                  << "° → 目标 " << target_yaw << "° (基准 " << align_yaw_deg
                  << "° + 前馈 " << JUMP_YAW_FF_DEG << "°)" << std::endl;
        if (std::fabs(yaw_error) >= JUMP_YAW_ALIGN_TOL
            && !turnToYawDeg(sport, target_yaw, JUMP_YAW_ALIGN_TOL,
                             /*stop_at_end=*/true)) {
            std::cout << "[前跳取消] 起跳前航向未在限时内对齐，禁止继续 FrontJump"
                      << std::endl;
            return false;
        }
    }
    // ② IMU 姿态稳定门
    {
        auto t0 = std::chrono::steady_clock::now();
        while (g_running) {
            double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - t0).count() / 1000.0;
            float roll_deg  = std::fabs(rad2deg(g_roll.load()));
            float pitch_deg = std::fabs(rad2deg(g_pitch.load()));
            bool  att_ok = (roll_deg < PREJUMP_ATT_TOL_DEG)
                        && (pitch_deg < PREJUMP_ATT_TOL_DEG);
            if (el >= min_stand && att_ok) {
                std::cout << "[前跳] 姿态就绪 (roll=" << roll_deg
                          << "° pitch=" << pitch_deg << "°, 站稳 " << el
                          << "s)" << std::endl;
                break;
            }
            if (el >= max_stand) {
                std::cout << "[前跳] 姿态稳定等待超时 (roll=" << roll_deg
                          << "° pitch=" << pitch_deg << "°), 兜底起跳" << std::endl;
                break;
            }
            usleep(20 * 1000);
        }
    }
    if (!g_running) {
        std::cout << "[前跳取消] 起跳前收到中断，未调用 FrontJump" << std::endl;
        return false;
    }
    float yaw_before_jump = g_yaw_deg.load();
    std::cout << "[前跳] FrontJump()..." << std::endl;
    int jump_ret = sport.FrontJump();
    if (jump_ret != 0) {
        std::cout << "[前跳失败] FrontJump() ret=" << jump_ret
                  << "，不再按成功落地继续" << std::endl;
        sport.Move(0.0f, 0.0f, 0.0f);
        return false;
    }
    // ★ 等动画 + 落地姿态看门狗 (前 POST_JUMP_GUARD_START_SEC 秒是动画本身, 不查)
    postJumpAttitudeGuard(sport, (double)FRONTJUMP_WAIT_SEC, POST_JUMP_GUARD_START_SEC);
    float yaw_after_jump = g_yaw_deg.load();
    g_last_jump_yaw_delta_deg = normalize_180(yaw_after_jump - yaw_before_jump);
    std::cout << "[前跳偏置] 起跳前 yaw=" << yaw_before_jump
              << "° 落地后 yaw=" << yaw_after_jump
              << "° ★Δ=" << g_last_jump_yaw_delta_deg
              << "° (正=左偏, 负=右偏; 多次跑取均值反号填 JUMP_YAW_FF_DEG)"
              << std::endl;
    return g_running;
}

// ============================================================
//  执行警示动作: 1=伸懒腰(Stretch) 2=打招呼(Hello) 3=闪前灯三次
// ============================================================
void executeWarnAction(unitree::robot::go2::SportClient& sport, int action_id) {
    std::cout << "[警示动作] 执行动作 " << action_id << ": ";
    // ★ 2026-07-06 第五轮: 全身动作 (伸懒腰/打招呼) 前先站稳 —— 复用前跳姿态门方案
    //   (0706 第七次: 巡线急停后 1s 直接 Hello, 重心不稳 → 人工干预)。闪灯不需要。
    // ★ 八轮: 删掉这里的 BalanceStand 重踩 —— 上游已改 softStop 松杆式停车,
    //   狗停在步态自然收尾的站立态 (四脚踩方, 遥控器实测该状态下动作 N 次全安全);
    //   再发 BalanceStand 反而把它切回阻尼态毁掉好站姿。姿态门只读不发令, 保留。
    if (action_id == 1 || action_id == 2) {
        auto t0 = std::chrono::steady_clock::now();
        while (g_running) {
            double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - t0).count() / 1000.0;
            float roll_deg  = std::fabs(rad2deg(g_roll.load()));
            float pitch_deg = std::fabs(rad2deg(g_pitch.load()));
            bool  att_ok = (roll_deg < PREJUMP_ATT_TOL_DEG)
                        && (pitch_deg < PREJUMP_ATT_TOL_DEG);
            if (el >= WARN_ACT_MIN_STAND_SEC && att_ok) {
                std::cout << "\n[警示动作] 姿态就绪 (roll=" << roll_deg
                          << "° pitch=" << pitch_deg << "°, 站稳 " << el
                          << "s) → ";
                break;
            }
            if (el >= WARN_ACT_MAX_STAND_SEC) {
                std::cout << "\n[警示动作] 姿态稳定等待超时 (roll=" << roll_deg
                          << "° pitch=" << pitch_deg << "°), 兜底照做 → ";
                break;
            }
            usleep(20 * 1000);
        }
    }
    // ★ 七轮 (0706 2258: 姿态门 roll=0.5°/pitch=0.6° 完美通过, Hello 途中照样整只翻倒
    //   —— 背载机械臂重心太高, 动作本身就在翻倒边缘, 入场门槛拦不住):
    //   动作期间改为【姿态监视等待】替代盲睡: |roll|>40° 或 |pitch|>60°
    //   (远超动作正常幅度 = 正在翻) 立即 RecoveryStand 自救翻回来,
    //   保住背上设备、免掉人工干预。
    auto guardedActWait = [&]() {
        bool fell = false;
        auto tw = std::chrono::steady_clock::now();
        while (g_running) {
            double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - tw).count() / 1000.0;
            if (el >= (double)WARN_ACT_WAIT_SEC) break;
            float r = std::fabs(rad2deg(g_roll.load()));
            float p = std::fabs(rad2deg(g_pitch.load()));
            if (r > 40.0f || p > 60.0f) {
                fell = true;
                std::cout << "\n[警示动作] ★ 检测到倾覆 (roll=" << r
                          << "° pitch=" << p << "°), 立即 RecoveryStand 自救!" << std::endl;
                break;
            }
            usleep(20 * 1000);
        }
        if (fell) {
            sport.RecoveryStand();
            sleep(3);
            sport.BalanceStand();
            usleep(500 * 1000);
            std::cout << "[警示动作] 自救完成, 继续流程" << std::endl;
        }
    };
    if (action_id == 1) {
        std::cout << "伸懒腰 (Stretch)" << std::endl;
        sport.Stretch();
        guardedActWait();
    } else if (action_id == 2) {
        std::cout << "打招呼 (Hello)" << std::endl;
        sport.Hello();
        guardedActWait();
    } else if (action_id == 3) {
        std::cout << "闪烁前灯三次" << std::endl;
        if (g_vui) {
            for (int i = 0; i < 3; ++i) {
                g_vui->SetBrightness(10);
                usleep(300 * 1000);
                g_vui->SetBrightness(0);
                usleep(300 * 1000);
            }
        } else {
            std::cout << "  [警告] VuiClient 未初始化, 跳过闪灯" << std::endl;
        }
    } else {
        std::cout << "未知动作号, 跳过" << std::endl;
    }
    std::cout << "[警示动作] 完成" << std::endl;
}

// ============================================================
//  ★ 白色横杆障碍识别 (起点/终点障碍, 触发前跳)
//    输入: 本帧黑线 mask (全图)、模糊后的 L 通道、当前线中心 x。
//    沿线中心竖条带逐行统计"该行是否有线", 找 "线段-亮缺口-线段" 图案:
//      - 缺口高度 ∈ [OB_GAP_MIN_PX, OB_GAP_MAX_PX]
//      - 缺口平均亮度 ≥ 黑线阈值 + OB_GAP_BRIGHT_MARGIN (证明是亮物, 不是阴影)
//      - 缺口上下两段线各 ≥ OB_SEG_MIN_PX 行 (证明是"截断", 不是线的尽头)
//    命中返回 true 并给出缺口下沿行号 (由下往上取最靠近狗的那个缺口)。
// ============================================================
bool detectObstacleGap(const cv::Mat& mask, const cv::Mat& blurred_L,
                       int line_cx, cv::Mat* draw_frame,
                       int& gap_bottom, int& gap_height) {
    gap_bottom = -1; gap_height = 0;
    if (mask.empty() || blurred_L.empty()) return false;
    if (mask.size() != blurred_L.size()) return false;
    int W = mask.cols, H = mask.rows;
    int x0 = std::max(0, line_cx - OB_STRIP_HALF_W_PX);
    int x1 = std::min(W, line_cx + OB_STRIP_HALF_W_PX);
    if (x1 - x0 < 20) return false;
    int y0 = (int)(H * OB_SCAN_TOP_FRAC);
    int y1 = (int)(H * OB_SCAN_BOTTOM_FRAC);
    y0 = std::max(0, std::min(y0, H - 2));
    y1 = std::max(y0 + 1, std::min(y1, H));

    // 逐行: 条带内黑线像素数 → 该行是否"有线"
    std::vector<char> row_has_line(y1 - y0, 0);
    for (int y = y0; y < y1; ++y) {
        const unsigned char* mp = mask.ptr<unsigned char>(y);
        int cnt = 0;
        for (int x = x0; x < x1; ++x) if (mp[x]) cnt++;
        row_has_line[y - y0] = (cnt >= OB_ROW_LINE_MIN_PX) ? 1 : 0;
    }

    // 提取"有线行"的连续段 (全图行号, [a,b) 区间)
    struct ObSeg { int a, b; };
    std::vector<ObSeg> segs;
    int seg_start = -1;
    for (int i = 0; i <= y1 - y0; ++i) {
        bool on = (i < y1 - y0) && row_has_line[i];
        if (on && seg_start < 0) seg_start = i;
        if (!on && seg_start >= 0) {
            segs.push_back({y0 + seg_start, y0 + i});
            seg_start = -1;
        }
    }
    if (segs.size() < 2) return false;

    int thr_ref = (g_dbg_thr > 0) ? g_dbg_thr : g_max_brightness;

    // 由下往上找第一个满足条件的缺口 (最靠近狗)
    for (int i = (int)segs.size() - 1; i >= 1; --i) {
        const ObSeg& below = segs[i];
        const ObSeg& above = segs[i - 1];
        if ((below.b - below.a) < OB_SEG_MIN_PX) continue;
        if ((above.b - above.a) < OB_SEG_MIN_PX) continue;
        int ga = above.b;          // 缺口顶 = 上段线的底
        int gb = below.a;          // 缺口底 = 下段线的顶
        int gh = gb - ga;
        if (gh < OB_GAP_MIN_PX || gh > OB_GAP_MAX_PX) continue;

        // 缺口亮度检查: 条带内缺口行的平均 L
        double sum = 0.0; long n = 0;
        for (int y = ga; y < gb; ++y) {
            const unsigned char* lp = blurred_L.ptr<unsigned char>(y);
            for (int x = x0; x < x1; ++x) { sum += lp[x]; n++; }
        }
        double meanL = (n > 0) ? sum / (double)n : 0.0;
        if (meanL < (double)(thr_ref + OB_GAP_BRIGHT_MARGIN)) continue;

        gap_bottom = gb;
        gap_height = gh;
        if (draw_frame) {
            cv::rectangle(*draw_frame, cv::Point(x0, ga), cv::Point(x1, gb),
                          cv::Scalar(255, 0, 255), 2);
            char t[96];
            snprintf(t, sizeof(t), "OBSTACLE GAP h=%d meanL=%.0f thr=%d",
                     gh, meanL, thr_ref);
            cv::putText(*draw_frame, t, cv::Point(x0, std::max(20, ga - 8)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
        }
        return true;
    }
    return false;
}

// ============================================================
//  巡线主循环 (源自 TrackLoop, 改成可参数化退出条件的函数)
//
//  mode:
//    TO_LINE_LOST —— 备用分支:首次看不到黑线即返回 AVOIDANCE_TRIGGER (主流程当前不用)
//    TIMED        —— 巡线 timer_sec 秒后返回 TIMER_DONE
//    FOREVER      —— 持续巡线,只有 ESC / Ctrl+C 才退出
//
//  返回:
//    AVOIDANCE_TRIGGER / TIMER_DONE —— 正常退出,★不调 softStop (保持运动无缝衔接下一阶段)
//    ABORTED                         —— ESC 或 Ctrl+C,已 softStop
// ============================================================
enum class LineMode   { TO_LINE_LOST, TIMED, FOREVER, PLATFORM_DETECT, THREE_TURN_DETECT, DUAL_PLATFORM_DETECT,
                        TO_OBSTACLE /* ★ 巡线 + 白色横杆检测, 命中或窗口到点返回 OBSTACLE_TRIGGER (前跳用) */ };
enum class LineResult { AVOIDANCE_TRIGGER, TIMER_DONE, ABORTED, PLATFORM_REACHED, THREE_TURN_DONE, DUAL_PLATFORM_REACHED,
                        DETECT_TIMEOUT, /* ★ 检测模式看门狗超时, 主流程按各自策略降级 */
                        OBSTACLE_TRIGGER /* ★ TO_OBSTACLE: 识别到横杆 (或窗口上限兜底), 该起跳了 */ };

// ============================================================
//  ★ 红圆命中后的统一动作序列: 停车 → 警示动作 → 左平移 → 置完成标志
//    两个调用方: ① FOREVER 模式红圆正常触发; ② FOREVER 红圈等待超时降级。
//    返回 TIMER_DONE 供 runLineFollowing 直接 return。
// ============================================================
LineResult execRedActionSequence(unitree::robot::go2::SportClient& sport,
                                 const cv::Mat* autoid_frame = nullptr,
                                 cv::VideoCapture* cover_cap = nullptr) {
    // ★ 八轮改二 (用户: 红圈没被完全盖住): 视觉伺服盖圈前压 —— 边巡线纠偏边前压,
    //   红圈连续丢帧 = 已进身下, 再收尾推一小段; 无相机句柄退回固定时长盲压。
    if (g_running) {
        auto t_cv = std::chrono::steady_clock::now();
        auto cover_elapsed = [&]() {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - t_cv).count() / 1000.0;
        };
        if (cover_cap) {
            std::cout << "[红圆] 视觉伺服前压盖圈: vx=" << RED_COVER_FWD_VX
                      << " (连丢 " << RED_COVER_LOST_FRAMES << " 帧后 +"
                      << RED_COVER_EXTRA_SEC << "s 收尾, 上限 "
                      << RED_COVER_MAX_SEC << "s)" << std::endl;
            cv::Mat cv_f, cv_m, cv_d;
            int    lost = 0;
            double extra_until = -1.0;   // <0=还在盯圈; >=0=收尾推进的截止时刻
            while (g_running) {
                double el = cover_elapsed();
                if (el >= RED_COVER_MAX_SEC) {
                    std::cout << "[红圆] 前压达上限 " << RED_COVER_MAX_SEC
                              << "s, 停止前压" << std::endl;
                    break;
                }
                if (extra_until >= 0.0 && el >= extra_until) {
                    std::cout << "[红圆] 盖圈收尾完成 (t=" << el << "s)" << std::endl;
                    break;
                }
                float omega = 0.0f;
                *cover_cap >> cv_f;
                if (!cv_f.empty()) {
                    // 黑线纠偏: 下半幅黑线质心 → 小增益 omega (丢线 omega=0 直走)
                    if (extractBlackLine(cv_f, cv_m, cv_d, 300)) {
                        cv::Mat lower = cv_m.rowRange(cv_m.rows / 2, cv_m.rows);
                        cv::Moments mm = cv::moments(lower, true);
                        if (mm.m00 > 300) {
                            float errp = (float)(mm.m10 / mm.m00)
                                         - cv_f.cols * 0.5f;
                            omega = clampf(-RED_COVER_YAW_KP * errp,
                                           -RED_COVER_OMEGA_MAX,
                                            RED_COVER_OMEGA_MAX);
                        }
                    }
                    if (extra_until < 0.0) {
                        if (detectRedCircle(cv_f)) {
                            lost = 0;
                        } else if (++lost >= RED_COVER_LOST_FRAMES) {
                            extra_until = el + RED_COVER_EXTRA_SEC;
                            std::cout << "[红圆] 圈已进身下 (连丢 " << lost
                                      << " 帧, t=" << el << "s) → 收尾推进 "
                                      << RED_COVER_EXTRA_SEC << "s" << std::endl;
                        }
                    }
                }
                sport.Move(RED_COVER_FWD_VX, 0.0f, omega);
                usleep(10 * 1000);   // cap>> 本身 ~30ms, 循环 ~25Hz
            }
        } else {
            // 兜底: 无相机句柄, 固定 0.8s 盲压
            std::cout << "[红圆] (无相机句柄) 盲压 0.8s 覆盖红圈" << std::endl;
            while (g_running && cover_elapsed() < 0.8) {
                sport.Move(RED_COVER_FWD_VX, 0.0f, 0.0f);
                usleep(20 * 1000);
            }
        }
    }
    // ★ 八轮: 原"带着巡线速度直接硬停 + 睡 1s"是全套代码里最硬的一次急停,
    //   后面紧跟的就是 Hello/伸懒腰 —— 软停替换 (零速流让步态自己减速收尾)
    softStop(sport, SOFTSTOP_SETTLE_SEC);
    g_action_id = 0;
    if (autoid_frame && !autoid_frame->empty()) {
        cv::Mat dbg;
        int auto_action = autoid::classifyWarningAction(
            *autoid_frame, g_gui_enabled ? &dbg : nullptr);
        if (auto_action >= 1 && auto_action <= 3) {
            std::cout << "[AUTOID] 警示标志识别成功: action_id="
                      << auto_action << std::endl;
            g_action_id = auto_action;
            if (g_gui_enabled && !dbg.empty()) {
                cv::imshow("AutoID", dbg);
                guiWaitKey(1);
            }
        } else {
            std::cout << "[AUTOID] 警示标志未识别, 不执行警示动作" << std::endl;
        }
    } else {
        std::cout << "[AUTOID] 无可用警示标志画面, 不执行警示动作" << std::endl;
    }
    if (g_action_id > 0) {
        executeWarnAction(sport, g_action_id);
    } else {
        std::cout << "[红圆] 未获得有效识别结果 (action_id=0), 跳过警示动作" << std::endl;
    }
    if (g_running) {
        // ★ 九轮 (用户): 仅动作 1 (伸懒腰) 左移时长 +1s, 动作 2/3 不变
        double red_lshift_sec = (g_action_id == 1)
            ? POST_RED_ACTION_LSHIFT_SEC_STRETCH : POST_RED_ACTION_LSHIFT_SEC;
        std::cout << "[红圆] 动作完成后向左平移 vy="
                  << POST_RED_ACTION_LSHIFT_VY << " "
                  << red_lshift_sec << "s (action_id=" << g_action_id << ")" << std::endl;
        sport.StaticWalk();
        usleep(300 * 1000);
        auto t_red_shift = std::chrono::steady_clock::now();
        while (g_running) {
            double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - t_red_shift).count() / 1000.0;
            if (el >= red_lshift_sec) break;
            sport.Move(0.0f, POST_RED_ACTION_LSHIFT_VY, 0.0f);
            usleep(20 * 1000);
                  }
        softStop(sport, SOFTSTOP_SETTLE_SEC);
        usleep(200 * 1000);
        if (g_running && g_action_id == 1) {
            std::cout << "[红圆] 动作1伸懒腰左移后额外左转 "
                      << POST_RED_STRETCH_LTURN_DEG << "°" << std::endl;
            turnInPlace(sport, +POST_RED_STRETCH_LTURN_DEG, RED_TURN_TOL);
        }
    }
    g_warn_action_done = true;
    return LineResult::TIMER_DONE;
}

LineResult runLineFollowing(unitree::robot::go2::SportClient& sport,
                            cv::VideoCapture& cap,
                            LineMode mode, double timer_sec,
                            float yaw_baseline_rad = 0.0f,
                            bool stop_on_exit = true,
                            double fixed_speed = -1.0,
                            double fixed_speed_switch_sec = -1.0,
                             double fixed_speed_after = -1.0,
                             double rotation_mult = 1.0,
                             double rot_mult_ramp_sec = 0.0,
                             double ob_window_min_sec = 0.0,
                             bool assert_static_walk = true) {
    // 巡线统一用常规步态 —— 每次进入都自断言一次 (与 line.cpp 原行为一致)
    //   阶段1: 初始切换;  阶段3: 避障 softStop 后必须重切;  阶段5: 保险重切
    if (assert_static_walk) {
        std::cout << "[步态] StaticWalk (常规步态)..." << std::endl;
        sport.StaticWalk();
        usleep(1000000);
    } else {
        std::cout << "[步态] 沿用刚完成航向复位的 StaticWalk，跳过重复切换"
                  << std::endl;
    }
    // 清掉进入巡线前残留的横移/转向速度, 避免视觉未 valid 时沿用上一阶段 Move。
    sport.Move(0.0f, 0.0f, 0.0f);
    usleep(20 * 1000);

    const char* mode_str = (mode == LineMode::TO_LINE_LOST)         ? "备用:首次丢线触发避障"
                         : (mode == LineMode::TIMED)                ? "限时巡线"
                         : (mode == LineMode::TO_OBSTACLE)          ? "横杆检测巡线(前跳)"
                         : (mode == LineMode::PLATFORM_DETECT)      ? "弧形→平台停位"
                         : (mode == LineMode::THREE_TURN_DETECT)    ? "三连转检测"
                         : (mode == LineMode::DUAL_PLATFORM_DETECT) ? "双侧平台检测"
                                                                    : "持续巡线";
    std::cout << "[巡线] 模式: " << mode_str;
    if (mode == LineMode::TIMED) std::cout << " (" << timer_sec << "s)";
    if (mode == LineMode::TO_OBSTACLE) {
        std::cout << " (检测窗 [" << ob_window_min_sec << ", " << timer_sec
                  << "]s, obdetect=" << (g_ob_detect_enabled ? 1 : 0)
                  << ", 触发行=" << OB_TRIGGER_ROW_FRAC << "H)";
    }
    if (fixed_speed > 0.0) std::cout << " fixed_vx=" << fixed_speed;
    if (fixed_speed_after > 0.0 && fixed_speed_switch_sec >= 0.0) {
        std::cout << "->" << fixed_speed_after
                  << " @" << fixed_speed_switch_sec << "s";
    }
    if (rotation_mult != 1.0) std::cout << " rot_x=" << rotation_mult;
    if (mode == LineMode::PLATFORM_DETECT) {
        std::cout << " (baseline=" << rad2deg(yaw_baseline_rad) << "°"
                  << " target_delta=" << ARC_YAW_TARGET_DELTA_DEG << "°"
                  << " lidar<" << ARC_LIDAR_LEFT_THRESH_M << "m)";
    }
    if (mode == LineMode::THREE_TURN_DETECT) {
        std::cout << " (baseline=" << rad2deg(yaw_baseline_rad) << "°"
                  << " phase0=+" << POST_PLAT_PHASE0_THRESH_DEG
                  << "° phase1=-" << POST_PLAT_PHASE1_THRESH_DEG
                  << "° phase 2 终止=local cum≥+" << POST_PLAT_FINAL_THRESH_DEG
                  << "° + global>|" << INITIAL_YAW_CUM_TRIGGER_DEG
                  << "°|)";
    }
    if (mode == LineMode::DUAL_PLATFORM_DETECT) {
        std::cout << " (左右 lidar < " << DUAL_PLAT_LIDAR_THRESH_M
                  << "m 连续 " << DUAL_PLAT_STABLE_FRAMES << " 帧)";
    }
    std::cout << std::endl;

    cv::Mat frame, mask, dbg_L;
    int    lost_count = 0;
    int    last_dir   = 0;
    double prev_error = 0.0;

    ResetTrajectory();      // 每次进入巡线清空轨迹记忆 (避障/台阶后画面已变)

    int  no_line_consec = 0;        // 第一阶段丢线触发计数
    bool line_lost_trigger = false; // 第一阶段触发避障标志
    bool black_line_visible = true; // HUD 显示用

    // ★ TO_OBSTACLE 模式状态 (横杆检测)
    int  ob_found_stable    = 0;    // 连续看到"线被亮带截断"图案的帧数
    int  ob_last_gap_bottom = -1;   // 最近一次缺口下沿行号

    auto t_start = std::chrono::steady_clock::now();

    bool red_ever_seen   = false; // 已确认见过红圆, 之后等待它离开视野
    int  red_seen_consec = 0;     // 红圆连续命中帧数
    int  red_lost_consec = 0;     // 见过红圆后, 红圆连续消失帧数

    // 累计 yaw 状态 (PLATFORM_DETECT 和 THREE_TURN_DETECT 共用)
    //   ★ 用累计积分法避免 ±π 跳变: 每帧算 dyaw (wrap 到 [-π,π]),
    //     累加到 arc_cum_delta (无界、单调)。
    float arc_yaw_prev           = yaw_baseline_rad;               // 上一帧 yaw
    float arc_cum_delta          = 0.0f;                           // 累计转角 (无界)

    // PLATFORM_DETECT 专用状态
    int   plat_stable_cnt        = 0;                              // 连续满足帧数
    // ★ 弧形入口补救 (2026-07-05 重写): "看到线" = near ROI 连续锁上, 不是画面里有黑块
    int   arc_start_line_lock_cnt  = 0;    // 连续 valid 帧计数
    bool  arc_start_shifting       = false;
    bool  arc_start_shift_done     = false;
    auto  arc_start_shift_t0       = t_start;
    float arc_start_shift_yaw_hold = 0.0f; // 右移期间锁定的朝向
    // THREE_TURN_DETECT 专用状态
    //   phase 0: 相对三连转初始 yaw 达到 +46° 即切相位 (左转)
    //   phase 1: 相对 phase 0 触发 yaw 达到 -31° 即切相位 (右转)
    //   phase 2: 相对三连转初始 yaw 达到 POST_PLAT_FINAL_THRESH_DEG 且全局累计 yaw>302° 才停
    int   tt_phase               = 0;                              // 当前相位
    int   tt_stable_cnt          = 0;
    // ★ 三连转丢线保护:
    //   phase0/1: near ROI 连续无有效线 TT_LINE_LOST_BACK_SEC 后, 后退一步重找线;
    //   phase2: 仍沿用末段卡死语义 —— 未达成、无有效线、卡在最终目标附近且角度基本不动才退。
    bool  tt_line_lost_active    = false;
    auto  tt_line_lost_since     = std::chrono::steady_clock::now();
    int   tt_line_lost_nudges    = 0;
    float tt_phase2_stall_ref_delta_deg = -999.0f;
    auto  tt_phase2_stall_ref_time      = std::chrono::steady_clock::now();
    float tt_initial_yaw_rad     = yaw_baseline_rad;               // 三连转初始 yaw
    float tt_phase_ref_yaw_rad   = yaw_baseline_rad;               // 当前相位比较基准 yaw
    float tt_phase0_thresh_rad   = deg2rad(POST_PLAT_PHASE0_THRESH_DEG); // +46°
    float tt_phase1_thresh_rad   = deg2rad(POST_PLAT_PHASE1_THRESH_DEG); // -31°
    float tt_final_thresh_rad    = deg2rad(POST_PLAT_FINAL_THRESH_DEG);

    // DUAL_PLATFORM_DETECT 专用状态
    int   dual_plat_stable_cnt   = 0;

    // ★ 断流保护计数 (跳跃/台阶震动可能震松 USB)
    int   empty_frames           = 0;

    while (g_running) {
        cap >> frame;
        if (frame.empty()) {
            // ★ 断流保护: 连续空帧 → 停车 + 尝试重连
            if (++empty_frames >= CAM_EMPTY_FRAMES_REOPEN) {
                std::cout << "[相机] 连续 " << empty_frames
                          << " 空帧 → softStop + 尝试重连巡线相机..." << std::endl;
                softStop(sport, SOFTSTOP_SETTLE_SEC);
                cap.release();
                if (!openAndConfigLineCamera(cap)) {
                    std::cout << "[相机] 重连失败, 0.5s 后再试" << std::endl;
                    usleep(500 * 1000);
                }
                empty_frames = 0;
            }
            usleep(20 * 1000);
            continue;
        }
        empty_frames = 0;
        int W = frame.cols;
        int H = frame.rows;

        // 红圆检测用原始画面做,不要等 ROI/HUD/轮廓线画上去之后再识别。
        // ★ 门控到 FOREVER 模式: 红圆只在三连转之后那段路出现; 其他阶段开着检测
        //   会被抓取平台立板上的红色同心圆识别标志/场外红物误触发 (整幅画面检测!)。
        bool red_seen_this_frame = false;
        if (!g_warn_action_done && mode == LineMode::FOREVER) {
            cv::Mat red_src = frame.clone();
            red_seen_this_frame = detectRedCircle(red_src);
        }

        cv::Rect roi_near(0, int(H * ROI_NEAR_TOP), W,
                          int(H * (ROI_NEAR_BOTTOM - ROI_NEAR_TOP)));
        cv::Rect roi_far(0, int(H * ROI_FAR_TOP), W,
                         int(H * (ROI_FAR_BOTTOM - ROI_FAR_TOP)));

        bool have_real_line = extractBlackLine(frame, mask, dbg_L);

        int far_dir  = INT_MIN;
        int near_dir = INT_MIN;
        bool far_cross  = false;
        bool near_cross = false;
        if (have_real_line) {
            far_dir  = computeRoiDirection(mask, roi_far, frame,
                                           cv::Scalar(0, 200, 200),
                                           g_last_cx_far, g_fork_bias_far,
                                           &far_cross);
            near_dir = computeRoiDirection(mask, roi_near, frame,
                                           cv::Scalar(0, 255, 0),
                                           g_last_cx_near, g_fork_bias_near,
                                           &near_cross);
        }

        cv::line(frame, cv::Point(W / 2, 0), cv::Point(W / 2, H),
                 cv::Scalar(255, 255, 0), 1);

        bool valid = have_real_line && (near_dir != INT_MIN);

        // ---------- ★ 第一阶段触发条件:视野中首次看不到黑线 ----------
        // 用整幅 mask 判断“视野中是否还有黑线”,而不是只看 near ROI。
        // 这样能避免黑线还在远处/边缘时被误判为入口触发。
        black_line_visible = have_real_line && !mask.empty() && (cv::countNonZero(mask) > 0);
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - t_start).count() / 1000.0;
        line_lost_trigger = false;
        if (mode == LineMode::TO_LINE_LOST && !g_line_lost_avoidance_triggered) {
            if (!black_line_visible) no_line_consec++;
            else                     no_line_consec = 0;

            if (no_line_consec >= LINE_LOST_TRIGGER_CONSEC) {
                g_line_lost_avoidance_triggered = true;
                line_lost_trigger = true;
            }
        }

        // ---------- ★ 弧形入口找线补救 (2026-07-05 重写) ----------
        //   旧版 bug (现场表现: "看不到黑线也不平移"):
        //     ① "看到线"用的是 black_line_visible = 整幅画面里有没有足够大的黑色块。
        //        下台阶后视野里几乎必然有黑色台阶体/平台侧面/狗影 → 第一帧就把
        //        arc_start_line_seen 置真 → 右移补救永远不会启动;
        //        而 near ROI 又锁不上真正的导引线 (valid=false) → 狗走 "NO VALID
        //        NEAR LINE - HOLD" 分支, Move(0,0,0) 原地罚站到看门狗超时。
        //     ② 即使补救启动, 也只是一次性右移 POST_STAIR_SHIFT_SEC 秒就放弃,
        //        不是"移到看到线为止"。
        //   新版:
        //     ① "看到线" = near ROI 真正锁上导引线 (valid) 且连续
        //        ARC_START_LINE_LOCK_FRAMES 帧, 画面里的黑色杂物不算;
        //     ② 补救 = 持续右移直到锁上线, 上限 ARC_START_SHIFT_MAX_SEC 兜底,
        //        右移期间带 yaw 保持防止走歪。
        bool arc_start_shift_override = false;
        if (mode == LineMode::PLATFORM_DETECT && !arc_start_shift_done) {
            if (valid) arc_start_line_lock_cnt++;
            else       arc_start_line_lock_cnt = 0;

            if (arc_start_line_lock_cnt >= ARC_START_LINE_LOCK_FRAMES) {
                arc_start_shift_done = true;
                if (arc_start_shifting) {
                    arc_start_shifting = false;
                    std::cout << "[弧形] 右移补救中锁上导引线 (连续 "
                              << ARC_START_LINE_LOCK_FRAMES
                              << " 帧 valid), 恢复正常巡线" << std::endl;
                } else {
                    std::cout << "[弧形] 启动即锁上导引线, 无需右移补救" << std::endl;
                }
            } else if (!arc_start_shifting && elapsed >= ARC_START_LINE_CHECK_SEC) {
                arc_start_shifting = true;
                arc_start_shift_t0 = std::chrono::steady_clock::now();
                arc_start_shift_yaw_hold = g_yaw_deg.load();
                std::cout << "[弧形] 启动 " << ARC_START_LINE_CHECK_SEC
                          << "s 内未锁上导引线, 持续右移找线 vy="
                          << POST_STAIR_SHIFT_VY << " (上限 "
                          << ARC_START_SHIFT_MAX_SEC << "s, yaw 保持 "
                          << arc_start_shift_yaw_hold << "°)" << std::endl;
            }

            if (arc_start_shifting) {
                double shift_el = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::steady_clock::now() - arc_start_shift_t0).count() / 1000.0;
                if (shift_el < ARC_START_SHIFT_MAX_SEC) {
                    arc_start_shift_override = true;
                } else {
                    arc_start_shifting = false;
                    arc_start_shift_done = true;
                    std::cout << "[弧形] 右移找线达到上限 " << ARC_START_SHIFT_MAX_SEC
                              << "s 仍未锁上线, 停止右移交回巡线逻辑" << std::endl;
                }
            }
        }

        if (arc_start_shift_override) {
            float shift_yaw_err = normalize_180(
                arc_start_shift_yaw_hold - g_yaw_deg.load());
            float shift_omega = clampf(0.02f * shift_yaw_err, -0.30f, 0.30f);
            sport.Move(0.0f, POST_STAIR_SHIFT_VY, shift_omega);
            prev_error = 0.0;
            cv::putText(frame, "ARC START NO LINE - SHIFT RIGHT UNTIL LOCK",
                        cv::Point(40, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.85,
                        cv::Scalar(0, 165, 255), 2);
        } else if (!valid) {
            sport.Move(0.0f, 0.0f, 0.0f);
            prev_error = 0.0;
            cv::putText(frame, "NO VALID NEAR LINE - HOLD",
                        cv::Point(40, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.85,
                        cv::Scalar(0, 0, 255), 2);
            // ★ 丢线保护已注释 (用户认为没啥用):
            //   - 原 HARD LOST (≥30帧丢线): 硬停 (检测模式则盲走)
            //   - 原 SOFT LOST (≥8帧丢线): 发 omega 原地搜索
            //   现在无 valid near 线时保持 0 速度, 下一帧若 valid 自动恢复 PD;
            //   若长期丢线, 后果由调用方的 mode 退出条件 (TIMED 计时 / DETECT 模式
            //   的 yaw+lidar / FOREVER 不退出) 自行处理。
            // lost_count++;
            // if (lost_count >= LOST_HARD_THRESHOLD) {
            //     bool detect_mode = (mode == LineMode::PLATFORM_DETECT
            //                      || mode == LineMode::THREE_TURN_DETECT
            //                      || mode == LineMode::DUAL_PLATFORM_DETECT);
            //     if (detect_mode) {
            //         sport.Move(LOST_BLIND_VX, 0.0f, 0.0f);
            //         cv::putText(frame, "HARD LOST - BLIND FWD",
            //                     cv::Point(40, 40),
            //                     cv::FONT_HERSHEY_SIMPLEX, 0.9,
            //                     cv::Scalar(0, 200, 255), 3);
            //     } else {
            //         softStop(sport, SOFTSTOP_SETTLE_SEC);
            //         cv::putText(frame, "HARD LOST - STOP", cv::Point(40, 40),
            //                     cv::FONT_HERSHEY_SIMPLEX, 1.0,
            //                     cv::Scalar(0, 0, 255), 3);
            //     }
            //     prev_error = 0.0;
            //     ResetTrajectory();
            // } else if (lost_count >= LOST_SOFT_THRESHOLD) {
            //     double recovery_yaw = (last_dir >= 0) ? SEARCH_YAW : -SEARCH_YAW;
            //     sport.Move(0.0, 0.0, -recovery_yaw);
            //     cv::putText(frame, "LOST - SEARCHING", cv::Point(40, 40),
            //                 cv::FONT_HERSHEY_SIMPLEX, 1.0,
            //                 cv::Scalar(0, 165, 255), 3);
            // }
        } else {
            lost_count = 0;
            last_dir   = near_dir;

            // ★ 穿越十字: 近 ROI 是对称横杠时, 改用远端 ROI (远处仍是直行竖线) 导向;
            //   远端也被横杠占住才维持上一帧 (≈直行)。对直角拐弯无害: 即便近端误判,
            //   远端看到的也是拐弯方向, 仍会正确转。
            double error;
            if (near_cross) {
                if (far_dir != INT_MIN && !far_cross) {
                    error = (double)far_dir;
                    cv::putText(frame, "CROSS BAR -> steer by FAR",
                                cv::Point(40, 70), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                                cv::Scalar(0, 165, 255), 2);
                } else {
                    error = prev_error;  // 远近都被横杠占住, 维持上一帧
                    cv::putText(frame, "CROSS BAR -> hold heading",
                                cv::Point(40, 70), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                                cv::Scalar(0, 165, 255), 2);
                }
            } else {
                error = (double)near_dir;
            }

            // PD 控制
            double d_error = error - prev_error;
            prev_error     = error;

            double Kp = g_kp_x1000 / 1000.0;
            double Kd = g_kd_x1000 / 1000.0;
            double rot_mult = (rotation_mult > 0.0) ? rotation_mult : 1.0;
            // ★ 转向倍率渐升 (第一跳落地保护): 前 rot_mult_ramp_sec 秒从 x1 线性升到目标值
            if (rot_mult_ramp_sec > 0.0 && elapsed < rot_mult_ramp_sec && rot_mult > 1.0) {
                rot_mult = 1.0 + (rot_mult - 1.0) * (elapsed / rot_mult_ramp_sec);
            }
            double rotation = (Kp * error + Kd * d_error) * rot_mult;
            double rotation_limit = ROTATION_LIMIT * rot_mult;
            if (rotation > rotation_limit)  rotation = rotation_limit;
            if (rotation < -rotation_limit) rotation = -rotation_limit;

            // 自适应速度
            double abs_err = std::abs(error);
            double speed_scale = 1.0 - (1.0 - MIN_SPEED_RATIO) *
                                 std::min(1.0, abs_err / SPEED_DECAY_PIVOT);
            // ★ THREE_TURN_DETECT 模式下, 进入 phase 2 (第二个 90° 已触发) 后速度减半,
            //   退出本函数(THREE_TURN_DONE)时局部变量回收, 自然恢复
            double tt_speed_mult = 1.0;
            if (mode == LineMode::THREE_TURN_DETECT && tt_phase >= 2) {
                tt_speed_mult = TT_PHASE2_SPEED_MULT;
            }
            double speed;
            if (fixed_speed > 0.0) {
                speed = fixed_speed;
                if (fixed_speed_after > 0.0
                    && fixed_speed_switch_sec >= 0.0
                    && elapsed >= fixed_speed_switch_sec) {
                    speed = fixed_speed_after;
                }
            } else {
                speed = MAX_SPEED * speed_scale * tt_speed_mult;
            }

            sport.Move(speed, 0.0, -rotation);

            // line.cpp 原有的逐帧控制信息 (画在 HUD 上方一行)
            char info[200];
            snprintf(info, sizeof(info),
                     "near=%d far=%d  vx=%.2f yaw=%.2f  Kp=%.3f Kd=%.3f  bias_n=%d",
                     near_dir, far_dir, speed, -rotation, Kp, Kd, g_fork_bias_near);
            cv::putText(frame, info, cv::Point(20, H - 45),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55,
                        cv::Scalar(255, 255, 255), 2);
        }

        // ---------- ★ 红圆检测: 先看到红圆, 再等红圆离开视野 (仅 FOREVER 模式) ----------
        if (!g_warn_action_done && mode == LineMode::FOREVER) {
            if (!red_ever_seen) {
                if (red_seen_this_frame) {
                    cv::putText(frame, "RED SEEN", cv::Point(40, 80),
                                cv::FONT_HERSHEY_SIMPLEX, 0.9,
                                cv::Scalar(0, 0, 255), 2);
                }
                if (red_seen_this_frame) red_seen_consec++;
                else                     red_seen_consec = 0;
                if (red_seen_consec >= RED_SEEN_CONFIRM_FRAMES) {
                    red_ever_seen = true;
                    red_lost_consec = 0;
                    std::cout << "\n[红圆] 已看到红色圆形, 开始等待红圆离开视野 (lost > "
                              << RED_LOST_TRIGGER_FRAMES << " 帧触发停车)" << std::endl;
                }
            } else {
                if (red_seen_this_frame) {
                    red_lost_consec = 0;
                    cv::putText(frame, "RED TRACK", cv::Point(40, 80),
                                cv::FONT_HERSHEY_SIMPLEX, 0.9,
                                cv::Scalar(0, 0, 255), 2);
                } else {
                    red_lost_consec++;
                    char red_lost_hud[64];
                    snprintf(red_lost_hud, sizeof(red_lost_hud), "RED LOST %d/%d",
                             red_lost_consec, RED_LOST_TRIGGER_FRAMES);
                    cv::putText(frame, red_lost_hud, cv::Point(40, 80),
                                cv::FONT_HERSHEY_SIMPLEX, 0.9,
                                cv::Scalar(0, 0, 255), 2);
                }

                if (red_lost_consec > RED_LOST_TRIGGER_FRAMES) {
                    std::cout << "[红圆] 已见过红圆, 且红圆连续消失 "
                              << red_lost_consec << " 帧 (> "
                              << RED_LOST_TRIGGER_FRAMES
                              << ") → 立即停车 → 执行动作 → 退出巡线" << std::endl;
                    return execRedActionSequence(sport, &frame, &cap);
                }
            }
        }

        // ---------- ★ TO_OBSTACLE: 白色横杆检测 → 触发前跳 ----------
        if (mode == LineMode::TO_OBSTACLE) {
            bool ob_trigger = false;
            const char* ob_why = "";
            int trig_row = (int)(H * g_ob_trigger_row_frac);  // ★ 九轮: 可被收尾前跳覆盖
            if (g_ob_detect_enabled && elapsed >= ob_window_min_sec && have_real_line) {
                int strip_cx = (g_last_cx_near >= 0) ? g_last_cx_near
                             : (g_last_cx_far  >= 0) ? g_last_cx_far
                                                     : (W / 2);
                // ★ 每帧画出扫描条带 (黄框, 顶到画面上方) 和触发行 (红线), 现场标定用
                {
                    int sx0 = std::max(0, strip_cx - OB_STRIP_HALF_W_PX);
                    int sx1 = std::min(W, strip_cx + OB_STRIP_HALF_W_PX);
                    int sy0 = (int)(H * OB_SCAN_TOP_FRAC);
                    int sy1 = (int)(H * OB_SCAN_BOTTOM_FRAC);
                    cv::rectangle(frame, cv::Point(sx0, sy0), cv::Point(sx1, sy1),
                                  cv::Scalar(0, 255, 255), 1);
                    cv::line(frame, cv::Point(sx0, trig_row), cv::Point(sx1, trig_row),
                             cv::Scalar(0, 0, 255), 2);
                    cv::putText(frame, "TRIG", cv::Point(sx1 + 4, trig_row + 5),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                }
                int gb = -1, gh = 0;
                bool found_gap = detectObstacleGap(mask, dbg_L, strip_cx, &frame, gb, gh);
                if (found_gap) {
                    ob_found_stable++;
                    ob_last_gap_bottom = gb;
                    char obhud[128];
                    snprintf(obhud, sizeof(obhud),
                             "OB gap_bottom=%d/%d h=%d stable=%d/%d",
                             gb, trig_row, gh, ob_found_stable, OB_STABLE_FRAMES);
                    cv::putText(frame, obhud, cv::Point(40, 110),
                                cv::FONT_HERSHEY_SIMPLEX, 0.7,
                                cv::Scalar(255, 0, 255), 2);
                    std::cout << "[障碍] 图案命中 gap_bottom=" << gb
                              << " (触发行=" << trig_row << ") h=" << gh
                              << " stable=" << ob_found_stable << "/"
                              << OB_STABLE_FRAMES << std::endl;
                    if (ob_found_stable >= OB_STABLE_FRAMES && gb >= trig_row) {
                        ob_trigger = true;
                        ob_why = "缺口下沿到达触发行";
                    }
                } else {
                    // 图案已确认过、且上次缺口已走到画面下半 → 横杆冲出近视野, 同样触发
                    if (ob_found_stable >= OB_STABLE_FRAMES
                        && ob_last_gap_bottom >= (int)(H * 0.45)) {
                        ob_trigger = true;
                        ob_why = "横杆冲出近视野 (图案已确认)";
                    } else {
                        ob_found_stable = 0;
                    }
                }
            }
            if (!ob_trigger && elapsed >= timer_sec) {
                ob_trigger = true;
                ob_why = g_ob_detect_enabled
                    ? "时间窗上限兜底 (窗口内未识别到横杆, 按计时触发)"
                    : "obdetect=0, 纯计时触发";
            }
            if (ob_trigger) {
                std::cout << "\n[障碍] 触发前跳: " << ob_why
                          << " (t=" << elapsed << "s, 窗口 [" << ob_window_min_sec
                          << ", " << timer_sec << "]s, gap_bottom="
                          << ob_last_gap_bottom << ", 触发行=" << trig_row << ")"
                          << std::endl;
                return LineResult::OBSTACLE_TRIGGER;  // ★ 不 softStop, doFrontJump 内先减速再停稳
            }
        }

        // 当前版本不再做“入口”文字/模板匹配;第一阶段只看黑线是否消失。

        // ---------- HUD ----------
        char hud[256];
        if (mode == LineMode::TIMED || mode == LineMode::TO_OBSTACLE) {
            snprintf(hud, sizeof(hud), "[%s] t=%.1f/%.1fs  near=%d far=%d",
                     mode_str, elapsed, timer_sec, near_dir, far_dir);
        } else if (mode == LineMode::TO_LINE_LOST) {
            snprintf(hud, sizeof(hud),
                     "[%s] lineVisible=%d lost=%d/%d  near=%d far=%d",
                     mode_str, black_line_visible ? 1 : 0,
                     no_line_consec, LINE_LOST_TRIGGER_CONSEC, near_dir, far_dir);
        } else {
            snprintf(hud, sizeof(hud), "[%s] near=%d far=%d", mode_str,
                     near_dir, far_dir);
        }
        cv::putText(frame, hud, cv::Point(20, H - 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

        // ★ 自适应阈值调试数 (标定时盯这四个值, 含义见使用手册)
        char adp[128];
        snprintf(adp, sizeof(adp), "ADP minL=%d floor=%d contrast=%d thr=%d%s",
                 g_dbg_minL, g_dbg_floorL, g_dbg_contrast, g_dbg_thr,
                 g_use_adaptive ? "" : " (OFF)");
        cv::putText(frame, adp, cv::Point(20, H - 70),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);

        if (g_gui_enabled) {
            cv::imshow("Original", frame);
            cv::imshow("Mask", mask);
            cv::imshow("L channel", dbg_L);
        }

        int key = guiWaitKey(1);
        if (key == 27) {                       // ESC: 统一中断 (仅 GUI 模式)
            g_running = false;
            softStop(sport, SOFTSTOP_SETTLE_SEC);
            return LineResult::ABORTED;
        }

        // ---------- 退出判据 ----------
        if (mode == LineMode::TO_LINE_LOST && line_lost_trigger) {
            std::cout << "\n[触发避障] 首次检测到视野中看不到黑线"
                      << " (lost=" << no_line_consec << "/" << LINE_LOST_TRIGGER_CONSEC
                      << ") → 退出巡线,进入避障" << std::endl;
            return LineResult::AVOIDANCE_TRIGGER; // ★ 不 softStop,保持运动无缝进避障
        }
        if (mode == LineMode::TIMED && elapsed >= timer_sec) {
            std::cout << "\n[巡线计时] " << timer_sec
                      << "s 到 → 退出巡线" << std::endl;
            return LineResult::TIMER_DONE; // ★ 不 softStop,由主流程决定下一阶段
        }

        // ---------- ★ FOREVER 红圈等待超时降级 ----------
        //   超时按"已到检测点"处理: 原地停车做动作, 置完成标志, 返回后进收尾。
        //   后面的双侧平台检测靠雷达, 物理位置对了照样触发, 放置/终点分保得住。
        if (mode == LineMode::FOREVER && !g_warn_action_done
            && elapsed >= g_red_timeout_sec) {
            std::cout << "\n[红圆超时] FOREVER 巡线 " << g_red_timeout_sec
                      << "s 未触发红圆 → 降级: 原地执行动作序列后进收尾"
                      << " (若正常行程超过此值, 用 redtimeout= 调大)" << std::endl;
            return execRedActionSequence(sport, &frame, &cap);
        }

        // ---------- ★ 检测模式看门狗: 超时返回 DETECT_TIMEOUT, 降级由主流程决定 ----------
        if (mode == LineMode::PLATFORM_DETECT && elapsed >= ARC_WATCHDOG_SEC) {
            std::cout << "\n[看门狗] 弧形→平台检测 " << ARC_WATCHDOG_SEC
                      << "s 未触发 → softStop, 返回 DETECT_TIMEOUT (主流程跳过第一抓取)"
                      << std::endl;
            softStop(sport, SOFTSTOP_SETTLE_SEC);
            return LineResult::DETECT_TIMEOUT;
        }
        if (mode == LineMode::THREE_TURN_DETECT && elapsed >= TT_WATCHDOG_SEC) {
            std::cout << "\n[看门狗] 三连转检测 " << TT_WATCHDOG_SEC
                      << "s 未完成 → softStop, 返回 DETECT_TIMEOUT (主流程直接进红圈等待)"
                      << std::endl;
            softStop(sport, SOFTSTOP_SETTLE_SEC);
            return LineResult::DETECT_TIMEOUT;
        }
        if (mode == LineMode::DUAL_PLATFORM_DETECT && elapsed >= DUAL_WATCHDOG_SEC) {
            std::cout << "\n[看门狗] 双侧平台检测 " << DUAL_WATCHDOG_SEC
                      << "s 未触发 → 按已触发降级 (不 softStop, 主流程接盲走)"
                      << std::endl;
            return LineResult::DETECT_TIMEOUT;
        }

        // ---------- DUAL_PLATFORM_DETECT 退出判据: 左右 lidar 同时近 + 连续帧 ----------
        //   ★ 触发即退出 (不 softStop, 保持运动), 触发后的 1s 盲走由主流程接 runBlindForward。
        if (mode == LineMode::DUAL_PLATFORM_DETECT) {
            float left_d  = g_left_dist.load();
            float right_d = g_right_dist.load();
            bool  left_ok  = left_d  < DUAL_PLAT_LIDAR_THRESH_M;
            bool  right_ok = right_d < DUAL_PLAT_LIDAR_THRESH_M;

            if (left_ok && right_ok) dual_plat_stable_cnt++;
            else                     dual_plat_stable_cnt = 0;

            std::cout << "[双侧] left=" << left_d
                      << "m right=" << right_d
                      << "m (th<" << DUAL_PLAT_LIDAR_THRESH_M << "m) "
                      << "left_ok=" << left_ok
                      << " right_ok=" << right_ok
                      << " stable=" << dual_plat_stable_cnt << "/"
                      << DUAL_PLAT_STABLE_FRAMES << std::endl;

            if (dual_plat_stable_cnt >= DUAL_PLAT_STABLE_FRAMES) {
                std::cout << "\n[双侧] 左右双侧检测到放置平台 (left="
                          << left_d << "m, right=" << right_d
                          << "m) → 退出巡线 (不 softStop), 主流程接 "
                          << DUAL_PLAT_POST_BLIND_SEC << "s 盲走" << std::endl;
                return LineResult::DUAL_PLATFORM_REACHED; // ★ 不 softStop, 主流程接盲走
            }
        }

        // ---------- 累计 yaw 跟踪 (PLATFORM_DETECT / THREE_TURN_DETECT 共用) ----------
        //   ★ 累计积分: 每帧 dyaw = wrap(yaw_now - yaw_prev),
        //     dyaw 单帧增量必然在 (-π, π) 内 (50Hz下不可能转半圈),
        //     wrap 之后无歧义。 累加得到 arc_cum_delta (无界、连续)。
        //     无需关心 yaw_now 本身的 ±π 跳变。
        if (mode == LineMode::PLATFORM_DETECT || mode == LineMode::THREE_TURN_DETECT) {
            float yaw_now = g_yaw_rad.load();
            float dyaw    = yaw_now - arc_yaw_prev;
            while (dyaw >  3.14159265f) dyaw -= 2.0f * 3.14159265f;
            while (dyaw < -3.14159265f) dyaw += 2.0f * 3.14159265f;
            arc_cum_delta += dyaw;
            arc_yaw_prev   = yaw_now;

            // ---------- PLATFORM_DETECT 退出: 全局 yaw 回环 + 左 lidar 近 ----------
            if (mode == LineMode::PLATFORM_DETECT) {
                float global_cum = g_global_yaw_cum_rad.load();
                float init_err   = normalize_180_rad(yaw_now - g_initial_yaw_rad.load());
                float left_d     = g_left_dist.load();
                bool  cum_ok     = g_initial_yaw_ready.load()
                                && (std::fabs(global_cum) > deg2rad(INITIAL_YAW_CUM_TRIGGER_DEG));
                bool  yaw_ok     = g_initial_yaw_ready.load()
                                && (std::fabs(init_err) < deg2rad(INITIAL_YAW_MATCH_TOL_DEG));
                bool  lidar_ok   = left_d < ARC_LIDAR_LEFT_THRESH_M;

                if (cum_ok && yaw_ok && lidar_ok) plat_stable_cnt++;
                else                              plat_stable_cnt = 0;

                std::cout << "[弧形] yaw=" << rad2deg(yaw_now)
                          << "° local_cum=" << rad2deg(arc_cum_delta)
                          << "° global_cum=" << rad2deg(global_cum)
                          << "° (need>|" << INITIAL_YAW_CUM_TRIGGER_DEG
                          << "°|) init_err=" << rad2deg(init_err)
                          << "° (tol<" << INITIAL_YAW_MATCH_TOL_DEG << "°)"
                          << "° left=" << left_d << "m"
                          << " cum_ok=" << cum_ok
                          << " yaw_ok=" << yaw_ok
                          << " lidar_ok=" << lidar_ok
                          << " stable=" << plat_stable_cnt << "/"
                          << ARC_STABLE_FRAMES << std::endl;

                if (plat_stable_cnt >= ARC_STABLE_FRAMES) {
                    std::cout << "\n[弧形] 到达抓取平台侧 (global_cum="
                              << rad2deg(global_cum) << "°, init_err="
                              << rad2deg(init_err) << "°, left_dist="
                              << left_d << "m)"
                              << (stop_on_exit ? " → softStop" : " → 不 softStop (外部续 Move)")
                              << std::endl;
                    if (stop_on_exit) softStop(sport, SOFTSTOP_SETTLE_SEC);
                    return LineResult::PLATFORM_REACHED;
                }
            }

            // ---------- THREE_TURN_DETECT 退出: 三相位状态机 ----------
            //   phase 0: 当前 yaw - 三连转初始 yaw ≥ +46° → 进 phase 1 (左转)
            //   phase 1: 当前 yaw - phase 0 触发 yaw ≤ -31° → 进 phase 2 (右转)
            //   phase 2: 当前 yaw - 三连转初始 yaw ≥ POST_PLAT_FINAL_THRESH_DEG 且全局累计 yaw > 302° → 停车
            //   当前 POST_PLAT_STABLE_FRAMES=1,角度条件命中一次就切换。
            if (mode == LineMode::THREE_TURN_DETECT) {
                float phase_delta = normalize_180_rad(yaw_now - tt_phase_ref_yaw_rad);
                float final_delta = normalize_180_rad(yaw_now - tt_initial_yaw_rad);
                float global_cum  = g_global_yaw_cum_rad.load();
                bool  global_cum_ok = g_initial_yaw_ready.load()
                                    && (std::fabs(global_cum) > deg2rad(INITIAL_YAW_CUM_TRIGGER_DEG));
                bool  phase_met;
                float target_show;
                float delta_show;
                const char* target_ref;
                if (tt_phase == 0) {
                    phase_met = (phase_delta >=  tt_phase0_thresh_rad);
                    target_show = +POST_PLAT_PHASE0_THRESH_DEG;
                    delta_show = rad2deg(phase_delta);
                    target_ref = "yaw - tt_initial";
                } else if (tt_phase == 1) {
                    phase_met = (phase_delta <= -tt_phase1_thresh_rad);
                    target_show = -POST_PLAT_PHASE1_THRESH_DEG;
                    delta_show = rad2deg(phase_delta);
                    target_ref = "yaw - phase0_yaw";
                } else { // phase 2: 和三连转初始 yaw 比 POST_PLAT_FINAL_THRESH_DEG,并叠加全局累计转角
                    bool local_final_ok = (final_delta >= tt_final_thresh_rad);
                    phase_met = local_final_ok && global_cum_ok;
                    target_show = +POST_PLAT_FINAL_THRESH_DEG;
                    delta_show = rad2deg(final_delta);
                    target_ref = "yaw - tt_initial + global cum";
                }

                // ---- ★ 三连转丢线保护 ----
                auto do_tt_line_lost_back = [&](const char* reason, double sec) {
                    if (tt_line_lost_nudges < TT_LINE_LOST_MAX_NUDGES) {
                        tt_line_lost_nudges++;
                        std::cout << "\n[三连转] ★ 丢线保护: " << reason
                                  << ", 持续 " << sec
                                  << "s, 后退一步重找线 ("
                                  << tt_line_lost_nudges << "/"
                                  << TT_LINE_LOST_MAX_NUDGES << ")" << std::endl;
                        auto t_bk = std::chrono::steady_clock::now();
                        while (g_running) {
                            double bel = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - t_bk).count() / 1000.0;
                            if (bel >= TT_LINE_LOST_BACK_MOVE_SEC) break;
                            sport.Move(TT_LINE_LOST_BACK_VX, 0.0f, 0.0f);
                            usleep(20 * 1000);
                        }
                        sport.Move(0.0f, 0.0f, 0.0f);
                        tt_line_lost_active = false;
                        tt_line_lost_since = std::chrono::steady_clock::now();
                        tt_phase2_stall_ref_delta_deg = -999.0f;
                        tt_phase2_stall_ref_time = std::chrono::steady_clock::now();
                        tt_stable_cnt = 0;
                    }
                };

                if (!valid) {
                    auto now_guard = std::chrono::steady_clock::now();
                    if (tt_phase < 2) {
                        if (!tt_line_lost_active) {
                            tt_line_lost_active = true;
                            tt_line_lost_since = now_guard;
                        }
                        double lost_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now_guard - tt_line_lost_since).count() / 1000.0;
                        if (lost_sec >= TT_LINE_LOST_BACK_SEC) {
                            do_tt_line_lost_back("phase0/1 near ROI 无有效线", lost_sec);
                        }
                    } else if (!phase_met) {
                        float fd_deg = rad2deg(final_delta);
                        bool near_target = (POST_PLAT_FINAL_THRESH_DEG - fd_deg)
                                           <= TT_PHASE2_NEAR_TARGET_DEG
                                        && fd_deg < POST_PLAT_FINAL_THRESH_DEG;
                        if (!near_target
                            || std::fabs(fd_deg - tt_phase2_stall_ref_delta_deg)
                                   > TT_PHASE2_STALL_DELTA_DEG) {
                            tt_phase2_stall_ref_delta_deg = fd_deg;
                            tt_phase2_stall_ref_time = now_guard;
                        } else {
                            double stall_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                                now_guard - tt_phase2_stall_ref_time).count() / 1000.0;
                            if (stall_sec >= TT_LINE_LOST_BACK_SEC) {
                                do_tt_line_lost_back("phase2 未达成且卡在最终目标附近无线", stall_sec);
                            }
                        }
                    } else {
                        tt_line_lost_active = false;
                        tt_line_lost_since = now_guard;
                        tt_phase2_stall_ref_delta_deg = -999.0f;
                        tt_phase2_stall_ref_time = now_guard;
                    }
                } else {
                    auto now_guard = std::chrono::steady_clock::now();
                    tt_line_lost_active = false;
                    tt_line_lost_since = now_guard;
                    tt_phase2_stall_ref_delta_deg = -999.0f;
                    tt_phase2_stall_ref_time = now_guard;
                }

                if (phase_met) tt_stable_cnt++;
                else           tt_stable_cnt = 0;

                std::cout << "[三连转] phase=" << tt_phase
                          << " yaw=" << rad2deg(yaw_now)
                          << "° ref_yaw=" << rad2deg(tt_phase_ref_yaw_rad)
                          << "° init_yaw=" << rad2deg(tt_initial_yaw_rad)
                          << "° cum=" << rad2deg(arc_cum_delta)
                          << "° phase_delta=" << rad2deg(phase_delta)
                          << "° final_delta=" << rad2deg(final_delta)
                          << "° (" << target_ref << " target=" << target_show << "°)"
                          << " delta_used=" << delta_show << "°"
                          << " global_cum=" << rad2deg(global_cum)
                          << "° global_ok=" << global_cum_ok
                          << " met=" << phase_met
                          << " stable=" << tt_stable_cnt << "/"
                          << POST_PLAT_STABLE_FRAMES << std::endl;

                if (tt_stable_cnt >= POST_PLAT_STABLE_FRAMES) {
                    tt_stable_cnt = 0;
                    if (tt_phase == 2) {
                        std::cout << "\n[三连转] 三相位全达成 (final_delta="
                                  << rad2deg(final_delta) << "° 严格 ≥ "
                                  << POST_PLAT_FINAL_THRESH_DEG
                                  << "°, global_cum=" << rad2deg(global_cum)
                                  << "°) → softStop, 中转平台前" << std::endl;
                        softStop(sport, SOFTSTOP_SETTLE_SEC);
                        return LineResult::THREE_TURN_DONE;
                    } else {
                        std::cout << "[三连转] phase " << tt_phase << " 达成 (phase_delta="
                                  << rad2deg(phase_delta) << "°), 进 phase "
                                  << (tt_phase + 1) << " (新基准 yaw="
                                  << rad2deg(yaw_now) << "°)" << std::endl;
                        tt_phase_ref_yaw_rad = yaw_now;
                        tt_phase++;
                    }
                }
            }
        }
    }

    // g_running 被外部置 false
    softStop(sport, SOFTSTOP_SETTLE_SEC);
    return LineResult::ABORTED;
}

// ============================================================
//  弧形 → 抓取平台停位 (wrapper)
//    输入: yaw_baseline_rad —— 进 runStairs 之前快照的 g_yaw_rad
//    内部: 调 runLineFollowing(PLATFORM_DETECT) 继续巡线, 检测到位后 softStop
// ============================================================
LineResult runArcToPlatform(unitree::robot::go2::SportClient& sport,
                            cv::VideoCapture& cap,
                            float yaw_baseline_rad,
                            bool stop_on_exit = true) {
    std::cout << "\n[弧形] 进入 弧形→抓取平台 模式, baseline yaw="
              << rad2deg(yaw_baseline_rad) << "°"
              << (stop_on_exit ? "" : " (退出不 softStop)") << std::endl;
    return runLineFollowing(sport, cap, LineMode::PLATFORM_DETECT, 0.0,
                            yaw_baseline_rad, stop_on_exit);
}

// ============================================================
//  三连转检测巡线 (wrapper)
//    输入: yaw_baseline_rad —— 进此阶段前快照的 g_yaw_rad (即平台停位时的 yaw)
//    内部: 调 runLineFollowing(THREE_TURN_DETECT) 继续巡线, 检测三相位完成后 softStop
// ============================================================
LineResult runThreeTurnDetect(unitree::robot::go2::SportClient& sport,
                              cv::VideoCapture& cap,
                              float yaw_baseline_rad) {
    std::cout << "\n[三连转] 进入 三连转检测 模式, baseline yaw="
              << rad2deg(yaw_baseline_rad) << "°" << std::endl;
    return runLineFollowing(sport, cap, LineMode::THREE_TURN_DETECT, 0.0,
                            yaw_baseline_rad);
}

// ============================================================
//  双侧放置平台检测巡线 (wrapper)
//    继续巡线, 直到左右雷达同时检测到放置平台 (< 阈值) 连续若干帧后 softStop
//    用于: 红圆动作完成 → 进 FINISH_LINE1 之间的双侧平台对位
// ============================================================
LineResult runUntilDualPlatform(unitree::robot::go2::SportClient& sport,
                                cv::VideoCapture& cap) {
    std::cout << "\n[双侧] 进入 双侧平台检测 模式" << std::endl;
    return runLineFollowing(sport, cap, LineMode::DUAL_PLATFORM_DETECT, 0.0);
}


// ============================================================
//  第一阶段盲走:不再依赖视觉,保持 StaticWalk 直行指定时间
//    返回 true = 正常走完; false = ESC / Ctrl+C 中断
// ============================================================
bool runBlindForward(unitree::robot::go2::SportClient& sport,
                     double duration_sec,
                     double vx,
                     float target_yaw_deg = NAN) {
    // ★ 盲走加 yaw 保持: 锁定进入时刻的朝向, 步态漂移由 P 控制纠正
    //   (原版 Move(vx,0,0) 无航向闭环, 盲走 2s 进避障入口姿态会歪)
    const float BLIND_YAW_KP        = 0.02f;
    const float BLIND_YAW_OMEGA_MAX = 0.30f;
    float yaw_hold = std::isnan(target_yaw_deg) ? g_yaw_deg.load()
                                                 : target_yaw_deg;
    std::cout << "[盲走] 直行 " << duration_sec
              << "s, vx=" << vx << ", yaw 锁定 " << yaw_hold << "°"
              << (std::isnan(target_yaw_deg) ? " (当前朝向)" : " (指定基准)")
              << std::endl;

    auto t_start = std::chrono::steady_clock::now();
    while (g_running) {
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - t_start).count() / 1000.0;
        if (elapsed >= duration_sec) break;

        float yaw_err = normalize_180(yaw_hold - g_yaw_deg.load());
        float omega   = clampf(BLIND_YAW_KP * yaw_err,
                               -BLIND_YAW_OMEGA_MAX, BLIND_YAW_OMEGA_MAX);
        sport.Move((float)vx, 0.0f, omega);

        int key = guiWaitKey(1);
        if (key == 27) {
            g_running = false;
            softStop(sport, SOFTSTOP_SETTLE_SEC);
            return false;
        }
        usleep(20 * 1000);  // 50 Hz
    }

    if (!g_running) {
        sport.Move(0.0f, 0.0f, 0.0f);
        std::cout << "[盲走] 收到中断，未正常走完" << std::endl;
        return false;
    }
    std::cout << "[盲走] 完成,进入避障" << std::endl;
    return true;  // ★ 不 softStop,保持运动无缝进避障
}

// ============================================================
// ============================================================
//  模块 2: 避障  (源自 avoid_END.cpp,逻辑原样保留)
//    重名消解: g_yaw→g_yaw_deg, YAW_TOLERANCE_DEG→AV_YAW_TOL,
//             TURN_OMEGA→AV_TURN_OMEGA, WATCHDOG_SEC→AV_WATCHDOG
// ============================================================
// ============================================================

// ---------- 路径序列 (现场可改) ----------
// ★ 八轮重构 (用户要求, 治转弯摆头): 90°+90° 合并成【一个 180° 目标】——
//   航向表里不存在中间 waypoint, 结构上就不可能有中途判定/停顿。
//   旧五段表保留, AV_MERGE_TURNS=false 一键回退。
//   合并的两个 180° 必须用显式方向表定向: 0→180 的角度误差恰为 ±180°,
//   正负号纯是噪声, 按误差符号定向 50% 概率反向扎进挡板。
//   转1=左(+1), 转2=右(-1); 出口转 0→90 无歧义, 按误差符号。
const bool  AV_MERGE_TURNS = false;  // ★ 八轮二: 合并弧实测过快过宽 = 错误尝试 →
                                     //   回旧五段表; 摆头改"转前平移缓冲"治 (LAT_PRE2/4)
const float VX_TURN_MERGED = 0.08f;   // 合并弧统一线速度: 半径 r=vx/0.45,
                                      //   180° 横向位移≈2r≈0.36m;
                                      //   内侧蹭板→加大, 出弧离板远/撞外侧→减小
const std::vector<float> SEG_YAW_LEGACY = { 0.0f, 90.0f, 180.0f, 90.0f, 0.0f };
const std::vector<float> SEG_YAW_MERGED = { 0.0f, 180.0f, 0.0f };
const std::vector<float>& SEGMENT_YAW_DEG =
    AV_MERGE_TURNS ? SEG_YAW_MERGED : SEG_YAW_LEGACY;
// 合并模式方向表: 第 N 个转弯的强制初始方向 (+1=左, -1=右, 0=按误差符号)
inline int mergedTurnDir(int upcoming_turn) {
    if (!AV_MERGE_TURNS) return 0;
    if (upcoming_turn == 1) return +1;   // 0→180: 左
    if (upcoming_turn == 2) return -1;   // 180→0: 右
    return 0;                            // 出口转 (→90): 按误差符号
}
const float EXIT_YAW_DEG = 90.0f;

// ---------- 控制参数 (现场可调) ----------
const float TRIGGER_DIST     = 0.20f;   // 前方激光 <20cm 触发下一步
const float TRIGGER_DIST_2   = 0.20f;   // 第 2 个转弯触发距离
const float TRIGGER_DIST_4   = 0.20f;   // 第 4 个转弯触发距离
const float EXIT_OPEN_DIST   = 2.0f;
const float AV_YAW_TOL       = 1.0f;    // (原 YAW_TOLERANCE_DEG,代码使用 <1°)

const float BASE_VX          = 0.13f;
const float SLOW_VX          = 0.06f;
const float NEAR_FRONT_DIST  = 0.40f;   // 前方激光 <40cm 开始减速
const float AV_TURN_OMEGA    = 0.45f;   // (原 TURN_OMEGA)
const float MAX_OMEGA        = 0.50f;
const float KP_YAW           = 0.025f;
const float VX_TURN          = 0.06f;
const float VX_TURN_FAST     = 0.08f;   // 第 3 个 90° 转弯专用
const float VX_TURN_4        = 0.08f;   // ★ 六轮 (0706 第四次后腿踢板): 第 4 个转弯专用,
                                        //   转弯半径略增, 后腿离内侧挡板远一点
// (★ 八轮: AV_MERGE_TURNS / VX_TURN_MERGED 已随重构上移到"路径序列"块)

const float WALL_AVOID_DIST  = 0.22f;
const float WALL_AVOID_OMEGA = 0.15f;
// ★ 2026-07-06 (治"弯角前左拧右拧→前激光斜视→触发迟→径直撞板", 五连测第3次-20):
//   wall_corr=±0.15 对 KP_YAW=0.025 稳态可拽偏 ~6° 航向; 弯角前右墙必然变近 →
//   左拧 → 激光看进斜对角, TRIGGER_DIST 迟迟不满足。距弯角此距离内锁死贴墙修正。
const float AV_CORNER_LOCK_DIST  = 0.70f; // 前距小于此值 → 只信航向保持, 不做贴墙修正
const float AV_TURN_RELATCH_DEG  = 30.0f; // TURN 态方向闩锁: 剩余角大于此值期间绝不换向
// ★ 2026-07-06 二轮 (治"转弯处轻微摆头"): FORWARD 的小幅 P 保持 → TURN 的 ±0.45
//   bang-bang 是瞬间跳变, 入弯那一下狗头会甩。入弯前 AV_TURN_RAMP_SEC 内角速度
//   从 30% 线性升到 100%, 平滑接入。
const double AV_TURN_RAMP_SEC    = 0.4;
const float  AV_TURN_RAMP_MIN    = 0.30f; // 斜坡起点比例 (太低转不动)

const int   AV_WATCHDOG      = 90;      // (原 WATCHDOG_SEC)

// 三段平移参数
const int   LAT_PRE_TURN_IDX = 1;
const float LAT_PRE_VY       = +0.05f;
const float LAT_PRE_TIME_SEC = 1.0f;
// ★ 八轮二 (用户观察: 凡是转弯前有平移段的弯都不摆头 —— 平移段让步态从前进
//   动量里静下来, 入弯准静态): 转2/转4 也加转前平移缓冲, 参数用户给定
const float LAT_PRE2_VX       = -0.07f;  // 转2前: 向后
const float LAT_PRE2_VY       = 0.0f;
const float LAT_PRE2_TIME_SEC = 1.0f;
const float LAT_PRE4_VX       = -0.05f;  // 转4前: 向后
const float LAT_PRE4_VY       = -0.05f;  // 转4前第二个后退改回右后退 (vy<0=右)
const float LAT_PRE4_TIME_SEC = 1.0f;
const double LAT_PRE_RETREAT_REARM_SEC = 0.25; // 转2/4后退结束后, 丢掉旧前距帧再重新布防触发

const int   LAT_MID_TURN_IDX = AV_MERGE_TURNS ? 1 : 2;  // ★ 八轮: 合并模式=第1个180°后
const float LAT_MID_VY       = -0.05f;
const float LAT_MID_TIME_SEC = 2.0f;

const int   LAT_END_TURN_IDX = AV_MERGE_TURNS ? 2 : 4;  // ★ 八轮: 合并模式=第2个180°后
const float LAT_END_VY       = +0.05f;
const float LAT_END_TIME_SEC = 1.5f;

// === 出口前右移: 最后一个转弯后向右平移一段再结束避障 (独立参数, 可改) ===
const float EXIT_SHIFT_VY       = -0.05f;  // 向右平移速度 (m/s, -右 / +左)
const float EXIT_SHIFT_TIME_SEC = 2.0f;    // 平移时长 (秒)

enum class State {
    FORWARD, LATERAL_PRE_TURN, TURN, LATERAL_POST_TURN,
    EXIT_TURN, EXIT_SHIFT, EXIT_WALK, DONE
};
const char* stateName(State s) {
    switch (s) {
        case State::FORWARD:           return "FORWARD";
        case State::LATERAL_PRE_TURN:  return "LAT_PRE";
        case State::TURN:              return "TURN";
        case State::LATERAL_POST_TURN: return "LAT_POST";
        case State::EXIT_TURN:         return "EXIT_TURN";
        case State::EXIT_SHIFT:        return "EXIT_SHIFT";
        case State::EXIT_WALK:         return "EXIT_WALK";
        case State::DONE:              return "DONE";
    }
    return "?";
}

// ============================================================
//  避障主循环 (源自 avoid main 的状态机部分, 改成函数)
//    ★ 入口不再断言步态 —— 前一段巡线已保证 StaticWalk 且持续 Move,
//      此处无缝衔接 "丢线→避障",不留指令空窗 (handoff: 空窗会切回 AI 步态)
//  返回 true = 正常完成 (DONE);  false = 超时 / 中断
// ============================================================
bool runAvoidance(unitree::robot::go2::SportClient& sport) {
    std::cout << "[步态] 沿用 StaticWalk (常规步态),无缝进入避障" << std::endl;

    float entry_yaw = g_initial_yaw_ready.load()
        ? g_initial_yaw_deg.load()
        : g_yaw_deg.load();
    std::cout << "\n========== 避障路径计划 ==========" << std::endl;
    std::cout << "避障基准 entry_yaw = " << entry_yaw
              << " deg (" << (g_initial_yaw_ready.load() ? "程序初始yaw" : "当前yaw")
              << ")" << std::endl;
    std::cout << "段数: " << SEGMENT_YAW_DEG.size() << std::endl;
    for (size_t i = 0; i < SEGMENT_YAW_DEG.size(); ++i) {
        std::cout << "  段 " << (i + 1) << ": 目标 yaw = "
                  << normalize_180(entry_yaw + SEGMENT_YAW_DEG[i])
                  << " (offset +" << SEGMENT_YAW_DEG[i] << ")" << std::endl;
    }
    std::cout << "出口朝向 (offset +" << EXIT_YAW_DEG << "): "
              << normalize_180(entry_yaw + EXIT_YAW_DEG) << std::endl;
    std::cout << "==================================" << std::endl;

    State state         = State::FORWARD;
    int   seg_idx       = 0;
    auto  seg_start     = std::chrono::steady_clock::now();
    auto  start_all     = std::chrono::steady_clock::now();
    auto  lateral_start = std::chrono::steady_clock::now();
    float lateral_vx_active   = 0.0f;   // ★ 八轮二: 转前平移可带 vx (向后缓冲)
    float lateral_vy_active   = 0.0f;
    float lateral_time_active = 0.0f;
    int   loop_cnt = 0;
    int   turn_dir_latch = 0;   // ★ TURN 态方向闩锁: 0=未锁, +1=左转, -1=右转
    auto  t_turn_entry   = std::chrono::steady_clock::now();  // ★ 本次转弯起点 (角速度斜坡用)
    std::vector<bool> pre_turn_retreat_done(SEGMENT_YAW_DEG.size() + 2, false);
    bool  lateral_recheck_front_after_retreat = false;
    int   lateral_pending_turn = 0;
    auto  front_trigger_rearm_until = std::chrono::steady_clock::now();

    while (g_running && state != State::DONE) {
        loop_cnt++;

        auto now = std::chrono::steady_clock::now();
        int total_sec = std::chrono::duration_cast<std::chrono::seconds>(
                            now - start_all).count();
        if (total_sec > AV_WATCHDOG) {
            std::cout << "\n[避障超时] " << AV_WATCHDOG
                      << " 秒未完成,强制停止" << std::endl;
            break;
        }

        float cur_target_offset = (seg_idx < (int)SEGMENT_YAW_DEG.size())
                                      ? SEGMENT_YAW_DEG[seg_idx]
                                      : EXIT_YAW_DEG;
        float target_yaw = normalize_180(entry_yaw + cur_target_offset);
        float yaw_err    = normalize_180(target_yaw - g_yaw_deg);

        if (state == State::FORWARD) {
            float omega_yaw = clampf(KP_YAW * yaw_err, -MAX_OMEGA, MAX_OMEGA);
            float wall_corr = 0.0f;
            float front_dist = frontLaserDistFresh();
            // ★ 2026-07-06 (治五连测第3次"弯角前左拧右拧→激光斜视→触发迟→撞板"):
            //   弯角附近 (前距 < AV_CORNER_LOCK_DIST) 锁死贴墙修正 —— 此处右墙
            //   必然渐近, wall_corr 会把航向拽偏 ~6°, 前激光看进斜对角导致
            //   TRIGGER_DIST 迟迟不满足。弯角前只信航向保持。
            if (front_dist >= AV_CORNER_LOCK_DIST) {
                if (g_right_dist < WALL_AVOID_DIST)      wall_corr = +WALL_AVOID_OMEGA;
                else if (g_left_dist < WALL_AVOID_DIST)  wall_corr = -WALL_AVOID_OMEGA;
            }
            float omega = clampf(omega_yaw + wall_corr, -MAX_OMEGA, MAX_OMEGA);
            float vx    = (front_dist < NEAR_FRONT_DIST) ? SLOW_VX : BASE_VX;

            sport.Move(vx, 0.0f, omega);

            // 第 2/4 个转弯可单独调触发距离; 其余维持 TRIGGER_DIST。
            int upcoming_turn = seg_idx + 1;
            float trig_dist_now = TRIGGER_DIST;
            if (!AV_MERGE_TURNS && upcoming_turn == 2) trig_dist_now = TRIGGER_DIST_2;
            if (!AV_MERGE_TURNS && upcoming_turn == 4) trig_dist_now = TRIGGER_DIST_4;
            bool front_trigger_armed = (now >= front_trigger_rearm_until);
            if (front_trigger_armed && front_dist < trig_dist_now) {
                if (upcoming_turn == LAT_PRE_TURN_IDX) {
                    std::cout << "\n[段 " << (seg_idx + 1)
                              << " 结束] 前激光 " << front_dist << " < " << trig_dist_now
                              << ",切换 LATERAL_PRE_TURN (" << LAT_PRE_TIME_SEC
                              << "s @ vy=" << LAT_PRE_VY << ")" << std::endl;
                    lateral_vx_active   = 0.0f;
                    lateral_vy_active   = LAT_PRE_VY;
                    lateral_time_active = LAT_PRE_TIME_SEC;
                    lateral_recheck_front_after_retreat = false;
                    lateral_pending_turn = upcoming_turn;
                    lateral_start       = std::chrono::steady_clock::now();
                    state = State::LATERAL_PRE_TURN;
                } else if ((upcoming_turn == 2 || upcoming_turn == 4)
                           && !pre_turn_retreat_done[upcoming_turn]) {
                    // ★ 转2/转4 转前后退缓冲只做一次。后退结束后不直接转,
                    //   回到 FORWARD 重新等前激光再次触发, 防止沿用后退前的旧触发结果。
                    lateral_vx_active   = (upcoming_turn == 2) ? LAT_PRE2_VX
                                                               : LAT_PRE4_VX;
                    lateral_vy_active   = (upcoming_turn == 2) ? LAT_PRE2_VY
                                                               : LAT_PRE4_VY;
                    lateral_time_active = (upcoming_turn == 2) ? LAT_PRE2_TIME_SEC
                                                               : LAT_PRE4_TIME_SEC;
                    std::cout << "\n[段 " << (seg_idx + 1)
                              << " 结束] 前激光 " << front_dist << " < " << trig_dist_now
                              << ",切换 LATERAL_PRE_TURN (转" << upcoming_turn
                              << "前缓冲 " << lateral_time_active << "s @ vx="
                              << lateral_vx_active << " vy=" << lateral_vy_active
                              << ")" << std::endl;
                    lateral_recheck_front_after_retreat = true;
                    lateral_pending_turn = upcoming_turn;
                    lateral_start       = std::chrono::steady_clock::now();
                    state = State::LATERAL_PRE_TURN;
                } else {
                    std::cout << "\n[段 " << (seg_idx + 1)
                              << " 结束] 前激光 " << front_dist << " < " << trig_dist_now
                              << ",切换 TURN (圆弧)" << std::endl;
                    state = State::TURN;
                }
            }
        }
        else if (state == State::LATERAL_PRE_TURN) {
            float omega = clampf(KP_YAW * yaw_err, -MAX_OMEGA, MAX_OMEGA);
            sport.Move(lateral_vx_active, lateral_vy_active, omega);

            float lat_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    now - lateral_start).count() / 1000.0f;
            if (lat_elapsed >= lateral_time_active) {
                if (lateral_recheck_front_after_retreat) {
                    if (lateral_pending_turn >= 0
                        && lateral_pending_turn < (int)pre_turn_retreat_done.size()) {
                        pre_turn_retreat_done[lateral_pending_turn] = true;
                    }
                    front_trigger_rearm_until =
                        now + std::chrono::milliseconds((int)(LAT_PRE_RETREAT_REARM_SEC * 1000.0));
                    std::cout << "[转前后退完成] 转" << lateral_pending_turn
                              << " 不直接转弯, 回到 FORWARD 重新等待前激光触发 ("
                              << LAT_PRE_RETREAT_REARM_SEC << "s 后开始判定)"
                              << "  L=" << g_left_dist.load()
                              << " R=" << g_right_dist.load() << std::endl;
                    lateral_recheck_front_after_retreat = false;
                    lateral_pending_turn = 0;
                    state = State::FORWARD;
                    seg_start = std::chrono::steady_clock::now();
                } else {
                    std::cout << "[侧移完成] 切换 TURN (圆弧)  L=" << g_left_dist.load()
                              << " R=" << g_right_dist.load() << std::endl;
                    state = State::TURN;
                }
            }
        }
        else if (state == State::TURN) {
            int next_idx = seg_idx + 1;
            float next_target_offset;
            if (next_idx < (int)SEGMENT_YAW_DEG.size()) {
                next_target_offset = SEGMENT_YAW_DEG[next_idx];
            } else {
                next_target_offset = EXIT_YAW_DEG;
            }
            float next_target_yaw = normalize_180(entry_yaw + next_target_offset);
            float next_yaw_err    = normalize_180(next_target_yaw - g_yaw_deg);

            if (std::abs(next_yaw_err) < AV_YAW_TOL) {
                turn_dir_latch = 0;   // ★ 转弯完成, 解除方向闩锁
                seg_idx++;
                int just_finished_turn = seg_idx;
                int next_upcoming_turn = seg_idx + 1;
                if ((next_upcoming_turn == 2 || next_upcoming_turn == 4)
                    && next_upcoming_turn < (int)pre_turn_retreat_done.size()) {
                    pre_turn_retreat_done[next_upcoming_turn] = false;
                }
                if (seg_idx >= (int)SEGMENT_YAW_DEG.size()) {
                    std::cout << "[转向完成] 最后一个转弯已完成,先向右平移 ("
                              << EXIT_SHIFT_TIME_SEC << "s @ vy=" << EXIT_SHIFT_VY
                              << ")" << std::endl;
                    lateral_vy_active   = EXIT_SHIFT_VY;
                    lateral_time_active = EXIT_SHIFT_TIME_SEC;
                    lateral_start       = std::chrono::steady_clock::now();
                    state = State::EXIT_SHIFT;
                } else if (just_finished_turn == LAT_MID_TURN_IDX) {
                    std::cout << "[转向完成] 第 " << just_finished_turn
                              << " 次转弯,触发 LATERAL_POST_TURN ("
                              << LAT_MID_TIME_SEC << "s @ vy=" << LAT_MID_VY << ")"
                              << std::endl;
                    lateral_vy_active   = LAT_MID_VY;
                    lateral_time_active = LAT_MID_TIME_SEC;
                    lateral_start       = std::chrono::steady_clock::now();
                    state = State::LATERAL_POST_TURN;
                } else if (just_finished_turn == LAT_END_TURN_IDX) {
                    std::cout << "[转向完成] 第 " << just_finished_turn
                              << " 次转弯,触发 LATERAL_POST_TURN ("
                              << LAT_END_TIME_SEC << "s @ vy=" << LAT_END_VY << ")"
                              << std::endl;
                    lateral_vy_active   = LAT_END_VY;
                    lateral_time_active = LAT_END_TIME_SEC;
                    lateral_start       = std::chrono::steady_clock::now();
                    state = State::LATERAL_POST_TURN;
                } else {
                    // (AV_MERGE_TURNS=false 时转 1/3 走这里: 进 FORWARD, 等激光再转下一个;
                    //  八轮已默认合并连转, 此分支现仅剩非合并模式使用)
                    std::cout << "[转向完成] 进入段 " << (seg_idx + 1)
                              << " (前进, 待前方太近再转下一个 90°)" << std::endl;
                    state = State::FORWARD;
                    seg_start = std::chrono::steady_clock::now();
                }
            } else {
                // ★ 2026-07-06 方向闩锁: 转弯开始定方向, 剩余角 > AV_TURN_RELATCH_DEG
                //   期间绝不换向 (防 yaw 噪声/毛刺把 bang-bang 转向来回翻);
                //   进入收尾小角度后才允许按误差符号做细修。
                if (turn_dir_latch == 0) {
                    // ★ 八轮: 合并 180° 转弯必须查方向表 —— 0→180 误差恰为 ±180°,
                    //   符号纯是噪声, 按符号定向 50% 概率反向扎进挡板
                    int forced = mergedTurnDir(seg_idx + 1);
                    turn_dir_latch = (forced != 0)
                                         ? forced
                                         : ((next_yaw_err > 0) ? +1 : -1);
                    t_turn_entry   = now;   // ★ 记录入弯时刻 (角速度斜坡)
                }
                int dir = (std::abs(next_yaw_err) > AV_TURN_RELATCH_DEG)
                              ? turn_dir_latch
                              : ((next_yaw_err > 0) ? +1 : -1);
                // ★ 入弯角速度斜坡: 30%→100% 线性升, 压掉 FORWARD→TURN 切换甩头
                double turn_el = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     now - t_turn_entry).count() / 1000.0;
                float ramp = (turn_el >= AV_TURN_RAMP_SEC)
                                 ? 1.0f
                                 : (AV_TURN_RAMP_MIN
                                    + (1.0f - AV_TURN_RAMP_MIN)
                                          * (float)(turn_el / AV_TURN_RAMP_SEC));
                float omega = dir * AV_TURN_OMEGA * ramp;
                float vy    = (dir > 0) ? 0.02f : -0.01f;
                int   turning_idx = seg_idx + 1;
                float vx_turn = (turning_idx == 4) ? VX_TURN_4
                              : (turning_idx == 3) ? VX_TURN_FAST : VX_TURN;
                if (AV_MERGE_TURNS) vx_turn = VX_TURN_MERGED;   // ★ 八轮: 合并弧统一速度
                sport.Move(vx_turn, vy, omega);
            }

            yaw_err    = next_yaw_err;
            target_yaw = next_target_yaw;
        }
        else if (state == State::LATERAL_POST_TURN) {
            float omega = clampf(KP_YAW * yaw_err, -MAX_OMEGA, MAX_OMEGA);
            sport.Move(0.0f, lateral_vy_active, omega);

            float lat_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    now - lateral_start).count() / 1000.0f;
            if (lat_elapsed >= lateral_time_active) {
                std::cout << "[侧移完成] 进入段 " << (seg_idx + 1)
                          << " (左距=" << g_left_dist.load()
                          << " 右距=" << g_right_dist.load() << ")" << std::endl;
                state = State::FORWARD;
                seg_start = std::chrono::steady_clock::now();
            }
        }
        else if (state == State::EXIT_SHIFT) {
            // ★ 出口前向右平移 (-vy = 右), P 控制保持出口朝向, 平移结束即结束避障
            float omega = clampf(KP_YAW * yaw_err, -MAX_OMEGA, MAX_OMEGA);
            sport.Move(0.0f, lateral_vy_active, omega);

            float lat_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    now - lateral_start).count() / 1000.0f;
            if (lat_elapsed >= lateral_time_active) {
                std::cout << "[出口右移完成] 避障流程结束" << std::endl;
                state = State::DONE;
            }
        }
        else if (state == State::EXIT_WALK) {
            float omega = clampf(KP_YAW * yaw_err, -MAX_OMEGA, MAX_OMEGA);
            sport.Move(BASE_VX, 0.0f, omega);

            float front_dist = frontLaserDistFresh();
            if (front_dist > EXIT_OPEN_DIST) {
                std::cout << "\n[出口] 前方开阔 (" << front_dist
                          << "m),完成避障" << std::endl;
                sport.StopMove();
                state = State::DONE;
            }
        }

        if (loop_cnt % 5 == 0) {
            printf("[%-9s seg=%d/%zu  t=%2ds]  yaw=%+7.1f tgt=%+7.1f err=%+6.1f"
                   "  L=%.2f F=%.2f R=%.2f\n",
                   stateName(state), seg_idx + 1, SEGMENT_YAW_DEG.size(),
                   total_sec,
                   g_yaw_deg.load(), target_yaw, yaw_err,
                   g_left_dist.load(), frontLaserDistFresh(), g_right_dist.load());
            fflush(stdout);
        }

        int key = guiWaitKey(1);      // 刷新 GUI 窗口 + 支持 ESC 统一中断 (nogui 下仅 Ctrl+C)
        if (key == 27) { g_running = false; break; }

        usleep(100 * 1000);   // 10 Hz
    }

    sport.StopMove();
    usleep(500 * 1000);

    auto end = std::chrono::steady_clock::now();
    int elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                      end - start_all).count();
    std::cout << "\n========== 避障结束 ==========" << std::endl;
    std::cout << "最终状态:    " << stateName(state) << std::endl;
    std::cout << "完成段数:    " << seg_idx << "/" << SEGMENT_YAW_DEG.size()
              << std::endl;
    std::cout << "总耗时:      " << elapsed << " s" << std::endl;
    std::cout << "最终 yaw:    " << g_yaw_deg.load() << " deg (相对 entry: "
              << normalize_180(g_yaw_deg.load() - entry_yaw) << ")" << std::endl;
    std::cout << "==============================" << std::endl;

    return (state == State::DONE);
}

// ============================================================
// ============================================================
//  模块 3: 爬台阶  (9=正爬，10=退开转180°后倒爬)
//    流程: UP → SUMMIT_STOP(含越沿短后退复核) → SUMMIT_TURN
//          → PRE_DOWN_STOP(★八轮:0s) → DOWN → EXIT
//    重名消解: g_yaw→g_yaw_rad, YAW_TOLERANCE_DEG→ST_YAW_TOL,
//             TURN_OMEGA→ST_TURN_OMEGA, WATCHDOG_SEC→ST_WATCHDOG
//    g_init_x/y 改为 runStairs 内部局部变量; 不再用 foot_force / 转向策略
// ============================================================
// ============================================================

// ---------- 速度 (AI 步态最低 vx 实测 0.20-0.25) ----------
const float VX_UP   = 0.35f;
const int   ST_UP_FAST_CLIMBS = 1;     // 第1运动阶段=3.5s冲锋；第2阶段连续补完
// ★ 国赛硬海绵五次验证版:
//   保留已经能起步的 3.5s @0.35 冲锋；冲锋后不再停1.6s再反复重启，
//   而是继续同一条0.35指令最多1.4s。只有冲锋完成后才开放到顶候选。
const double ST_UP_ASSAULT_SEC = 3.5;  // 冲锋段时长 (0.35×3.5s ≈ 1.23m 名义)
const double ST_UP_CONTINUE_SEC = 1.4; // 冲锋后连续爬上限；不是低速蠕动
const float VX_UP_LATE        = 0.35f;
const float VX_DOWN = 0.35f;

// 倒爬分支: 台阶前先退开约0.25m给180°转身留空间；转完后这段距离并入
// 第一次连续倒爬，不在台阶脚下再次停车。主爬结束后必须先零速复核2s；
// 倒爬不再做负速度补爬，稳定卡住时也允许以正vx受控右转90°并接DOWN。
const float  ST_REVERSE_PRETURN_BACK_VX = -0.20f;
const double ST_REVERSE_PRETURN_BACK_SEC = 1.25;
const float  ST_REVERSE_TURN_ANGLE_DEG   = 90.0f;
const float  ST_REVERSE_TURN_VX          = +0.10f;
const double ST_REVERSE_TURN_RAMP_SEC    = 0.30;
const float  ST_REVERSE_UP_DIST_MAX_M    = 1.35f;
const float  ST_REVERSE_STUCK_REL_MIN_DEG = -30.0f;
const float  ST_REVERSE_STUCK_ROLL_OFFSET_MAX_DEG = 8.0f;
const float  ST_REVERSE_TURN_ROLL_ABORT_DEG = 18.0f;
const int    ST_REVERSE_TURN_ROLL_ABORT_FRAMES = 10;

// ---------- 顶部转向 ----------
const float TURN_ANGLE_DEG  = 100.0f;
const float ST_TURN_OMEGA   = 1.57f;   // 原地转角速度 rad/s (原 TURN_OMEGA)
const float TURN_VX         = 0.10f;   // 恢复原策略：整个转身过程持续带0.10前进
// ★ 2026-07-09 十三轮: 转身欠转前馈 —— 0708 五连跑转身完成误差全部同向欠转
//   +4.8/+6.1/+6.3/+6.4/+6.5°(7° 容差第一次进窗即退出, 系统性停早 ~6°)。
//   完成判定的目标角向转动方向多压 ST_TURN_FEEDFWD_DEG, 实际停位≈真目标±1°。
//   注意: 只作用于 SUMMIT_TURN 的"转到位"判定; DOWN 的直线保持仍用真目标角,
//   下楼方向不受此前馈影响。
const float ST_TURN_FEEDFWD_DEG = 5.0f;
const float ST_YAW_TOL      = 7.0f;    // ★ 转到位容差 (原 1.0; 甩腿快转 1.57rad/s 在 50Hz
                                        //   下每帧走 1.8°, 1° 窗必被跨过 → 顶上来回扭到
                                        //   10s 超时才下。放宽到 7° 让第一次到达就干净退出,
                                        //   甩腿动作本身完全不变)

// ---------- 朝向保持 (UP / DOWN 走直线) ----------
const float YAW_KP        = 0.020f;
const float MAX_OMEGA_FWD = 0.25f;

// ---------- ★ 国赛硬海绵: UP 到顶检测 (实测簇 + STOP 段滚动窗) ----------
// 目标由“四脚全上平顶”改为安全的20分停位: 前后足约差一级时即可准备转身。
// 一层差实测 rel≈-8.5°, 两层差约-18~-24°, 因此以 -14° 作单侧分界。
// 里程不证明到顶，仅作为过长推进的故障上界；历史回升也只打印提示。
const float PITCH_RECOVERY_DEG       = 13.0f;
const float ST_SLOPE_ARM_DELTA_DEG   = 18.0f;
const int   ST_SLOPE_ARM_FRAMES      = 10;

const float ST_TOP_REL_MIN_DEG       = -14.0f; // rel≥此值才进入目标停位簇
const float ST_TOP_OVERRUN_DELTA_DEG = +8.0f;
const double ST_MOVING_TOP_HOLD_SEC  = 0.25;   // 连续爬中进入目标簇需保持的真实时间
const int    ST_MOVING_TOP_MIN_SAMPLES = 6;   // 且必须来自新IMU帧

const float ST_TOP_DEEP_EVIDENCE_DEG = -20.0f; // 本次UP必须确实经历过深坡态
const int   ST_TOP_MIN_MOVE_PHASES   = 2;      // 已完成冲锋并进入连续补完阶段
const float ST_UP_DIST_MAX_M         = 1.10f;  // 里程只作异常推进上界

const double ST_TOP_VERIFY_WINDOW_SEC = 0.5;
const float ST_TOP_VERIFY_RANGE_DEG   = 4.0f;
const int   ST_TOP_VERIFY_MIN_SAMPLES = 12;
const float ST_TOP_VERIFY_ROLL_OFFSET_DEG = 6.0f;
const float ST_TOP_VERIFY_ROLL_RANGE_DEG  = 4.0f;

const float  ST_OVERRUN_BACK_VX      = 0.30f;
const double ST_OVERRUN_BACK_SEC     = 0.4;
const int    ST_OVERRUN_BACK_MAX     = 2;

// 静止复核失败后不用低速蠕行, 重新给一个完整、能启动的爬步脉冲。
const int    ST_MAX_RELAUNCHES    = 2;
const float  ST_RELAUNCH_VX_FIRST = 0.35f;
const float  ST_RELAUNCH_VX_SECOND= 0.40f;
const double ST_RELAUNCH_SEC      = 0.7;

// ---------- DOWN 到底检测 (回平判据) ----------
const float DOWN_DESCEND_CONFIRM_DEG = 10.0f; // pitch 至少正过此值才算下过台阶
const float DOWN_FLAT_TOL_DEG        = 5.0f;  // |pitch| 小于此值算回到平地
const float DOWN_STUCK_POS_PITCH_DEG = 0.0f;  // pitch 为正时才启用下台阶卡住检测
const double DOWN_STUCK_CHECK_SEC    = 1.0;   // 正 pitch 持续多久检测一次是否几乎没动
const float DOWN_STUCK_MOVE_THRESH   = 0.05f; // 1s 内平面位移小于此值视为几乎没动
const float DOWN_STUCK_VX_STEP       = 0.05f; // 下台阶卡住时每次上推速度
const float DOWN_STUCK_VX_MAX        = 0.45f; // 下台阶速度上限

// ---------- 到顶后处理 (零速静止) ----------
const float SUMMIT_STOP_SEC  = 2.0f;   // 零速静止时长
const float PRE_DOWN_STOP_SEC = 0.0f;  // 转身完成后、下楼梯前零速停顿
                                       // (★ 八轮: 用户实测没用, 2.0→0 = 直接下)

// 常规“确认后固定回退”已删除；只有正倾越沿候选使用上面的限次短后退。

// ---------- 阶段超时 ----------
const int   UP_MAX_SEC   = 25;  // 到时安全失败; 绝不能把半坡超时解释成“已到顶”
const int   TURN_MAX_SEC = 10;
const int   ST_TURN_POSE_FAIL_FRAMES = 10; // 正爬转身中重新持续跌回两级差姿态则刹停
const int   DOWN_MIN_SEC = 3;
const int   DOWN_MAX_SEC = 7;
const int   ST_WATCHDOG  = 90;        // 总看门狗 (原 WATCHDOG_SEC)
const double ST_STATE_STALE_SEC = 0.30; // SportModeState 超过此时长无新帧则安全停

enum class Phase {
    UP, SUMMIT_STOP, SUMMIT_TURN, PRE_DOWN_STOP, DOWN, EXIT, DONE
};
const char* phaseName(Phase p) {
    switch (p) {
        case Phase::UP:           return "UP         ";
        case Phase::SUMMIT_STOP:  return "SUMMIT_STOP";
        case Phase::SUMMIT_TURN:  return "SUMMIT_TURN";
        case Phase::PRE_DOWN_STOP:return "PRE_DOWN";
        case Phase::DOWN:         return "DOWN       ";
        case Phase::EXIT:         return "EXIT       ";
        case Phase::DONE:         return "DONE       ";
    }
    return "?";
}

enum class TurnDir { LEFT, RIGHT };
enum class StairClimbMode { FORWARD = 9, REVERSE = 10 };
const char* stairClimbModeName(StairClimbMode mode) {
    return mode == StairClimbMode::REVERSE ? "REVERSE(10)" : "FORWARD(9)";
}

// ============================================================
//  爬台阶主循环  v13 「起步冲刺 + 连续变速爬转 + 带符号 roll 闸门」 CONTINUOUS ARC-CLIMB  (2026-08-11)
//
//  ============ v13 相对 v12 的改动 (基于 1423/1430 两次实跑) ============
//
//  【好消息】v12 的 roll 闸门确实解决了掉腿: 1423/1430 两次 ★后腿悬空都是 0 次
//    (对照 2053 坏跑是 3.60s)。方向对了。
//
//  【但闸门刹错了方向】把 roll 残差按符号拆开看四次跑:
//        跑次        内侧最大  内侧>6°累计 | 外侧最大  外侧>6°累计 | 后腿悬空
//        2047(好)      3.7°     0.00s     |  10.1°     0.61s     |  0.50s
//        2053(坏)     15.4°     2.82s     |   8.6°     0.43s     |  3.60s
//        1423(冲过)    4.1°     0.00s     |  10.1°     1.34s     |  0 次
//        1430(正常)    8.6°     0.73s     |   7.7°     0.29s     |  0 次
//    只有【内侧】残差能区分好坏 (2053 独一档); 外侧残差在好跑次里同样有 10.1°,
//    根本不是故障信号。v12 用 |残差| 把外侧也算了进去。
//
//  ① ★闸门改带符号, 只对"向转弯内侧歪"刹车
//        内侧量 = -turn_sign · (rel_roll - roll_geo)     // 左转: 左侧沉为正
//        gate   = clamp(1 - (max(0,内侧量) - 6)/(12 - 6), 0, 1)
//     回放: 2053 的 2.82s 保护原样保留; 1423 的 1.34s、2047 的 0.61s 误刹全部消失。
//
//  ② ★这同时修掉了"冲过一点点"
//     闸门只收转速不收前进 ⇒ 误刹 2 秒 = 转身晚完成 2 秒 = 这 2 秒继续往前走:
//        1423 转角满时 dist=1.067m rel=+32.9° (已陡着头冲下远端)
//        1430 转角满时 dist=0.795m rel=+15.6°
//     多走了 27cm, 而 1423 的内侧残差全程只有 4.1°, 腿一点问题没有 —— 纯白刹。
//
//  ③ 二值"越沿减速"换成连续下坡收油
//     v12 是 rel≥+12° 一刀切 ×0.5 (再被下限托住, 实际几乎没减), 而冲过那一段
//     rel 从 +12 一路涨到 +32.9°, 减速力度却全程不变。改成 rel 由 0° 到 +20°
//     把 vx 连续收到 0.15 —— 越往下栽收得越狠。SA_EDGE_VX_MIN 那个补丁一并作废。
//
//  ④ 总转角 90° → 100° (出口面相邻本身是 90°, 多的 10° 是左偏余量, 右边是悬崖)
//
//  ⑤ 修 bug: [roll 刹车] 日志里"几何应有"打成了恒等于残差本身的表达式, 现在打真值。
//
//  ============ 以下为 v12 相对 v11 的改动 (仍然成立) ============
//
//  【复盘 2047(好) vs 2053(坏)】两次同一份代码, 结果天差地别:
//        2047: roll 全程 ≤10.4°, 左后腿最长只虚 0.50s, 6.0s 转满 90°,
//              航向误差 0.96°, 12.3s 走完 —— 历史最快
//        2053: roll 到 -18.2°, 左后腿(ff[2]) 连续悬空 3.60s, 撞 12s 兜底,
//              航向误差 8.13°, 18.2s
//    ★ 差别整个都在左右上。而 v10/v11 的转速只看 rel_pitch, 也就是
//      【只看前后, 不看左右】—— 缺的就是这一路反馈。
//    ★ 另外这两次连同 1928 一起确认了足力数组映射: 掉队的一直是 ff[2] = 左后腿,
//      正是左转时内侧那条腿。最早那套"左转把内侧后脚往下坡方向拉"的几何推论成立。
//
//  ① ★roll 闸门 (核心): 转速再乘一个"身子有没有歪"的系数
//        roll_geo = turn_sign · min(0, rel_pitch) · tan(已转角度)   // 几何应有的 roll
//        res      = |rel_roll − roll_geo|
//        gate     = clamp(1 − (res−6)/(12−6), 0, 1)
//        转速     = (8 + 31·u) · gate
//     必须扣几何分量: 机体在坡上偏航本来就会产生 roll (深跨级偏航 30° 自带约 17°),
//     不扣的话闸门会把自己锁死、90° 根本转不完。
//     日志回放 (只统计真正转向那一段):
//        2047 |res| 最大 9.2°, >10° 累计 0.00s → 闸门几乎不介入
//        2053 |res| 最大 14.3°, >8° 累计 2.64s, >12° 累计 0.34s → 被强力刹住
//        且 2053 残差首次破 8° 在 5.10s, 左后腿真正塌掉在 6.00s —— 早 0.9 秒预警
//
//  ② ★删掉"后腿悬空脱困"的动作, 只保留检测和日志
//     2053 实测该脱困触发 3 次, 而 ff[2] 从 6.0s 到 8.9s 一直是 0 —— 完全没救回来。
//     根因: 它是固定 0.6s, 时间一到腿还悬着就照样接着以 39°/s 拧。
//     roll 闸门是它的严格加强版 (只要身子还歪着就一直刹住), 留两套只会互相打架。
//     足力信号本身很有价值 (指认了是左后腿、悬空多久), 所以检测保留, 只是不驱动动作。
//
//  ③ 越沿减速加下限 SA_EDGE_VX_MIN=0.20
//     2053 里 SA_VX_CLIMB_TOP=0.25 再 ×0.5 = 0.125, 卡住那 3 秒 dist 从 0.66
//     只挪到 0.70 —— 推力低到脱不了困。
//
//  ============ 以下为 v11 相对 v10 的改动 (仍然成立) ============
//
//  【背景】v10 的转向是从【坡态锁存】才开始的, 而实测锁存时刻是
//      1928=1.78s  1753=1.91s  1759=2.04s  1740=2.17s
//    ——全都晚于 1.5s。也就是说原本就已经有一段 1.8~2.2s 的纯直行,
//    新加的 1.5s 冲刺整个包在里面, 不会推迟任何转向。
//
//  【真正的改动只有一条】把这 1.5s 的推力从 0.28 提到 0.40。
//    起步段 rel≈0 ⇒ u=1 ⇒ vx 被调制压到 SA_VX_CLIMB_TOP(0.28),
//    而实测【前 1.4 秒只推进 0.17~0.19m】, 是全程最慢的一段 ——
//    狗正在够第一级立面, 最费劲, 偏偏拿到的推力最小。这里补上。
//
//  【冲刺优先】提速之后坡态有可能提前到 1.5s 以内锁存; 此时仍然把冲刺跑满再开转,
//    代价最多 0.2~0.3s, 换起步段绝对干净、完全确定。打滑加力在冲刺段照常生效。
//
//  【参数回退】SA_VX_CLIMB_DEEP/TOP 从现场临时改的 0.40/0.30 退回 0.35/0.28
//    —— 这两个是历史上验证过能稳定起步爬升的值; 0.40 只出现在冲刺段。
//    深跨级是四脚最受约束的时候, 那里加推力有可能反而增加打滑。
//
//  ============ 以下为 v10 相对 v9 的改动 (核心, 仍然成立) ============
//
//  【为什么推倒重来】v9 之前是三段式: 冲锋(预转 12.3°/s) → 顶部停顿 → 弧线(39°/s)。
//    1928 那次日志把病因钉死了:
//        弧线段开始         t=6.15s
//        某后足失去承力     t=6.40s   (ff ≤ 8, 之后连续悬空 3.30 秒)
//    ★ 腿是在角速度阶跃之后 0.25 秒失去承力的。而在此之前, 狗顶着 20° 预转、
//      以 10°/s 慢转爬了 4 秒多, 那条腿只在 4.80~5.50 虚了 0.7s, 没出事。
//    如果病因是"偏航张角把两只后脚拉开", 它应该在预转转到位(3.86s)之后就发作,
//    而不是老实等到弧线开始。真正的触发点是【角速度从 12.3°/s 跳到 33.3°/s
//    这个 3 倍阶跃】: 步态正在坡上一级一级找落脚点, 转速突然翻三倍,
//    那条最吃亏的腿的落脚点被瞬间挪走, 就够不着了。
//    ⇒ 病在接缝上, 那就把接缝去掉。
//
//  ① ★取消 ASSAULT / SETTLE / ARC 三段, 合并成单一 CLIMB 阶段
//     从坡态锁存到转身完成是一个连续动作, 中间没有任何模式切换、没有阶跃。
//
//  ② ★角速度由 rel_pitch 连续调制 (核心)
//        u = clamp( (rel - REL_LO) / (REL_HI - REL_LO), 0, 1 )
//        ω = ω_slow + (ω_fast - ω_slow) * u
//     - 深跨级时 (rel ≈ -26°, 后脚还低两级) → ω = 8°/s, 慢慢转,
//       给那条吃亏的腿留出重新找落脚点的时间
//     - 身体接近水平时 (rel ≥ -8°, 后脚跟上来了) → ω = 39°/s, 全速甩完剩余角度
//     - 中间连续过渡, 一阶连续, 没有任何跳变
//     ★ 为什么不用"固定角速度跑完全程": 锁存→登顶的时长跑次间差近一倍
//       (1753=1.96s, 1928=3.68s)。按快的定, 慢的跑次会在半坡上就转到 60~70°,
//       那时后脚还在中间踏面上, 落脚点错位最大, 恰恰是最危险的时候;
//       按慢的定, 快的跑次转不完。转速必须由"爬到哪儿了"决定, 不能由时间决定
//       —— 这和当初把定时器换成状态判据是同一个道理。
//
//  ③ ★取消顶部停顿
//     数据早就说明它没用: 1928 那 0.6 秒里 roll 只从 -13.08° 变到 -12.51°,
//     零速指令下步态根本不迈步, 腿自然上不来, 纯白花 0.6 秒还往前多蹭了 1.5cm。
//
//  ④ ★vx 也随 u 反向taper: 0.35 (爬升中) → 0.28 (顶部快转时)
//     ★ 注意这里和讨论时我说的方向相反, 是复核了"腿到底什么时候失效"之后改的:
//       1928 那条腿是在 rel≈0 (即 u≈1, 已接近水平) 时失效的, 不是在深跨级时。
//       所以要在【转得快的那一段】给步态更多循环次数 ⇒ 那一段 vx 要小。
//       深跨级段本来就转得慢, 干扰小, 保持 0.35 的爬升推力不动它。
//
//  ⑤ ★新增足力脱困判据 (纯位移判据抓不到的那种卡滞)
//     1928 那 3.3 秒悬空期间, 弧线每秒位移 0.036~0.045m, 而旧阈值是 1.0s/0.03m,
//     差 1~1.5cm 就是没触发, 保护机制眼睁睁看着。现在两套并行:
//       - 骑棱卡死 (位移判据): 1.0s 位移 < 0.03m → ω=0, vx=0.40 直冲 0.6s 脱困
//       - 后腿悬空 (足力判据): 任一后足 ff ≤ 8 持续 0.8s 且 1s 位移 < 0.06m
//                              → ω=0, vx=0.15 慢行 0.6s, 给它时间把腿放下来找踏面
//     ★ 两者响应相反是故意的: 骑在棱上要猛推才能脱开, 后腿悬空猛推只会让前身
//       更往下栽, 要的是慢下来让步态有机会重新落脚。
//     ★ 后足取 ff[2]/ff[3]: 常见的两种排布 [FR,FL,RR,RL] 和 [FL,FR,RL,RR]
//       都把后腿一对放在下标 2/3, 所以不依赖左右映射, 两只都查。
//
//  ⑥ 半坡转角上限保险: rel ≤ -20° (深跨级) 期间累计转角不允许超过 35°,
//     防止极慢的跑次在半坡上越转越多。
//
//  ⑦ 转角指令改为【积分式】而不是时间斜坡: turn_cmd += ω(u)·dt。
//     好处是脱困时"冻结斜坡"变成"这一帧不积分", 不用再维护 arc_frozen_sec。
//     目标仍是绝对角度 entry_yaw + SA_TURN_DEG, 所以狗转过头了 P 环会自己收回来。
//
//  ⑧ 原来那套开转判据 (最深回升 + 绝对门槛 + 防抖) 不再控制任何东西,
//     降级成日志事件 [登顶判定], 只为了继续积累"登顶时 dist 是多少"的数据。
//
//  几何前提 (2026-08 现场实测, 改参数前先确认这些没变):
//    单级高 15cm / 踏面深 20cm  → 坡度 arctan(15/20)=36.9°, 三级总高 45cm
//    顶面 50x50, 二级 70x70 (可踩的是外圈 10cm 环 x2 边)
//    足端矩形长 ≈ 39cm (外接圆直径 48.4cm) ⇒ 前后脚天然跨约两级踏面
//    ★ 只有两个【相邻】侧面是台阶 (A=进入面, B=出口面, 拐角处踏面连续同高),
//      另两个相邻侧面是 15/30/45cm 直墙 = 悬崖。
//    ★ 左转 90° 上A面进、B面出 ⇒ 爬升中【左边是台阶(安全), 右边是悬崖】。
//      所有余量往左给, 右侧一点都不留。弧线转弯天然把狗向左推离悬崖。
//
//  ★ 现场只需要调这四个旋钮 (其余先别动):
//      SA_TURN_RATE_SLOW_DPS —— 深跨级时转多慢 (腿跟不上就调小)
//      SA_TURN_RATE_FAST_DPS —— 到顶后转多快
//      SA_TURN_MOD_REL_LO/HI —— 从慢到快的过渡区间 (调制曲线的两端)
//      SA_VX_CLIMB_TOP       —— 顶部快转时的推进速度 (腿跟不上就调小)
//  ★ 一次只动一个变量; 每跑先看日志里 [转身完成] 的用时和 [脱困] 次数,
//    再看 u 的变化是否平滑 —— u 突变说明 rel 噪声大, 该调滤波而不是调转速。
//
//  ============ 以下为历史版本要点 (仍然成立, 供追溯) ============
//  v6: 冲锋上限 3.5→5.5s; 开转判据换成"最深回升法"; 弧线卡死脱困; DOWN 用相对 pitch
//  v7: 冲锋段预转; 弧线从已预转角度接续; 越沿只减 vx 不减 omega
//  v8: 重新引入绝对门槛 rel ≥ -16°(治触发过早); 总转角 100°→90°; 日志加不缠绕 Y0
//  v9: 预转 20°→30° / 12.3°/s; 弧线 33.3→39°/s; 相应放开两处角速度限幅
//  返回 false (main 会零速 hold 等人工) 只剩三种真危险:
//      ① |roll| 持续超限 (疑似侧翻)  ② IMU 出 NaN/Inf  ③ DDS 断流 >2s
//    其余 (转不到位、卡死、超时、总看门狗) 全部返回 true 照常往下走
//    —— 台阶最多 30 分, 后面还有 110 分, 不能用 110 赌 30。
// ============================================================

// ---------------- 速度 ----------------
const float  SA_VX_CLIMB_DEEP = 0.35f;  // 深跨级段推进速度 (u=0, 已验证能起步爬升)
const float  SA_VX_CLIMB_TOP  = 0.25f;  // 顶部快转段推进速度 (u=1, 慢一点=每单位转角步态循环更多)
const float  SA_VX_DOWN       = 0.35f;  // 下台阶速度

// ---------------- ★v11 新增: 起步冲刺 (纯直行, 不转) ----------------
//   【为什么加】实测前 1.4 秒只推进 0.17~0.19m, 是全程最慢的一段 —— 狗正在够
//   第一级立面, 最费劲。而这一段 rel≈0 ⇒ u=1 ⇒ vx 被压到 SA_VX_CLIMB_TOP(0.28),
//   恰恰是全程推力最小的时候。这里固定给一段大推力直冲, 把起步这一下做实。
//   【和转向的关系】v10 的转向本来就是从坡态锁存才开始的, 而实测锁存全都在
//   1.78~2.17s, 已经晚于 1.5s。所以这一段冲刺整个包在原有的直行段里,
//   不会推迟任何转向 —— 唯一的改变就是这 1.5s 的推力从 0.28 提到 0.40。
//   【冲刺优先】提速后锁存有可能提前到 1.5s 以内; 此时仍然把冲刺跑满再开转,
//   代价最多 0.2~0.3s, 换起步段绝对干净、完全确定。
const double SA_LAUNCH_SEC    = 2.5;    // 起步冲刺时长
const float  SA_LAUNCH_VX     = 0.40f;  // 起步冲刺推进速度 (仅此段, 不影响后面的调制)

// ---------------- 坡态锁存 ----------------
const float  SA_SLOPE_ARM_DEG    = 15.0f; // rel <= -此值
const int    SA_SLOPE_ARM_FRAMES = 8;     // 连续 fresh 帧数
const double SA_NOSLOPE_TURN_SEC = 7.0;   // 到点仍未锁上坡态 -> 认为没爬上台阶, 强制按 u=1 开转

// ---------------- ★v10 核心: 角速度随 rel 连续调制 ----------------
const float  SA_TURN_DEG            = 100.0f;// ★v13 90→100 入口→出口总转角 (左正), 绝对目标
                                             //   出口面B与进入面A相邻本身是 90°, 多的 10° 是左偏余量
                                             //   (左边是B面台阶=安全, 右边是悬崖, 余量全往左给)
const float  SA_TURN_RATE_SLOW_DPS  = 8.0f;  // u=0 (深跨级) 时的转速
const float  SA_TURN_RATE_FAST_DPS  = 39.0f; // u=1 (接近水平) 时的转速
const float  SA_TURN_MOD_REL_LO     = -26.0f;// rel ≤ 此值 -> u=0 (稳态爬升实测 -25~-29°)
const float  SA_TURN_MOD_REL_HI     = -8.0f; // rel ≥ 此值 -> u=1 (登顶实测 -8° 附近)
const int    SA_REL_FILT_N          = 5;     // rel 滑动平均帧数 (只用于调制, 不改日志里的原始值)
const float  SA_TURN_MAX_LAG_DEG    = 8.0f;  // 实际航向落后指令超此值 -> 这一帧不积分, 等它跟上
//   ★v14 新增: 深跨级确认门 —— 在本次跑真正见过一次深跨级之前, u 一律锁 0。
//   【为什么】rel 在爬升中是"浅—深—浅", 所以 rel 浅有两种完全相反的含义:
//        ① 快到顶了 (该快转)      ② 压根还没爬上去 (最该慢转)
//   起步冲刺是拿定时 (2.5s) 去跨过这个歧义区的, 但"定时器不能用来判断爬到
//   哪儿了"这条早就验证过了 —— 冲刺跑满时爬到哪儿, 完全取决于起步打不打滑。
//   0811 三次跑正好把这个洞踩穿了:
//        跑次   冲刺结束 rel/dist    开转 u / 转速    登顶时累计转角   净 dy
//        1538   -34.2° / 0.377m     0    /  8.0°/s      22.8°        -0.26m
//        1545   -28.9° / 0.398m     0    /  8.0°/s      24.3°        -0.38m
//        1552   -18.0° / 0.231m    0.44  / 21.7°/s   ★ 42.9°      ★ -0.90m
//   1552 起步就打滑 (1s 只走 4cm), 2.5s 到点时人还骑在第一级立面上, rel 只有
//   -18° 被读成"接近顶部", 直接给 21.7°/s。到 3.4s 才真进深跨级, 这 0.9 秒白
//   转了约 12°; 登顶时已经转了 42.9° (好跑次 23°), 机体在半坡上就严重偏航,
//   按几何 (偏航 θ 时靠后那只后脚多退 0.14·sinθ) 左后脚落点整个偏掉,
//   到顶面远端边沿时够不到下一级 —— 这才是"左后腿没上去"的起点。
//   【判据】只要本次跑的 min_rel 曾经 ≤ 此门槛, 就认为深跨级已确认, u 解锁。
//   ★ 对好跑次是零影响: 1538/1545 开转时 rel 已是 -34/-29, u 本来就是 0。
//   【兜底】万一坡度浅到永远够不着 -24° (或 pitch 通道异常), 到点强制解锁,
//   否则 u 卡在 0 = 8°/s, 100° 要 12.5s, 会撞 SA_CLIMB_MAX_SEC。
const float  SA_DEEP_ARM_REL_DEG    = -24.0f;// min_rel 到过此值 -> 认为已确实进入深跨级
const double SA_DEEP_ARM_MAX_SEC    = 2.0;   // 冲刺结束后再等这么久仍没到 -> 强制解锁
//   ★v12 新增: roll 闸门 —— 转速再乘一个"身子有没有歪"的系数
//   v10/v11 的转速只看 rel_pitch, 也就是【只看前后, 不看左右】。而 2047(好) 与
//   2053(坏) 的差别整个都在左右上:
//        2047: roll 全程 ≤10.4°, 左后腿最长只虚 0.50s, 6.0s 转满, 12.3s 走完
//        2053: roll 到 -18.2°, 左后腿连续悬空 3.60s, 撞 12s 兜底, 航向差 8.13°
//   ★ 必须扣掉几何应有的 roll: 机体在坡上偏航本来就会产生 roll
//     (深跨级偏航 30° 就有约 17°), 不扣的话闸门会把自己锁死、90° 转不完。
//        roll_geo = turn_sign · min(0, rel_pitch) · tan(已转角度)
//        res      = |rel_roll − roll_geo|
//     rel_pitch>0 (已在下坡) 时机体已大致对齐新坡的下降方向, 几何分量按 0 算。
//   日志回放 (只统计真正转向的那一段):
//        2047 |res| 最大 9.2°, >10° 累计 0.00s  → 闸门几乎不介入
//        2053 |res| 最大 14.3°, >10° 累计 0.68s, >8° 累计 2.64s
//        且 2053 残差首次破 8° 在 t=5.10s, 而左后腿真正塌掉在 6.00s —— 早 0.9 秒
//   ★v13 改为【带符号】: 只对"向转弯内侧歪"刹车, 向外侧歪一律不管。
//   1423/1430 两次实跑把 v12 的问题暴露得很干净 —— 按符号拆开残差:
//        跑次        内侧最大  内侧>6°累计 | 外侧最大  外侧>6°累计 | 后腿悬空
//        2047(好)      3.7°     0.00s     |  10.1°     0.61s     |  0.50s
//        2053(坏)     15.4°     2.82s     |   8.6°     0.43s     |  3.60s
//        1423(冲过)    4.1°     0.00s     |  10.1°     1.34s     |  0 次
//        1430(正常)    8.6°     0.73s     |   7.7°     0.29s     |  0 次
//   ⇒ 只有内侧残差能区分好坏 (2053 独一档); 外侧残差在好跑次里同样有 10.1°,
//     它根本不是故障信号。v12 用 |残差| 把外侧也算进去, 结果 1423 那 2.05 秒
//     刹车几乎全是外侧误刹 (内侧累计 0.00s)。
//   ⇒ 而闸门只收转速不收前进, 误刹 2 秒 = 转身晚完成 2 秒 = 这 2 秒继续往前走
//     = 冲过头: 1423 转角满时 dist=1.067m/rel=+32.9° (已陡着头冲下远端),
//               1430 转角满时 dist=0.795m/rel=+15.6°, 多走了 27cm。
//   改带符号后回放: 2053 的 2.82s 保护原样保留, 1423/2047 的误刹全部消失。
const float  SA_ROLL_GATE_SOFT_DEG  = 6.0f;  // 内侧残差超此值开始收转速
const float  SA_ROLL_GATE_HARD_DEG  = 12.0f; // 内侧残差到此值完全停转 (只停转, 不停走)
const float  SA_ROLL_GEO_CAP_DEG    = 60.0f; // 算几何分量时 tan 的角度上限 (防止发散)
//   ★v15 【删除】SA_DEEP_REL_DEG / SA_DEEP_TURN_CAP_DEG (深跨级封顶)
//   它的门槛 -20° 整个落在真正危险的区间之外, 0811 五次跑一次都没触发过。
//   由下面的【登顶前转角闩锁封顶】取代 —— 同一个意图, 门槛挪到对的地方。
//
//   ★v15 新增: 登顶前转角闩锁封顶
//   【1636 的病历】relF 在 -15° 悬了将近一秒, 期间机体仍以 17cm/s 在走 ——
//   现场确认: 它是在二级面(70×70 的外圈 10cm 环)上斜着横移, 不是往上蹬。
//   而 turn_cmd += rate·dt 是按【时间】积分的, 爬升进度停了、时间没停,
//   于是这一秒里照常按 u≈0.56 (约 25°/s) 白转。把 relF ∈ [-20°,-12°]
//   这个"半坡平台期"单独拎出来看, 五次跑分得极干净:
//        跑次        停留     cmd 从→到        烧掉转角   平均u   结果
//        1538       0.3s     19.4→26.0         6.6°      0.54   好
//        1545       0.5s     18.9→28.9        10.0°      0.54   好
//        1643       0.7s     12.9→26.6        13.7°      0.56   好
//        1552       1.6s      0.5→54.5      ★ 54.0°      0.49   坏(左后)
//        1636       1.7s     10.7→45.7      ★ 35.0°      0.56   坏(左后)
//   u 五次跑几乎一样, 唯一变量就是"在平台期待了多久"。
//   【后果由几何唯一决定】偏航 θ 时两只后脚在踏面纵深方向被拉开 0.28·sinθ,
//   踏面只有 20cm ⇒ 临界角 θc = arcsin(0.20/0.28) = 45.6°。
//   取"机体压平上顶面"那一刻 (relF 首次 ≥ -10°) 的累计转角:
//        1538 29.1° (13.6cm) | 1545 33.5° (15.5cm) | 1643 29.6° (13.8cm)  好
//        1636 48.0° (20.8cm) | 1552 56.8° (23.4cm)                      ★坏
//   五次跑没有一个例外 —— 超过 45.6°, 左后脚在几何上就不可能和右后脚落在
//   同一级, 必然被留在下一级。"左后足没上去"是被控制器逼出来的, 不是运气。
//   【做法】relF 本次跑首次达到 SA_TOP_ARM_REL_DEG 之前, turn_cmd 硬封在
//   SA_TOP_TURN_CAP_DEG; 一旦达到就【永久】解除, 之后不再限制 (闩锁, 不来回抖)。
//   回放: 好跑次到 relF=-10 时 cmd 是 29.1/33.5/29.6, 一次都不碰 40°,
//         最小余量 6.5° ⇒ 三次好跑次逐帧零影响。
//         1636 封住省下 8°, 1552 封住省下 17°。
//   【兜底】万一 relF 永远上不到 -10° (没真正登顶/pitch 通道异常), 开转后
//   SA_TOP_ARM_MAX_SEC 强制解除, 否则封在 40° 就永远转不满 100°。
const float  SA_TOP_ARM_REL_DEG     = -10.0f;// relF 到过此值 -> 认为已压平上顶面, 永久解除封顶
const float  SA_TOP_TURN_CAP_DEG    = 40.0f; // 解除之前累计转角的硬上限
const double SA_TOP_ARM_MAX_SEC     = 4.0;   // 开转后这么久仍没到 -> 强制解除
const float  SA_TURN_DONE_TOL       = 6.0f;  // 指令转满后, 误差进此窗即算转到位
const double SA_CLIMB_MAX_SEC       = 12.0;  // CLIMB 段总兜底上限

// ---------------- 卡滞与脱困 ----------------
//   ★ 两套判据并行, 响应相反 —— 见文件头 ⑤
//   骑棱卡死 (位移判据): 猛推脱开
//   ★v15 阈值 0.03 → 0.045: 1636 卡了 5 秒多, 第一次猛推却晚了约 2.8 秒 ——
//   狗站住时机体仍在晃, 里程计每秒读出 2~4cm, 一直贴着 3cm 擦边躲过判据
//   (实际触发值 0.0280/0.0100/0.0244/0.0298, 全部刚好压线)。
//   而三次好跑次 CLIMB 段 1s 窗口位移的【最小值】是 0.058/0.054/0.063m,
//   从来没低于 5.4cm ⇒ 放到 4.5cm 仍有约 20% 余量, 三次好跑次零触发。
const double SA_STUCK_POS_WIN_SEC   = 1.0;
const float  SA_STUCK_POS_MOVE_M    = 0.045f;// ★v15 0.03→0.045
const float  SA_ESC_PUSH_VX         = 0.40f;
//   ★v12 后腿悬空: 降级为【只检测、只打日志, 不再有动作】
//   2053 实测: 该脱困触发 3 次, 每次 0.6s, 而 ff[2] 从 6.0s 到 8.9s 一直是 0 ——
//   固定 0.6s 一到, 腿还悬着就照样接着以 39°/s 拧, 完全没救回来。
//   roll 闸门是它的严格加强版 (只要身子还歪着就一直刹住), 留两套只会互相打架。
//   足力信号本身很有价值 (它指认了是左后腿、悬空多久), 所以检测保留, 只是不再驱动动作。
//   ★v14 阈值 8 → 15: 8 这个数来自 2053 那次 (ff[2] 悬空时读数就是 0)。
//   0811 三次跑 (1538/1545/1552) 里 ff[2] 的最小值分别是 13 / 10 / 9, 10 分位
//   数是 14 / 14 / 12 —— 这条腿完全悬空时读数就在 12~14, "≤8" 一次都不可能
//   成立, 检测器等于是关着的 (三次跑结束块全部打印"后腿悬空 0 次")。
//   而 1552 实际发生了什么: 从 6.1s 到 10.0s 左后腿基本没承过力,
//   其中 7.80~9.90s 连续 2.10s 贴在地板值上, 同期机体位移 4.5cm/1.2s。
//   按 15 重算三次跑的最长连续段: 1538=0.90s, 1545=0.80s, 1552=2.10s ——
//   好坏依然分得开 (判据"单次最长 <1s"照旧可用), 且 0.8s 的持续要求本身
//   就把正常摆动相 (~0.2~0.3s) 挡在外面, 不会误报。
//   ★ 只影响日志, 不驱动任何动作 (动作在 v12 就删了)。
const int    SA_FOOT_LOW_TH         = 15;    // 足力 ≤ 此值视为不承力 (★v14 8→15)
const double SA_FOOT_LOW_SEC        = 0.8;   // 持续这么久就记一次事件
//   公共
//   ★v15 脱困退出条件从【固定 0.6s】改成【按位移退出】。
//   1636 四次猛推的实测: 时长 0.60/0.60/0.60/0.40s, 位移 2.2/3.0/7.1/4.0cm。
//   第 3、4 次已经跑到 0.118/0.100 m/s —— 正起效, 却被计时器掐断。
//   而 12s 兜底进 DOWN 之后 (vx 恒 0.35, 不收油): 0.6s 走 17.0cm,
//   1.2s 走 30.6cm, 2.0s 走 49.4cm, 一下就出来了。
//   ⇒ 不是推力不够, 是【持续时间】和【脱困一结束就被收油掐死】。
//   这跟 v12 删掉后腿脱困动作时批评的是同一个毛病: 用定时器决定"够了没有"。
//   SA_ESC_GOAL_M 取 0.15m, 依据就是 DOWN 前 0.6s 的 17cm。
const float  SA_ESC_GOAL_M          = 0.15f; // 本次脱困累计推进到此值即算脱开
const double SA_ESC_MAX_SEC         = 1.5;   // 脱困动作时长上限 (兜底, 不是正常退出条件)
const double SA_ESC_MIN_SEC         = 0.3;   // 至少推这么久 (给步态起步时间)
const double SA_ESC_COOLDOWN_SEC    = 0.8;   // 两次脱困之间的冷却
//   ★v15 脱困后保持期: 刚推出来时 rel 往往已 >0, 下坡收油会立刻把 vx 打回
//   0.19 —— 1636 三次猛推结束后的 0.6s 只走了 1.4/2.2/1.0cm, 马上又停。
//   DOWN 的 0.35 能出来、CLIMB 的 0.19 出不来, 差的就是这一档。
const double SA_ESC_HOLD_SEC        = 1.0;   // 脱困结束后维持推力的时长
const float  SA_ESC_HOLD_VX_MIN     = 0.30f; // 保持期内 vx 的下限 (绕过下坡收油)
//   打滑加力 (爬不动就加推力)
const double SA_SLIP_CHECK_SEC      = 1.0;
const float  SA_SLIP_MOVE_M         = 0.05f;
const float  SA_SLIP_VX_STEP        = 0.05f;
const float  SA_SLIP_VX_BOOST_MAX   = 0.15f;
//   越沿减速: 前脚已探出远端 -> 只减 vx, omega 保持 (要接着把身子甩过来)
//   ★v13 二值"越沿减速"换成【连续下坡收油】, 和 u 调制同一套思路:
//   v12 是 rel≥+12° 一刀切 ×0.5(再被下限托住, 实际几乎没减), 而 1423 冲过那一段
//   rel 从 +12 一路涨到 +32.9°, 减速力度全程不变 —— 越往下栽越该收, 不该是台阶函数。
//        rel 从 REL0 → REL1, vx 从当前值连续降到 SA_DESC_VX_MIN
//   ★v14 修正【作用顺序】: v13 是先 vx_cmd = vx_base + vx_boost, 再拿这个含加力
//   的值往 SA_DESC_VX_MIN 收, 且 vx_cmd = min(vx_cmd, vx_desc) —— 加力越多被
//   收掉的也越多, 等于打滑加力在下坡段被整个抵消。1552 卡住那一刻代入:
//        vx_base=0.25, vx_boost=+0.15 (已加满), vx_cmd=0.40
//        dcs = (11.9-0)/(20-0) = 0.595
//        vx_desc = 0.40 + (0.15-0.40)*0.595 = 0.251   ← 日志实测 vx=0.23~0.25
//   而加力的判据恰恰是"1 秒走不到 5cm", 也就是【最需要它的时候它被 min 掉了】。
//   改成: 收油只作用在 vx_base 上, vx_boost 在收油之后再加。同样代入:
//        vx_desc = 0.25 + (0.15-0.25)*0.595 = 0.19,  +0.15 = 0.34
//   ★ 对好跑次是恒等变换: 1538/1545 的 vx_boost 全程为 0, 两式结果完全一样。
//     它只在"已经走不动了"这个前提下才生效, 符合"保护性改动只在坏跑次上生效"。
const float  SA_DESC_REL0_DEG       = 0.0f;  // 开始收油的相对 pitch
const float  SA_DESC_REL1_DEG       = +20.0f;// 收到底的相对 pitch
const float  SA_DESC_VX_MIN         = 0.15f; // 收到底时的推进速度

// ---------------- yaw 闭环 (前馈 + P) ----------------
const float  SA_YAW_KP     = 0.030f; // rad/s per deg
const float  SA_YAW_FB_MAX = 0.50f;  // 反馈项限幅
const float  SA_OMEGA_MAX  = 1.40f;  // 总角速度限幅 (要装下前馈 0.681 + 反馈 0.50)
const float  SA_YAW_KP_STRAIGHT    = 0.020f; // 锁存前/DOWN/EXIT 的直线保持
const float  SA_OMEGA_MAX_STRAIGHT = 0.25f;

// ---------------- 安全 (只有这几条会 return false) ----------------
const float  SA_ROLL_ABORT_DEG   = 28.0f;
const double SA_ROLL_ABORT_SEC   = 0.60;
const double SA_STATE_STALE_SEC  = 0.30;
const double SA_STATE_DEAD_SEC   = 2.00;
const int    SA_WATCHDOG_SEC     = 45;

// ---------------- DOWN ----------------
const float  SA_DOWN_CONFIRM_REL_DEG = 10.0f;
const float  SA_DOWN_FLAT_REL_DEG    = 5.0f;
const double SA_DOWN_MIN_SEC = 2.5;
const double SA_DOWN_MAX_SEC = 7.0;
const double SA_DOWN_STUCK_CHECK_SEC = 1.0;
const float  SA_DOWN_STUCK_MOVE_M    = 0.05f;
const float  SA_DOWN_STUCK_VX_STEP   = 0.05f;
const float  SA_DOWN_STUCK_VX_MAX    = 0.45f;

// ---------------- 步态切换等待 ----------------
const double SA_FREEWALK_WAIT_SEC   = 1.5;
const double SA_STATICWALK_WAIT_SEC = 1.5;

// ---------------- 仅用于日志的"登顶判定" (不控制任何东西) ----------------
const float  SA_SUMMIT_DEPTH_DEG    = -20.0f;
const float  SA_SUMMIT_RECOVERY_DEG = 7.0f;
const float  SA_SUMMIT_CEIL_DEG     = -16.0f;

// 合成倾角: 把 roll/pitch 合成为"机体相对重力的总倾角"。
// 斜着爬时地形法向会被分到 roll 和 pitch 两个通道, 单看 pitch 会低估坡度。
// 本版判据仍以 rel_pitch 为主 (与历史阈值可比), tilt 只进日志。
static inline float tiltDeg(float roll_deg, float pitch_deg) {
    float c = std::cos(deg2rad(roll_deg)) * std::cos(deg2rad(pitch_deg));
    c = clampf(c, -1.0f, 1.0f);
    return rad2deg(std::acos(c));
}

enum class SaPhase { CLIMB, DOWN, EXIT, DONE };
static const char* saPhaseName(SaPhase p) {
    switch (p) {
        case SaPhase::CLIMB: return "CLIMB";
        case SaPhase::DOWN:  return "DOWN ";
        case SaPhase::EXIT:  return "EXIT ";
        case SaPhase::DONE:  return "DONE ";
    }
    return "?    ";
}

enum class SaEsc { NONE, PUSH };  // ★v12 LEG 已删除, 见文件头

// ============================================================
//  runStairs v13: 起步冲刺 -> 一段式连续变速爬转(带符号 roll 闸门) -> 下台阶 -> 切回常规步态
//  返回 true  = 走完 (含各种降级路径, main 照常继续后面 110 分)
//  返回 false = 真危险, main 会零速 hold 等人工接管
// ============================================================
bool runStairs(unitree::robot::go2::SportClient& sport, TurnDir turn_dir,
               float entry_yaw_forced = NAN,
               StairClimbMode climb_mode = StairClimbMode::FORWARD) {
    if (climb_mode == StairClimbMode::REVERSE) {
        std::cout << "[台阶v13] ★ 倒爬(10)分支已废弃, 本版只支持正爬; 按正爬执行"
                  << std::endl;
    }
    const float turn_sign = (turn_dir == TurnDir::LEFT) ? +1.0f : -1.0f;

    // ---------- 切 AI 步态 ----------
    std::cout << "[步态] StaticWalk → FreeWalk (AI 灵动, 支持爬楼梯)..." << std::endl;
    int ret = sport.FreeWalk();
    std::cout << "  FreeWalk() ret=" << ret << std::endl;
    {
        auto t = std::chrono::steady_clock::now();
        while (g_running) {
            double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - t).count() / 1000.0;
            if (el >= SA_FREEWALK_WAIT_SEC) break;
            sport.Move(0.0f, 0.0f, 0.0f);
            usleep(20 * 1000);
        }
    }
    if (!g_running) { sport.Move(0, 0, 0); return true; }

    // ---------- 入口 pitch/roll 基准 (5ms 轮询, 1.5s 截止) ----------
    float entry_pitch_sum = 0.0f, entry_roll_sum = 0.0f;
    int   entry_n = 0;
    uint64_t bseq = g_state_seq.load(std::memory_order_acquire);
    auto bdl = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
    while (g_running && entry_n < 30 && std::chrono::steady_clock::now() < bdl) {
        uint64_t s = g_state_seq.load(std::memory_order_acquire);
        if (s != bseq) {
            bseq = s;
            entry_pitch_sum += rad2deg(g_pitch);
            entry_roll_sum  += rad2deg(g_roll);
            ++entry_n;
        }
        usleep(5 * 1000);
    }
    float entry_pitch_baseline, entry_roll_baseline;
    if (entry_n >= 10) {
        entry_pitch_baseline = entry_pitch_sum / entry_n;
        entry_roll_baseline  = entry_roll_sum  / entry_n;
    } else {
        entry_pitch_baseline = rad2deg(g_pitch.load());
        entry_roll_baseline  = rad2deg(g_roll.load());
        std::cout << "[台阶v13][告警] 入口基准只采到 " << entry_n
                  << " 个新帧, 退化用当前单帧值 (pitch=" << entry_pitch_baseline
                  << "° roll=" << entry_roll_baseline << "°), 继续爬" << std::endl;
    }

    const float init_x = g_pos_x.load();
    const float init_y = g_pos_y.load();
    const float entry_yaw = std::isnan(entry_yaw_forced) ? g_yaw_rad.load()
                                                         : entry_yaw_forced;
    const float final_yaw = normalize_180_rad(
        entry_yaw + turn_sign * deg2rad(SA_TURN_DEG));

    std::cout << "\n========== 台阶 v15 一段式连续爬转 ==========" << std::endl;
    std::cout << "入口 yaw:        " << rad2deg(entry_yaw) << "°" << std::endl;
    std::cout << "入口 pitch 基准: " << entry_pitch_baseline
              << "°   roll 基准: " << entry_roll_baseline
              << "°  (基于 " << entry_n << " 帧)" << std::endl;
    std::cout << "入口雷达:        F=" << g_front_dist.load()
              << "m L=" << g_left_dist.load()
              << "m R=" << g_right_dist.load() << "m" << std::endl;
    std::cout << "★结构:          单一 CLIMB 阶段, 无停顿/无角速度阶跃" << std::endl;
    std::cout << "★起步冲刺:      前 " << SA_LAUNCH_SEC << "s 纯直行 vx="
              << SA_LAUNCH_VX << " (一度不转; 冲刺优先, 坡态提前锁存也等冲刺跑满)"
              << std::endl;
    std::cout << "★转速调制:      rel≤" << SA_TURN_MOD_REL_LO << "° → "
              << SA_TURN_RATE_SLOW_DPS << "°/s ("
              << deg2rad(SA_TURN_RATE_SLOW_DPS) << " rad/s);  rel≥"
              << SA_TURN_MOD_REL_HI << "° → " << SA_TURN_RATE_FAST_DPS
              << "°/s (" << deg2rad(SA_TURN_RATE_FAST_DPS)
              << " rad/s);  中间线性连续" << std::endl;
    std::cout << "★推进调制:      vx " << SA_VX_CLIMB_DEEP << " (u=0) → "
              << SA_VX_CLIMB_TOP << " (u=1);  打滑最多加力 +"
              << SA_SLIP_VX_BOOST_MAX << std::endl;
    std::cout << "总转角:          " << (turn_sign > 0 ? "+" : "-") << SA_TURN_DEG
              << "° (绝对目标 yaw=" << rad2deg(final_yaw) << "°)" << std::endl;
    std::cout << "★v15 登顶前封顶: relF 到过 " << SA_TOP_ARM_REL_DEG
              << "° 之前, 累计转角硬封在 " << SA_TOP_TURN_CAP_DEG
              << "° (到了就永久解除); 开转后 " << SA_TOP_ARM_MAX_SEC
              << "s 仍没到则兜底解除" << std::endl;
    std::cout << "                 依据: 上顶面时偏航 θ ⇒ 后脚纵深拉开 0.28·sinθ,"
              << " 踏面 20cm ⇒ 临界 45.6°;  好跑次 29~34°, 坏跑次 48~57°"
              << std::endl;
    std::cout << "★roll 闸门:     ★只对向内侧歪刹车(外侧不管); 内侧残差(扣几何) 超 "
              << SA_ROLL_GATE_SOFT_DEG << "° 开始收转速, 到 " << SA_ROLL_GATE_HARD_DEG
              << "° 完全停转 (只停转, 不停走)" << std::endl;
    std::cout << "★v14 深跨级门:  min_rel 到过 " << SA_DEEP_ARM_REL_DEG
              << "° 之前 u 锁 0 (转速恒 " << SA_TURN_RATE_SLOW_DPS
              << "°/s); 冲刺后 " << SA_DEEP_ARM_MAX_SEC << "s 仍没到则兜底解锁"
              << std::endl;
    std::cout << "★下坡收油:      rel 从 " << SA_DESC_REL0_DEG << "° 到 "
              << SA_DESC_REL1_DEG << "°, 基础 vx 连续收到 " << SA_DESC_VX_MIN
              << " (omega 保持);  ★v14 打滑加力加在收油之后, 不再被 min 掉"
              << std::endl;
    std::cout << "★脱困(骑棱):    " << SA_STUCK_POS_WIN_SEC << "s 位移<"
              << SA_STUCK_POS_MOVE_M << "m → ω=0, vx=" << SA_ESC_PUSH_VX
              << ";  ★v15 按位移退出: 推进到 " << SA_ESC_GOAL_M << "m 即止 (最短 "
              << SA_ESC_MIN_SEC << "s, 上限 " << SA_ESC_MAX_SEC << "s)" << std::endl;
    std::cout << "★v15 脱困保持期: 脱困结束后 " << SA_ESC_HOLD_SEC << "s 内 vx 下限 "
              << SA_ESC_HOLD_VX_MIN << " (绕过下坡收油, 防止刚推出来又被按回去)"
              << std::endl;
    std::cout << "★脱困(后腿悬空):后足力≤" << SA_FOOT_LOW_TH << " 持续 "
              << SA_FOOT_LOW_SEC << "s → ★只记事件, 不再有动作 (交给 roll 闸门处理)"
              << std::endl;
    std::cout << "危险 hold 条件:  |roll|>" << SA_ROLL_ABORT_DEG << "° 持续 "
              << SA_ROLL_ABORT_SEC << "s / NaN / DDS 死 " << SA_STATE_DEAD_SEC
              << "s   —— 其余全部照常往下走" << std::endl;
    std::cout << "策略ID:          STAIR_CONT_V15_20260811" << std::endl;
    std::cout << "=============================================" << std::endl;

    // ---------- 状态 ----------
    SaPhase phase = SaPhase::CLIMB;
    auto now0 = std::chrono::steady_clock::now();
    auto start_all   = now0;
    auto phase_start = now0;
    auto last_fresh  = now0;
    auto last_loop   = now0;
    uint64_t last_seq = bseq;

    int   slope_arm_cnt = 0;
    bool  slope_seen    = false;
    auto  slope_time    = now0;
    float min_rel_seen  = 0.0f;
    float max_rel_seen  = -999.0f;

    // ★v14 深跨级确认门: 没见过一次 rel ≤ SA_DEEP_ARM_REL_DEG 之前, u 锁 0
    bool   deep_armed      = false;
    bool   deep_armed_fb   = false;   // 是否走的兜底解锁 (而不是真见到深跨级)
    double deep_armed_t    = -1.0;    // 解锁时刻 (phase_sec), 仅日志

    // rel 滑动平均 (只喂调制, 日志仍打原始值)
    float rel_buf[SA_REL_FILT_N];
    int   rel_buf_n = 0, rel_buf_i = 0;

    // 转角指令 (积分式)
    float turn_cmd_deg   = 0.0f;
    bool  turn_cmd_full  = false;
    bool  turn_started   = false;
    bool  lag_pausing    = false;
    float lag_max        = 0.0f;
    double turn_start_sec = -1.0;   // ★v15 开转时刻 (给封顶兜底计时用)
    bool  top_armed      = false;   // ★v15 relF 到过 -10° -> 永久解除转角封顶
    bool  top_capped     = false;   // 当前是否正被封顶 (仅日志防抖)
    double top_armed_t   = -1.0;
    bool  top_armed_fb   = false;
    float top_armed_cmd  = -1.0f;   // 解除那一刻的累计转角 (★核心观察量)
    auto  esc_ref_x = 0.0f; auto esc_ref_y = 0.0f;   // 本次脱困起点
    auto  esc_hold_until = now0 - std::chrono::seconds(10);
    bool  esc_holding    = false;
    double esc_hold_sec_total = 0.0;
    bool  forced_turn    = false;   // 从未锁上坡态, 到点强制开转
    bool  launch_done_logged = false;  // ★v11 起步冲刺结束只打一次日志

    // 位移历史环形缓冲 (用来算"最近 W 秒的位移")
    const int MVN = 96;
    float mv_x[MVN], mv_y[MVN];
    double mv_t[MVN];
    int mv_n = 0, mv_i = 0;

    // 脱困
    SaEsc esc = SaEsc::NONE;
    auto  esc_start = now0;
    auto  esc_last_end = now0 - std::chrono::seconds(10);
    int   esc_push_cnt = 0;
    double esc_total_sec = 0.0;

    // 后足失力计时
    std::chrono::steady_clock::time_point foot_low_since[2] = { now0, now0 };
    bool  foot_low_act[2]   = { false, false };
    bool  foot_low_logged[2] = { false, false };
    int   foot_low_events   = 0;
    double foot_low_worst   = 0.0;   // 单次最长悬空时长 (调参用的关键指标)

    // ★v12 roll 闸门
    float roll_gate     = 1.0f;
    float roll_res_mag  = 0.0f;   // ★v13 只统计"向内侧歪"的残差, 外侧记 0
    float roll_geo_dbg  = 0.0f;   // 几何应有的 roll (日志用)
    float roll_res_worst = 0.0f;
    bool  roll_braking  = false;
    double roll_brake_sec = 0.0;

    // 打滑加力
    float vx_boost = 0.0f;
    auto  slip_ref_t = now0;

    // 越沿
    bool  edge_slow = false;

    // 仅日志: 登顶判定
    bool  summit_logged = false;
    float summit_dist = -1.0f;
    double summit_t = -1.0;
    float summit_turn = -1.0f;   // ★v14 登顶时的累计转角 (好跑次 ~23°, 1552 坏跑次 42.9°)

    int   roll_bad_cnt = 0;
    const int roll_bad_frames_need = (int)(SA_ROLL_ABORT_SEC * 50.0);

    float down_vx = SA_VX_DOWN;
    float down_ref_x = g_pos_x.load(), down_ref_y = g_pos_y.load();
    auto  down_ref_t = now0;
    bool  descended = false;

    bool danger_hold = false;
    long n_loops = 0;

    while (g_running && phase != SaPhase::DONE) {
        ++n_loops;
        auto now = std::chrono::steady_clock::now();
        double total_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                               now - start_all).count() / 1000.0;
        double phase_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                               now - phase_start).count() / 1000.0;
        double dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - last_loop).count() / 1000.0;
        last_loop = now;
        if (dt < 0.0)  dt = 0.0;
        if (dt > 0.10) dt = 0.10;   // 卡顿保护: 一帧最多算 100ms, 防止转角指令跳变

        // ---------- DDS 新鲜度 ----------
        uint64_t seq = g_state_seq.load(std::memory_order_acquire);
        bool fresh = (seq != last_seq);
        if (fresh) { last_seq = seq; last_fresh = now; }
        double stale = std::chrono::duration_cast<std::chrono::milliseconds>(
                           now - last_fresh).count() / 1000.0;
        if (stale >= SA_STATE_DEAD_SEC) {
            std::cout << "\n[危险停止] SportModeState 已 " << stale
                      << "s 无新帧, 零速 hold 等人工" << std::endl;
            sport.Move(0, 0, 0);
            danger_hold = true;
            break;
        }
        if (stale >= SA_STATE_STALE_SEC) {
            sport.Move(0.0f, 0.0f, 0.0f);
            usleep(20 * 1000);
            continue;
        }

        // ---------- 姿态 ----------
        float pitch_deg = rad2deg(g_pitch.load());
        float roll_deg  = rad2deg(g_roll.load());
        float yaw_now   = g_yaw_rad.load();
        float rel_pitch = pitch_deg - entry_pitch_baseline;
        float rel_roll  = roll_deg  - entry_roll_baseline;
        float tilt      = tiltDeg(roll_deg, pitch_deg);
        float rel_tilt  = tiltDeg(rel_roll, rel_pitch);
        float px = g_pos_x.load(), py = g_pos_y.load();
        float dx = px - init_x, dy = py - init_y;
        float dist = std::sqrt(dx * dx + dy * dy);

        if (!std::isfinite(pitch_deg) || !std::isfinite(roll_deg)
            || !std::isfinite(yaw_now) || !std::isfinite(dx) || !std::isfinite(dy)) {
            std::cout << "\n[危险停止] IMU/里程出现 NaN/Inf: pitch=" << pitch_deg
                      << " roll=" << roll_deg << " yaw=" << yaw_now
                      << " dx=" << dx << " dy=" << dy << std::endl;
            sport.Move(0, 0, 0);
            danger_hold = true;
            break;
        }

        // rel 滑动平均 (调制专用)
        if (fresh) {
            rel_buf[rel_buf_i] = rel_pitch;
            rel_buf_i = (rel_buf_i + 1) % SA_REL_FILT_N;
            if (rel_buf_n < SA_REL_FILT_N) ++rel_buf_n;
        }
        float rel_f = rel_pitch;
        if (rel_buf_n > 0) {
            float s = 0.0f;
            for (int k = 0; k < rel_buf_n; ++k) s += rel_buf[k];
            rel_f = s / rel_buf_n;
        }

        // 位移历史
        mv_x[mv_i] = px; mv_y[mv_i] = py; mv_t[mv_i] = total_sec;
        mv_i = (mv_i + 1) % MVN;
        if (mv_n < MVN) ++mv_n;
        // 最近 W 秒的位移 (取窗口内最老的一条比)
        auto movedOver = [&](double W) -> float {
            float bx = px, by = py;
            double best = -1.0;
            for (int k = 0; k < mv_n; ++k) {
                double age = total_sec - mv_t[k];
                if (age >= W && (best < 0.0 || age < best)) {
                    best = age; bx = mv_x[k]; by = mv_y[k];
                }
            }
            if (best < 0.0) return 999.0f;   // 历史还不够 W 秒, 视为"有在动"
            return std::sqrt((px - bx) * (px - bx) + (py - by) * (py - by));
        };

        // ---------- 侧翻守卫 ----------
        if (fresh) {
            if (std::fabs(roll_deg) > SA_ROLL_ABORT_DEG) ++roll_bad_cnt;
            else                                          roll_bad_cnt = 0;
        }
        if (roll_bad_cnt >= roll_bad_frames_need) {
            std::cout << "\n[危险停止] |roll|=" << std::fabs(roll_deg)
                      << "° 已持续 " << SA_ROLL_ABORT_SEC << "s (>"
                      << SA_ROLL_ABORT_DEG << "°), 疑似侧翻, 零速 hold 等人工"
                      << std::endl;
            sport.Move(0, 0, 0);
            danger_hold = true;
            break;
        }

        // ---------- 总看门狗 ----------
        if (total_sec > SA_WATCHDOG_SEC && phase != SaPhase::EXIT) {
            std::cout << "\n[看门狗] 台阶总耗时 " << total_sec << "s 超 "
                      << SA_WATCHDOG_SEC << "s, 直接进 EXIT 切回常规步态继续跑"
                      << std::endl;
            phase = SaPhase::EXIT;
            phase_start = now;
        }

        // ---------- 极值跟踪 ----------
        if (fresh) {
            if (phase == SaPhase::CLIMB && !turn_cmd_full) {
                min_rel_seen = std::min(min_rel_seen, rel_pitch);
            }
            if (rel_pitch > 0.0f || phase == SaPhase::DOWN) {
                max_rel_seen = std::max(max_rel_seen, rel_pitch);
            }
        }

        float vx_cmd = 0.0f;
        float omega_cmd = 0.0f;
        float u = 0.0f;
        float turn_rate = 0.0f;
        float yaw_ref = entry_yaw;
        float yaw_err_deg = 0.0f;

        // ============ CLIMB: 一段式连续爬转 ============
        if (phase == SaPhase::CLIMB) {

            // ---- 坡态锁存 ----
            if (fresh && !slope_seen) {
                if (rel_pitch <= -SA_SLOPE_ARM_DEG) {
                    if (++slope_arm_cnt >= SA_SLOPE_ARM_FRAMES) {
                        slope_seen = true;
                        slope_time = now;
                        slip_ref_t = now;
                        std::cout << "\n[坡态锁存] t=" << phase_sec
                                  << "s rel_pitch=" << rel_pitch << "° tilt=" << tilt
                                  << "° dist=" << dist
                                  << "m —— 已确实骑在台阶上, 开始连续转向" << std::endl;
                    }
                } else {
                    slope_arm_cnt = 0;
                }
            }
            if (!slope_seen && !forced_turn && phase_sec >= SA_NOSLOPE_TURN_SEC) {
                forced_turn = true;
                slip_ref_t = now;
                std::cout << "\n[强制开转] " << SA_NOSLOPE_TURN_SEC
                          << "s 到点仍未锁上坡态 (可能根本没爬上台阶), "
                          << "按 u=1 全速转, 不判失败" << std::endl;
            }
            // ---- ★v11 起步冲刺: 前 SA_LAUNCH_SEC 秒纯直行, 一度都不转 ----
            //   冲刺优先: 就算坡态提前锁存, 也把冲刺跑满再开转。
            bool in_launch = (phase_sec < SA_LAUNCH_SEC);
            if (!in_launch && !launch_done_logged) {
                launch_done_logged = true;
                slip_ref_t = now;
                std::cout << "\n[起步冲刺结束] " << SA_LAUNCH_SEC << "s @ vx="
                          << SA_LAUNCH_VX << " 完成: rel_pitch=" << rel_pitch
                          << "° dist=" << dist << "m 坡态="
                          << (slope_seen ? "已锁存" : "未锁存")
                          << " —— 转入连续变速爬转" << std::endl;
            }

            bool turning_allowed = (slope_seen || forced_turn) && !in_launch;

            // ---- ★v14 深跨级确认门 ----
            //   rel 浅有两种相反含义 (快到顶 / 压根没爬上去), 只有"已经深过一次"
            //   能把两者分开。没深过之前一律按最慢转速走, 别在半坡上把角度用光。
            if (!deep_armed && min_rel_seen <= SA_DEEP_ARM_REL_DEG) {
                deep_armed  = true;
                deep_armed_t = phase_sec;
                std::cout << "\n[深跨级确认] t=" << phase_sec << "s min_rel="
                          << min_rel_seen << "° (门槛 " << SA_DEEP_ARM_REL_DEG
                          << "°) dist=" << dist << "m —— u 解锁, 转速恢复由 rel 调制"
                          << std::endl;
            }
            if (!deep_armed && turning_allowed
                && phase_sec >= SA_LAUNCH_SEC + SA_DEEP_ARM_MAX_SEC) {
                deep_armed    = true;
                deep_armed_fb = true;
                deep_armed_t  = phase_sec;
                std::cout << "\n[深跨级确认-兜底] t=" << phase_sec
                          << "s 冲刺后 " << SA_DEEP_ARM_MAX_SEC
                          << "s 仍未见 rel≤" << SA_DEEP_ARM_REL_DEG
                          << "° (最深只到 " << min_rel_seen
                          << "°), 强制解锁 u, 免得 8°/s 转不完 100°" << std::endl;
            }

            // ---- 调制量 u ----
            if (forced_turn && !slope_seen) {
                u = 1.0f;
            } else if (!deep_armed) {
                u = 0.0f;                       // ★v14 深跨级确认前一律最慢转速
            } else {
                u = (rel_f - SA_TURN_MOD_REL_LO)
                  / (SA_TURN_MOD_REL_HI - SA_TURN_MOD_REL_LO);
                u = clampf(u, 0.0f, 1.0f);
            }
            turn_rate = SA_TURN_RATE_SLOW_DPS
                      + (SA_TURN_RATE_FAST_DPS - SA_TURN_RATE_SLOW_DPS) * u;

            // ---- ★v12 roll 闸门: 身子歪了就别再拧, 先让腿站稳 ----
            //   机体在坡上偏航本来就会产生 roll, 必须先扣掉这部分几何分量,
            //   否则深跨级偏航 30° 自带的 ~17° roll 会把闸门锁死、90° 转不完。
            {
                float theta = std::fabs(turn_cmd_deg);
                if (theta > SA_ROLL_GEO_CAP_DEG) theta = SA_ROLL_GEO_CAP_DEG;
                // rel_pitch>0 (已在下坡) 时机体已大致对齐新坡的下降方向, 几何分量按 0 算
                roll_geo_dbg = turn_sign * std::min(0.0f, rel_pitch)
                             * std::tan(deg2rad(theta));
                // ★v13 带符号: 只取"向转弯内侧歪"的分量 (左转=左侧沉), 外侧一律不管
                float inward = -turn_sign * (rel_roll - roll_geo_dbg);
                roll_res_mag = std::max(0.0f, inward);
                if (roll_res_mag > roll_res_worst) roll_res_worst = roll_res_mag;
                roll_gate = 1.0f - (roll_res_mag - SA_ROLL_GATE_SOFT_DEG)
                                 / (SA_ROLL_GATE_HARD_DEG - SA_ROLL_GATE_SOFT_DEG);
                roll_gate = clampf(roll_gate, 0.0f, 1.0f);
            }
            if (turning_allowed) {
                if (roll_gate < 1.0f) roll_brake_sec += dt;
                if (!roll_braking && roll_gate < 0.5f) {
                    roll_braking = true;
                    std::cout << "\n[roll 刹车] 内侧残差 " << roll_res_mag
                              << "° (rel_roll=" << rel_roll << "° 几何应有="
                              << roll_geo_dbg << "°), 闸门降到 "
                              << roll_gate << " —— 机体向内侧歪了, 先收转速让腿站稳"
                              << " [rel=" << rel_pitch << "° 转角=" << turn_cmd_deg
                              << "°]" << std::endl;
                } else if (roll_braking && roll_gate > 0.85f) {
                    roll_braking = false;
                    std::cout << "\n[roll 刹车解除] 内侧残差回到 " << roll_res_mag
                              << "°, 闸门 " << roll_gate << ", 恢复转速" << std::endl;
                }
            }
            turn_rate *= roll_gate;

            // ---- 后足失力计时 ----
            for (int k = 0; k < 2; ++k) {
                int ffv = g_foot_force[2 + k].load();
                if (ffv <= SA_FOOT_LOW_TH) {
                    if (!foot_low_act[k]) { foot_low_act[k] = true; foot_low_since[k] = now; }
                } else {
                    foot_low_act[k] = false;
                }
            }
            for (int k = 0; k < 2; ++k) {
                if (foot_low_act[k]) {
                    double d = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   now - foot_low_since[k]).count() / 1000.0;
                    if (d > foot_low_worst) foot_low_worst = d;
                    // ★v12 只记事件, 不做动作 (下标 2 = 左后腿, 实测 1928/2053 都是它)
                    if (d >= SA_FOOT_LOW_SEC && !foot_low_logged[k]) {
                        foot_low_logged[k] = true;
                        ++foot_low_events;
                        std::cout << "\n[后腿悬空-记录] ff[" << (2 + k)
                                  << "] 已连续 " << d << "s ≤ " << SA_FOOT_LOW_TH
                                  << " (rel=" << rel_pitch << "° roll=" << roll_deg
                                  << "° roll残差=" << roll_res_mag
                                  << "° 闸门=" << roll_gate << " 转角=" << turn_cmd_deg
                                  << "°) —— 不触发动作, 由 roll 闸门负责" << std::endl;
                    }
                } else {
                    foot_low_logged[k] = false;
                }
            }

            // ---- 脱困状态机 ----
            //   ★v15 退出条件从"固定 0.6s"改成"按位移退出": 推进到 SA_ESC_GOAL_M
            //   即算脱开, 没到就一直推 (上限 SA_ESC_MAX_SEC 兜底)。1636 的四次
            //   猛推里有两次已经跑到 0.10~0.12 m/s, 正起效时被 0.6s 掐断。
            if (esc != SaEsc::NONE) {
                double e = std::chrono::duration_cast<std::chrono::milliseconds>(
                               now - esc_start).count() / 1000.0;
                float  emv = std::sqrt((px - esc_ref_x) * (px - esc_ref_x)
                                     + (py - esc_ref_y) * (py - esc_ref_y));
                bool   done_goal = (e >= SA_ESC_MIN_SEC) && (emv >= SA_ESC_GOAL_M);
                bool   done_cap  = (e >= SA_ESC_MAX_SEC);
                if (done_goal || done_cap) {
                    std::cout << "\n[脱困结束] 猛推 " << e << "s, 推进 " << emv
                              << "m —— " << (done_goal ? "已达标脱开" : "★到上限仍未达标")
                              << ", 恢复连续转向 (当前转角指令 " << turn_cmd_deg
                              << "°, rel=" << rel_pitch << "°);  保持期 "
                              << SA_ESC_HOLD_SEC << "s 内 vx 下限 " << SA_ESC_HOLD_VX_MIN
                              << std::endl;
                    esc_total_sec += e;
                    esc = SaEsc::NONE;
                    esc_last_end = now;
                    // ★v15 脱困后保持期: 别让下坡收油立刻把 vx 按回 0.19
                    esc_hold_until = now + std::chrono::milliseconds(
                                         (int)(SA_ESC_HOLD_SEC * 1000));
                    // 脱困后重置计时, 别立刻二次触发
                    for (int k = 0; k < 2; ++k) foot_low_act[k] = false;
                    slip_ref_t = now;
                }
            } else if (turning_allowed) {
                double cool = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  now - esc_last_end).count() / 1000.0;
                if (cool >= SA_ESC_COOLDOWN_SEC) {
                    float mv1 = movedOver(SA_STUCK_POS_WIN_SEC);
                    if (mv1 < SA_STUCK_POS_MOVE_M) {
                        esc = SaEsc::PUSH; esc_start = now; ++esc_push_cnt;
                        esc_ref_x = px; esc_ref_y = py;
                        std::cout << "\n[骑棱卡死] " << SA_STUCK_POS_WIN_SEC
                                  << "s 只走了 " << mv1 << "m (rel=" << rel_pitch
                                  << "° roll=" << roll_deg << "°) —— 停转 + vx="
                                  << SA_ESC_PUSH_VX << " 猛推到推进 " << SA_ESC_GOAL_M
                                  << "m 为止 (上限 " << SA_ESC_MAX_SEC
                                  << "s) 脱困 (第 " << esc_push_cnt << " 次)"
                                  << std::endl;
                    }
                }
            }

            // ---- 转角指令积分 ----
            float yaw_rel_deg = turn_sign
                              * rad2deg(normalize_180_rad(yaw_now - entry_yaw));
            float lag = turn_cmd_deg - yaw_rel_deg;
            if (lag > lag_max) lag_max = lag;

            bool can_integrate = turning_allowed && !turn_cmd_full
                              && (esc == SaEsc::NONE);
            if (can_integrate && lag > SA_TURN_MAX_LAG_DEG) {
                can_integrate = false;
                if (!lag_pausing) {
                    lag_pausing = true;
                    std::cout << "\n[转角暂停] 实际 " << yaw_rel_deg
                              << "° 落后指令 " << turn_cmd_deg << "° 达 " << lag
                              << "° (>" << SA_TURN_MAX_LAG_DEG
                              << "°), 暂停积分等它跟上" << std::endl;
                }
            } else if (lag_pausing && lag <= SA_TURN_MAX_LAG_DEG) {
                lag_pausing = false;
                std::cout << "\n[转角恢复] 已跟上 (落后 " << lag << "°), 继续转"
                          << std::endl;
            }
            // ★v15 登顶前转角闩锁封顶 (取代 v10~v14 的"深跨级封顶")
            //   旧版判据 rel ≤ -20 且 cmd ≥ 35: 门槛整个落在危险区之外,
            //   0811 五次跑一次都没触发过 —— 因为半坡平台期在 -12~-18。
            //   新判据: relF 本次跑到过 -10° (机体已压平上顶面) 之前, cmd 硬封 40°。
            if (!top_armed && rel_f >= SA_TOP_ARM_REL_DEG && turn_started) {
                top_armed = true;
                top_armed_t = phase_sec;
                top_armed_cmd = turn_cmd_deg;
                std::cout << "\n[登顶前封顶-解除] t=" << phase_sec << "s relF="
                          << rel_f << "° (门槛 " << SA_TOP_ARM_REL_DEG
                          << "°), 此刻累计转角 " << turn_cmd_deg
                          << "° (好跑次 29~34°, 坏跑次 48~57°, 几何临界 45.6°)"
                          << " —— 封顶永久解除" << std::endl;
            }
            if (!top_armed && turn_started
                && phase_sec >= turn_start_sec + SA_TOP_ARM_MAX_SEC) {
                top_armed = true;
                top_armed_fb = true;
                top_armed_t = phase_sec;
                top_armed_cmd = turn_cmd_deg;
                std::cout << "\n[登顶前封顶-兜底解除] t=" << phase_sec
                          << "s 开转后 " << SA_TOP_ARM_MAX_SEC << "s 仍没到 relF≥"
                          << SA_TOP_ARM_REL_DEG << "° (当前 " << rel_f
                          << "°), 强制解除, 否则封在 " << SA_TOP_TURN_CAP_DEG
                          << "° 永远转不满 " << SA_TURN_DEG << "°" << std::endl;
            }
            if (can_integrate && !top_armed
                && turn_cmd_deg >= SA_TOP_TURN_CAP_DEG) {
                can_integrate = false;
                if (!top_capped) {
                    top_capped = true;
                    std::cout << "\n[登顶前封顶] 累计转角已到 " << turn_cmd_deg
                              << "° 上限, 而 relF=" << rel_f << "° 还没压平上顶面"
                              << " —— 暂停转向, 先把台阶爬上去 (再转下去左后脚会够不到)"
                              << std::endl;
                }
            } else if (top_capped && top_armed) {
                top_capped = false;
            }

            if (can_integrate) {
                if (!turn_started) {
                    turn_started = true;
                    turn_start_sec = phase_sec;
                    std::cout << "\n[开始转向] t=" << phase_sec << "s rel="
                              << rel_pitch << "° u=" << u << " 起始转速 "
                              << turn_rate << "°/s —— 全程连续变速, 目标 "
                              << SA_TURN_DEG << "°" << std::endl;
                }
                turn_cmd_deg += turn_rate * (float)dt;
                if (turn_cmd_deg >= SA_TURN_DEG) {
                    turn_cmd_deg = SA_TURN_DEG;
                    turn_cmd_full = true;
                    std::cout << "\n[转角指令满] t=" << phase_sec << "s 指令已到 "
                              << SA_TURN_DEG << "°, 实际 " << yaw_rel_deg
                              << "° (落后 " << lag << "°), rel=" << rel_pitch
                              << "° dist=" << dist << "m —— 转入纯 P 收尾"
                              << std::endl;
                }
            }

            // ---- yaw 闭环 ----
            yaw_ref = normalize_180_rad(entry_yaw + turn_sign * deg2rad(turn_cmd_deg));
            yaw_err_deg = rad2deg(normalize_180_rad(yaw_ref - yaw_now));
            float omega_ff = 0.0f;
            float kp = SA_YAW_KP_STRAIGHT, om_max = SA_OMEGA_MAX_STRAIGHT;
            float fb_max = SA_OMEGA_MAX_STRAIGHT;
            if (turning_allowed) {
                kp = SA_YAW_KP; om_max = SA_OMEGA_MAX; fb_max = SA_YAW_FB_MAX;
                if (can_integrate) omega_ff = turn_sign * deg2rad(turn_rate);
            }
            float omega_fb = clampf(kp * yaw_err_deg, -fb_max, fb_max);
            omega_cmd = clampf(omega_ff + omega_fb, -om_max, om_max);

            // ---- vx ----
            float vx_base = SA_VX_CLIMB_DEEP
                          + (SA_VX_CLIMB_TOP - SA_VX_CLIMB_DEEP) * u;
            if (in_launch) vx_base = SA_LAUNCH_VX;   // ★v11 起步冲刺段用固定大推力

            // 打滑加力 (脱困中不加)
            if ((turning_allowed || in_launch) && esc == SaEsc::NONE) {
                double since = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   now - slip_ref_t).count() / 1000.0;
                if (since >= SA_SLIP_CHECK_SEC) {
                    float mv = movedOver(SA_SLIP_CHECK_SEC);
                    if (mv < SA_SLIP_MOVE_M
                        && vx_boost < SA_SLIP_VX_BOOST_MAX - 1e-6f) {
                        float old = vx_boost;
                        vx_boost = std::min(SA_SLIP_VX_BOOST_MAX,
                                            vx_boost + SA_SLIP_VX_STEP);
                        std::cout << "\n[打滑加力] " << SA_SLIP_CHECK_SEC
                                  << "s 只走了 " << mv << "m (rel=" << rel_pitch
                                  << "°), vx 加力 " << old << " → " << vx_boost
                                  << std::endl;
                    }
                    slip_ref_t = now;
                }
            }
            // ★v13 连续下坡收油: rel 从 REL0 涨到 REL1, vx 连续降到 SA_DESC_VX_MIN
            //   (取代 v12 的二值越沿减速 —— 越往下栽越该收, 不该是台阶函数)
            // ★v14 收油只作用在 vx_base 上, 打滑加力在收油之后再加。
            //   v13 是拿含加力的 vx_cmd 去收, 结果加力被 min 掉 (1552: 加满 +0.15
            //   仍然只发出 0.25), 而加力的判据本身就是"走不动了" —— 最需要的时候
            //   被抵消。vx_boost 全程为 0 的跑次 (1538/1545) 两式完全等价。
            float vx_ramp = vx_base;
            if (rel_pitch > SA_DESC_REL0_DEG) {
                float dcs = (rel_pitch - SA_DESC_REL0_DEG)
                          / (SA_DESC_REL1_DEG - SA_DESC_REL0_DEG);
                dcs = clampf(dcs, 0.0f, 1.0f);
                float vx_desc = vx_base + (SA_DESC_VX_MIN - vx_base) * dcs;
                vx_ramp = std::min(vx_base, vx_desc);
                if (!edge_slow && dcs > 0.5f) {
                    edge_slow = true;
                    std::cout << "\n[下坡收油] rel_pitch=" << rel_pitch
                              << "°, 基础 vx 收到 " << vx_ramp << " (收油系数 " << dcs
                              << "), 加力 +" << vx_boost << " 后实发 "
                              << (vx_ramp + vx_boost)
                              << " (omega 保持, 继续转)" << std::endl;
                } else if (edge_slow && dcs < 0.25f) {
                    edge_slow = false;
                    std::cout << "\n[下坡收油解除] rel_pitch=" << rel_pitch
                              << "°, 恢复正常推进" << std::endl;
                }
            } else if (edge_slow) {
                edge_slow = false;
                std::cout << "\n[下坡收油解除] rel_pitch=" << rel_pitch
                          << "°, 恢复正常推进" << std::endl;
            }
            vx_cmd = vx_ramp + vx_boost;   // ★v14 加力加在收油之后

            // ★v15 脱困后保持期: 刚推出来时 rel 往往已 >0, 收油会立刻把 vx 打回
            //   0.19, 1636 三次猛推结束后的 0.6s 只走了 1.4/2.2/1.0cm 就又停了。
            //   这里给 SA_ESC_HOLD_SEC 秒的下限保护, 让它先把身子带出来。
            if (esc == SaEsc::NONE && now < esc_hold_until
                && vx_cmd < SA_ESC_HOLD_VX_MIN) {
                if (!esc_holding) {
                    esc_holding = true;
                    std::cout << "\n[脱困保持期] vx " << vx_cmd << " → "
                              << SA_ESC_HOLD_VX_MIN << " (rel=" << rel_pitch
                              << "°, 收油暂时让位 " << SA_ESC_HOLD_SEC << "s)"
                              << std::endl;
                }
                vx_cmd = SA_ESC_HOLD_VX_MIN;
                esc_hold_sec_total += dt;
            } else if (esc_holding && now >= esc_hold_until) {
                esc_holding = false;
                std::cout << "\n[脱困保持期结束] 恢复正常收油 (rel=" << rel_pitch
                          << "°)" << std::endl;
            }

            // 脱困动作覆盖
            if (esc == SaEsc::PUSH) { vx_cmd = SA_ESC_PUSH_VX; omega_cmd = 0.0f; }

            // ---- 仅日志: 登顶判定 (不控制任何东西) ----
            if (!summit_logged && slope_seen
                && min_rel_seen <= SA_SUMMIT_DEPTH_DEG
                && (rel_f - min_rel_seen) >= SA_SUMMIT_RECOVERY_DEG
                && rel_f >= SA_SUMMIT_CEIL_DEG) {
                summit_logged = true;
                summit_dist = dist;
                summit_t = phase_sec;
                summit_turn = turn_cmd_deg;
                std::cout << "\n[登顶判定] t=" << phase_sec << "s dist=" << dist
                          << "m rel=" << rel_f << "° (最深 " << min_rel_seen
                          << "°), 此刻累计转角 " << turn_cmd_deg
                          << "° —— 仅记录, 不触发任何动作" << std::endl;
            }

            // ---- 转身完成 -> DOWN ----
            bool aligned = (std::fabs(yaw_err_deg) < SA_TURN_DONE_TOL);
            if (turn_cmd_full && aligned && esc == SaEsc::NONE) {
                std::cout << "\n[转身完成] t=" << phase_sec << "s  yaw="
                          << rad2deg(yaw_now) << "° 目标=" << rad2deg(final_yaw)
                          << "° 误差=" << yaw_err_deg << "°  rel_pitch=" << rel_pitch
                          << "° roll=" << roll_deg << "° dist=" << dist
                          << "m → 进 DOWN" << std::endl;
                phase = SaPhase::DOWN;
                phase_start = now;
                down_vx = SA_VX_DOWN;
                down_ref_x = px; down_ref_y = py;
                down_ref_t = now;
                descended = (max_rel_seen > SA_DOWN_CONFIRM_REL_DEG);
                continue;
            }
            if (phase_sec >= SA_CLIMB_MAX_SEC) {
                std::cout << "\n[CLIMB 兜底] " << SA_CLIMB_MAX_SEC
                          << "s 到点, 转角指令 " << turn_cmd_deg << "°, yaw 误差 "
                          << yaw_err_deg << "° (脱困 猛推" << esc_push_cnt
                          << "次) —— 不判失败, 直接进 DOWN (DOWN 会继续把航向拉回目标)"
                          << std::endl;
                phase = SaPhase::DOWN;
                phase_start = now;
                down_vx = SA_VX_DOWN;
                down_ref_x = px; down_ref_y = py;
                down_ref_t = now;
                descended = (max_rel_seen > SA_DOWN_CONFIRM_REL_DEG);
                continue;
            }
        }
        // ============ DOWN: 下台阶 ============
        else if (phase == SaPhase::DOWN) {
            yaw_ref = final_yaw;
            yaw_err_deg = rad2deg(normalize_180_rad(yaw_ref - yaw_now));
            omega_cmd = clampf(SA_YAW_KP_STRAIGHT * yaw_err_deg,
                               -SA_OMEGA_MAX_STRAIGHT, SA_OMEGA_MAX_STRAIGHT);

            if (rel_pitch > 0.0f) {
                double since = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   now - down_ref_t).count() / 1000.0;
                if (since >= SA_DOWN_STUCK_CHECK_SEC) {
                    float mdx = px - down_ref_x, mdy = py - down_ref_y;
                    float moved = std::sqrt(mdx * mdx + mdy * mdy);
                    if (moved < SA_DOWN_STUCK_MOVE_M
                        && down_vx < SA_DOWN_STUCK_VX_MAX - 1e-6f) {
                        float old = down_vx;
                        down_vx = std::min(SA_DOWN_STUCK_VX_MAX,
                                           down_vx + SA_DOWN_STUCK_VX_STEP);
                        std::cout << "\n[下台阶卡住] rel_pitch=" << rel_pitch
                                  << "° 但 " << SA_DOWN_STUCK_CHECK_SEC
                                  << "s 只走了 " << moved << "m, vx " << old
                                  << " → " << down_vx << std::endl;
                    }
                    down_ref_x = px; down_ref_y = py; down_ref_t = now;
                }
            } else {
                down_ref_x = px; down_ref_y = py; down_ref_t = now;
            }

            vx_cmd = down_vx;

            if (!descended && max_rel_seen > SA_DOWN_CONFIRM_REL_DEG) {
                descended = true;
                std::cout << "\n[下坡确认] 相对 pitch 最大已到 " << max_rel_seen
                          << "° (>" << SA_DOWN_CONFIRM_REL_DEG << "°)" << std::endl;
            }
            bool back_flat = (std::fabs(rel_pitch) < SA_DOWN_FLAT_REL_DEG);

            if (phase_sec >= SA_DOWN_MIN_SEC && descended && back_flat) {
                std::cout << "\n[到底] max_rel=" << max_rel_seen << "° 当前 rel="
                          << rel_pitch << "° 已回平地, 进 EXIT" << std::endl;
                phase = SaPhase::EXIT;
                phase_start = now;
            } else if (phase_sec >= SA_DOWN_MAX_SEC) {
                std::cout << "\n[超时] DOWN " << SA_DOWN_MAX_SEC << "s, max_rel="
                          << max_rel_seen << "° 当前 rel=" << rel_pitch
                          << "° —— 不判失败, 强制 EXIT" << std::endl;
                phase = SaPhase::EXIT;
                phase_start = now;
            }
        }
        // ============ EXIT: 切回常规步态 ============
        else if (phase == SaPhase::EXIT) {
            softStop(sport, 0.5);
            std::cout << "\n[步态] FreeWalk → StaticWalk (台阶完成, 切回常规步态)..."
                      << std::endl;
            ret = sport.StaticWalk();
            std::cout << "  StaticWalk() ret=" << ret << std::endl;
            {
                auto t = std::chrono::steady_clock::now();
                while (g_running) {
                    double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - t).count() / 1000.0;
                    if (el >= SA_STATICWALK_WAIT_SEC) break;
                    sport.Move(0.0f, 0.0f, 0.0f);
                    usleep(20 * 1000);
                }
            }
            phase = SaPhase::DONE;
            continue;
        }

        // ---------- 下发 ----------
        sport.Move(vx_cmd, 0.0f, omega_cmd);

        // ---------- 全量日志 (50Hz) ----------
        //   u    = 转速调制量 (0=深跨级慢转, 1=接近水平全速)
        //   rate = 本帧转速指令 °/s
        //   cmd  = 累计转角指令 °
        //   esc  = N 正常 / P 猛推脱困 / L 后腿悬空慢行 / S 越沿减速
        printf("[STAIR][%s t=%5.2f/%5.1f] p=%+6.1f r=%+6.1f y=%+7.1f Y0=%+7.1f | tilt=%5.1f "
               "| rel_p=%+6.1f relF=%+6.1f rel_t=%5.1f min=%+6.1f max=%+6.1f "
               "| F=%.2f L=%.2f R=%.2f | dx=%+.2f dy=%+.2f d=%.2f "
               "| ff[%d,%d,%d,%d] | u=%.2f rate=%4.1f cmd=%5.1f rIn=%4.1f gate=%.2f "
               "| vx=%.2f om=%+.2f yawT=%+7.1f e=%+6.1f | slope=%c esc=%c\n",
               saPhaseName(phase), phase_sec, total_sec,
               pitch_deg, roll_deg, rad2deg(yaw_now),
               rad2deg(g_global_yaw_cum_rad.load()), tilt,
               rel_pitch, rel_f, rel_tilt, min_rel_seen, max_rel_seen,
               g_front_dist.load(), g_left_dist.load(), g_right_dist.load(),
               dx, dy, dist,
               g_foot_force[0].load(), g_foot_force[1].load(),
               g_foot_force[2].load(), g_foot_force[3].load(),
               u, turn_rate, turn_cmd_deg, roll_res_mag, roll_gate,
               vx_cmd, omega_cmd, rad2deg(yaw_ref), yaw_err_deg,
               slope_seen ? 'Y' : (forced_turn ? 'F' : 'N'),
               esc == SaEsc::PUSH ? 'P' : (esc_holding ? 'H' : (edge_slow ? 'S' : 'N')));
        fflush(stdout);

        if (guiWaitKey(1) == 27) { g_running = false; break; }
        usleep(20 * 1000);   // 50 Hz
    }

    if (phase != SaPhase::DONE) {
        sport.Move(0, 0, 0);
        usleep(200 * 1000);
    }

    double run_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - start_all).count() / 1000.0;
    std::cout << "\n========== 台阶 v15 结束 ==========" << std::endl;
    std::cout << "最终阶段:     " << saPhaseName(phase) << std::endl;
    std::cout << "起步冲刺:     " << SA_LAUNCH_SEC << "s @ vx=" << SA_LAUNCH_VX
              << (launch_done_logged ? " (已完成)" : " (★未跑完就退出了)") << std::endl;
    std::cout << "坡态:         " << (slope_seen ? "已锁存" : (forced_turn ? "★从未锁存(强制开转)" : "从未锁存"))
              << std::endl;
    std::cout << "转角指令:     " << turn_cmd_deg << "° / " << SA_TURN_DEG
              << "° (最大落后 " << lag_max << "°"
              << (turn_cmd_full ? ", 已转满" : ", ★未转满") << ")" << std::endl;
    std::cout << "登顶判定:     ";
    if (summit_logged) std::cout << "t=" << summit_t << "s dist=" << summit_dist
                                 << "m ★此刻累计转角=" << summit_turn
                                 << "°  (好跑次 1538/1545 = 22.8/24.3°;  坏跑次 1552 = 42.9°)";
    else               std::cout << "(全程未满足, 仅日志用, 不影响动作)";
    std::cout << std::endl;
    std::cout << "脱困:         猛推 " << esc_push_cnt << " 次 (累计 "
              << esc_total_sec << "s);  ★v15 保持期生效累计 " << esc_hold_sec_total
              << "s   (1636 坏跑次: 4 次 × 0.6s 定时, 单次只推进 2.2~7.1cm)" << std::endl;
    std::cout << "★登顶前封顶:  ";
    if (top_armed) {
        std::cout << "解除于 t=" << top_armed_t << "s, 此刻累计转角 "
                  << top_armed_cmd << "°"
                  << (top_armed_fb ? " ★兜底解除(relF 全程没到 -10°)" : "");
        if (top_armed_cmd >= SA_TOP_TURN_CAP_DEG - 0.5f)
            std::cout << "  ★曾被封顶 (半坡上转角本来会超)";
    } else {
        std::cout << "★全程未解除 (转角一直封在 " << SA_TOP_TURN_CAP_DEG << "°)";
    }
    std::cout << "   (好跑次 29~34° / 坏跑次 48~57° / 几何临界 45.6°)" << std::endl;
    std::cout << "★深跨级确认:  ";
    if (deep_armed) std::cout << "t=" << deep_armed_t << "s"
                              << (deep_armed_fb ? " ★兜底解锁(全程没到过门槛)" : " (min_rel 到达门槛)");
    else            std::cout << "★全程未确认 (u 一直锁 0, 转速恒 "
                              << SA_TURN_RATE_SLOW_DPS << "°/s)";
    std::cout << std::endl;
    std::cout << "★后腿悬空:    " << foot_low_events << " 次, 单次最长 "
              << foot_low_worst << "s   (阈值 ff≤" << SA_FOOT_LOW_TH
              << "; 按此阈值回放 0811: 1538=0.90s / 1545=0.80s / 1552坏=2.10s)" << std::endl;
    std::cout << "★roll 闸门:   ★内侧残差最大 " << roll_res_worst << "° (软"
              << SA_ROLL_GATE_SOFT_DEG << "/硬" << SA_ROLL_GATE_HARD_DEG
              << "), 刹车累计 " << roll_brake_sec
              << "s   (2053坏: 内侧15.4°/2.82s;  1423误刹: 内侧仅4.1°)" << std::endl;
    std::cout << "打滑加力:     +" << vx_boost << " (上限 +" << SA_SLIP_VX_BOOST_MAX << ")"
              << std::endl;
    std::cout << "平均循环频率: " << (run_sec > 0 ? n_loops / run_sec : 0) << " Hz" << std::endl;
    std::cout << "净位移:       dx=" << (g_pos_x.load() - init_x)
              << "m dy=" << (g_pos_y.load() - init_y) << "m" << std::endl;
    std::cout << "UP 最深 rel:  " << min_rel_seen << "°" << std::endl;
    std::cout << "下行 max rel: " << max_rel_seen << "°" << std::endl;
    std::cout << "最终 yaw:     " << rad2deg(g_yaw_rad.load()) << "° (相对入口 "
              << rad2deg(normalize_180_rad(g_yaw_rad.load() - entry_yaw))
              << "°, 目标 " << turn_sign * SA_TURN_DEG << "°)" << std::endl;
    std::cout << "结果:         " << (danger_hold ? "★危险 hold (等人工)" : "正常走完")
              << std::endl;
    std::cout << "===================================" << std::endl;

    return !danger_hold;
}

// ============================================================
//  机械臂抓取模块 (从 shibie.cpp 移植, 放进独立命名空间避免与巡线全局重名)
//    - 检测 = shibie step-2 修复版 (暗板中位数基准 + 板内 MAD 阈值 + 板范围限制
//      + solidity 打分); 默认参数与 shibie 当前一致 → 现场在 shibie 调好后同步到这里。
//    - 串口 115200 8N1:
//        第一抓取 "1\n"=就位, "2,cx,cy\n"=抓取;
//        第二抓取 "3\n"=进入识别状态, "4,cx,cy\n"=第二抓取。
//    - runMaterialGrab(sport, window): 独立打开物资摄像头 /dev/video1, 显示识别窗口, 受 ::g_running 控制;
//      串口打不开 / 窗口内未锁定目标都安全跳过, 绝不阻塞整条流程。
// ============================================================
namespace material_grab {

// ---- 检测可调参数 (MIRROR shibie.cpp, 现场调好后同步过来) ----
int g_roi_left_pct = 8, g_roi_right_pct = 92, g_roi_top_pct = 8, g_roi_bottom_pct = 92;
int g_thr_mode = 0;       // 0 = median+MAD, 1 = Otsu
int g_mad_k_x10 = 35;     // MAD 倍率 x10
int g_diff_thr = 25;      // 距离阈值地板
int g_min_area = 900;
int g_max_area_p = 80;
int g_blur_size = 5;
int g_morph_size = 7;
int g_edge_margin = 10;
int g_use_bright_gate = 0;
int g_obj_v_min = 90;
int g_obj_v_delta = 35;
int g_solidity_min = 55;
int g_central_w = 0;

// ---- 机械臂 / 抓取可调参数 ----
std::string  g_arm_serial_dev = "/dev/ttyUSB1"; // ★ 命令行 armdev= 可覆盖; dogonly 下不会打开
const double ARM_BOOT_WAIT_SEC = 6.0;       // ★ 中途重新打开串口后的等待。v3 起正常流程
                                             //   串口在 main 启动时就打开且全程持久, 这个等待
                                             //   只在"比赛中途 USB 掉线重连"的兜底路径才会走到;
                                             //   臂若因掉电重启需要 ~10s, 2.0 太短, 加到 6.0
                                             //   (再配合 sendArmLineAcked 的 ACK 重试兜底)。
// (协议 v1 遗留, 握手 v2 后不再使用, 留作参考值)
const double ARM_CMD1_MONITOR_SEC = 1.5;
const double ARM_SETTLE_SEC = 10.0;         // 旧: 发"1"后固定等待
const int    ARM_STABLE_FRAMES = 8;         // 连续多少帧中心稳定才发抓取
const int    ARM_STABLE_TOL_PX = 12;        // 稳定判据: 中心位移阈值(px)
const int    ARM_GRAB_REPLY_TIMEOUT_MS = 10000;
const double ARM_AFTER_REPLY_WAIT_SEC = 15.0; // 第一抓取坐标发出后固定等待, 不等机械臂回复
const double SECOND_GRAB_WAIT_SEC = 100.0;    // 第二抓取总等待窗口
const double ARM_SECOND_AFTER_REPLY_WAIT_SEC = 30.0; // 第二抓取坐标发出后固定等待, 不等机械臂回复
const double GRAB_NO_TARGET_SEC = 5.0;       // ★ 九轮 (用户: 能扫到就扫得快): 8→5,
                                             //   5s 没扫到就后退补救, 省时又多争取补救机会
// ★ 2026-07-06 第五轮 (0706 第六次实验: 第一抓取一直识别不到, 靠手推才触发 → -10/-20):
//   识别失败链的最后一环兜底 —— 补救都用完、之后又连续 GRAB_NO_TARGET_SEC
//   没有任何有效识别 → 不再干等窗口耗尽, 直接对【标定网格中心点】盲发抓取坐标。
//   物资摆放公差通常以标定中心为基准, 盲抓中心点成功率远高于什么都不发 (0 分)。
//   坐标 = 各自标定网格的中心点 (见 CAL_GRID_1/2), 现场如实测物资常偏某方向可微调。
const int    GRAB_BLIND_FALLBACK_ENABLE = 1;      // 0 = 关闭盲抓兜底
const double GRAB1_BLIND_WX = 0.0;                // 第一抓取兜底 = CAL_GRID_1 中心 {0,340}
const double GRAB1_BLIND_WY = 340.0;
const double GRAB2_BLIND_WX = -20.0;              // 第二抓取兜底 = CAL_GRID_2 中心 {-20,-350}
const double GRAB2_BLIND_WY = -350.0;
const int    GRAB_NO_TARGET_MAX_NUDGES = 3;  // ★ 第二抓取升级补救总段数 (退/左/前);
                                             //   且改为"滚动窗口": 距上一次有效识别满 5s 就触发,
                                             //   偶发一帧误检只是重置计时, 不再永久禁用回退。
// ★ 十轮 (用户, 仅第一抓取/抓取平台一): 补救从"两退"改为三段升级 ——
//   ①后退一步 → 仍无识别 5s → ②水平向右挪一步 → 仍无识别 5s → ③再后退一步
//   → 仍无识别 5s → 盲发标定中心点。每段移动都复用后退的同一组数值参数
//   (0.15 m/s 起步踢 0.5s = 一步 ~7.5cm), 只换方向。
const int    GRAB1_ESCALATE_STAGES = 3;      // 第一抓取升级补救总段数 (退/右/退)
const float  GRAB_NO_TARGET_BACK_KICK_VX = -0.15f; // 未见物资时,先用较大速度启动后退
const double GRAB_NO_TARGET_BACK_KICK_SEC = 0.50;  // 后退启动脉冲时长
const float  GRAB_NO_TARGET_BACK_VX = -0.15f;      // 保持可启动速度, 通过缩短时长控距离
const double GRAB_NO_TARGET_BACK_SEC = 0.50; // 未见物资时,狗后退补救总时长
// matcam= 可以是索引 ("1") 也可以是稳定路径 ("/dev/v4l/by-id/...-video-index0")
std::string g_material_cam_id = "1";         // material camera, independent from line camera
const bool   MAT_CAM_LOCK_WB = true;
const int    MAT_CAM_WB_TEMP = 4600;
const bool   MAT_CAM_LOCK_EXPOSURE = true;
const int    MAT_CAM_EXPOSURE_VAL = 156;
const bool   MAT_CAM_LOCK_GAIN = true;
const int    MAT_CAM_GAIN_VAL = 0;
int g_exposure = MAT_CAM_EXPOSURE_VAL;
int g_gain = MAT_CAM_GAIN_VAL;
// ★ 两个抓取点位光照可能不同 (抓取平台在场地中央, 中转平台在角落):
//   分别给曝光值, 命令行 matexp1= / matexp2= 覆盖, 各自标定。
int g_exposure_first  = MAT_CAM_EXPOSURE_VAL;   // 第一抓取 (抓取平台)
int g_exposure_second = MAT_CAM_EXPOSURE_VAL;   // 第二抓取 (中转平台)
bool g_second_grab_nudged = false;              // 第二抓取是否触发过至少一次无识别补救
// ★ 黑板判定健康度回写 (HUD 显示用): dark_frac 是 Otsu 暗簇占比, >=0.15 才算找到黑板
double g_last_dark_frac = 0.0;
const int    GRAB_FLUSH_FRAMES = 5;         // 检测前丢弃的陈旧缓存帧

// ★ 十轮: 通用"挪一步"补救 —— 后退/右移/前移共用同一组速度/时长参数
//   (kick 段和保持段分别给 vx/vy, 只是方向不同; 移完停稳 + 排空相机缓存帧)
static void nudgeMoveAfterNoMaterial(unitree::robot::go2::SportClient& sport,
                                     cv::VideoCapture& cap,
                                     const char* tag,
                                     float vx_kick, float vy_kick,
                                     float vx_hold, float vy_hold,
                                     const char* desc) {
    std::cout << "[" << tag << "] " << GRAB_NO_TARGET_SEC
              << "s 内未识别到物资, 常规步态" << desc << ": vx="
              << vx_kick << " vy=" << vy_kick << ", "
              << GRAB_NO_TARGET_BACK_SEC << "s 后继续识别" << std::endl;
    sport.StaticWalk();
    usleep(500 * 1000);

    auto t_back = std::chrono::steady_clock::now();
    while (::g_running) {
        double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t_back).count() / 1000.0;
        if (el >= GRAB_NO_TARGET_BACK_SEC) break;
        float vx = (el < GRAB_NO_TARGET_BACK_KICK_SEC) ? vx_kick : vx_hold;
        float vy = (el < GRAB_NO_TARGET_BACK_KICK_SEC) ? vy_kick : vy_hold;
        sport.Move(vx, vy, 0.0f);
        usleep(20 * 1000);
    }

    sport.Move(0.0f, 0.0f, 0.0f);
    usleep(300 * 1000);
    for (int i = 0; i < GRAB_FLUSH_FRAMES; ++i) { cv::Mat f; cap >> f; }
}

// 单步后退补救封装; 三段升级逻辑可直接调用 nudgeMoveAfterNoMaterial 组合方向。
static void backOffAfterNoMaterial(unitree::robot::go2::SportClient& sport,
                                   cv::VideoCapture& cap,
                                   const char* tag) {
    nudgeMoveAfterNoMaterial(sport, cap, tag,
                             GRAB_NO_TARGET_BACK_KICK_VX, 0.0f,
                             GRAB_NO_TARGET_BACK_VX, 0.0f, "后退");
}

// ---- ★ 机械臂握手协议 v2 (2026-07-03, 07-04 第二抓改先识别再放) ----
//   第一抓: "1"→R1(识别位), "2,x,y"→D2(抓取)。
//   第二抓: "3"→R3(握料摆识别位), "4,x,y"→D4(放物资1到x-80,y + 抓物资2 x,y)。
//   其它: "0"→D0(复位), "5"→D5/"6"→D6(双平台放置)。"7"→D7 已废弃(旧先放后识别)。
//   狗端每条命令只发一次, 等确认或超时按兜底继续 (绝不重发, 重发会让臂重复动作抽搐)。坐标=毫米。
// ★ 不重发: 超时=固定兜底等待。收不到回执时狗就等满这个时间再继续, 所以要 ≥ 臂动作耗时,
//   否则会在动作没做完时提前开走。宁可等久点也别提前走。
// ★ 九轮 (用户: 看门狗时限 = 六份日志实测最长回复 + 5s, 臂真挂时更早兜底省时):
//   实测最长 R1=3.83 R3=5.32 D2=18.97 D4=24.85 D0=7.82 D5/D6=13.06
const double HS_R1_TIMEOUT_SEC = 9.0;   // "1"识别位就绪 (实测最长 3.83s +5)
const double HS_R3_TIMEOUT_SEC = 11.0;  // "3"第二识别位就绪 (实测最长 5.32s +5)
const double HS_D2_TIMEOUT_SEC = 24.0;  // 第一抓取完成 (实测最长 18.97s +5)
                                        // ★ 十一轮: 32→24 回退 (用户指令)。0708 日志证实故障是
                                        //   USB 设备级掉线 (dmesg: 2.4 口 80min 掉线 130 次),
                                        //   不是回执迟到, 加长超时无意义; 周期重问已能兜住
                                        //   空夹重试的迟到回执 (超时后重问命中臂端去重缓存)
const double HS_D7_TIMEOUT_SEC = 25.0;  // [已废弃] 旧"7"放料+识别位
const double HS_D4_TIMEOUT_SEC = 30.0;  // 第二抓取完成 (实测最长 24.85s +5)
                                        // ★ 十一轮: 38→30 回退 (用户指令), 理由同 D2
const double HS_D0_TIMEOUT_SEC = 13.0;  // "0"复位巡线姿态完成 (实测最长 7.82s +5)
// ---- ★ 协议 v3 (2026-07-05, 与臂固件 v3 配套): 命令级 ACK + 有限重发 ----
//   臂端收到任何一行命令后【毫秒级】先回 "ACK<首字符>" (如 "1"→ACK1, "2,..."→ACK2),
//   动作完成再回 R1/D2/...。狗端发送后等 ACK, 没等到就重发 (最多 ARM_SEND_MAX_TRIES 次)。
//   ★ 重发是安全的: 臂固件 v3 对"与上一条已完成命令相同"的重复行只补发缓存回执、
//     绝不重复执行动作 (去重窗口 2min)。必须与新版臂固件一起使用!
const double ARM_ACK_TIMEOUT_SEC   = 2.0;  // 单次发送后等 ACK 的时长
const int    ARM_SEND_MAX_TRIES    = 3;    // ★ 十轮: 恢复"没等到 ACK 也重发" (总共最多发 3 次)。
                                           //   为什么现在敢: 臂固件 v3 的重复命令去重已经在跑
                                           //   (同一行只补发缓存回执、绝不重做动作), 且串口是
                                           //   按序处理 —— 第一份必然先执行完并缓存, 重发份到达
                                           //   时 100% 命中去重; 而 20~40% 的"狗收不到回复"是
                                           //   臂→狗方向单向丢字 (臂确实收到并执行了), 重发/重问
                                           //   是狗端唯一能主动恢复这条链路的手段。
                                           //   ★ 必须与臂固件 v3/v3.1 (带去重) 配套使用!
const double ARM_STARTUP_DRAIN_SEC = 3.0;  // main 启动预连接后监听臂输出的时长
// ★ 十一轮 (0708 dmesg 实锤): 臂的 CH340 (hub 2.4 口) 80 分钟内掉线/重枚举 130 次,
//   掉线窗从 1s 到 2 分半不等 (descriptor read error -32/-71 = 电气级信号/供电问题)。
//   1251 轮第二抓取失败的直接软件原因: 到位时恰逢掉线窗, 而入口 armPortEnsureOpen
//   只试一次 (armFindDevice 首次 miss 即返回空) = 给设备回来的时间是【0 秒】。
//   这里给关键入口一个有界的重连宽限窗: 每 0.5s 重试一次, 设备回来立即继续。
//   12s 能吃掉 dmesg 里的大部分短掉线窗; 吃不掉长窗时照旧跳过, 不无限等。
const double ARM_OPEN_GRACE_SEC = 12.0;   // 抓取/放置入口等串口重枚举的宽限窗

// ---- ★ 像素→世界(mm) 标定: 单应矩阵 (homography), 每个抓取点位一个 ----
//   标定方法见 xbtss_field_manual.md "物资抓取标定" 一节:
//   用臂在平台上点出 9 个网格点(世界坐标已知) → calmat 模式把物资摆到每个点按 c 采像素
//   → 自动 findHomography 拟合 → 存到 mat_cal1.txt / mat_cal2.txt (启动时自动加载)。
//   ★ 相机装在臂上俯视: 标定时用真实物资采点(检测的是物资顶面中心像素, 记录的是
//     底座中心世界坐标), 顶面视差/斜视/镜头倾斜全部被矩阵一并建模, 无需单独修正。
//   未标定 (valid=0) 时拒绝发送坐标 —— 新臂旧系数不可信, 宁可跳过抓取也不发垃圾。
double g_H1[9] = {1,0,0, 0,1,0, 0,0,1};  int g_H1_valid = 0;   // 第一抓取
double g_H2[9] = {1,0,0, 0,1,0, 0,0,1};  int g_H2_valid = 0;   // 第二抓取
// 标定网格的世界坐标 (臂基座系 mm), 行优先 3x3, 顺序=采点顺序。
// ★ 下面是按旧多项式工作区推的初值, 现场用臂点位后按实际值替换 (手册有流程)。
const int CAL_PTS = 9;
// ★ 2026-07-04 已按实贴标记更新 (值 = 标记时 T:104 发的原始坐标, 不带任何偏移;
//   movecatch 的内部偏移由狗端发送时自动补偿, 见 GRAB1_SEND_X_OFF)
double CAL_GRID_1[CAL_PTS][2] = {
    {-50, 310}, {0, 310}, {50, 310},
    {-50, 340}, {0, 340}, {50, 340},
    {-50, 380}, {0, 380}, {50, 380},
};
// ★ 2026-07-04 按实标网格更新 (采集顺序 = 提示顺序, 残差 3.6mm)
double CAL_GRID_2[CAL_PTS][2] = {
    {-20, -350}, {-20, -310}, {-20, -390},
    { 30, -390}, { 30, -350}, { 30, -310},
    {-60, -310}, {-60, -350}, {-60, -390},
};

// ★ 2026-07-04 定为 0: 标定约定 = 物资直接摆在"发该坐标能抓到"的位置,
//   movecatch 内偏 (x-5/y-20) 被矩阵一并吸收, 狗端不再补偿。
//   若实抓发现系统性偏差 (总往同一方向偏固定距离), 调这两个旋钮, 不用重标。
const double GRAB1_SEND_X_OFF = 0.0;
const double GRAB2_SEND_Y_OFF = 0.0;

static const char* matCalFileName(bool second) {
    return second ? "mat_cal2.txt" : "mat_cal1.txt";
}

// 启动时自动加载标定文件 (9 个 double, 空白分隔); 不存在则保持未标定。
bool loadMatCal(bool second) {
    double* H = second ? g_H2 : g_H1;
    int&  valid = second ? g_H2_valid : g_H1_valid;
    FILE* f = fopen(matCalFileName(second), "r");
    if (!f) {
        std::cout << "[标定] " << matCalFileName(second)
                  << " 不存在, 抓取点位 " << (second ? 2 : 1)
                  << " 未标定 (calmat 采点后自动生成)" << std::endl;
        return false;
    }
    double tmp[9];
    int n = 0;
    for (; n < 9; ++n) { if (fscanf(f, "%lf", &tmp[n]) != 1) break; }
    fclose(f);
    if (n != 9) {
        std::cout << "[标定] " << matCalFileName(second) << " 格式错误 (读到 "
                  << n << "/9 个数), 忽略" << std::endl;
        return false;
    }
    for (int i = 0; i < 9; ++i) H[i] = tmp[i];
    valid = 1;
    std::cout << "[标定] 已加载 " << matCalFileName(second) << std::endl;
    return true;
}

bool saveMatCal(bool second) {
    const double* H = second ? g_H2 : g_H1;
    FILE* f = fopen(matCalFileName(second), "w");
    if (!f) { std::cout << "[标定] 写 " << matCalFileName(second) << " 失败!" << std::endl; return false; }
    for (int r = 0; r < 3; ++r)
        fprintf(f, "%.10g %.10g %.10g\n", H[r*3], H[r*3+1], H[r*3+2]);
    fclose(f);
    std::cout << "[标定] 已保存 " << matCalFileName(second) << std::endl;
    return true;
}

// 像素 → 世界mm (透视除法)。valid=0 或退化时返回 false。
bool pixelToWorldMM(const double H[9], int valid, double px, double py,
                    double& wx, double& wy) {
    if (!valid) return false;
    double w = H[6]*px + H[7]*py + H[8];
    if (std::fabs(w) < 1e-9) return false;
    wx = (H[0]*px + H[1]*py + H[2]) / w;
    wy = (H[3]*px + H[4]*py + H[5]) / w;
    return true;
}

// calmat 采点模式用: updateMaterialPreview 每帧把按键回写到这里 (-1=无)
int g_last_key = -1;

static int oddKernel(int v) { v = std::max(1, v); if ((v % 2) == 0) v += 1; return v; }

static cv::Rect makeRoi(int w, int h) {
    int l = std::clamp(g_roi_left_pct, 0, 99);
    int r = std::clamp(g_roi_right_pct, 1, 100);
    int t = std::clamp(g_roi_top_pct, 0, 99);
    int b = std::clamp(g_roi_bottom_pct, 1, 100);
    if (r <= l + 2) r = std::min(100, l + 3);
    if (b <= t + 2) b = std::min(100, t + 3);
    int x0 = w * l / 100, x1 = w * r / 100, y0 = h * t / 100, y1 = h * b / 100;
    x0 = std::clamp(x0, 0, w - 2); y0 = std::clamp(y0, 0, h - 2);
    x1 = std::clamp(x1, x0 + 2, w); y1 = std::clamp(y1, y0 + 2, h);
    return cv::Rect(x0, y0, x1 - x0, y1 - y0);
}

static bool touchesEdge(const cv::Rect& box, cv::Size size, int margin) {
    margin = std::max(0, margin);
    return box.x <= margin || box.y <= margin ||
           box.x + box.width >= size.width - margin ||
           box.y + box.height >= size.height - margin;
}

static double channelMedian(const cv::Mat& ch8u, const cv::Mat& mask = cv::Mat()) {
    int histSize = 256; float range[] = {0.f, 256.f}; const float* histRange = range;
    cv::Mat hist;
    cv::calcHist(&ch8u, 1, 0, mask, hist, 1, &histSize, &histRange);
    double total = mask.empty() ? (double)ch8u.total() : (double)cv::countNonZero(mask);
    if (total < 1.0) return 127.0;
    double accum = 0.0;
    for (int i = 0; i < histSize; ++i) {
        accum += (double)hist.at<float>(i);
        if (accum >= total * 0.5) return (double)i;
    }
    return 127.0;
}

struct DetectResult {
    bool found = false;
    bool board_found = false;   // ★ 黑板是否锁定 (板范围限制是否真正生效)
    cv::Point bbox_center{0, 0};
    cv::Point moment_center{0, 0};
    cv::Rect bbox;
    double area = 0.0;
    std::vector<cv::Point> contour;
};

static bool detectMaterial(const cv::Mat& frame, DetectResult& out,
                           cv::Mat& full_mask, cv::Rect& roi) {
    out = DetectResult{};
    full_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    roi = makeRoi(frame.cols, frame.rows);

    cv::Mat crop = frame(roi).clone();
    int blur_k = oddKernel(g_blur_size);
    if (blur_k > 1) cv::GaussianBlur(crop, crop, cv::Size(blur_k, blur_k), 0);

    cv::Mat lab; cv::cvtColor(crop, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> labch; cv::split(lab, labch);

    // 背景 = 暗簇(黑板)中位数; 阈值统计也只在黑板像素上算 (见 shibie 注释)
    cv::Mat dark_mask;
    cv::threshold(labch[0], dark_mask, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    double dark_frac = (double)cv::countNonZero(dark_mask) / (double)labch[0].total();
    g_last_dark_frac = dark_frac;   // ★ 回写给 HUD, 现场标定时盯这个数
    bool have_board = dark_frac >= 0.15;
    cv::Mat stat_mask = have_board ? dark_mask : cv::Mat();

    double medL = channelMedian(labch[0], stat_mask);
    double meda = channelMedian(labch[1], stat_mask);
    double medb = channelMedian(labch[2], stat_mask);

    cv::Mat board_region;
    if (have_board) {
        cv::Mat closed;
        cv::Mat bk = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
        cv::morphologyEx(dark_mask, closed, cv::MORPH_CLOSE, bk);
        std::vector<std::vector<cv::Point>> bcs;
        cv::findContours(closed, bcs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        double bbest = 0.0; int bidx = -1;
        for (int i = 0; i < (int)bcs.size(); ++i) {
            double a = cv::contourArea(bcs[i]);
            if (a > bbest) { bbest = a; bidx = i; }
        }
        if (bidx >= 0) {
            cv::Rect bb = cv::boundingRect(bcs[bidx]);
            double frac = (double)bb.area() / (double)(crop.cols * crop.rows);
            if (frac >= 0.10) {
                int pad = (int)(0.04 * std::max(crop.cols, crop.rows));
                bb.x -= pad; bb.y -= pad; bb.width += 2 * pad; bb.height += 2 * pad;
                bb &= cv::Rect(0, 0, crop.cols, crop.rows);
                board_region = cv::Mat::zeros(crop.size(), CV_8UC1);
                board_region(bb).setTo(255);
            }
        }
    }
    out.board_found = !board_region.empty();   // ★ 板范围限制真正生效才算锁定

    cv::Mat dL, da, db;
    cv::absdiff(labch[0], cv::Scalar(medL), dL);
    cv::absdiff(labch[1], cv::Scalar(meda), da);
    cv::absdiff(labch[2], cv::Scalar(medb), db);
    cv::Mat dLf, daf, dbf;
    dL.convertTo(dLf, CV_32F); da.convertTo(daf, CV_32F); db.convertTo(dbf, CV_32F);
    cv::Mat dist = 0.70f * dLf + 1.25f * daf + 1.25f * dbf;
    cv::Mat distc = cv::min(dist, 255.0);
    cv::Mat dist8u; distc.convertTo(dist8u, CV_8U);

    int thr_final;
    if (g_thr_mode == 1) {
        cv::Mat tmp;
        double otsu = cv::threshold(dist8u, tmp, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        thr_final = (int)std::max(otsu, (double)g_diff_thr);
    } else {
        double med = channelMedian(dist8u, stat_mask);
        cv::Mat absdev; cv::absdiff(dist8u, cv::Scalar(med), absdev);
        double mad = channelMedian(absdev, stat_mask);
        double k = std::max(1, g_mad_k_x10) / 10.0;
        thr_final = (int)std::max(med + k * 1.4826 * mad, (double)g_diff_thr);
    }
    thr_final = std::clamp(thr_final, 1, 254);

    cv::Mat mask_roi;
    cv::threshold(dist8u, mask_roi, thr_final, 255, cv::THRESH_BINARY);
    if (!board_region.empty()) cv::bitwise_and(mask_roi, board_region, mask_roi);

    if (g_use_bright_gate) {
        cv::Mat hsv; cv::cvtColor(crop, hsv, cv::COLOR_BGR2HSV);
        std::vector<cv::Mat> hsvch; cv::split(hsv, hsvch);
        double medV = channelMedian(hsvch[2]);
        int v_thr = std::clamp(std::max(g_obj_v_min, (int)(medV + g_obj_v_delta)), 1, 255);
        cv::Mat bright; cv::threshold(hsvch[2], bright, v_thr, 255, cv::THRESH_BINARY);
        cv::bitwise_and(mask_roi, bright, mask_roi);
    }

    int morph_k = oddKernel(g_morph_size);
    if (morph_k > 1) {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morph_k, morph_k));
        cv::morphologyEx(mask_roi, mask_roi, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask_roi, mask_roi, cv::MORPH_OPEN, kernel);
    }

    mask_roi.copyTo(full_mask(roi));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_score = 0.0, best_area = 0.0; int best_idx = -1;
    double max_area = roi.area() * (std::clamp(g_max_area_p, 5, 100) / 100.0);
    double solidity_min = std::clamp(g_solidity_min, 0, 100) / 100.0;
    cv::Point2f img_c(mask_roi.cols * 0.5f, mask_roi.rows * 0.5f);
    double max_d = std::sqrt((double)img_c.x * img_c.x + (double)img_c.y * img_c.y);
    if (max_d < 1.0) max_d = 1.0;
    for (int i = 0; i < (int)contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area < std::max(1, g_min_area)) continue;
        if (area > max_area) continue;
        cv::Rect box = cv::boundingRect(contours[i]);
        if (touchesEdge(box, mask_roi.size(), g_edge_margin)) continue;
        std::vector<cv::Point> hull; cv::convexHull(contours[i], hull);
        double hull_area = cv::contourArea(hull);
        double solidity = hull_area > 1.0 ? area / hull_area : 0.0;
        if (solidity < solidity_min) continue;
        cv::Point2f bc(box.x + box.width * 0.5f, box.y + box.height * 0.5f);
        double cdist = std::sqrt((double)(bc.x - img_c.x) * (bc.x - img_c.x) +
                                 (double)(bc.y - img_c.y) * (bc.y - img_c.y));
        double centrality = 1.0 - std::min(1.0, cdist / max_d);
        double cfac = 1.0 + (std::max(0, g_central_w) / 100.0) * centrality;
        double score = area * solidity * cfac;
        if (score > best_score) { best_score = score; best_idx = i; best_area = area; }
    }
    if (best_idx < 0) return false;

    std::vector<cv::Point> shifted = contours[best_idx];
    for (auto& p : shifted) { p.x += roi.x; p.y += roi.y; }
    cv::Rect box = cv::boundingRect(shifted);
    cv::Moments m = cv::moments(shifted, false);
    if (std::abs(m.m00) < 1e-6) return false;

    out.found = true; out.area = best_area; out.contour = shifted; out.bbox = box;
    out.bbox_center = cv::Point(box.x + box.width / 2, box.y + box.height / 2);
    out.moment_center = cv::Point((int)(m.m10 / m.m00), (int)(m.m01 / m.m00));
    return true;
}

// ---- 串口 (115200 8N1) ----
static void setupMaterialWindows() {
    if (!g_gui_enabled) return;   // ★ nogui 比赛模式不开窗口
    cv::namedWindow("Material Original", cv::WINDOW_NORMAL);
    cv::namedWindow("Material Mask", cv::WINDOW_NORMAL);
    cv::namedWindow("Material Params", cv::WINDOW_NORMAL);
    cv::resizeWindow("Material Original", 900, 650);
    cv::resizeWindow("Material Mask", 700, 500);
    cv::resizeWindow("Material Params", 520, 520);

    cv::createTrackbar("roi_left",   "Material Params", &g_roi_left_pct,   99);
    cv::createTrackbar("roi_right",  "Material Params", &g_roi_right_pct,  100);
    cv::createTrackbar("roi_top",    "Material Params", &g_roi_top_pct,    99);
    cv::createTrackbar("roi_bottom", "Material Params", &g_roi_bottom_pct, 100);
    cv::createTrackbar("thr_mode",   "Material Params", &g_thr_mode,         1);
    cv::createTrackbar("mad_k_x10",  "Material Params", &g_mad_k_x10,      100);
    cv::createTrackbar("diff_thr",   "Material Params", &g_diff_thr,       160);
    cv::createTrackbar("min_area",   "Material Params", &g_min_area,     20000);
    cv::createTrackbar("max_area_%", "Material Params", &g_max_area_p,     100);
    cv::createTrackbar("blur",       "Material Params", &g_blur_size,       31);
    cv::createTrackbar("morph",      "Material Params", &g_morph_size,      31);
    cv::createTrackbar("edge_margin", "Material Params", &g_edge_margin,    80);
    cv::createTrackbar("bright_gate", "Material Params", &g_use_bright_gate, 1);
    cv::createTrackbar("obj_v_min",  "Material Params", &g_obj_v_min,      255);
    cv::createTrackbar("obj_v_delta", "Material Params", &g_obj_v_delta,   120);
    cv::createTrackbar("solidity_min", "Material Params", &g_solidity_min, 100);
    cv::createTrackbar("central_w",  "Material Params", &g_central_w,      200);
    cv::createTrackbar("exposure",   "Material Params", &g_exposure,      1000);
    cv::createTrackbar("gain",       "Material Params", &g_gain,           255);
}

static void destroyMaterialWindows() {
    if (!g_gui_enabled) return;
    cv::destroyWindow("Material Original");
    cv::destroyWindow("Material Mask");
    cv::destroyWindow("Material Params");
}

static void applyMaterialCameraControls(cv::VideoCapture& cap) {
    if (MAT_CAM_LOCK_EXPOSURE) cap.set(cv::CAP_PROP_EXPOSURE, g_exposure);
    if (MAT_CAM_LOCK_GAIN)     cap.set(cv::CAP_PROP_GAIN, g_gain);
}

static bool openMaterialCamera(cv::VideoCapture& cap) {
    std::cout << "[抓取相机] 打开 " << g_material_cam_id
              << " (V4L2, 独立于寻线相机)" << std::endl;
    if (!openCameraByIdOrIndex(cap, g_material_cam_id, "抓取相机")) {
        std::cerr << "[抓取相机] 无法打开 " << g_material_cam_id
                  << " (matcam= 可改索引或 /dev/v4l/by-id 路径)" << std::endl;
        return false;
    }

    cap.set(cv::CAP_PROP_CONVERT_RGB, 1);
    bool ok_buf = cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    bool ok_awb = MAT_CAM_LOCK_WB ? cap.set(cv::CAP_PROP_AUTO_WB, 0)
                                  : cap.set(cv::CAP_PROP_AUTO_WB, 1);
    bool ok_wb = MAT_CAM_LOCK_WB
        ? cap.set(cv::CAP_PROP_WB_TEMPERATURE, MAT_CAM_WB_TEMP)
        : false;
    bool ok_auto_exp = MAT_CAM_LOCK_EXPOSURE
        ? cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1)
        : cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);
    bool ok_exp = MAT_CAM_LOCK_EXPOSURE
        ? cap.set(cv::CAP_PROP_EXPOSURE, g_exposure)
        : false;
    bool ok_gain = MAT_CAM_LOCK_GAIN
        ? cap.set(cv::CAP_PROP_GAIN, g_gain)
        : false;

    std::cout << "[抓取相机] backend=" << cap.getBackendName()
              << " buf=" << ok_buf
              << " awb=" << ok_awb
              << " wb=" << ok_wb
              << " auto_exp=" << ok_auto_exp
              << " exp=" << ok_exp
              << " gain=" << ok_gain
              << " exp_read=" << cap.get(cv::CAP_PROP_EXPOSURE)
              << " gain_read=" << cap.get(cv::CAP_PROP_GAIN)
              << std::endl;
    return true;
}

static bool updateMaterialPreview(cv::VideoCapture& cap,
                                  const std::string& status,
                                  DetectResult& res,
                                  cv::Mat& mask,
                                  cv::Rect& roi) {
    applyMaterialCameraControls(cap);

    cv::Mat frame;
    cap >> frame;
    if (frame.empty()) {
        usleep(10 * 1000);
        return false;
    }

    bool found = detectMaterial(frame, res, mask, roi);
    cv::rectangle(frame, roi, cv::Scalar(0, 255, 255), 2);

    if (found) {
        std::vector<std::vector<cv::Point>> draw_contours{res.contour};
        cv::drawContours(frame, draw_contours, -1, cv::Scalar(0, 255, 0), 2);
        cv::rectangle(frame, res.bbox, cv::Scalar(255, 0, 0), 2);
        cv::drawMarker(frame, res.bbox_center, cv::Scalar(0, 0, 255),
                       cv::MARKER_CROSS, 28, 3);
        cv::circle(frame, res.moment_center, 5, cv::Scalar(255, 0, 255), -1);

        char text[256];
        std::snprintf(text, sizeof(text),
                      "center=(%d,%d) moment=(%d,%d) area=%.0f",
                      res.bbox_center.x, res.bbox_center.y,
                      res.moment_center.x, res.moment_center.y,
                      res.area);
        cv::putText(frame, text, cv::Point(20, 35),
                    cv::FONT_HERSHEY_SIMPLEX, 0.75,
                    cv::Scalar(0, 0, 255), 2);
    } else {
        cv::putText(frame, "NO TARGET", cv::Point(20, 35),
                    cv::FONT_HERSHEY_SIMPLEX, 0.85,
                    cv::Scalar(0, 0, 255), 2);
    }

    cv::putText(frame, status, cv::Point(20, 70),
                cv::FONT_HERSHEY_SIMPLEX, 0.70,
                cv::Scalar(0, 0, 255), 2);

    // ★ 黑板判定健康度: dark_frac>=0.15 且板范围生效才是 OK。
    //   现场标定物资相机曝光时, 首要目标就是让这行常显 BOARD OK。
    char board_hud[96];
    std::snprintf(board_hud, sizeof(board_hud), "BOARD %s  dark_frac=%.2f",
                  res.board_found ? "OK" : "MISS", g_last_dark_frac);
    cv::putText(frame, board_hud, cv::Point(20, 105),
                cv::FONT_HERSHEY_SIMPLEX, 0.70,
                res.board_found ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 0, 255), 2);

    if (g_gui_enabled) {
        cv::imshow("Material Original", frame);
        cv::imshow("Material Mask", mask);
    }

    int key = guiWaitKey(1);
    g_last_key = key;   // ★ calmat 采点模式读这个 (c/z/r)
    if (key == 27 || key == 'q' || key == 'Q') ::g_running = false;
    return found;
}

static bool configureSerial115200(int fd) {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) { std::cerr << "[ARM] tcgetattr: " << std::strerror(errno) << std::endl; return false; }
    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200); cfsetospeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    // ★ 2026-07-05: 必须清 HUPCL! 否则 close() 时内核拉低 DTR/RTS,
    //   ESP32 臂板的自动下载电路会被复位 → 臂静默重启 10s+ (前激光串口早就清了,
    //   臂串口漏掉了 —— 这就是"每条命令 open/close 一次"方案回执丢失的根因之一)。
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | HUPCL);
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { std::cerr << "[ARM] tcsetattr: " << std::strerror(errno) << std::endl; return false; }
    tcflush(fd, TCIOFLUSH);
    return true;
}

static void releaseSerialModemLines(int fd) {
#if defined(TIOCM_DTR) && defined(TIOCM_RTS)
    int lines = TIOCM_DTR | TIOCM_RTS;
    if (ioctl(fd, TIOCMBIC, &lines) == 0) std::cout << "[ARM] released DTR/RTS" << std::endl;
#endif
}

static int openArmSerial(const std::string& dev) {
    int fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) { std::cerr << "[ARM] open " << dev << " failed: " << std::strerror(errno) << std::endl; return -1; }
    if (!configureSerial115200(fd)) { close(fd); return -1; }
    releaseSerialModemLines(fd);
    std::cout << "[ARM] serial opened: " << dev << " 115200 8N1" << std::endl;
    return fd;
}

// ============================================================
//  ★ 机械臂串口持久连接 (2026-07-05) —— "回执经常收不到"的核心修复
//  旧版每条命令 open→write→(wait)→close, 三个坑叠加:
//    ① close 拉低 DTR/RTS (HUPCL 没清) → ESP32 臂板被自动下载电路复位,
//       臂静默重启 10~15s (setup 里有舵机初始化 + xunxian0 + 各种 delay);
//    ② 臂在"狗端口已关闭"的窗口里发出的回执被内核直接丢弃 (没有持有 fd 就没有缓冲);
//    ③ 下一条命令 open 时 tcflush 又把刚缓冲到的残余回执字节冲掉, 且臂若还在
//       重启中, 回执会拖到狗的握手超时之后才发出 → 狗按超时兜底, 现场看起来就是
//       "命令都执行了 (RX 有 256B 硬件缓冲, 重启完照样跑), 但回执总是收不到"。
//  修复: 整个比赛进程只 open 一次、全程不 close (HUPCL 已清, 进程退出也不会复位臂)。
// ============================================================
int g_arm_fd = -1;   // 持久 fd; -1 = 尚未打开/打开失败

// ★ 2026-07-06 (治"第二抓取/双侧触发时串口报错"): 持久 fd 的新弱点是
//   比赛中途 USB 掉线 (台阶/跳跃震动、臂舵机大电流 brownout、EMI) —— 旧版每次
//   open/close 反而"顺手"重连了, 持久化之后 fd 变僵尸: write/read 全部 EIO,
//   而且掉线重插后设备号会漂移 (旧 fd 还占着 ttyUSB1 → 重插的臂变成 ttyUSB2)。
//   所以这里补三件事: ① TIOCMGET 健康检查识别僵尸 fd; ② 关掉僵尸 fd 后按候选表
//   重新扫描设备 (跳过前激光!); ③ sendArmLineAcked / waitArmReply 里失败自动重连。

void armPortMarkDead(const char* why) {
    if (g_arm_fd >= 0) { close(g_arm_fd); g_arm_fd = -1; }
    std::cout << "[ARM] ★ 串口失效 (" << why << "), 已关闭, 将自动扫描重连" << std::endl;
}

// 僵尸 fd 探测: 设备掉线/重枚举后旧 fd 表面还"开着", 但任何 IO 都是 EIO/ENODEV
bool armPortAlive() {
    if (g_arm_fd < 0) return false;
    int m = 0;
    if (ioctl(g_arm_fd, TIOCMGET, &m) < 0
        && (errno == EIO || errno == ENXIO || errno == ENODEV)) return false;
    return true;
}

// 把符号链接 (/dev/serial/by-path|by-id/...) 解析成实际节点 (/dev/ttyUSBx)
static std::string resolveReal(const std::string& p) {
    char buf[PATH_MAX];
    if (::realpath(p.c_str(), buf)) return std::string(buf);
    return p;   // 解析失败原样返回 (节点暂时不存在等)
}

// ★ 2026-07-06 v2 (适配 by-path/by-id 启动): 用户经 run.sh 传入的是
//   /dev/serial/by-path/... 固定路径 —— 它钉的是【物理 USB 口】, 掉线重插重枚举后
//   链接会自动指回新节点 (ttyUSB1→ttyUSB2 的漂移被 by-path 天然吸收)。所以:
//   ① 永远优先等配置路径回来 (重枚举窗口里链接会短暂消失几百 ms~2s);
//   ② 只有配置路径【持续消失 ≥3s】(线彻底松脱/换了口) 才扫描裸 ttyUSB 候选兜底;
//   ③ 候选与前激光的比对用 realpath 后的实际节点 —— 激光同样是 by-path 配置,
//      直接字符串比对 "/dev/ttyUSB0" 永远不相等, 旧写法有把 "3" 发给激光的风险!
static std::string armFindDevice() {
    static bool was_missing = false;
    static std::chrono::steady_clock::time_point missing_since;
    if (access(g_arm_serial_dev.c_str(), F_OK) == 0) {
        was_missing = false;
        return g_arm_serial_dev;
    }
    auto now = std::chrono::steady_clock::now();
    if (!was_missing) {
        was_missing = true;
        missing_since = now;
        std::cout << "[ARM] 配置路径暂时消失 (" << g_arm_serial_dev
                  << "), 等待其重新出现 (重枚举窗口)..." << std::endl;
        return "";
    }
    double miss_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now - missing_since).count() / 1000.0;
    if (miss_sec < 3.0) return "";   // 给 by-path 链接回来的时间, 别急着抓裸节点

    std::string laser_real = resolveReal(::g_front_laser_dev);
    const char* cands[] = { "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
                            "/dev/ttyUSB3", "/dev/ttyUSB4",
                            "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2" };
    for (const char* c : cands) {
        if (access(c, F_OK) != 0) continue;
        if (resolveReal(c) == laser_real) continue;               // ★ 绝不碰前激光
        if (resolveReal(c) == resolveReal(g_arm_serial_dev)) continue;
        std::cout << "[ARM] ★ 配置路径消失已 " << miss_sec << "s, 兜底改用裸节点 "
                  << c << " (请确认这是臂不是别的设备!)" << std::endl;
        return std::string(c);
    }
    return "";
}

// 确保臂串口已打开且健康; freshly_opened 回传"这次调用刚(重新)打开"
// (刚打开可能伴随臂板复位/重启, 调用方应多等一会儿或做重连排空)。
bool armPortEnsureOpen(bool* freshly_opened = nullptr) {
    if (freshly_opened) *freshly_opened = false;
    if (g_arm_fd >= 0) {
        if (armPortAlive()) return true;
        armPortMarkDead("健康检查失败: 设备已掉线/重枚举");
    }
    std::string dev = armFindDevice();
    if (dev.empty()) {
        std::cout << "[ARM] 找不到可用串口设备 (armdev=" << g_arm_serial_dev
                  << " 已消失且无候选)" << std::endl;
        return false;
    }
    g_arm_fd = openArmSerial(dev);
    if (g_arm_fd < 0) return false;
    std::cout << "[ARM] 实际节点: " << dev << " → " << resolveReal(dev) << std::endl;
    if (dev != g_arm_serial_dev) {
        std::cout << "[ARM] ★ 切换到兜底裸节点: " << g_arm_serial_dev << " → " << dev
                  << std::endl;
        g_arm_serial_dev = dev;
    }
    if (freshly_opened) *freshly_opened = true;
    return true;
}

// ★ 十一轮: 带宽限窗的串口打开 —— 设备正在掉线/重枚举时不立刻放弃,
//   每 0.5s 重试直到 max_wait_sec。armFindDevice 内部的"路径消失 ≥3s 才扫裸节点"
//   逻辑与 0.5s 轮询天然衔接 (前 3s 等 by-path 回来, 之后开始扫候选)。
//   注意: 宽限窗只在设备真挂时消耗; 设备健康时首次调用即返回, 零开销。
bool armPortEnsureOpenWait(double max_wait_sec, bool* freshly_opened = nullptr) {
    auto t0 = std::chrono::steady_clock::now();
    int tries = 0;
    while (::g_running) {
        if (armPortEnsureOpen(freshly_opened)) {
            if (tries > 0)
                std::cout << "[ARM] 宽限窗内重连成功 (等了 "
                          << std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::steady_clock::now() - t0).count() / 1000.0
                          << "s)" << std::endl;
            return true;
        }
        double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count() / 1000.0;
        if (el >= max_wait_sec) {
            std::cout << "[ARM] 宽限窗 " << max_wait_sec
                      << "s 用尽, 设备仍未回来" << std::endl;
            return false;
        }
        if (tries == 0)
            std::cout << "[ARM] 串口暂不可用, 进入重连宽限窗 (最长 "
                      << max_wait_sec << "s, 每 0.5s 重试)..." << std::endl;
        ++tries;
        usleep(500 * 1000);
    }
    return false;
}

void armPortCloseAtExit() {
    if (g_arm_fd >= 0) {
        close(g_arm_fd);
        g_arm_fd = -1;
        std::cout << "[ARM] 串口已在程序退出时关闭" << std::endl;
    }
}

static bool writeSerialLine(int fd, const std::string& line) {
    const char* p = line.c_str(); size_t left = line.size();
    while (left > 0) {
        ssize_t n = write(fd, p, left);
        if (n < 0) { if (errno == EINTR) continue; std::cerr << "[ARM] write: " << std::strerror(errno) << std::endl; return false; }
        p += n; left -= (size_t)n;
    }
    tcdrain(fd);
    return true;
}

// ★ 2026-07-04 修复: 旧版超时会把没收完的半行当整行返回, "D0" 被切成 "D"+"0" 两行,
//   回执精确匹配永远失败 → 握手全部退化成超时兜底 (实测复现)。
//   现在半行留在调用方传入的 partial 里, 下次调用接着收, 只有见到换行才算一行。
static bool readSerialLine(int fd, std::string& partial, std::string& line, int timeout_ms) {
    line.clear();
    auto t0 = std::chrono::steady_clock::now();
    while (true) {
        char c = 0; ssize_t n = read(fd, &c, 1);
        if (n > 0) {
            if (c == '\n' || c == '\r') {
                if (!partial.empty()) { line.swap(partial); return true; }
                // \r\n 的第二个字符/连续空行 → 忽略, 继续收
            } else {
                partial.push_back(c);
                if (partial.size() > 200) partial.clear();  // 垃圾流保护
            }
        }
        else if (n < 0 && errno != EINTR && errno != EAGAIN) { std::cerr << "[ARM] read: " << std::strerror(errno) << std::endl; return false; }
        int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - t0).count();
        if (elapsed >= timeout_ms) return false;   // ★ 半行不上交, 留在 partial
        if (n <= 0) usleep(10 * 1000);   // 有数据时不睡, 尽快把整行收完
    }
}

static void monitorSerialInput(int fd, double seconds, const std::string& tag) {
    if (fd < 0 || seconds <= 0.0) return;
    auto t0 = std::chrono::steady_clock::now();
    std::string partial;
    while (::g_running) {
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - t0).count() / 1000.0;
        if (elapsed >= seconds) break;
        std::string line;
        if (readSerialLine(fd, partial, line, 50)) { std::cout << "[ARM<-] " << tag << ": " << line << std::endl; }
        else usleep(20 * 1000);
    }
}

// ---- ★ 握手等待: 读串口直到收到期望的确认行或超时 ----
//   cap != nullptr 时每轮刷新物资相机预览窗口 (保持 GUI 活着 + 观察画面)。
//   超时返回 false, 调用方按"旧固定等待已过"继续 —— 兜底行为等于协议 v1。
//   ★ v3 说明: "完成回执" (R1/D2/...) 本身仍然只等不重发;
//     "命令发送"层的重发在 sendArmLineAcked 里做, 且只在没收到 ACK 时发生,
//     配合臂固件 v3 的去重, 绝不会让臂把同一动作做两遍。
static bool waitArmReply(int fd, const char* expect, double timeout_sec,
                         cv::VideoCapture* cap, const char* status) {
    auto t0 = std::chrono::steady_clock::now();
    DetectResult res; cv::Mat mask; cv::Rect roi;
    std::string partial;
    std::string token_window;
    // ★ 2026-07-06: fd 可能在等待期间掉线 (僵尸化)。只对臂的持久口做自动重连;
    //   掉线后不能立刻 return false —— 臂多半已收到命令并且【正在做动作】(25s 的
    //   movecatch!), 立刻返回 = 狗提前开走撞臂。必须等满超时, 期间周期性尝试重连,
    //   重连成功还有机会收到迟来的完成回执。
    bool is_arm_port = (fd == g_arm_fd);
    int  cur_fd = fd;
    auto last_reconnect_try = t0;
    while (::g_running) {
        auto now_tp = std::chrono::steady_clock::now();
        double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now_tp - t0).count() / 1000.0;
        if (el >= timeout_sec) {
            std::cout << "[ARM] 等待 " << expect << " 超时 (" << timeout_sec
                      << "s), 按已完成兜底继续" << std::endl;
            return false;
        }

        // ---- 掉线恢复: 每 1s 尝试重连一次, 其余时间只耗超时 ----
        if (is_arm_port && cur_fd < 0) {
            double since_try = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   now_tp - last_reconnect_try).count() / 1000.0;
            if (since_try >= 1.0) {
                last_reconnect_try = now_tp;
                bool fresh = false;
                if (armPortEnsureOpen(&fresh)) {
                    cur_fd = g_arm_fd;
                    std::cout << "[ARM] 等待 " << expect
                              << " 期间重连成功, 继续收回执" << std::endl;
                }
            }
            if (cap) updateMaterialPreview(*cap, status, res, mask, roi);
            else usleep(50 * 1000);
            continue;
        }

        bool got_char = false;
        while (::g_running) {
            char c = 0;
            ssize_t n = read(cur_fd, &c, 1);
            if (n > 0) {
                got_char = true;
                token_window.push_back(c);
                if (token_window.size() > 200) token_window.erase(0, token_window.size() - 200);

                if (c == '\n' || c == '\r') {
                    if (!partial.empty()) {
                        std::cout << "[ARM<-] " << partial << std::endl;
                        partial.clear();
                    }
                } else {
                    partial.push_back(c);
                    if (partial.size() > 200) partial.clear();
                }

                // 直接扫字节窗口: 回包没换行、粘了调试日志、或半行残留时也能认到 D2/D4/D5/D6/R1。
                if (token_window.find(expect) != std::string::npos) {
                    std::cout << "[ARM<-] " << expect << " 确认 (耗时 "
                              << el << "s";
                    if (!partial.empty()) std::cout << ", 当前片段\"" << partial << "\"";
                    std::cout << ")" << std::endl;
                    return true;
                }
                continue;
            }

            if (n < 0) {
                if (errno == EINTR) continue;
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::cerr << "[ARM] read: " << std::strerror(errno) << std::endl;
                    if (is_arm_port) {
                        // ★ 掉线: 标记失效, 不返回 —— 臂可能正在做动作, 等满超时保安全
                        armPortMarkDead("等待回执期间 read 失败 (设备掉线?)");
                        cur_fd = -1;
                        break;
                    }
                    return false;   // 非臂持久口: 维持旧行为
                }
            }
            break;
        }
        if (cap) updateMaterialPreview(*cap, status, res, mask, roi);
        else if (!got_char) usleep(20 * 1000);
    }
    return false;
}

// ============================================================
//  ★ 发送一行命令 + 等 ACK (★ 2026-07-06 用户要求: 【绝不重发】)
//    write 成功后无论有没有等到 ACK 都只发这一次 —— 命令可能已被臂收到,
//    重发存在"同一动作做多遍"的风险 (去重兜不住时是灾难), 宁可按
//    完成回执的超时兜底继续。只有"根本没送出去"(串口不可用 / write 失败)
//    才自动重连换口重试 (这不算重发: 臂从未收到)。
//    返回 true = 至少成功 write 一次 (ACK 到没到都算)。
// ============================================================
// 中途重连之后: 臂若因掉电重启, 会先打印 setup 日志 (10s 级, 期间还收不了命令)。
// 监听输出直到"安静 3.5s"或上限 15s; 全程无输出 (臂独立供电没重启) 则 2s 就走。
static void armPortDrainAfterReopen() {
    if (g_arm_fd < 0) return;
    std::cout << "[ARM] 中途重连成功, 监听臂输出等其就绪 (安静3.5s/无输出2s/上限15s)..."
              << std::endl;
    auto t0 = std::chrono::steady_clock::now();
    auto last_rx = t0;
    std::string partial;
    bool any = false;
    while (::g_running) {
        auto now = std::chrono::steady_clock::now();
        double total = std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count() / 1000.0;
        double quiet = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx).count() / 1000.0;
        if (total >= 15.0) break;
        if (any  && quiet >= 3.5) break;   // 有过启动输出 → 等它安静 (setup 里有 >2s 的 delay)
        if (!any && total >= 2.0) break;   // 全程没输出 → 臂没重启, 直接继续
        std::string ln;
        if (readSerialLine(g_arm_fd, partial, ln, 50)) {
            any = true;
            last_rx = std::chrono::steady_clock::now();
            std::cout << "[ARM<-] reboot: " << ln << std::endl;
        }
    }
}

bool sendArmLineAcked(const std::string& line,
                      cv::VideoCapture* cap, const char* status) {
    char ack_tok[8];
    std::snprintf(ack_tok, sizeof(ack_tok), "ACK%c", line.empty() ? '?' : line[0]);
    // ★ 十轮: write 失败(没送出去)重试 + 没等到 ACK 也重发, 总共最多发 ARM_SEND_MAX_TRIES 次
    //   (重发安全性依赖臂固件 v3/v3.1 的重复命令去重, 见 ARM_SEND_MAX_TRIES 注释)
    bool wrote_once = false;
    for (int attempt = 1; attempt <= ARM_SEND_MAX_TRIES && ::g_running; ++attempt) {
        // ★ 每轮都确保串口健康 (掉线/漂移自动重连); 刚重连则先等臂就绪
        bool fresh = false;
        if (!armPortEnsureOpen(&fresh)) {
            std::cout << "[ARM] 串口不可用 (第 " << attempt << "/"
                      << ARM_SEND_MAX_TRIES << " 次), 0.5s 后重试" << std::endl;
            usleep(500 * 1000);
            continue;
        }
        if (fresh) armPortDrainAfterReopen();

        std::cout << "[ARM->] 发送 (第 " << attempt << "/" << ARM_SEND_MAX_TRIES
                  << " 次): " << line;
        if (!writeSerialLine(g_arm_fd, line)) {
            armPortMarkDead("write 失败 (USB 掉线/重枚举?)");
            continue;   // 命令没送出去, 重连后再发不算重发
        }
        wrote_once = true;
        // ---- write 成功: 等 ACK; 没等到就重发 (★ 十轮恢复重发) ----
        //   安全性论证: 臂端 go2SerialPoll 单线程按序处理 → 第一份先执行完并缓存到
        //   go2LastDoneLine, 重发份必然命中去重 (只补发缓存回执, 不重做动作);
        //   臂正在执行第一份时, 重发份排在 RX 缓冲里, 执行完成后才被去重处理并
        //   【再补发一遍完成回执】—— 重发同时就是"回执丢了再要一份"的重问通道。
        if (waitArmReply(g_arm_fd, ack_tok, ARM_ACK_TIMEOUT_SEC, cap, status)) {
            return true;   // 臂已确认收到这行
        }
        if (!armPortAlive()) {
            // 掉线: 标记失效, 下一轮循环自动重连后重发 (臂端去重保证不重做动作)
            armPortMarkDead("等 ACK 期间设备掉线, 重连后重发 (臂端去重防重复动作)");
            continue;
        }
        if (attempt < ARM_SEND_MAX_TRIES) {
            std::cout << "[ARM] " << ARM_ACK_TIMEOUT_SEC << "s 未收到 " << ack_tok
                      << ", ★ 重发 (第 " << (attempt + 1) << "/" << ARM_SEND_MAX_TRIES
                      << " 次; 臂端去重保证不重做动作)" << std::endl;
            continue;
        }
        std::cout << "[ARM] " << ARM_SEND_MAX_TRIES << " 次发送均未收到 " << ack_tok
                  << ", 按已发出继续, 由完成回执 (含中途重问) 超时兜底" << std::endl;
        return true;   // 已发出, 交给完成回执/重问/超时兜底
    }
    if (wrote_once) {
        // 边角: 最后一轮 write 成功但等 ACK 时掉线 —— 命令可能已送达,
        // 按已发出继续, 由完成回执路径 (waitArmReply 内含自动重连) 兜底
        std::cout << "[ARM] 发送过至少一次但未确认 ACK, 按已发出继续: " << line;
        return true;
    }
    std::cout << "[ARM] ★ 命令始终未能发出 (串口多次重连失败): " << line;
    return false;
}

// ============================================================
//  ★ 十轮: 等完成回执 + 中途"重问" —— 治 20~40% "臂做了动作但狗收不到回执,
//    且一丢就连丢几条"。机制: 先等 60% 超时; 没扫到 token 就把【原命令行】原样
//    重发一遍 —— 臂端去重命中: 动作已完成 → 立刻补发缓存回执; 还在执行 → 重问
//    行排在 RX 缓冲, 执行完成后被去重处理再补发。两条路都不会让臂重做动作,
//    但都给了狗第二次收回执的机会 (配合臂端回执 5 连发拉开间隔, 突发丢字可恢复)。
//    然后再等余下 40% 超时 + 3s (给去重补发留传输时间)。
// ============================================================
static bool waitArmReplyReask(const std::string& line, const char* expect,
                              double timeout_sec, cv::VideoCapture* cap,
                              const char* status) {
    double first = timeout_sec * 0.6;
    if (waitArmReply(g_arm_fd, expect, first, cap, status)) return true;
    if (!::g_running) return false;
    // ★ 十一轮: 单次重问 → 周期重问 (最多 3 轮)。0708 dmesg 证实链路会反复瞬断
    //   (掉线→1~15s 后自动回来), 一次重问的回执同样可能整体被掉线窗吃掉;
    //   周期重问 + 每轮前 ensureOpen (瞬断后自动换新 fd), 只要链路在超时内
    //   任何一刻恢复, 就能从臂端去重缓存把回执要回来。绝不重做动作 (臂端去重)。
    const int REASK_MAX = 3;
    double remain = timeout_sec - first + 3.0;   // +3s: 给去重补发留传输时间
    double slice  = remain / REASK_MAX;
    for (int k = 1; k <= REASK_MAX && ::g_running; ++k) {
        if (armPortEnsureOpen()) {
            std::cout << "[ARM] 未见 " << expect << " → 重问 (" << k << "/"
                      << REASK_MAX << ", 臂端去重: 只补回执, 绝不重做动作): " << line;
            writeSerialLine(g_arm_fd, line);
        }
        if (waitArmReply(g_arm_fd, expect, slice, cap, status)) return true;
    }
    return false;
}

// ---- 简单机械臂命令: 发一行 (带 ACK/重发), 不等待完成回执 ----
bool sendSimpleArmCommand(int cmd_id) {
    if (cmd_id != 0 && cmd_id != 5 && cmd_id != 6) {
        std::cout << "[ARM] simple command disabled/invalid: " << cmd_id << std::endl;
        return false;
    }
    char line[16];
    std::snprintf(line, sizeof(line), "%d\n", cmd_id);
    std::cout << "[ARM] send simple command: " << cmd_id
              << " (no done-reply wait)" << std::endl;
    return sendArmLineAcked(line, nullptr, "");
}

// ---- ★ 简单命令 + 握手: 发一行 (带 ACK/重发), 等完成回执或超时 ----
//   返回 true=收到完成回执; false=超时/串口失败 (调用方无需额外等待, 直接继续)。
//   ★ v3: 不再每次 open/close 串口 —— 那是回执丢失的根因 (见 armPortEnsureOpen 注释)。
bool sendSimpleArmCommandWaitReply(int cmd_id, const char* expect,
                                   double timeout_sec) {
    if (cmd_id != 0 && cmd_id != 5 && cmd_id != 6) {
        std::cout << "[ARM] simple command disabled/invalid: " << cmd_id << std::endl;
        return false;
    }
    if (!armPortEnsureOpen()) {
        std::cout << "[ARM] 串口打开失败 (命令 " << cmd_id << "), 继续流程" << std::endl;
        return false;
    }
    char line[16];
    std::snprintf(line, sizeof(line), "%d\n", cmd_id);
    std::cout << "[ARM] 发送: " << cmd_id << ", 等待 " << expect
              << " (超时 " << timeout_sec << "s)" << std::endl;
    // ★ 十一轮: 发送前先过宽限窗 (阶段4.5 的 "0" 和双侧平台的 "5"/"6" 都走这里;
    //   1251 轮放置平台"自发恢复"纯属运气 —— 设备恰好在那之前回来了, 现在不赌运气)
    if (!armPortEnsureOpenWait(ARM_OPEN_GRACE_SEC, nullptr)) {
        std::cout << "[ARM] 简单命令 " << cmd_id
                  << " 放弃 (宽限窗内设备未回来)" << std::endl;
        return false;
    }
    // ★ 十轮: 完成回执改走"等待+中途重问"版 (waitArmReplyReask)
    return sendArmLineAcked(line, nullptr, "")
        && waitArmReplyReask(line, expect, timeout_sec, nullptr, "");
}

// ---- 抓取主流程: 独立使用物资摄像头; 返回 true=已发送抓取指令 ----
bool runMaterialGrab(unitree::robot::go2::SportClient& sport,
                     double window_sec) {
    // ★ 未标定不进流程: 坐标换算在狗端, 没有 H1 发出去的全是垃圾
    if (!g_H1_valid) {
        std::cout << "[抓取] ★ 第一抓取未标定 (无 mat_cal1.txt), 跳过抓取流程!"
                  << " 先按手册跑 calmat 采点标定" << std::endl;
        return false;
    }
    g_exposure = g_exposure_first;   // ★ 第一抓取点位曝光 (matexp1= 标定)
    auto t_start = std::chrono::steady_clock::now();
    auto win_elapsed = [&]() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - t_start).count() / 1000.0;
    };

    cv::VideoCapture cap;
    setupMaterialWindows();
    auto cleanup = [&]() {
        // ★ v3: 串口持久化, 这里不再 close (close 会复位臂板 + 丢后续回执)
        cap.release();
        destroyMaterialWindows();
    };

    if (!openMaterialCamera(cap)) {
        cleanup();
        return false;
    }

    DetectResult preview_res;
    cv::Mat preview_mask;
    cv::Rect preview_roi;

    bool arm_fresh_open = false;
    // ★ 十一轮: 入口给设备重枚举留宽限窗 (旧版首次 miss 即弃 = 0 秒等待)
    if (!armPortEnsureOpenWait(ARM_OPEN_GRACE_SEC, &arm_fresh_open)) {
        std::cout << "[抓取] 串口打开失败 (含 " << ARM_OPEN_GRACE_SEC
                  << "s 宽限窗) → 跳过抓取 (检查机械臂连线/设备名 armdev="
                  << g_arm_serial_dev << ")" << std::endl;
        cleanup();
        return false;
    }

    // ★ v3: 正常情况下串口早在 main 启动时就打开了 (arm_fresh_open=false),
    //   这里只做 0.3s 排空; 只有兜底"现在才第一次打开"时才等满 ARM_BOOT_WAIT_SEC。
    double arm_boot_wait = arm_fresh_open ? ARM_BOOT_WAIT_SEC : 0.3;
    std::cout << "[抓取] 串口" << (arm_fresh_open ? "刚打开, 等 " : "已持久打开, 排空 ")
              << arm_boot_wait << "s" << std::endl;
    {
        auto ts = std::chrono::steady_clock::now();
        std::string partial;
        while (::g_running) {
            double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - ts).count() / 1000.0;
            if (el >= arm_boot_wait) break;
            std::string line;
            if (readSerialLine(g_arm_fd, partial, line, 10)) {
                std::cout << "[ARM<-] boot: " << line << std::endl;
            }
            updateMaterialPreview(cap, "ARM BOOT WAIT", preview_res, preview_mask, preview_roi);
        }
    }
    if (!::g_running) {
        cleanup();
        return false;
    }

    std::cout << "[抓取] 发送: 1 (机械臂就位), 等待 R1 确认 (超时 "
              << HS_R1_TIMEOUT_SEC << "s)" << std::endl;
    sendArmLineAcked("1\n", &cap, "SEND CMD 1");
    // ★ 十轮: 完成回执 R1 走"等待+中途重问"版 (去重保证不重做动作), 超时仍兜底
    waitArmReplyReask("1\n", "R1", HS_R1_TIMEOUT_SEC, &cap, "WAIT ARM R1");
    if (!::g_running || win_elapsed() >= window_sec) {
        if (::g_running) std::cout << "[抓取] 窗口耗尽(就位阶段), 跳过" << std::endl;
        cleanup();
        return false;
    }

    for (int i = 0; i < GRAB_FLUSH_FRAMES; ++i) { cv::Mat f; cap >> f; }  // 丢弃陈旧帧

    std::cout << "[抓取] 开始检测物料, 连续 " << ARM_STABLE_FRAMES
              << " 帧稳定后发送抓取指令" << std::endl;
    bool last_valid = false; int stable = 0; cv::Point last_c{0, 0};
    bool sent = false; int notgt_log = 0;
    int  nudge_cnt   = 0;                                    // 已补救段数
    auto last_useful = std::chrono::steady_clock::now();     // 上一次有效识别时刻 (滚动窗口)
    while (::g_running && win_elapsed() < window_sec) {
        DetectResult res; cv::Mat mask; cv::Rect roi;
        bool found = updateMaterialPreview(cap, "TARGET SEARCH", res, mask, roi);
        // ★ 2026-07-06 第四轮: "有效识别 = 有候选 且 黑板已锁定"; 无效识别按【滚动窗口】
        //   累计 —— 距上一次有效识别满 GRAB_NO_TARGET_SEC 就后退补救, 最多
        //   GRAB_NO_TARGET_MAX_NUDGES 次。偶发一帧误检只是重置计时, 不再永久禁用回退
        //   (旧"看到过一帧就禁用"在第二抓取是致命的: 臂举着物资1悬在平台边被误检一帧,
        //    回退就永远不触发)。
        bool useful = found && res.board_found;
        if (!useful) {
            stable = 0; last_valid = false;
            if ((notgt_log++ % 15) == 0) {
                if (!found)
                    std::cout << "[抓取] 未检测到物料..." << std::endl;
                else
                    std::cout << "[抓取] 黑板未锁定 (dark_frac=" << g_last_dark_frac
                              << " <0.15 或板太小), 不发坐标 — 调物资相机曝光 matexp1="
                              << std::endl;
            }
            double no_target_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                                       std::chrono::steady_clock::now() - last_useful).count() / 1000.0;
            // ★ 滚动窗口: 距上一次有效识别 ≥8s 就补救, 最多 2 次。旧"看到过一帧就永久
            //   禁用"在第二抓取是致命的 —— 臂正举着物资1悬在平台边, 只要它被误检成
            //   候选一帧 (黑板又是锁定的), 回退就永远不会触发。
            // ★ 十轮 (用户): 第一抓取补救从"两退"改为三段升级 ——
            //   ①后退一步 → ②水平向右挪一步 → ③再后退一步 (参数全部复用后退那组),
            //   每段之间照旧等满 GRAB_NO_TARGET_SEC 的滚动窗口; 三段用尽仍无识别
            //   才盲发标定中心点。
            if (nudge_cnt < GRAB1_ESCALATE_STAGES && no_target_sec >= GRAB_NO_TARGET_SEC) {
                nudge_cnt++;
                if (nudge_cnt == 1) {
                    std::cout << "[抓取] 无有效识别 " << no_target_sec
                              << "s → 补救① 后退一步 (1/" << GRAB1_ESCALATE_STAGES
                              << ")" << std::endl;
                    nudgeMoveAfterNoMaterial(sport, cap, "抓取",
                        GRAB_NO_TARGET_BACK_KICK_VX, 0.0f,
                        GRAB_NO_TARGET_BACK_VX, 0.0f, "后退一步");
                } else if (nudge_cnt == 2) {
                    // vy < 0 = 右 (与全文 FINAL_SHIFT_VY 等约定一致); 速度/时长同后退
                    std::cout << "[抓取] 后退后仍无有效识别 " << no_target_sec
                              << "s → 补救② 向右横移一步 (2/" << GRAB1_ESCALATE_STAGES
                              << ")" << std::endl;
                    nudgeMoveAfterNoMaterial(sport, cap, "抓取",
                        0.0f, -std::fabs(GRAB_NO_TARGET_BACK_KICK_VX),
                        0.0f, -std::fabs(GRAB_NO_TARGET_BACK_VX), "向右横移一步");
                } else {
                    std::cout << "[抓取] 右移后仍无有效识别 " << no_target_sec
                              << "s → 补救③ 再后退一步 (3/" << GRAB1_ESCALATE_STAGES
                              << ")" << std::endl;
                    nudgeMoveAfterNoMaterial(sport, cap, "抓取",
                        GRAB_NO_TARGET_BACK_KICK_VX, 0.0f,
                        GRAB_NO_TARGET_BACK_VX, 0.0f, "再后退一步");
                }
                last_useful = std::chrono::steady_clock::now();
                notgt_log = 0;
            }
            // ★ 2026-07-06 第五轮兜底 (十轮: 触发条件改为三段补救全部用尽):
            //   之后又连续 GRAB_NO_TARGET_SEC 无有效识别 → 盲抓标定中心点
            //   (0706 第六次: 一直识别不到只能靠手推, 现在宁可对中心点试一把,
            //    也不把 80s 窗口白白耗完)。
            else if (GRAB_BLIND_FALLBACK_ENABLE
                     && nudge_cnt >= GRAB1_ESCALATE_STAGES
                     && no_target_sec >= GRAB_NO_TARGET_SEC) {
                char cmd[64];
                std::snprintf(cmd, sizeof(cmd), "2,%.1f,%.1f\n",
                              GRAB1_BLIND_WX + GRAB1_SEND_X_OFF, GRAB1_BLIND_WY);
                std::cout << "[抓取] ★ 三段补救(退/右/退)用尽后仍无有效识别 "
                          << no_target_sec << "s → 盲抓标定中心点, 发送: " << cmd;
                if (sendArmLineAcked(cmd, &cap, "SEND CMD 2 (BLIND)")) {
                    sent = true;
                    waitArmReplyReask(cmd, "D2", HS_D2_TIMEOUT_SEC, &cap, "WAIT ARM D2");
                }
                break;
            }
            continue;
        }
        last_useful = std::chrono::steady_clock::now();   // 有效识别 = 黑板锁定 + 有候选
        int dx = res.bbox_center.x - last_c.x, dy = res.bbox_center.y - last_c.y;
        if (!last_valid || dx * dx + dy * dy > ARM_STABLE_TOL_PX * ARM_STABLE_TOL_PX) stable = 1;
        else stable++;
        last_c = res.bbox_center; last_valid = true;
        std::cout << "[抓取] center=(" << res.bbox_center.x << "," << res.bbox_center.y
                  << ") area=" << res.area << " 稳定 " << stable << "/" << ARM_STABLE_FRAMES << std::endl;
        if (stable >= ARM_STABLE_FRAMES) {
            // ★ 协议 v2: 狗端像素→毫米 (H1 单应), 发毫米给臂
            double wx = 0, wy = 0;
            if (!pixelToWorldMM(g_H1, g_H1_valid,
                                res.bbox_center.x, res.bbox_center.y, wx, wy)) {
                std::cout << "[抓取] 像素→毫米换算失败, 不发坐标" << std::endl;
                break;
            }
            char cmd[64];
            std::snprintf(cmd, sizeof(cmd), "2,%.1f,%.1f\n",
                          wx + GRAB1_SEND_X_OFF, wy);   // 补偿 movecatch 内偏 x-5
            std::cout << "[抓取] 发送: " << cmd
                      << "  (像素 " << res.bbox_center.x << ","
                      << res.bbox_center.y << " → 毫米)" << std::endl;
            if (sendArmLineAcked(cmd, &cap, "SEND CMD 2")) {
                sent = true;
                // ★ 等 D2 (夹稳) 替代旧固定 15s (臂实际要 ~20s, 旧值会提前走!)
                //   十轮: 改走"等待+中途重问"版, 回执单向丢字可恢复
                waitArmReplyReask(cmd, "D2", HS_D2_TIMEOUT_SEC, &cap, "WAIT ARM D2");
            }
            break;
        }
    }
    if (!sent) std::cout << "[抓取] 窗口内未稳定锁定目标, 未发送抓取指令" << std::endl;
    cleanup();
    return sent;
}

// ---- 第二抓取流程 (协议 v2, 2026-07-04 改"先识别再放") ----
//   ① 发 "3": 臂握着物资1摆到第二识别位 (不放料) → 回 R3
//   ② 收到 R3 后打开相机识别物资2 → H2 单应算世界坐标 (wx,wy)
//   ③ 发 "4,wx,wy" → 臂先放物资1到 (wx-80,wy)、再抓物资2 (wx,wy) → 回 D4
//   为何先识别: 若先放料, 物资1落在中转点会和物资2同时出现在画面 → 可能误识别成两个。
//   握着物资1识别时爪没放料, 画面里只有物资2 (实测爪不挡镜头)。
bool runSecondMaterialGrab(unitree::robot::go2::SportClient& sport,
                           double window_sec) {
    g_second_grab_nudged = false;
    if (!g_H2_valid) {
        std::cout << "[第二抓取] ★ 第二抓取未标定 (无 mat_cal2.txt), 跳过抓取流程!"
                  << " 先按手册跑 calmat2 采点标定" << std::endl;
        return false;
    }
    g_exposure = g_exposure_second;   // ★ 第二抓取点位曝光 (matexp2= 标定)
    auto t_start = std::chrono::steady_clock::now();
    auto win_elapsed = [&]() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - t_start).count() / 1000.0;
    };

    cv::VideoCapture cap;
    bool windows_ready = false;
    auto cleanup = [&]() {
        // ★ v3: 串口持久化, 这里不再 close (close 会复位臂板 + 丢后续回执)
        cap.release();
        if (windows_ready) {
            destroyMaterialWindows();
            windows_ready = false;
        }
    };

    bool arm_fresh_open = false;
    // ★ 十一轮: 1251 轮第二抓取正是死在这里 (到位时恰逢掉线窗, 0 秒等待即弃);
    //   宽限窗 12s 占 100s 窗口的 12%, 只在设备真挂时消耗, 值得等。
    if (!armPortEnsureOpenWait(ARM_OPEN_GRACE_SEC, &arm_fresh_open)) {
        std::cout << "[第二抓取] 串口打开失败 (含 " << ARM_OPEN_GRACE_SEC
                  << "s 宽限窗) → 跳过第二抓取 (检查机械臂连线/设备名 armdev="
                  << g_arm_serial_dev << ")" << std::endl;
        return false;
    }

    double arm_boot_wait = arm_fresh_open ? ARM_BOOT_WAIT_SEC : 0.3;
    std::cout << "[第二抓取] 串口" << (arm_fresh_open ? "刚打开, 等 " : "已持久打开, 排空 ")
              << arm_boot_wait << "s" << std::endl;
    {
        auto ts = std::chrono::steady_clock::now();
        std::string partial;
        while (::g_running) {
            double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - ts).count() / 1000.0;
            if (el >= arm_boot_wait || win_elapsed() >= window_sec) break;
            std::string line;
            if (readSerialLine(g_arm_fd, partial, line, 10)) {
                std::cout << "[ARM<-] second boot: " << line << std::endl;
            } else {
                usleep(10 * 1000);
            }
        }
    }
    if (!::g_running || win_elapsed() >= window_sec) {
        cleanup();
        return false;
    }

    // ---- ① 摆第二识别位 (命令 "3", 臂握着物资1、不放料), 等 R3 ----
    //   ★ 2026-07-04 改为"先识别再放": 识别时物资1还在爪里(没落平台), 视野里只有物资2,
    //     不会误识别成两个。识别后发 "4" 让臂放物资1到 (wx-80,wy) 再抓物资2 (wx,wy)。
    std::cout << "[第二抓取] 发送: 3 (摆第二识别位, 握着物资1), 等待 R3 (超时 "
              << HS_R3_TIMEOUT_SEC << "s)" << std::endl;
    if (!sendArmLineAcked("3\n", nullptr, "")) {
        cleanup();
        return false;
    }
    {
        // ★ 十轮: R3 也走"等待+中途重问"版
        double t_left = window_sec - win_elapsed();
        waitArmReplyReask("3\n", "R3",
                          std::min(HS_R3_TIMEOUT_SEC, std::max(1.0, t_left)),
                          nullptr, "");
    }
    if (!::g_running || win_elapsed() >= window_sec) {
        cleanup();
        return false;
    }

    // ---- ② 臂停在识别位、物资1仍在爪里 —— 打开相机识别物资2 ----
    setupMaterialWindows();
    windows_ready = true;
    if (!openMaterialCamera(cap)) {
        cleanup();
        return false;
    }

    for (int i = 0; i < GRAB_FLUSH_FRAMES; ++i) { cv::Mat f; cap >> f; }

    std::cout << "[第二抓取] 开始检测物料, 连续 " << ARM_STABLE_FRAMES
              << " 帧稳定后发送第二抓取坐标" << std::endl;
    bool last_valid = false; int stable = 0; cv::Point last_c{0, 0};
    bool sent = false; int notgt_log = 0;
    int  nudge_cnt   = 0;                                    // 已后退补救次数
    auto last_useful = std::chrono::steady_clock::now();     // 上一次有效识别时刻 (滚动窗口)
    while (::g_running && win_elapsed() < window_sec) {
        DetectResult res; cv::Mat mask; cv::Rect roi;
        bool found = updateMaterialPreview(cap, "SECOND TARGET SEARCH", res, mask, roi);
        // ★ 同第一抓取 2026-07-06 第三轮: "有效识别 = 有候选 且 黑板已锁定",
        //   否则累计 5s 无目标照常触发三段补救 (旧版被误检/黑板未锁定分支永久卡死)。
        bool useful = found && res.board_found;
        if (!useful) {
            stable = 0; last_valid = false;
            if ((notgt_log++ % 15) == 0) {
                if (!found)
                    std::cout << "[第二抓取] 未检测到物料..." << std::endl;
                else
                    std::cout << "[第二抓取] 黑板未锁定 (dark_frac=" << g_last_dark_frac
                              << "), 不发坐标 — 调物资相机曝光 matexp2=" << std::endl;
            }
            double no_target_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                                       std::chrono::steady_clock::now() - last_useful).count() / 1000.0;
            if (nudge_cnt < GRAB_NO_TARGET_MAX_NUDGES && no_target_sec >= GRAB_NO_TARGET_SEC) {
                nudge_cnt++;
                g_second_grab_nudged = true;
                if (nudge_cnt == 1) {
                    std::cout << "[第二抓取] 无有效识别 " << no_target_sec
                              << "s → 补救① 后退一步 (1/" << GRAB_NO_TARGET_MAX_NUDGES
                              << ")" << std::endl;
                    backOffAfterNoMaterial(sport, cap, "第二抓取");
                } else if (nudge_cnt == 2) {
                    // vy > 0 = 左; 速度/时长复用后退那组, 只换方向。
                    std::cout << "[第二抓取] 后退后仍无有效识别 " << no_target_sec
                              << "s → 补救② 向左横移一步 (2/" << GRAB_NO_TARGET_MAX_NUDGES
                              << ")" << std::endl;
                    nudgeMoveAfterNoMaterial(sport, cap, "第二抓取",
                        0.0f, +std::fabs(GRAB_NO_TARGET_BACK_KICK_VX),
                        0.0f, +std::fabs(GRAB_NO_TARGET_BACK_VX), "向左横移一步");
                } else {
                    std::cout << "[第二抓取] 左移后仍无有效识别 " << no_target_sec
                              << "s → 补救③ 前进一步 (3/" << GRAB_NO_TARGET_MAX_NUDGES
                              << ")" << std::endl;
                    nudgeMoveAfterNoMaterial(sport, cap, "第二抓取",
                        +std::fabs(GRAB_NO_TARGET_BACK_KICK_VX), 0.0f,
                        +std::fabs(GRAB_NO_TARGET_BACK_VX), 0.0f, "前进一步");
                }
                last_useful = std::chrono::steady_clock::now();
                notgt_log = 0;
            }
            // ★ 2026-07-06 第五轮兜底: 同第一抓取 —— 补救用尽后仍连续无有效识别
            //   → 盲发标定中心点坐标 (臂按协议先放物资1到 wx-80, 再抓 wx 处物资2)。
            else if (GRAB_BLIND_FALLBACK_ENABLE
                     && nudge_cnt >= GRAB_NO_TARGET_MAX_NUDGES
                     && no_target_sec >= GRAB_NO_TARGET_SEC) {
                char cmd[64];
                std::snprintf(cmd, sizeof(cmd), "4,%.1f,%.1f\n",
                              GRAB2_BLIND_WX, GRAB2_BLIND_WY + GRAB2_SEND_Y_OFF);
                std::cout << "[第二抓取] ★ 补救用尽后仍无有效识别 " << no_target_sec
                          << "s → 盲抓标定中心点, 发送: " << cmd;
                if (sendArmLineAcked(cmd, &cap, "SEND CMD 4 (BLIND)")) {
                    sent = true;
                    waitArmReplyReask(cmd, "D4", HS_D4_TIMEOUT_SEC, &cap, "WAIT ARM D4");
                }
                break;
            }
            continue;
        }
        last_useful = std::chrono::steady_clock::now();   // 有效识别才刷新滚动窗口

        int dx = res.bbox_center.x - last_c.x, dy = res.bbox_center.y - last_c.y;
        if (!last_valid || dx * dx + dy * dy > ARM_STABLE_TOL_PX * ARM_STABLE_TOL_PX) stable = 1;
        else stable++;
        last_c = res.bbox_center; last_valid = true;

        std::cout << "[第二抓取] center=(" << res.bbox_center.x << "," << res.bbox_center.y
                  << ") area=" << res.area << " 稳定 "
                  << stable << "/" << ARM_STABLE_FRAMES << std::endl;

        if (stable >= ARM_STABLE_FRAMES) {
            // ★ 协议 v2: 狗端像素→毫米 (H2 单应), 发毫米给臂
            double wx = 0, wy = 0;
            if (!pixelToWorldMM(g_H2, g_H2_valid,
                                res.bbox_center.x, res.bbox_center.y, wx, wy)) {
                std::cout << "[第二抓取] 像素→毫米换算失败, 不发坐标" << std::endl;
                break;
            }
            char cmd[64];
            std::snprintf(cmd, sizeof(cmd), "4,%.1f,%.1f\n",
                          wx, wy + GRAB2_SEND_Y_OFF);   // GRAB2 内偏已删, 发原始识别坐标
            std::cout << "[第二抓取] 发送: " << cmd
                      << "  (像素 " << res.bbox_center.x << ","
                      << res.bbox_center.y << " → 毫米)" << std::endl;
            if (sendArmLineAcked(cmd, &cap, "SEND CMD 4")) {
                sent = true;
                // ★ "4" = 臂放物资1到(wx-80,wy)+抓物资2(wx,wy), 两段~40s
                //   十轮: 改走"等待+中途重问"版
                waitArmReplyReask(cmd, "D4", HS_D4_TIMEOUT_SEC, &cap, "WAIT ARM D4");
            }
            break;
        }
    }

    if (!sent) std::cout << "[第二抓取] 窗口内未稳定锁定目标, 未发送第二抓取坐标" << std::endl;
    cleanup();
    return sent;
}

} // namespace material_grab

enum class CalibrationMode {
    NONE,
    LINE_RED,
    MATERIAL1,
    MATERIAL2
};

static void setupLineCalibrationWindows() {
    if (!g_gui_enabled) return;

    cv::namedWindow("Original", cv::WINDOW_NORMAL);
    cv::namedWindow("Mask", cv::WINDOW_NORMAL);
    cv::namedWindow("L channel", cv::WINDOW_NORMAL);
    cv::namedWindow("Red Mask", cv::WINDOW_NORMAL);
    cv::namedWindow("Params", cv::WINDOW_NORMAL);

    cv::resizeWindow("Original", 900, 650);
    cv::resizeWindow("Mask", 700, 500);
    cv::resizeWindow("L channel", 700, 500);
    cv::resizeWindow("Red Mask", 700, 500);
    cv::resizeWindow("Params", 560, 680);

    cv::createTrackbar("exposure",    "Params", &g_cam_exposure_val,   10000);
    cv::createTrackbar("adaptive",    "Params", &g_use_adaptive,           1);
    cv::createTrackbar("minContrast", "Params", &g_min_contrast,         200);
    cv::createTrackbar("frac",        "Params", &g_contrast_frac,        100);
    cv::createTrackbar("maxBright",   "Params", &g_max_brightness,       255);
    cv::createTrackbar("realLineMin", "Params", &g_real_line_min,        255);
    cv::createTrackbar("useOtsu",     "Params", &g_use_otsu,               1);
    cv::createTrackbar("minArea",     "Params", &g_min_area,           10000);
    cv::createTrackbar("blurSize",    "Params", &g_blur_size,             31);
    cv::createTrackbar("connectThr",  "Params", &g_connect_threshold,    600);
    cv::createTrackbar("redS",        "Params", &g_red_s_min,            255);
    cv::createTrackbar("redV",        "Params", &g_red_v_min,            255);
}

static void printLineCalibrationCommand() {
    std::cout << "[CAL] current params: "
              << visionProfileName(g_vision_profile)
              << " linecam=" << g_line_cam_id
              << " exp=" << g_cam_exposure_val
              << " adaptive=" << g_use_adaptive
              << " mincontrast=" << g_min_contrast
              << " frac=" << g_contrast_frac
              << " maxbright=" << g_max_brightness
              << " realmin=" << g_real_line_min
              << " minarea=" << g_min_area
              << " reds=" << g_red_s_min
              << " redv=" << g_red_v_min
              << std::endl;
    std::cout << "[CAL] dogonly template: ./xbtss eth0 "
              << visionProfileName(g_vision_profile)
              << " dogonly linecam=" << g_line_cam_id
              << " exp=" << g_cam_exposure_val
              << " adaptive=" << g_use_adaptive
              << " mincontrast=" << g_min_contrast
              << " frac=" << g_contrast_frac
              << " reds=" << g_red_s_min
              << " redv=" << g_red_v_min
              << std::endl;
}

static void applyLineCalibrationCameraControls(cv::VideoCapture& cap) {
    if (g_vision_profile == VisionProfile::SUN && CAM_LOCK_EXPOSURE) {
        cap.set(cv::CAP_PROP_EXPOSURE, g_cam_exposure_val);
    }
}

static bool runLineRedCalibrationMode() {
    g_gui_enabled = true;
    g_running = true;
    ResetTrajectory();
    setupLineCalibrationWindows();

    std::cout << "============================================" << std::endl;
    std::cout << "[CAL] line/red static calibration mode" << std::endl;
    std::cout << "[CAL] no DDS init, no SportClient, no Move command." << std::endl;
    std::cout << "[CAL] ESC/q quit, p prints current command params." << std::endl;
    std::cout << "============================================" << std::endl;

    cv::VideoCapture cap;
    if (!openAndConfigLineCamera(cap)) return false;

    for (int i = 0; i < 5; ++i) {
        cv::Mat tmp;
        cap >> tmp;
        usleep(20 * 1000);
    }

    printLineCalibrationCommand();

    while (g_running) {
        applyLineCalibrationCameraControls(cap);

        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            usleep(20 * 1000);
            continue;
        }

        cv::Mat mask, dbg_L;
        bool have_real_line = extractBlackLine(frame, mask, dbg_L);
        cv::Mat vis = frame.clone();
        int W = vis.cols;
        int H = vis.rows;

        cv::Rect roi_near(0, int(H * ROI_NEAR_TOP), W,
                          int(H * (ROI_NEAR_BOTTOM - ROI_NEAR_TOP)));
        cv::Rect roi_far(0, int(H * ROI_FAR_TOP), W,
                         int(H * (ROI_FAR_BOTTOM - ROI_FAR_TOP)));
        cv::Rect roi_stat(0, int(H * ROI_FAR_TOP), W,
                          int(H * (ROI_NEAR_BOTTOM - ROI_FAR_TOP)));

        int far_dir  = INT_MIN;
        int near_dir = INT_MIN;
        bool far_cross = false;
        bool near_cross = false;
        if (have_real_line) {
            far_dir = computeRoiDirection(mask, roi_far, vis,
                                          cv::Scalar(0, 200, 200),
                                          g_last_cx_far, g_fork_bias_far,
                                          &far_cross);
            near_dir = computeRoiDirection(mask, roi_near, vis,
                                           cv::Scalar(0, 255, 0),
                                           g_last_cx_near, g_fork_bias_near,
                                           &near_cross);
        }
        bool line_ok = have_real_line && (near_dir != INT_MIN);

        cv::rectangle(vis, roi_stat, cv::Scalar(255, 0, 255), 1);
        cv::line(vis, cv::Point(W / 2, 0), cv::Point(W / 2, H),
                 cv::Scalar(255, 255, 0), 1);
        int red_gate_y = int(H * RED_MIN_Y_FRAC);
        cv::line(vis, cv::Point(0, red_gate_y), cv::Point(W, red_gate_y),
                 cv::Scalar(0, 0, 255), 2);
        cv::putText(vis, "FAR ROI", cv::Point(18, std::max(18, roi_far.y - 6)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 200, 200), 2);
        cv::putText(vis, "NEAR ROI", cv::Point(18, std::max(18, roi_near.y - 6)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2);
        cv::putText(vis, "RED lower-half gate", cv::Point(18, red_gate_y + 24),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 255), 2);

        cv::Mat roi_mask = cv::Mat::zeros(mask.size(), mask.type());
        if (!mask.empty()) {
            mask(roi_far).copyTo(roi_mask(roi_far));
            mask(roi_near).copyTo(roi_mask(roi_near));
        }
        cv::Mat mask_view;
        cv::cvtColor(roi_mask, mask_view, cv::COLOR_GRAY2BGR);
        cv::rectangle(mask_view, roi_far, cv::Scalar(0, 200, 200), 2);
        cv::rectangle(mask_view, roi_near, cv::Scalar(0, 255, 0), 2);

        bool red_ok = detectRedCircle(vis);

        char hud1[256];
        std::snprintf(hud1, sizeof(hud1),
                      "CAL %s line=%s near=%d far=%d exp=%d",
                      visionProfileName(g_vision_profile),
                      line_ok ? "OK" : "MISS",
                      near_dir, far_dir, g_cam_exposure_val);
        cv::putText(vis, hud1, cv::Point(18, 32),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65,
                    line_ok ? cv::Scalar(0, 180, 0) : cv::Scalar(0, 0, 255), 2);

        char hud2[256];
        std::snprintf(hud2, sizeof(hud2),
                      "adaptive=%d minL=%d floorL=%d contrast=%d thr=%d mincontrast=%d frac=%d red=%s S=%d V=%d",
                      g_use_adaptive, g_dbg_minL, g_dbg_floorL, g_dbg_contrast, g_dbg_thr,
                      g_min_contrast, g_contrast_frac,
                      red_ok ? "OK" : "MISS", g_red_s_min, g_red_v_min);
        cv::putText(vis, hud2, cv::Point(18, 62),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(0, 0, 255), 2);

        cv::imshow("Original", vis);
        cv::imshow("Mask", mask_view);
        cv::imshow("L channel", dbg_L);

        int key = guiWaitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') break;
        if (key == 'p' || key == 'P') printLineCalibrationCommand();
    }

    cap.release();
    cv::destroyWindow("Original");
    cv::destroyWindow("Mask");
    cv::destroyWindow("L channel");
    cv::destroyWindow("Red Mask");
    cv::destroyWindow("Params");
    return true;
}

// ★ calmat 终端按键兜底 (2026-07-03): cv::waitKey 只在图像窗口有键盘焦点时才收到按键,
//   现场极易把 c 敲进终端然后"没反应"。这里同时轮询 stdin: 在终端敲 c/z/r/q + 回车同样生效。
//   一次读一行只取第一个有效字符, 连敲 "ccc" 也只算一次采点。
static int pollStdinKey() {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    timeval tv{0, 0};
    if (select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv) <= 0) return -1;
    char buf[64];
    ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
    for (ssize_t k = 0; k < n; ++k) {
        char ch = buf[k];
        if (ch != '\n' && ch != '\r' && ch != ' ' && ch != '\t') return (unsigned char)ch;
    }
    return -1;
}

static bool runMaterialCalibrationMode(bool second_point) {
    g_gui_enabled = true;
    g_running = true;

    material_grab::g_exposure = second_point
        ? material_grab::g_exposure_second
        : material_grab::g_exposure_first;

    std::cout << "============================================" << std::endl;
    std::cout << "[CALMAT] material camera calibration mode"
              << (second_point ? " (point 2)" : " (point 1)") << std::endl;
    std::cout << "[CALMAT] no arm serial command, no dog Move command." << std::endl;
    std::cout << "[CALMAT] ESC/q quit. Use exposure trackbar, then record matexp"
              << (second_point ? "2" : "1") << "=" << std::endl;
    std::cout << "============================================" << std::endl;

    material_grab::setupMaterialWindows();
    cv::VideoCapture cap;
    if (!material_grab::openMaterialCamera(cap)) {
        material_grab::destroyMaterialWindows();
        return false;
    }

    material_grab::DetectResult res;
    cv::Mat mask;
    cv::Rect roi;

    // ---- ★ 单应标定采点 (2026-07-03): 把真实物资摆到网格点 i 上, 按 c 采一点 ----
    //   网格世界坐标 = CAL_GRID_1/2 (臂基座系 mm, 先用臂点位标好并填进代码)。
    //   9 点采满自动 findHomography 拟合并保存 mat_cal1/2.txt (启动自动加载)。
    //   ★ 用真实物资采点: 检测的是物资"顶面"中心像素, 记录的是"底座"中心世界坐标,
    //     顶面视差/相机斜视被矩阵一并建模 —— 这正是"不在正上方会偏"问题的解法。
    double (*grid)[2] = second_point ? material_grab::CAL_GRID_2
                                     : material_grab::CAL_GRID_1;
    std::vector<cv::Point2f> cap_px;   // 已采像素点
    bool fitted = false;
    std::cout << "[CALMAT] 采点键: c=采当前检测中心  z=撤销上一点  r=全部重来"
              << "  (物资摆到提示的网格点上再按 c)" << std::endl;
    std::cout << "[CALMAT] ★ 按键两种发法均可: ①鼠标点一下图像窗口(获得焦点)后直接按键;"
              << " ②在本终端敲 c/z/r + 回车。q+回车 退出。" << std::endl;

    while (g_running) {
        char status[160];
        if ((int)cap_px.size() < material_grab::CAL_PTS) {
            int i = (int)cap_px.size();
            std::snprintf(status, sizeof(status),
                          "CAL PT %d/%d -> world(%.0f,%.0f)  [c]cap [z]undo [r]reset",
                          i + 1, material_grab::CAL_PTS, grid[i][0], grid[i][1]);
        } else {
            std::snprintf(status, sizeof(status), "CAL DONE (saved %s)  [r]redo",
                          material_grab::matCalFileName(second_point));
        }
        bool found = material_grab::updateMaterialPreview(cap, status, res, mask, roi);

        int key = material_grab::g_last_key;
        material_grab::g_last_key = -1;
        if (key < 0) key = pollStdinKey();   // 窗口没焦点时终端敲键也生效
        if (key == 'q' || key == 'Q' || key == 27) { g_running = false; break; }
        if ((key == 'c' || key == 'C')
            && (int)cap_px.size() < material_grab::CAL_PTS) {
            if (!found) {
                std::cout << "[CALMAT] 当前帧没有检测到物资, 不采点" << std::endl;
            } else {
                if (!res.board_found)
                    std::cout << "[CALMAT] 警告: BOARD MISS 状态下采点, 中心可能不准" << std::endl;
                cap_px.push_back(cv::Point2f((float)res.bbox_center.x,
                                             (float)res.bbox_center.y));
                int i = (int)cap_px.size() - 1;
                std::cout << "[CALMAT] 采点 " << (i + 1) << "/" << material_grab::CAL_PTS
                          << ": 像素(" << res.bbox_center.x << "," << res.bbox_center.y
                          << ") <-> 世界(" << grid[i][0] << "," << grid[i][1] << ")mm"
                          << std::endl;
                // 采满 → 拟合 + 保存
                if ((int)cap_px.size() == material_grab::CAL_PTS && !fitted) {
                    std::vector<cv::Point2f> world_pts;
                    for (int k = 0; k < material_grab::CAL_PTS; ++k)
                        world_pts.push_back(cv::Point2f((float)grid[k][0],
                                                        (float)grid[k][1]));
                    cv::Mat Hm = cv::findHomography(cap_px, world_pts, 0);
                    if (Hm.empty()) {
                        std::cout << "[CALMAT] ★ findHomography 失败 (点共线/退化?), 按 r 重采"
                                  << std::endl;
                        cap_px.pop_back();
                    } else {
                        double* H = second_point ? material_grab::g_H2
                                                 : material_grab::g_H1;
                        for (int r2 = 0; r2 < 3; ++r2)
                            for (int c2 = 0; c2 < 3; ++c2)
                                H[r2 * 3 + c2] = Hm.at<double>(r2, c2);
                        (second_point ? material_grab::g_H2_valid
                                      : material_grab::g_H1_valid) = 1;
                        material_grab::saveMatCal(second_point);
                        fitted = true;
                        // 回代残差: 每个采样点换算回去和真值差多少 mm
                        std::cout << "[CALMAT] 拟合完成, 回代残差:" << std::endl;
                        double worst = 0;
                        for (int k = 0; k < material_grab::CAL_PTS; ++k) {
                            double wx, wy;
                            material_grab::pixelToWorldMM(H, 1, cap_px[k].x, cap_px[k].y, wx, wy);
                            double ex = wx - grid[k][0], ey = wy - grid[k][1];
                            double e = std::sqrt(ex * ex + ey * ey);
                            worst = std::max(worst, e);
                            printf("  点%d: (%.1f,%.1f) vs (%.0f,%.0f)  误差 %.1fmm\n",
                                   k + 1, wx, wy, grid[k][0], grid[k][1], e);
                        }
                        std::cout << "[CALMAT] 最大残差 " << worst
                                  << "mm (建议 <5mm; 过大=某点摆歪了, 按 r 重采)" << std::endl;
                    }
                }
            }
        } else if (key == 'z' || key == 'Z') {
            if (!cap_px.empty()) { cap_px.pop_back(); fitted = false;
                std::cout << "[CALMAT] 撤销, 当前 " << cap_px.size() << " 点" << std::endl; }
        } else if (key == 'r' || key == 'R') {
            cap_px.clear(); fitted = false;
            std::cout << "[CALMAT] 重新采点" << std::endl;
        }
    }

    std::cout << "[CALMAT] final exposure: matexp"
              << (second_point ? "2" : "1") << "="
              << material_grab::g_exposure
              << " dark_frac=" << material_grab::g_last_dark_frac
              << "  标定: " << (fitted ? "已保存" : "未完成/沿用旧文件")
              << std::endl;

    cap.release();
    material_grab::destroyMaterialWindows();
    return true;
}

void waitDogOnlyPlatform(unitree::robot::go2::SportClient& sport,
                         const std::string& label,
                         double wait_sec = DOG_ONLY_PLATFORM_WAIT_SEC) {
    std::cout << "[" << label << "] dogonly: 跳过机械臂/抓取相机, 原地停留 "
              << wait_sec << "s 确认停位" << std::endl;
    // ★ 保持常规步态站姿: 不硬停 (硬停→BalanceStand=灵动/AI 站立),
    //   改为 StaticWalk + 持续喂 Move(0,0,0), 原地 planted 站立, 全程不回 AI 步态。
    sport.StaticWalk();
    usleep(100 * 1000);
    auto t0 = std::chrono::steady_clock::now();
    while (g_running) {
        double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0).count() / 1000.0;
        if (el >= wait_sec) break;
        sport.Move(0.0f, 0.0f, 0.0f);
        usleep(20 * 1000);
    }
}

// ============================================================
//  ★ 续跑入口 (startat=) — 人工干预/二次机会后从中途阶段重启
//    每个入口对应一组虚拟 yaw 偏移 + 全局累计角预载:
//      yaw_offset = 入口点狗的朝向相对"启停区出发朝向"的地图夹角 (左转为正)。
//    ★ 除 arc (= 1free, 实测过) 外是按场地图估的初值, 必须现场核一次
//      (方法见 xbts_使用与标定手册.md), 临时可用 yawoff=<度> 覆盖。
// ============================================================
enum class StartStage { FULL = 0, AVOID, STAIRS, ARC, TT, RED, CLOSE };
struct StartPreset { const char* name; float yaw_offset_deg; float cum_preload_deg; };
const StartPreset START_PRESETS[] = {
    { "full",   0.0f,    0.0f },
    { "avoid",  0.0f,    0.0f },   // 避障入口: 朝向与出发一致 (沿右侧线向前)
    { "stairs", 90.0f,   0.0f },   // 台阶脚下, 正对台阶 (★ 现场核)
    { "arc",    180.0f, 180.0f },  // 弧形巡线前 (= 1free 原值)
    { "tt",     0.0f,   310.0f },  // 抓取平台停位后: yaw≈初始, 累计角预载过 302° 门槛
    { "red",    180.0f, 310.0f },  // 红圈段起点 (★ 现场核)
    { "close", -90.0f,  310.0f },  // 收尾段/双侧检测前 (★ 现场核)
};

// ============================================================
// ============================================================
//  主程序: 五阶段流水线状态机
// ============================================================
// ============================================================
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0]
                  << " <networkInterface> [left|right] [lab|sun] [9|10] [nogui]"
                  << " [dogonly] [no] [cal|calmat] [startat=X] [linecam=N] [key=value...]" << std::endl;
        std::cout << "  left|right     正爬(9)台阶顶层转向方向 (默认 left; 倒爬10停稳后带正vx右转90°)"
                  << std::endl;
        std::cout << "  lab|sun        视觉环境 profile (默认 lab; sun=强太阳光低曝光)"
                  << std::endl;
        std::cout << "  动作 1|2|3     由警示标志视觉识别自动选择, 不接受预先输入"
                  << std::endl;
        std::cout << "  指令 5|6       由放置标志视觉识别自动选择, 不接受预先输入"
                  << std::endl;
        std::cout << "  数字 9|10      台阶策略: 9=原正向爬; 10=退开转180°后倒爬, 停稳后带正vx右转90°"
                  << std::endl;
        std::cout << "  nogui          ★ 比赛模式: 不开任何窗口/滑动条 (省算力)"
                  << std::endl;
        std::cout << "  dogonly        ★ 只调机械狗: 禁用机械臂/抓取相机, 平台停位只停 5s"
                  << std::endl;
        std::cout << "  no             ★ 保底安全模式: 跳过台阶, 阶段3后后退/左移/前进/左转, 弧形入口复用台阶后右移找线"
                  << std::endl;
        std::cout << "  cal/calline/calred  静态巡线+红圈标定: 不初始化狗, 不下发运动"
                  << std::endl;
        std::cout << "  calmat/calmat1/calmat2  静态物资相机标定: 不发机械臂/狗命令"
                  << std::endl;
        std::cout << "  startat=X      ★ 续跑入口: avoid|stairs|arc|tt|red|close"
                  << " (1free=arc 别名)" << std::endl;
        std::cout << "  yawoff=D       覆盖续跑入口虚拟 yaw 偏移 (度)" << std::endl;
        std::cout << "  redtimeout=S   红圈等待超时秒数 (默认 40)" << std::endl;
        std::cout << "  plat1trim=D    平台一停位锚点角修正 (度, 左正, 默认 0)"
                  << std::endl;
        std::cout << "  linecam=N      寻线摄像头设备号" << std::endl;
        std::cout << "  laserdev=DEV   前方激光串口 (默认 /dev/ttyUSB0)" << std::endl;
        std::cout << "  armdev=DEV     机械臂串口 (默认 /dev/ttyUSB1)" << std::endl;
        std::cout << "  exp=V          sun 模式巡线相机曝光 (默认 156)" << std::endl;
        std::cout << "  adaptive=0|1 mincontrast=V frac=V   自适应阈值参数"
                  << std::endl;
        std::cout << "  maxbright=V realmin=V minarea=V     旧阈值/面积覆盖"
                  << std::endl;
        std::cout << "  reds=V redv=V                       红圈 HSV S/V 下限"
                  << std::endl;
        std::cout << "  obdetect=0|1   白色横杆识别触发前跳 (默认 1; 0=回退纯计时)"
                  << std::endl;
        std::cout << "  fbalign=0|1    收尾蓝色启停区边界前后对齐闭环 (默认 1)"
                  << std::endl;
        std::cout << "  matexp1=V matexp2=V                 两次抓取的物资相机曝光"
                  << std::endl;
        return -1;
    }

    // ---------- 解析命令行 (顺序无关) ----------
    TurnDir turn_dir = TurnDir::LEFT;
    StairClimbMode stair_climb_mode = StairClimbMode::FORWARD;
    StartStage start_stage = StartStage::FULL;
    CalibrationMode cal_mode = CalibrationMode::NONE;
    bool  has_yawoff = false;
    float yawoff_val = 0.0f;
    // 视觉参数覆盖暂存 (在 ApplyVisionProfileDefaults 之后生效, 防止被 profile 冲掉)
    int ov_maxbright = -1, ov_realmin = -1, ov_minarea = -1;

    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        try {
            if      (arg == "left")  turn_dir = TurnDir::LEFT;
            else if (arg == "right") turn_dir = TurnDir::RIGHT;
            else if (arg == "lab")   g_vision_profile = VisionProfile::LAB;
            else if (arg == "sun")   g_vision_profile = VisionProfile::SUN;
            else if (arg == "nogui") g_gui_enabled = false;
            else if (arg == "dogonly") g_dog_only_mode = true;
            else if (arg == "no") g_no_stairs_mode = true;
            else if (arg == "cal" || arg == "calline" || arg == "calred")
                cal_mode = CalibrationMode::LINE_RED;
            else if (arg == "calmat" || arg == "calmat1")
                cal_mode = CalibrationMode::MATERIAL1;
            else if (arg == "calmat2")
                cal_mode = CalibrationMode::MATERIAL2;
            else if (arg == "1free") start_stage = StartStage::ARC;   // 兼容别名
            else if (arg.rfind("startat=", 0) == 0) {
                std::string s = arg.substr(8);
                if      (s == "avoid")  start_stage = StartStage::AVOID;
                else if (s == "stairs") start_stage = StartStage::STAIRS;
                else if (s == "arc")    start_stage = StartStage::ARC;
                else if (s == "tt")     start_stage = StartStage::TT;
                else if (s == "red")    start_stage = StartStage::RED;
                else if (s == "close")  start_stage = StartStage::CLOSE;
                else std::cerr << "[WARN] 未知 startat=" << s
                               << ", 走完整流程" << std::endl;
            }
            else if (arg.rfind("yawoff=", 0) == 0) {
                yawoff_val = std::stof(arg.substr(7)); has_yawoff = true;
            }
            else if (arg.rfind("redtimeout=", 0) == 0)  g_red_timeout_sec  = std::stof(arg.substr(11));
            else if (arg.rfind("plat1trim=", 0) == 0)   g_plat1_trim_deg   = std::stof(arg.substr(10));
            else if (arg.rfind("obdetect=", 0) == 0)    g_ob_detect_enabled = (std::stoi(arg.substr(9)) != 0);
            else if (arg.rfind("fbalign=", 0) == 0)     g_fb_align_enabled  = (std::stoi(arg.substr(8)) != 0);
            else if (arg.rfind("exp=", 0) == 0)         g_cam_exposure_val = std::stoi(arg.substr(4));
            else if (arg.rfind("maxbright=", 0) == 0)   ov_maxbright       = std::stoi(arg.substr(10));
            else if (arg.rfind("realmin=", 0) == 0)     ov_realmin         = std::stoi(arg.substr(8));
            else if (arg.rfind("minarea=", 0) == 0)     ov_minarea         = std::stoi(arg.substr(8));
            else if (arg.rfind("adaptive=", 0) == 0)    g_use_adaptive     = std::stoi(arg.substr(9));
            else if (arg.rfind("mincontrast=", 0) == 0) g_min_contrast     = std::stoi(arg.substr(12));
            else if (arg.rfind("frac=", 0) == 0)        g_contrast_frac    = std::stoi(arg.substr(5));
            else if (arg.rfind("reds=", 0) == 0)        g_red_s_min        = std::stoi(arg.substr(5));
            else if (arg.rfind("redv=", 0) == 0)        g_red_v_min        = std::stoi(arg.substr(5));
            else if (arg.rfind("matexp1=", 0) == 0)     material_grab::g_exposure_first  = std::stoi(arg.substr(8));
            else if (arg.rfind("matexp2=", 0) == 0)     material_grab::g_exposure_second = std::stoi(arg.substr(8));
            else if (arg.rfind("linecam=", 0) == 0)     g_line_cam_id      = arg.substr(8);
            else if (arg.rfind("matcam=", 0) == 0)      material_grab::g_material_cam_id = arg.substr(7);
            else if (arg.rfind("laserdev=", 0) == 0)    g_front_laser_dev  = arg.substr(9);
            else if (arg.rfind("armdev=", 0) == 0)      material_grab::g_arm_serial_dev = arg.substr(7);
        } catch (...) {
            std::cerr << "[WARN] 参数解析失败, 忽略: " << arg << std::endl;
        }
    }

    // ---------- 解析台阶策略数字参数 ----------
    //   1/2/3 与 5/6 保留给视觉识别结果, 命令行只接受 9/10 台阶策略。
    for (int i = 2; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "1" || a == "2" || a == "3" || a == "5" || a == "6") {
            std::cerr << "[ERROR] " << a
                      << " 是视觉识别结果, 不能在启动时预先输入" << std::endl;
            return -1;
        }
        if (a == "9")  stair_climb_mode = StairClimbMode::FORWARD;
        else if (a == "10") stair_climb_mode = StairClimbMode::REVERSE;
    }

    ApplyVisionProfileDefaults();
    // ★ 命令行视觉参数覆盖 (在 profile 默认值之后生效)
    if (ov_maxbright >= 0) g_max_brightness = ov_maxbright;
    if (ov_realmin   >= 0) g_real_line_min  = ov_realmin;
    if (ov_minarea   >= 0) g_min_area       = ov_minarea;

    // ★ 抓取标定 (像素→毫米单应矩阵) 自动加载: 工作目录下 mat_cal1.txt / mat_cal2.txt
    //   (calmat 采点模式生成; 没有文件 = 未标定, 对应抓取流程会拒发坐标并跳过)
    material_grab::loadMatCal(false);
    material_grab::loadMatCal(true);

    if (cal_mode != CalibrationMode::NONE) {
        if (!g_gui_enabled) {
            std::cout << "[CAL] ignore nogui: calibration mode needs OpenCV windows." << std::endl;
            g_gui_enabled = true;
        }
        std::signal(SIGINT, signalHandler);
        bool ok_cal = false;
        if (cal_mode == CalibrationMode::LINE_RED) {
            ok_cal = runLineRedCalibrationMode();
        } else {
            ok_cal = runMaterialCalibrationMode(cal_mode == CalibrationMode::MATERIAL2);
        }
        return ok_cal ? 0 : -1;
    }

    const StartPreset& start_preset = START_PRESETS[(int)start_stage];
    float start_yawoff = has_yawoff ? yawoff_val : start_preset.yaw_offset_deg;

    std::cout << "============================================" << std::endl;
    std::cout << "  RAICOM 整合流程  raicom_run" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "  阶段 1  巡线 10s + 盲走 2s → 避障" << std::endl;
    std::cout << "  阶段 2  避障 (" << SEGMENT_YAW_DEG.size() << " 段 + 三段侧移)" << std::endl;
    std::cout << "  阶段 3  巡线 (限时 3.5s)" << std::endl;
    std::cout << "  阶段 4  "
              << (g_no_stairs_mode ? "no保底: 跳过台阶, 后退/左移/前进/左转"
                                    : "爬台阶")
              << " [" << stairClimbModeName(stair_climb_mode);
    if (stair_climb_mode == StairClimbMode::FORWARD) {
        std::cout << ", top=" << (turn_dir == TurnDir::LEFT ? "left100" : "right100")
                  << ", vx=" << TURN_VX;
    } else {
        std::cout << ", preturn=180, stop=2s, top=right90, vx="
                  << ST_REVERSE_TURN_VX;
    }
    std::cout << "]"
              << std::endl;
    std::cout << "  阶段 5  弧形巡线 → 抓取平台停位 (yaw+lidar) → "
              << (g_dog_only_mode ? "dogonly停位确认 5s" : "机械臂抓取流程")
              << std::endl;
    std::cout << "  阶段 6  三连转检测巡线 (+46/-31/+92) → 停在中转平台前" << std::endl;
    std::cout << "  调试模式: "
              << (g_dog_only_mode
                      ? "dogonly 启用, 不打开机械臂串口/抓取相机, 各平台停 5s"
                      : "完整流程, 启用机械臂抓取/放置")
              << std::endl;
    std::cout << "  启动模式: ";
    if (start_stage == StartStage::FULL) {
        std::cout << "完整流程";
    } else {
        std::cout << "startat=" << start_preset.name
                  << " (yawoff=" << start_yawoff
                  << "°, preload=" << start_preset.cum_preload_deg << "°)";
    }
    std::cout << std::endl;
    std::cout << "  视觉模式: " << visionProfileName(g_vision_profile)
              << " (maxBright=" << g_max_brightness
              << " realLineMin=" << g_real_line_min
              << " useOtsu=" << g_use_otsu
              << " minArea=" << g_min_area
              << " connectThr=" << g_connect_threshold << ")" << std::endl;
    std::cout << "  寻线摄像头: " << g_line_cam_id
              << " (linecam=" << g_line_cam_id << ")" << std::endl;
    if (g_dog_only_mode) {
        std::cout << "  抓取摄像头: dogonly 下不打开" << std::endl;
    } else {
        std::cout << "  抓取摄像头: " << material_grab::g_material_cam_id
                  << " (matcam=" << material_grab::g_material_cam_id
                  << ", 独立于寻线相机)" << std::endl;
    }
    if (!g_dog_only_mode && g_line_cam_id == material_grab::g_material_cam_id) {
        std::cout << "  [WARN] 当前 linecam 与 matcam 同一个设备, 请检查"
                  << std::endl;
    }
    std::cout << "  标志识别: 始终启用 (1/2/3 与 5/6 仅由视觉结果触发; 失败则安全跳过)"
              << std::endl;
    if (g_no_stairs_mode) {
        std::cout << "  no保底安全模式: 启用 (阶段3后跳过台阶, 后退 "
                  << NO_STAIR_PRE_BACK_VX << "m/s × "
                  << NO_STAIR_PRE_BACK_SEC
                  << "s; 固定左移 " << NO_STAIR_LEFT_FAST_VY
                  << "m/s × " << NO_STAIR_LEFT_FAST_SEC
                  << "s; 未锁线再左移 " << NO_STAIR_LEFT_SEARCH_VY
                  << "m/s; 锁线后前进 " << NO_STAIR_AFTER_LINE_FWD_VX
                  << "m/s × " << NO_STAIR_AFTER_LINE_FWD_SEC
                  << "s; 左转 " << NO_STAIR_TURN_LEFT_DEG
                  << "°; 转后前进 " << NO_STAIR_AFTER_TURN_FWD_VX
                  << "m/s × " << NO_STAIR_AFTER_TURN_FWD_SEC
                  << "s; 随后进入弧形入口, 复用台阶后右移找线 vy="
                  << POST_STAIR_SHIFT_VY << "m/s, 上限 "
                  << ARC_START_SHIFT_MAX_SEC << "s)" << std::endl;
    }
    std::cout << "  红圆检测: 启用 → 红圆触发后识别警示标志并选择动作 1/2/3;"
              << " 识别失败则跳过动作" << std::endl;
    if (g_dog_only_mode) {
        std::cout << "  双侧平台机械臂指令: dogonly 下禁用, 只停 "
                  << DOG_ONLY_PLATFORM_WAIT_SEC << "s" << std::endl;
    } else {
        std::cout << "  双侧平台机械臂指令: 由放置标志识别选择 5/6;"
                  << " 识别失败则不发送" << std::endl;
    }
    std::cout << "  横杆障碍识别: " << (g_ob_detect_enabled ? "启用" : "关闭 (obdetect=0)")
              << " (窗口=名义时刻±" << OB_WIN_HALF_SEC
              << "s, 窗口内未识别到则按原计时兜底前跳)" << std::endl;
    std::cout << "  收尾前后闭环: " << (g_fb_align_enabled
                  ? "启用 (蓝区边界对齐, 起跑前自动采起点参照)"
                  : "关闭 (fbalign=0)") << std::endl;
    std::cout << "  步态: 巡线/避障=StaticWalk, 台阶=FreeWalk" << std::endl;
    std::cout << "  GUI: " << (g_gui_enabled ? "开启 (调试)" : "关闭 (nogui 比赛模式)")
              << std::endl;
    std::cout << "  避障前距: 前方激光 " << g_front_laser_dev
              << " (laserdev= 可改; 断流回退 Go2 雷达; 左右用 Go2 LiDAR)" << std::endl;
    if (!g_dog_only_mode) {
        std::cout << "  机械臂串口: " << material_grab::g_arm_serial_dev
                  << " (armdev= 可改)" << std::endl;
    }
    std::cout << "============================================" << std::endl;

    std::signal(SIGINT, signalHandler);
    bool needs_front_laser_for_avoidance =
        (start_stage == StartStage::FULL || start_stage == StartStage::AVOID);
    std::thread front_laser_thread;
    auto stop_front_laser_thread = [&]() {
        g_front_laser_running = false;
        if (front_laser_thread.joinable()) front_laser_thread.join();
    };

    // ---------- DDS (ChannelFactory 全局只 Init 一次) ----------
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>
        state_sub("rt/sportmodestate");
    state_sub.InitChannel(StateHandler, 1);

    unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>
        lowstate_sub("rt/lowstate");
    lowstate_sub.InitChannel(LowStateHandler, 1);

    unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::PointStamped_>
        range_sub("rt/utlidar/range_info");
    range_sub.InitChannel(RangeInfoHandler, 1);

    // ---------- 等待三路数据 ----------
    std::cout << "[INFO] 等待 IMU / LowState / LiDAR 数据..." << std::endl;
    int wait = 0;
    while ((!g_state_received || !g_lowstate_received || !g_lidar_received)
           && wait < 100 && g_running) {
        usleep(100 * 1000);
        wait++;
    }
    if (!g_state_received || !g_lowstate_received || !g_lidar_received) {
        std::cerr << "[错误] 10s 未收齐数据 (sportstate="
                  << g_state_received.load() << " lowstate="
                  << g_lowstate_received.load() << " lidar="
                  << g_lidar_received.load() << "),检查网卡和 Go2 状态" << std::endl;
        return -1;
    }
    std::cout << "[OK] 三路数据通道已连通" << std::endl;
    ResetInitialYawBaseline();

    // ---------- 第一阶段触发条件 ----------
    // 当前版本不再加载/识别“入口”模板,主流程也不再用丢线判断触发避障。
    // 触发避障固定为:巡线 10s + 盲走 2s。
    g_line_lost_avoidance_triggered = false;

    // ---------- OpenCV 窗口 + 滑动条 (★ nogui 比赛模式不创建 —— 与物资/标定窗口
    //   的门控保持一致; 无 X 显示环境时 namedWindow 会抛异常直接崩在启动) ----------
    if (g_gui_enabled) {
        cv::namedWindow("Original", cv::WINDOW_NORMAL);
        cv::namedWindow("Mask", cv::WINDOW_NORMAL);
        cv::namedWindow("L channel", cv::WINDOW_NORMAL);
        cv::resizeWindow("Original", 800, 600);
        cv::resizeWindow("Mask", 600, 450);
        cv::resizeWindow("L channel", 600, 450);

        cv::namedWindow("Params", cv::WINDOW_NORMAL);
        cv::resizeWindow("Params", 520, 560);
        // 巡线参数
        cv::createTrackbar("maxBright",   "Params", &g_max_brightness,    255);
        cv::createTrackbar("realLineMin", "Params", &g_real_line_min,     255);
        cv::createTrackbar("useOtsu",     "Params", &g_use_otsu,            1);
        cv::createTrackbar("minArea",     "Params", &g_min_area,        10000);
        cv::createTrackbar("blurSize",    "Params", &g_blur_size,          21);
        cv::createTrackbar("connectThr",  "Params", &g_connect_threshold, 600);
        cv::createTrackbar("Kp_x1000",    "Params", &g_kp_x1000,           20);
        cv::createTrackbar("Kd_x1000",    "Params", &g_kd_x1000,           30);
    }

    // ---------- 摄像头 (开头打开一次,贯穿全程) ----------
    cv::VideoCapture cap;
    if (USE_USB_CAMERA) {
        std::cout << "[INFO] USB 摄像头: " << g_line_cam_id
                  << " (强制 V4L2 后端, 否则曝光控制不可用)" << std::endl;
        if (openCameraByIdOrIndex(cap, g_line_cam_id, "寻线相机")) {
            std::cout << "[OK] 打开成功, backend="
                      << cap.getBackendName() << std::endl;
        }
    } else {
        cap.open(GO2_CAM_PIPELINE, cv::CAP_GSTREAMER);
    }
    if (!cap.isOpened()) {
        std::cerr << "[错误] 无法打开摄像头" << std::endl;
        return -1;
    }
    {
        bool ok_buf = cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
        std::cout << "[摄像头] set BUFFERSIZE=1 ret=" << ok_buf
                  << " 回读=" << cap.get(cv::CAP_PROP_BUFFERSIZE) << std::endl;
    }

    // ---------- 摄像头曝光/白平衡锁定 ----------
    //   lab 模式主动恢复自动曝光; sun 模式沿用低曝光,应对强直射光。
    //   V4L2 设置可能持久化,所以 lab 也必须显式 set 回自动曝光。
    bool use_manual_exposure =
        (g_vision_profile == VisionProfile::SUN) && CAM_LOCK_EXPOSURE;
    if (use_manual_exposure) {
        std::cout << "[摄像头] sun 模式: 尝试锁定曝光 target="
                  << g_cam_exposure_val << std::endl;
        // 先关自动曝光 (V4L2: 1=手动 3=自动)
        bool ok_auto = cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        usleep(100 * 1000);
        // 再设手动曝光值
        bool ok_exp  = cap.set(cv::CAP_PROP_EXPOSURE, g_cam_exposure_val);
        usleep(100 * 1000);
        std::cout << "  set AUTO_EXPOSURE=1 ret=" << ok_auto
                  << "  set EXPOSURE=" << g_cam_exposure_val << " ret=" << ok_exp
                  << std::endl;
        std::cout << "  回读 AUTO_EXPOSURE=" << cap.get(cv::CAP_PROP_AUTO_EXPOSURE)
                  << "  EXPOSURE=" << cap.get(cv::CAP_PROP_EXPOSURE) << std::endl;

        // 若回读的 AUTO_EXPOSURE 不是 1, 试试 3 (不同摄像头约定不同)
        if ((int)cap.get(cv::CAP_PROP_AUTO_EXPOSURE) == 3) {
            std::cout << "  [警告] AUTO_EXPOSURE 还是 3 (自动), 试 set=3..." << std::endl;
            cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);
            usleep(100 * 1000);
            cap.set(cv::CAP_PROP_EXPOSURE, g_cam_exposure_val);
            std::cout << "  二次回读 AUTO_EXPOSURE="
                      << cap.get(cv::CAP_PROP_AUTO_EXPOSURE)
                      << "  EXPOSURE=" << cap.get(cv::CAP_PROP_EXPOSURE) << std::endl;
        }
    } else {
        std::cout << "[摄像头] lab 模式: 恢复自动曝光 (避免沿用 sun 低曝光)"
                  << std::endl;
        // V4L2 常见约定: 3=自动曝光,1=手动曝光。
        bool ok_auto = cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);
        usleep(200 * 1000);
        std::cout << "  set AUTO_EXPOSURE=3 ret=" << ok_auto
                  << "  回读 AUTO_EXPOSURE=" << cap.get(cv::CAP_PROP_AUTO_EXPOSURE)
                  << "  EXPOSURE=" << cap.get(cv::CAP_PROP_EXPOSURE) << std::endl;
    }

    if (CAM_LOCK_WB) {
        std::cout << "[摄像头] 尝试锁定白平衡: target=" << CAM_WB_TEMP << "K" << std::endl;
        bool ok_awb = cap.set(cv::CAP_PROP_AUTO_WB, 0);
        usleep(50 * 1000);
        bool ok_wb  = cap.set(cv::CAP_PROP_WB_TEMPERATURE, CAM_WB_TEMP);
        std::cout << "  set AUTO_WB=0 ret=" << ok_awb
                  << "  set WB_TEMPERATURE=" << CAM_WB_TEMP << " ret=" << ok_wb
                  << std::endl;
        std::cout << "  回读 AUTO_WB=" << cap.get(cv::CAP_PROP_AUTO_WB)
                  << "  WB_TEMPERATURE=" << cap.get(cv::CAP_PROP_WB_TEMPERATURE)
                  << std::endl;
    } else {
        // ★ V4L2 设置会持久化! 上次锁的 WB 会留在驱动里, 必须主动恢复自动 WB,
        //   否则画面会一直绿偏 (停留在 4600K 之类的旧手动值)
        std::cout << "[摄像头] 恢复自动白平衡 (避免持久化的旧 WB 锁定)" << std::endl;
        bool ok_awb = cap.set(cv::CAP_PROP_AUTO_WB, 1);
        usleep(50 * 1000);
        std::cout << "  set AUTO_WB=1 ret=" << ok_awb
                  << "  回读 AUTO_WB=" << cap.get(cv::CAP_PROP_AUTO_WB) << std::endl;
    }

    if (CAM_LOCK_GAIN) {
        bool ok_gain = cap.set(cv::CAP_PROP_GAIN, CAM_GAIN_VAL);
        std::cout << "[摄像头] set GAIN=" << CAM_GAIN_VAL << " ret=" << ok_gain
                  << "  回读=" << cap.get(cv::CAP_PROP_GAIN) << std::endl;
    }

    // 锁定后丢弃前几帧 (有些摄像头切换设置后头几帧是过渡帧)
    {
        cv::Mat tmp;
        for (int i = 0; i < 5; ++i) cap >> tmp;
    }

    // ---------- 前方激光测距 (仅避障阶段使用; 左右仍用 Go2 LiDAR) ----------
    if (needs_front_laser_for_avoidance) {
        g_front_laser_running = true;
        front_laser_thread = std::thread(frontLaserThread);
        std::cout << "[INFO] 等待前方激光数据..." << std::endl;
        for (int i = 0; i < 50 && g_running; ++i) {
            if (g_front_laser.ms.load() != 0) break;
            usleep(100 * 1000);
        }
        if (g_front_laser.ms.load() == 0) {
            // ★ 不再拒跑: 比赛只有两次机会, "拒跑"比"降级跑"贵得多。
            //   frontLaserDistFresh() 断流时自动回退 Go2 雷达前距, 避障仍能跑。
            //   调试时看到这条告警, 应 Ctrl+C 先查串口号(laserdev=)/权限/接线。
            std::cerr << "\n[警告] 5s 未收到前方激光数据 (laserdev="
                      << g_front_laser_dev << ")!\n"
                      << "[警告] 降级继续: 避障前距回退 Go2 雷达。"
                      << "若在调试, 建议 Ctrl+C 排查后再跑\n" << std::endl;
        } else {
            std::cout << "[前激光] front=" << frontLaserDistFresh() << "m" << std::endl;
        }
    }

    // ---------- SportClient ----------
    unitree::robot::go2::SportClient sport;
    sport.SetTimeout(10.0f);
    sport.Init();

    // ---------- VuiClient (动作 3 可能由视觉识别触发, 因此预先初始化) ----------
    g_vui = new unitree::robot::go2::VuiClient();
    g_vui->Init();
    std::cout << "[INFO] VuiClient 已初始化 (前灯控制)" << std::endl;

    // ---------- ★ 机械臂串口: 启动时打开一次, 全程持久保持 (2026-07-05) ----------
    //   如果 open 触发臂板复位, 也是在比赛跑动开始之前复位完毕 (第一条臂命令在
    //   下台阶之后, 距此有好几分钟), 不会再出现"比赛中途臂静默重启吃掉回执"。
    if (!g_dog_only_mode) {
        bool arm_fresh = false;
        if (material_grab::armPortEnsureOpen(&arm_fresh)) {
            std::cout << "[ARM] 启动预连接成功"
                      << (arm_fresh ? " (刚打开, 若臂板被复位会自行重启, 无碍)" : "")
                      << ", 监听臂输出 " << material_grab::ARM_STARTUP_DRAIN_SEC
                      << "s..." << std::endl;
            material_grab::monitorSerialInput(
                material_grab::g_arm_fd,
                material_grab::ARM_STARTUP_DRAIN_SEC, "startup");
        } else {
            std::cout << "[ARM] 启动预连接失败 (armdev="
                      << material_grab::g_arm_serial_dev
                      << "), 各阶段会自动重试打开" << std::endl;
        }
    }

    // ---------- ★ 收尾起点参照采集 (狗此刻就摆在启停区里, 人工摆正 = 正确答案) ----------
    //   ① 蓝区前边界行号 → 收尾③把边界伺服回这一行 = 复现起点前后摆位;
    //   ② 黑线在画面里的 x → 收尾①把线伺服回这个 x = 复现起点左右摆位
    //      (治五连测"次次偏右却不再调": 线在画面几何中心 ≠ 身体在区里居中)。
    //   只在完整流程时采集 (startat 续跑时狗不在启停区, 采了也是错的)。
    if (g_fb_align_enabled && start_stage == StartStage::FULL && g_running) {
        cv::Mat f, cap_m, cap_d;
        for (int i = 0; i < 8 && g_running; ++i) { cap >> f; usleep(30 * 1000); }  // 排空旧帧
        int row_sum = 0, row_cnt = 0;
        int x_sum = 0, x_cnt = 0;
        for (int i = 0; i < 5 && g_running; ++i) {
            cap >> f;
            int r = detectBlueZoneBoundary(f);
            if (r >= 0) { row_sum += r; row_cnt++; }
            int cx = -1;
            if (!f.empty()
                && getFinalLineCenterXNoBlue(f, cx, cap_m, cap_d,
                                             FINAL_CENTER_MIN_AREA)) {
                x_sum += cx; x_cnt++;   // ★ 排蓝: 垫子不再拉偏起点线参照
            }
            usleep(30 * 1000);
        }
        if (row_cnt >= 3) {
            g_final_fb_target_row = row_sum / row_cnt;
            std::cout << "[启停区] 起点参照采集成功: 蓝区边界行 = "
                      << g_final_fb_target_row << " (" << row_cnt
                      << "/5 帧有效; 收尾前后按此行对齐)" << std::endl;
        } else {
            std::cout << "[启停区] 起点参照采集失败 (5 帧仅 " << row_cnt
                      << " 帧看到蓝区) → 收尾退回默认目标行 "
                      << FINAL_FB_TARGET_ROW_FRAC_DEFAULT
                      << "×H (可调 FINAL_FB_* 阈值, 或 fbalign=0 关闭前后闭环)" << std::endl;
        }
        if (x_cnt >= 3) {
            g_final_lat_target_x = x_sum / x_cnt;
            std::cout << "[启停区] 起点左右参照: 黑线 x = " << g_final_lat_target_x
                      << " (" << x_cnt << "/5 帧有效; 收尾左右按此 x 对齐)" << std::endl;
        } else {
            std::cout << "[启停区] 起点左右参照未采到 (线不在视野?) → 收尾退回画面中心 x"
                      << std::endl;
        }
    }

    // ---------- 启动倒计时 (整套流程只此一个) ----------
    std::cout << "\n[INFO] 5 秒后启动 ";
    if (start_stage == StartStage::FULL) {
        std::cout << "整套流程,把狗摆在巡线起点";
    } else {
        std::cout << "startat=" << start_preset.name << " 续跑流程,按该入口摆位";
    }
    std::cout << ";Ctrl+C / ESC 中断" << std::endl;
    for (int i = 5; i > 0 && g_running; --i) {
        std::cout << i << "..." << std::flush;
        sleep(1);
    }
    std::cout << std::endl;
    if (!g_running) {
        stop_front_laser_thread();
        material_grab::armPortCloseAtExit();
        cap.release();
        if (g_gui_enabled) cv::destroyAllWindows();
        return 0;
    }

    // ============================================================
    //  五阶段流水线 + startat 续跑入口
    //    失败策略: ESC/Ctrl+C 才全停; 普通超时尽量继续保后续分。
    // ============================================================
    bool ok = true;
    bool hold_after_stair_failure = false;
    float yaw_baseline_stair = g_yaw_rad.load();
    bool  stair_baseline_from_avoid = false;  // ★ 八轮: 基准是否来自"避障刚结束"快照

    bool run_stage1      = (start_stage == StartStage::FULL);
    bool run_avoid_stage = (start_stage == StartStage::FULL || start_stage == StartStage::AVOID);
    bool run_pre_stairs  = (start_stage == StartStage::FULL || start_stage == StartStage::AVOID);
    bool run_no_stairs_bypass =
        g_no_stairs_mode && (start_stage == StartStage::FULL || start_stage == StartStage::AVOID);
    if (g_no_stairs_mode && !run_no_stairs_bypass) {
        std::cout << "[no] 当前 startat=" << start_preset.name
                  << ", no 保底只在完整流程/避障续跑时生效, 本次忽略 no" << std::endl;
    }
    bool run_stairs_stage= !run_no_stairs_bypass
                         && (start_stage == StartStage::FULL || start_stage == StartStage::AVOID
                          || start_stage == StartStage::STAIRS);
    bool run_post_stairs = run_stairs_stage;
    bool run_arm0_pause  = (start_stage == StartStage::FULL || start_stage == StartStage::AVOID
                         || start_stage == StartStage::STAIRS || start_stage == StartStage::ARC);
    bool run_arc_stage   = (start_stage == StartStage::FULL || start_stage == StartStage::AVOID
                         || start_stage == StartStage::STAIRS || start_stage == StartStage::ARC);
    bool run_tt_stage    = (start_stage == StartStage::FULL || start_stage == StartStage::AVOID
                         || start_stage == StartStage::STAIRS || start_stage == StartStage::ARC
                         || start_stage == StartStage::TT);
    bool run_red_wait    = (start_stage == StartStage::FULL || start_stage == StartStage::AVOID
                         || start_stage == StartStage::STAIRS || start_stage == StartStage::ARC
                         || start_stage == StartStage::TT || start_stage == StartStage::RED);

    if (start_stage != StartStage::FULL) {
        std::cout << "\n############ startat=" << start_preset.name << " 续跑模式 ############"
                  << std::endl;
        std::cout << "[startat] yawoff=" << start_yawoff
                  << "° preload_cum=" << start_preset.cum_preload_deg
                  << "°" << std::endl;
        yaw_baseline_stair = SetVirtualYawBaseline(
            start_yawoff, start_preset.cum_preload_deg, start_preset.name);
    }
    if (start_stage == StartStage::CLOSE) {
        g_warn_action_done = true;
        run_red_wait = false;
        std::cout << "[startat=close] 直接进入红圆后收尾序列 (双侧平台检测前)"
                  << std::endl;
    }

    struct DogOnlyTimingRow {
        std::string name;
        double dog_sec;
        double full_est_sec;
        bool has_full_est;
    };
    std::vector<DogOnlyTimingRow> dogonly_timing_rows;
    auto timingNow = []() {
        return std::chrono::steady_clock::now();
    };
    auto fmtSec = [](double sec) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.1f", sec);
        return std::string(buf);
    };
    auto addDogOnlyTiming = [&](const std::string& name,
                                std::chrono::steady_clock::time_point t0,
                                double full_extra_sec = 0.0,
                                const std::string& note = std::string()) {
        // ★ 2026-07-06 六轮 (用户要求): 完整模式也记录并打印每阶段耗时+总耗时
        //   (原先 !dogonly 直接 return, 正式跑一次计时都看不到)
        if (full_extra_sec < 0.0) full_extra_sec = 0.0;
        double dog_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - t0).count() / 1000.0;
        bool has_full = full_extra_sec > 0.05;
        double full_est = dog_sec + full_extra_sec;
        dogonly_timing_rows.push_back({name, dog_sec, full_est, has_full});

        std::cout << "[计时] " << name
                  << " 用时=" << fmtSec(dog_sec) << "s";
        if (has_full) {
            std::cout << " full_est=" << fmtSec(full_est)
                      << "s (+" << fmtSec(full_extra_sec)
                      << "s 机械臂/等待估算)";
        }
        if (!note.empty()) std::cout << " " << note;
        std::cout << std::endl;
    };
    auto printDogOnlyTimingSummary = [&]() {
        if (dogonly_timing_rows.empty()) return;   // ★ 六轮: 完整模式也打印汇总
        double dog_total = 0.0;
        double full_total = 0.0;
        bool any_full = false;
        std::cout << "\n[计时汇总] (各阶段 + 总计)" << std::endl;
        for (const auto& row : dogonly_timing_rows) {
            dog_total += row.dog_sec;
            full_total += row.full_est_sec;
            any_full = any_full || row.has_full_est;
            std::cout << "  - " << row.name
                      << ": " << fmtSec(row.dog_sec) << "s";
            if (row.has_full_est) {
                std::cout << " (dogonly 估完整=" << fmtSec(row.full_est_sec) << "s)";
            }
            std::cout << std::endl;
        }
        std::cout << "  ★ 总计=" << fmtSec(dog_total) << "s";
        if (any_full) std::cout << " (dogonly 估完整总计=" << fmtSec(full_total) << "s)";
        std::cout << std::endl;
    };

    // ----- 阶段 1: 起步巡线 1s → 前跳 → 计时巡线 + 盲走 → 避障 -----
    if (ok && g_running && run_stage1) {
        auto t_stage1 = timingNow();
        std::cout << "\n############ 阶段 1/6: 起步前跳 + 巡线 + 盲走 ############" << std::endl;
        const float stage1_initial_yaw = g_initial_yaw_ready.load()
            ? g_initial_yaw_deg.load() : g_yaw_deg.load();
        // ★ 2026-07-05: 首跳触发从"纯计时 START_LINE_SEC"改为"横杆识别 + 时间窗兜底":
        //   在 [名义-OB_WIN_HALF, 名义+OB_WIN_HALF] 内识别到"黑线被亮横带截断"且
        //   缺口下沿走到触发行 → 立刻起跳 (距离由画面位置决定, 不吃速度误差);
        //   窗口上限还没识别到 → 照跳 (最坏情况 = 原纯计时方案)。obdetect=0 回退纯计时。
        // ★ 2026-07-06: 名义时刻按距离等效换算 —— 原方案 START_LINE_SEC 秒 @ 0.20 m/s
        //   走过的距离, 换算成 "0.20x0.5s 起步踢 + 0.10 蠕行" 曲线所需时间 (≈3.5s)。
        double ob1_dist = START_LINE_SEC * START_PREJUMP_LINE_SPEED;   // ≈0.40 m
        double ob1_nom  = obNominalSecForDist(ob1_dist);
        double ob1_min  = std::max(OB_APPROACH_KICK_SEC, ob1_nom - OB_WIN_HALF_SEC);
        // obdetect=0 时窗口上限收缩到名义时刻本身 → 行为 = 纯计时 (等效距离不变)
        double ob1_max  = g_ob_detect_enabled ? (ob1_nom + OB_WIN_HALF_SEC) : ob1_nom;
        LineResult r0 = runLineFollowing(sport, cap, LineMode::TO_OBSTACLE, ob1_max,
                                         0.0f, true, OB_APPROACH_KICK_SPEED,
                                         OB_APPROACH_KICK_SEC, OB_APPROACH_SLOW_SPEED,
                                         1.0, 0.0, ob1_min);
        if (r0 == LineResult::ABORTED) ok = false;
        if (ok && g_running) {
            if (!doFrontJump(sport, (float)OB_APPROACH_SLOW_SPEED, /*quick=*/true,
                             /*align_yaw_deg=*/stage1_initial_yaw)) {
                ok = false;
            }
            if (ok && g_running) {
                std::cout << "[阶段1] 前跳落地, 原地站稳 " << POST_JUMP_SETTLE_SEC
                          << "s (带落地姿态看门狗), 再做航向复位..." << std::endl;
                postJumpAttitudeGuard(sport, POST_JUMP_SETTLE_SEC, 0.0);
                std::cout << "[阶段1] 前跳后航向复位: 当前 yaw=" << g_yaw_deg.load()
                          << "° → 初始 yaw=" << stage1_initial_yaw << "°" << std::endl;
                if (!turnToYawDeg(sport, stage1_initial_yaw, 1.0f,
                                  /*stop_at_end=*/true)) {
                    std::cout << "[阶段1] 航向复位失败，禁止带偏航进入后续巡线/避障"
                              << std::endl;
                    ok = false;
                } else {
                    std::cout << "[阶段1] 航向复位完成: yaw=" << g_yaw_deg.load()
                              << "°" << std::endl;
                    if (!shiftLeftAfterFirstJump(sport, stage1_initial_yaw)) {
                        std::cout << "[阶段1] 跳后固定左移未正常完成，禁止继续后续巡线/避障"
                                  << std::endl;
                        ok = false;
                    }
                }
            }
        }
        if (ok && g_running) {
            const double stage1_line_total =
                STAGE1_LINE_KICK_TIME_SEC + STAGE1_LINE_SLOW_TIME_SEC;
            std::cout << "[阶段1] 前跳后定时巡线: fixed vx="
                      << STAGE1_LINE_KICK_SPEED << " "
                      << STAGE1_LINE_KICK_TIME_SEC << "s, 然后 vx="
                      << STAGE1_LINE_SLOW_SPEED << " "
                      << STAGE1_LINE_SLOW_TIME_SEC << "s, 转向修正 x"
                      << STAGE1_LINE_ROT_MULT << std::endl;
            LineResult r1 = runLineFollowing(sport, cap, LineMode::TIMED,
                                             stage1_line_total,
                                             0.0f, true, STAGE1_LINE_KICK_SPEED,
                                             STAGE1_LINE_KICK_TIME_SEC,
                                             STAGE1_LINE_SLOW_SPEED,
                                             STAGE1_LINE_ROT_MULT,
                                             STAGE1_ROT_RAMP_SEC,
                                             /*ob_window_min_sec=*/0.0,
                                             /*assert_static_walk=*/false);
            if (r1 == LineResult::ABORTED) ok = false;
        }
        if (ok && g_running) {
            if (!runBlindForward(sport, STAGE1_BLIND_TIME_SEC,
                                 BLIND_FORWARD_SPEED, stage1_initial_yaw)) {
                ok = false;
            }
        }
        addDogOnlyTiming("阶段1 起步前跳+巡线+盲走", t_stage1);
    }

    // ----- 阶段 2: 避障 -----
    if (ok && g_running && run_avoid_stage) {
        auto t_stage2 = timingNow();
        std::cout << "\n############ 阶段 2/6: 避障 ############" << std::endl;
        bool avoid_ok = runAvoidance(sport);
        stop_front_laser_thread();
        std::cout << "[前激光] 避障结束, 已释放串口" << std::endl;
        // ★ 八轮 (用户): 台阶 yaw 基准改在【避障刚结束】快照 —— 之后的巡线会被
        //   画面里的黑色台阶带偏, 避障出口朝向才是准确朝向; 爬梯直线保持会把
        //   阶段3/3.5 弄歪的头主动拉回这个朝向。
        yaw_baseline_stair = g_yaw_rad.load();
        stair_baseline_from_avoid = true;
        std::cout << "[台阶基准] 避障结束快照 yaw = "
                  << rad2deg(yaw_baseline_stair) << "°" << std::endl;
        if (!avoid_ok) {
            if (g_running) {
                std::cout << "[警告] 避障未正常完成, 继续后续阶段尽量保分" << std::endl;
            } else {
                ok = false;
            }
        }
        addDogOnlyTiming("阶段2 避障", t_stage2);
    } else if (!run_avoid_stage) {
        stop_front_laser_thread();
    }

    // ----- 阶段 3: 巡线 (限时 3.5s) -----
    if (ok && g_running && run_pre_stairs) {
        auto t_stage3 = timingNow();
        std::cout << "\n############ 阶段 3/6: 巡线 (限时 3.5s) ############" << std::endl;
        // ★ 九轮 (1808 -30 停太早 + 用户"多走"): 3.0→3.5s, 更贴近台阶再开爬;
        //   台阶 yaw 基准仍用避障结束快照
        // ★ 十三轮 (0007 -30: 巡线相机看到黑色台阶被当成线, 向右踩一步没爬上):
        //   3.5→3.0s, 减少末段看见台阶误修正的窗口 (停车点约后移 0.1m)
        // ★ 国赛调试: 台阶前停位偏早, 3.0→3.5s, 向台阶再靠近约半秒。
        LineResult r3 = runLineFollowing(sport, cap, LineMode::TIMED, 3.5);
        if (r3 == LineResult::ABORTED) ok = false;
        addDogOnlyTiming("阶段3 台阶前巡线", t_stage3);
    }

    // ----- 阶段3.5: 向右平移 (常规步态), 给上台阶对位 -----
    if (ok && g_running && run_pre_stairs) {
        auto t_stage35 = timingNow();
        if (std::abs(PRE_STAIR_SHIFT_VY) < 1e-6f || PRE_STAIR_SHIFT_SEC <= 0.0) {
            std::cout << "\n[阶段3.5] 台阶前右移关闭, 不平移" << std::endl;
        } else {
            std::cout << "\n[阶段3.5] 向右平移 vy=" << PRE_STAIR_SHIFT_VY
                      << " " << PRE_STAIR_SHIFT_SEC << "s (常规步态)" << std::endl;
            sport.StaticWalk();
            usleep(300 * 1000);
            auto t_shift = std::chrono::steady_clock::now();
            while (g_running) {
                double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - t_shift).count() / 1000.0;
                if (el >= PRE_STAIR_SHIFT_SEC) break;
                sport.Move(0.0f, PRE_STAIR_SHIFT_VY, 0.0f);
                usleep(20 * 1000);  // 50Hz
            }
        }
        // ★ 九轮 (1729 没爬上就踉跄 = 带动量猛冲台阶、起爬姿态没站稳): 台阶前一律
        //   软急停 (零速流让步态把四脚踩方站稳) 再进 FreeWalk 爬; 右移开/关都执行
        softStop(sport, SOFTSTOP_SETTLE_SEC);
        addDogOnlyTiming("阶段3.5 台阶前停稳/对位", t_stage35);
    }

    // ----- no 保底: 跳过台阶, 后退/左移/前进/左转后进入弧形入口 -----
    if (ok && g_running && run_no_stairs_bypass) {
        auto t_no_stairs = timingNow();
        if (!runNoStairsSafetyBypass(sport, cap)) {
            if (g_running) {
                std::cout << "[警告] no 保底找线未完成, 继续尝试进入弧形" << std::endl;
            } else {
                ok = false;
            }
        }
        yaw_baseline_stair = g_yaw_rad.load();
        stair_baseline_from_avoid = false;
        std::cout << "[no] 弧形 yaw baseline 更新为当前 yaw = "
                  << rad2deg(yaw_baseline_stair) << "°" << std::endl;
        addDogOnlyTiming("阶段4 no保底跳过台阶找线", t_no_stairs);
    }

    // ----- 阶段 4: 爬台阶 (AI 步态) -----
    if (ok && g_running && run_stairs_stage) {
        auto t_stage4 = timingNow();
        // ★ 八轮: 完整流程不再在此重新快照 (会把阶段3被台阶带偏的朝向当基准);
        //   仅 startat 续跑等避障没跑过的情况才用当前朝向兜底。
        if (!stair_baseline_from_avoid) {
            yaw_baseline_stair = g_yaw_rad.load();
        }
        std::cout << "\n[弧形] yaw baseline = "
                  << rad2deg(yaw_baseline_stair) << "° ("
                  << (stair_baseline_from_avoid ? "避障结束快照" : "台阶前当前值")
                  << ")" << std::endl;
        std::cout << "\n############ 阶段 4/6: 爬台阶 (AI 步态, "
                  << stairClimbModeName(stair_climb_mode)
                  << ") ############" << std::endl;

        float stair_motion_yaw = yaw_baseline_stair;
        bool stair_entry_ready = true;
        if (stair_climb_mode == StairClimbMode::REVERSE) {
            // 当前正向停车点几乎贴台阶，先沿原赛道朝向后退约0.25m，
            // 给四足原地180°扫腿留空间；转完后这段平地距离已补进倒爬首段时长。
            std::cout << "[倒爬准备] 先退开转身空间: vx="
                      << ST_REVERSE_PRETURN_BACK_VX << " × "
                      << ST_REVERSE_PRETURN_BACK_SEC
                      << "s，yaw锁赛道基准 " << rad2deg(yaw_baseline_stair) << "°"
                      << std::endl;
            auto t_back = std::chrono::steady_clock::now();
            while (g_running) {
                double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - t_back).count() / 1000.0;
                if (el >= ST_REVERSE_PRETURN_BACK_SEC) break;
                float yerr = normalize_180(
                    rad2deg(yaw_baseline_stair) - g_yaw_deg.load());
                float omega = clampf(0.02f * yerr, -0.30f, 0.30f);
                sport.Move(ST_REVERSE_PRETURN_BACK_VX, 0.0f, omega);
                if (guiWaitKey(1) == 27) {
                    g_running = false;
                    break;
                }
                usleep(20 * 1000);
            }
            sport.Move(0.0f, 0.0f, 0.0f);
            if (g_running) softStop(sport, SOFTSTOP_SETTLE_SEC);

            if (g_running) {
                // 目标由“此刻实际yaw+180°”生成，避免±180°包角改变赛道基准；
                // turnToYawDeg 无论选择左/右旋，最终身体都会背对台阶。
                float reverse_target_yaw_deg = normalize_180(
                    g_yaw_deg.load() + 180.0f);
                std::cout << "[倒爬准备] 原地转180°: 当前 yaw="
                          << g_yaw_deg.load() << "° → target="
                          << reverse_target_yaw_deg << "°" << std::endl;
                if (!turnToYawDeg(sport, reverse_target_yaw_deg, 1.5f,
                                  /*stop_at_end=*/true)) {
                    std::cout << "[倒爬准备] 180°转身未到位，取消倒爬" << std::endl;
                    stair_entry_ready = false;
                } else {
                    stair_motion_yaw = g_yaw_rad.load();
                    std::cout << "[倒爬准备] 转身完成，倒爬身体yaw基准="
                              << rad2deg(stair_motion_yaw)
                              << "°；台阶后弧形仍保留赛道基准="
                              << rad2deg(yaw_baseline_stair) << "°" << std::endl;
                }
            } else {
                stair_entry_ready = false;
            }
        }

        bool stairs_completed = stair_entry_ready && g_running
            && runStairs(sport, turn_dir, stair_motion_yaw, stair_climb_mode);
        if (!stairs_completed) {
            std::cout << "[安全停止] 台阶未正常完成，保持当前步态零速并中止后续比赛阶段；"
                         "禁止在可能仍处于坡面时切步态或继续弧形"
                      << std::endl;
            sport.Move(0, 0, 0);
            hold_after_stair_failure = g_running;
            ok = false;
        }
        addDogOnlyTiming("阶段4 爬台阶", t_stage4);
    }

    // ★ 阶段4.5: 下台阶后先向机械臂发送 0, 原地等待 POST_STAIR_ARM0_WAIT_SEC 秒
    //   startat=arc 仍会执行 (方便单独测发 0)。
    //   串口打不开也安全跳过, 不阻塞后续流程。
    if (ok && g_running && run_arm0_pause) {
        auto t_arm0_pause = timingNow();
        if (g_dog_only_mode) {
            waitDogOnlyPlatform(sport, "阶段4.5 下台阶后停位");
        } else {
            std::cout << "\n[阶段4.5] 下台阶后向机械臂发送 0 (握手: 等 D0, 超时 "
                      << material_grab::HS_D0_TIMEOUT_SEC << "s)" << std::endl;
            softStop(sport, SOFTSTOP_SETTLE_SEC);
            // ★ 握手 v2: 等 D0 替代旧固定 10s (臂实际 ~9s; 收到即走, 超时兜底)
            material_grab::sendSimpleArmCommandWaitReply(
                0, "D0", material_grab::HS_D0_TIMEOUT_SEC);
        }
        addDogOnlyTiming(
            "阶段4.5 机械臂0停位",
            t_arm0_pause,
            POST_STAIR_ARM0_WAIT_SEC - DOG_ONLY_PLATFORM_WAIT_SEC,
            "(full_est按发送0后等待窗口估算)");
    }

    bool red_wait_already_done = false;

    // ----- 阶段 5: 弧形巡线 → 停在抓取平台侧边 -----
    //   退出条件: 全局 yaw 累计超过 302°、当前 yaw 回到初始 yaw 附近、且左 lidar < 0.7m。
    if (ok && g_running && run_arc_stage) {
        auto t_stage5 = timingNow();
        double stage5_full_extra = 0.0;
        std::cout << "\n############ 阶段 5/6: 弧形巡线 → 抓取平台停位 ############"
                  << std::endl;
        // ★ 关键: stop_on_exit=false 让 runArcToPlatform 在 PLATFORM_REACHED 时不 softStop,
        //   狗保持在 PD 巡线的最后一次 Move 状态, 然后紧接发左平移 Move (无缝衔接),
        //   避免 SDK 切回 BalanceStand 后 vy 被 deadband 吃掉 (跟最后右平移修复同套路)
        LineResult r5 = runArcToPlatform(sport, cap, yaw_baseline_stair,
                                         /*stop_on_exit=*/false);
        if (r5 == LineResult::PLATFORM_REACHED) {
            // ★ 八轮 (用户要求, 治"第一平台停不准"): 角度到位后不再带着弧线动量
            //   直接甩左移 —— 先 softStop 停一拍把动量清零、四脚踩方, 再左移。
            //   (★ 九轮二"视觉对正"已撤销: 此处线是弧线, 倾角参照不可比)
            float plat1_anchor_yaw = g_initial_yaw_ready.load()
                ? normalize_180(g_initial_yaw_deg.load() + g_plat1_trim_deg)
                : g_yaw_deg.load();
            std::cout << "[阶段5] 已到抓取平台侧 (角度到位) → softStop 停一拍再左移"
                      << std::endl;
            softStop(sport, SOFTSTOP_SETTLE_SEC);
            if (g_running) {
                std::cout << "[阶段5] 左平移 vy=" << PLATFORM_LSHIFT_VY
                          << " " << PLATFORM_LSHIFT_SEC << "s (先平移, yaw 锁当前朝向)"
                          << std::endl;
                float yaw_hold = g_yaw_deg.load();
                auto t_pl_shift = std::chrono::steady_clock::now();
                while (g_running) {
                    double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - t_pl_shift).count() / 1000.0;
                    if (el >= PLATFORM_LSHIFT_SEC) break;
                    float yerr  = normalize_180(yaw_hold - g_yaw_deg.load());
                    float omega = clampf(0.02f * yerr, -0.30f, 0.30f);
                    sport.Move(0.0f, PLATFORM_LSHIFT_VY, omega);
                    usleep(20 * 1000);
                }
                softStop(sport, SOFTSTOP_SETTLE_SEC);
                usleep(200 * 1000);
            }
            if (g_running) {
                float yaw_before = g_yaw_deg.load();
                std::cout << "[阶段5] 左平移后停位对准: 当前 yaw=" << yaw_before
                          << "° → 锚点(初始"
                          << (g_plat1_trim_deg >= 0.0f ? "+" : "")
                          << g_plat1_trim_deg << "°)=" << plat1_anchor_yaw
                          << "°" << std::endl;
                turnToYawDeg(sport, plat1_anchor_yaw, PLAT1_ALIGN_TOL_DEG,
                             /*stop_at_end=*/false);
                std::cout << "[阶段5] 左平移后停位对准完成: yaw=" << g_yaw_deg.load()
                          << "° (共转 "
                          << normalize_180(g_yaw_deg.load() - yaw_before)
                          << "°)" << std::endl;
            }
            if (g_running) {
                g_dual_arm_cmd = 0;
                std::cout << "[AUTOID] 识别抓取平台 1/2 号标志并选择放置指令 5/6"
                          << std::endl;
                int auto_cmd = autoid::recognizePlaceFromCamera(cap, 1.5);
                if (auto_cmd == 5 || auto_cmd == 6) {
                    std::cout << "[AUTOID] 放置平台识别成功: arm_cmd="
                              << auto_cmd << std::endl;
                    g_dual_arm_cmd = auto_cmd;
                } else {
                    std::cout << "[AUTOID] 放置平台标志未识别, 不发送 5/6 指令"
                              << std::endl;
                }
            }
            if (g_dog_only_mode) {
                waitDogOnlyPlatform(sport, "阶段5 抓取平台停位");
                stage5_full_extra = PLATFORM_WAIT_SEC - DOG_ONLY_PLATFORM_WAIT_SEC;
                std::cout << "[阶段5] dogonly 停位确认结束, 准备第一平台后右移" << std::endl;
            } else {
                // ★ 左移后: 运行机械臂抓取流程 (独立打开物资摄像头 /dev/video1, 窗口 PLATFORM_WAIT_SEC=80s)
                //   流程: 开串口 → 发"1"(就位) → 等就位 → 检测物料稳定 → 发"2,cx,cy" → 等回复。
                //   串口打不开 / 窗口内未锁定目标都安全跳过, 不阻塞后续。狗此期间保持站立不动。
                std::cout << "[阶段5] 平移完, 进入机械臂抓取流程 (窗口 "
                          << PLATFORM_WAIT_SEC << "s)" << std::endl;
                bool grabbed = material_grab::runMaterialGrab(sport, PLATFORM_WAIT_SEC);
                std::cout << "[阶段5] 抓取流程结束 (" << (grabbed ? "已发送抓取指令" : "未抓取")
                          << "), 准备第一平台后右移" << std::endl;
            }
            if (g_running) {
                std::cout << "[阶段5] 第一平台结束后向右平移 vy="
                          << PLATFORM_AFTER_GRAB_RSHIFT_VY << " "
                          << PLATFORM_AFTER_GRAB_RSHIFT_SEC
                          << "s (先平移, yaw 锁当前朝向)" << std::endl;
                float yaw_hold = g_yaw_deg.load();
                auto t_after_grab_shift = std::chrono::steady_clock::now();
                while (g_running) {
                    double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now()
                                    - t_after_grab_shift).count() / 1000.0;
                    if (el >= PLATFORM_AFTER_GRAB_RSHIFT_SEC) break;
                    float yerr = normalize_180(yaw_hold - g_yaw_deg.load());
                    float omega = clampf(0.02f * yerr, -0.30f, 0.30f);
                    sport.Move(0.0f, PLATFORM_AFTER_GRAB_RSHIFT_VY, omega);
                    usleep(20 * 1000);
                }
                softStop(sport, SOFTSTOP_SETTLE_SEC);
                usleep(200 * 1000);
                if (g_running) {
                    float yaw_before_realign = g_yaw_deg.load();
                    std::cout << "[阶段5] 右移后回正: 当前 yaw=" << yaw_before_realign
                              << "° → 平台一锚点=" << plat1_anchor_yaw
                              << "° (差值 "
                              << normalize_180(plat1_anchor_yaw - yaw_before_realign)
                              << "°)" << std::endl;
                    turnToYawDeg(sport, plat1_anchor_yaw, PLAT1_ALIGN_TOL_DEG,
                                 /*stop_at_end=*/false);
                }
            }
            std::cout << "[阶段5] 第一平台后右移完成, 进入阶段6" << std::endl;
        } else if (r5 == LineResult::ABORTED) {
            ok = false;
        } else if (r5 == LineResult::DETECT_TIMEOUT) {
            std::cout << "[阶段5] 弧形→平台检测超时, 跳过第一抓取, 直接进入阶段6"
                      << std::endl;
        } else {
            // 巡线返回非 PLATFORM_REACHED (理论不会发生, 防御性处理)
            ok = false;
        }
        addDogOnlyTiming(
            "阶段5 弧形到第一抓取平台",
            t_stage5,
            stage5_full_extra,
            stage5_full_extra > 0.0 ? "(full_est按第一抓取窗口估算)" : "");
    }

    // ----- 阶段 6: 继续巡线 → 检测 yaw 经历 +46/-31/+92 → 停在中转平台前 -----
    //   ★ 不是原地转, 是继续巡线, 让弧形线"自然带着"狗转;
    //     程序只检测 yaw 累计变化经过三相位, 第三相位达成时停。
    if (ok && g_running && run_tt_stage) {
        auto t_stage6 = timingNow();
        double stage6_full_extra = 0.0;
        bool stage6_platform_timed = false;
        std::cout << "\n############ 阶段 6/6: 三连转检测巡线 → 中转平台 ############"
                  << std::endl;
        float yaw_at_platform = g_yaw_rad.load();
        std::cout << "[阶段6] 平台停位 yaw baseline = "
                  << rad2deg(yaw_at_platform) << "°" << std::endl;
        LineResult r6 = runThreeTurnDetect(sport, cap, yaw_at_platform);
        if (r6 == LineResult::THREE_TURN_DONE) {
            if (g_dog_only_mode) {
                waitDogOnlyPlatform(sport, "阶段6 中转平台停位");
                stage6_full_extra =
                    material_grab::SECOND_GRAB_WAIT_SEC
                    - DOG_ONLY_PLATFORM_WAIT_SEC;
                std::cout << "[阶段6] dogonly 停位确认结束, 继续后续流程" << std::endl;
            } else {
                std::cout << "[阶段6] 三连转检测完成, 已停在中转平台前, 进入第二抓取流程 (窗口 "
                          << material_grab::SECOND_GRAB_WAIT_SEC << "s)" << std::endl;
                bool second_grabbed = material_grab::runSecondMaterialGrab(
                    sport, material_grab::SECOND_GRAB_WAIT_SEC);
                std::cout << "[阶段6] 第二抓取流程结束 ("
                          << (second_grabbed ? "已发送第二抓取坐标" : "未发送第二抓取坐标")
                          << "), 直接继续 (★ 2026-07-06 删除原 5s 停留)" << std::endl;
            }
            addDogOnlyTiming(
                "阶段6A 三连转到中转平台/第二抓取停位",
                t_stage6,
                stage6_full_extra,
                stage6_full_extra > 0.0 ? "(full_est按第二抓取窗口估算)" : "");
            stage6_platform_timed = true;

            // ★ 第二抓取结束 → 向左平移 POST_TT_SHIFT_SEC 秒 (常规步态)
            auto t_stage6_post = timingNow();
            if (g_running) {
                std::cout << "[阶段6] 向左平移 vy=" << POST_TT_SHIFT_VY
                          << " " << POST_TT_SHIFT_SEC << "s (常规步态)" << std::endl;
                sport.StaticWalk();
                usleep(300 * 1000);
                auto t_lshift = std::chrono::steady_clock::now();
                while (g_running) {
                    double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - t_lshift).count() / 1000.0;
                    if (el >= POST_TT_SHIFT_SEC) break;
                    sport.Move(0.0f, POST_TT_SHIFT_VY, 0.0f);
                    usleep(20 * 1000);
                }
                softStop(sport, SOFTSTOP_SETTLE_SEC);
            }

            // ★ 平移后原地左转 POST_TT_TURN_DEG° (严格)
            if (g_running) {
                std::cout << "[阶段6] 平移后原地左转 " << POST_TT_TURN_DEG
                          << "° (容差±" << POST_TT_TURN_TOL << "°)" << std::endl;
                turnInPlace(sport, +POST_TT_TURN_DEG, POST_TT_TURN_TOL,
                            /*stop_at_end=*/false);
                softStop(sport, SOFTSTOP_SETTLE_SEC);
            }
            // 左移+左转后补前进: 仅在第二抓取曾触发无识别补救时执行。
            if (g_running && material_grab::g_second_grab_nudged) {
                std::cout << "[阶段6] 左转后向前 vx=" << POST_TT_FWD_VX
                          << " " << POST_TT_FWD_SEC << "s (yaw 保持)" << std::endl;
                sport.StaticWalk();
                usleep(300 * 1000);
                float fwd_yaw = g_yaw_deg.load();
                auto t_fwd = std::chrono::steady_clock::now();
                while (g_running) {
                    double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - t_fwd).count() / 1000.0;
                    if (el >= POST_TT_FWD_SEC) break;
                    float yerr = normalize_180(fwd_yaw - g_yaw_deg.load());
                    float omega = clampf(0.02f * yerr, -0.30f, 0.30f);
                    sport.Move(POST_TT_FWD_VX, 0.0f, omega);
                    usleep(20 * 1000);
                }
                softStop(sport, SOFTSTOP_SETTLE_SEC);
            } else if (g_running) {
                std::cout << "[阶段6] 第二抓取未触发无识别补救, 跳过左转后的前进" << std::endl;
            }
            addDogOnlyTiming("阶段6B 中转后左移+转向", t_stage6_post);

            // ★ 恢复巡线 (FOREVER) → 期间检测红圆, 命中后走红圆收尾序列
            if (g_running) {
                auto t_red_wait = timingNow();
                std::cout << "[阶段6] 恢复巡线 (FOREVER), 等待红圆触发..." << std::endl;
                runLineFollowing(sport, cap, LineMode::FOREVER, 0.0);
                red_wait_already_done = true;
                addDogOnlyTiming("红圆等待巡线", t_red_wait);
            }
        } else if (r6 == LineResult::ABORTED) {
            ok = false;
        } else if (r6 == LineResult::DETECT_TIMEOUT) {
            std::cout << "[阶段6] 三连转检测超时, 跳过第二抓取/左移/转向, 直接进入红圆等待"
                      << std::endl;
        }
        if (!stage6_platform_timed) {
            addDogOnlyTiming("阶段6 三连转检测", t_stage6);
        }
    }

    if (ok && g_running && run_red_wait && !g_warn_action_done && !red_wait_already_done) {
        auto t_red_wait = timingNow();
        std::cout << "\n############ 红圆等待巡线 (FOREVER) ############" << std::endl;
        runLineFollowing(sport, cap, LineMode::FOREVER, 0.0);
        red_wait_already_done = true;
        addDogOnlyTiming("红圆等待巡线", t_red_wait);
    }

    // ----- 红圆收尾序列: 双侧检测→停5s→巡线6s→前跳→转到初始 yaw→右移→相机居中→结束 -----
    //   仅当红圆警示动作已执行 (runLineFollowing 因此返回) 才走这一段
    if (g_warn_action_done && g_running) {
        auto t_close = timingNow();
        double close_full_extra = 0.0;
        std::cout << "\n############ 红圆收尾序列 ############" << std::endl;

        // ★ 红圆动作完成 → 先强制巡线 DUAL_PLAT_MIN_LINE_SEC 秒 (防双侧立刻误触发)
        if (g_running) {
            std::cout << "[收尾] 红圆后强制巡线 " << DUAL_PLAT_MIN_LINE_SEC
                      << "s 再允许触发双侧" << std::endl;
            runLineFollowing(sport, cap, LineMode::TIMED, DUAL_PLAT_MIN_LINE_SEC);
        }

        // ★ 然后才允许双侧 lidar 触发, 停 DUAL_PLAT_WAIT_SEC 秒
        if (g_running) {
            std::cout << "[收尾] 双侧放置平台检测 (left/right lidar < "
                      << DUAL_PLAT_LIDAR_THRESH_M << "m)..." << std::endl;
            LineResult rdual = runUntilDualPlatform(sport, cap);
            if (rdual == LineResult::DUAL_PLATFORM_REACHED
                || rdual == LineResult::DETECT_TIMEOUT) {
                if (rdual == LineResult::DETECT_TIMEOUT) {
                    std::cout << "[收尾] 双侧平台检测超时, 按已触发降级继续收尾"
                              << std::endl;
                }
                // ★ 触发后 1s 盲走 (直行, 不依赖视觉, 逻辑同避障盲走), 再停车
                std::cout << "[收尾] 双侧触发后盲走 " << DUAL_PLAT_POST_BLIND_SEC
                          << "s (vx=" << POST_DUAL_FWD_VX << ")" << std::endl;
                runBlindForward(sport, DUAL_PLAT_POST_BLIND_SEC, POST_DUAL_FWD_VX);
                // 放置动作前软急停, 避免硬停切回不期望的站立状态
                softStop(sport, SOFTSTOP_SETTLE_SEC);
                if (g_dog_only_mode) {
                    waitDogOnlyPlatform(sport, "收尾 双侧平台停位");
                    double full_dual_wait =
                        (g_dual_arm_cmd == 5 || g_dual_arm_cmd == 6)
                            ? DUAL_ARM_AFTER_SEND_WAIT_SEC
                            : DUAL_PLAT_WAIT_SEC;
                    close_full_extra += full_dual_wait - DOG_ONLY_PLATFORM_WAIT_SEC;
                } else {
                    if (g_dual_arm_cmd == 5 || g_dual_arm_cmd == 6) {
                        // ★ 握手 v2: 等 D5/D6 完成确认, 超时按旧 25s 兜底
                        const char* expect = (g_dual_arm_cmd == 5) ? "D5" : "D6";
                        std::cout << "[收尾] 双侧检测到放置平台, 发送机械臂放置指令 "
                                  << g_dual_arm_cmd << " (握手: 等 " << expect
                                  << ", 超时 " << DUAL_ARM_AFTER_SEND_WAIT_SEC
                                  << "s)" << std::endl;
                        material_grab::sendSimpleArmCommandWaitReply(
                            g_dual_arm_cmd, expect, DUAL_ARM_AFTER_SEND_WAIT_SEC);
                    } else {
                        std::cout << "[收尾] 双侧检测到放置平台, 未获得有效 5/6 识别结果, 停留 "
                                  << DUAL_PLAT_WAIT_SEC << "s" << std::endl;
                        auto t_dwait = std::chrono::steady_clock::now();
                        while (g_running) {
                            double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                                            std::chrono::steady_clock::now() - t_dwait).count() / 1000.0;
                            if (el >= DUAL_PLAT_WAIT_SEC) break;
                            usleep(50 * 1000);
                        }
                    }
                }
            } else if (rdual == LineResult::ABORTED) {
                ok = false;
            }
        }

        // 巡线找终点横杆 (窗口兜底=原 FINAL_LINE_SEC 计时的距离等效) → 前跳
        if (g_running) {
            // ★ 九轮 (用户: 收尾前跳跳太晚): 收尾前跳触发行上移, 更早起跳
            g_ob_trigger_row_frac = OB_TRIGGER_ROW_FRAC_FINAL;
            std::cout << "[收尾] 终点前跳触发行上移: " << g_ob_trigger_row_frac
                      << "H (起步 " << OB_TRIGGER_ROW_FRAC << "H)" << std::endl;
            double ob2_dist = FINAL_LINE_SEC * FINAL_PREJUMP_LINE_SPEED;   // ≈1.0 m
            double ob2_nom  = obNominalSecForDist(ob2_dist);
            double ob2_min  = std::max(OB_APPROACH_KICK_SEC, ob2_nom - OB_WIN_HALF_SEC);
            double ob2_max  = g_ob_detect_enabled ? (ob2_nom + OB_WIN_HALF_SEC) : ob2_nom;
            runLineFollowing(sport, cap, LineMode::TO_OBSTACLE, ob2_max,
                             0.0f, true, OB_APPROACH_KICK_SPEED,
                             OB_APPROACH_KICK_SEC, OB_APPROACH_SLOW_SPEED,
                             1.0, 0.0, ob2_min);
        }
        if (g_running) doFrontJump(sport, (float)OB_APPROACH_SLOW_SPEED, /*quick=*/true);
        // ★ 2026-07-06 六轮 (本批 2/5 两次收尾前跳踉跄→人工干预): 收尾前跳一直没有
        //   阶段1那样的"落地站稳+姿态看门狗"待遇, 跳完直接转身 —— 落地踉跄没人管。
        //   补上同款守护: 站稳 POST_JUMP_SETTLE_SEC, 姿态坏了自动 BalanceStand/
        //   RecoveryStand 自救, 然后才允许左转。
        if (g_running) {
            std::cout << "[收尾] 前跳落地, 原地站稳 " << POST_JUMP_SETTLE_SEC
                      << "s (带落地姿态看门狗), 再左转回初始 yaw..." << std::endl;
            postJumpAttitudeGuard(sport, POST_JUMP_SETTLE_SEC, 0.0);
        }

        float final_base_yaw_deg = g_initial_yaw_ready.load()
            ? g_initial_yaw_deg.load()
            : g_yaw_deg.load();
        float final_target_yaw_deg = normalize_180(
            final_base_yaw_deg + FINAL_YAW_EXTRA_LEFT_DEG);

        // 转到程序启动时记录的初始 yaw → ★ stop_at_end=false, 末尾不 softStop
        //   让狗保持步态状态, 紧接着进入相机闭环找线/居中, 避免 SDK 切 BalanceStand
        if (g_running) {
            std::cout << "[收尾] 常规步态转到初始 yaw+" << FINAL_YAW_EXTRA_LEFT_DEG
                      << "°: target=" << final_target_yaw_deg
                      << "° (严格容差 " << FINAL_TURN_TOL
                      << "°, 末尾不 softStop)" << std::endl;
            turnToYawDeg(sport, final_target_yaw_deg, FINAL_TURN_TOL,
                         /*stop_at_end=*/false);
        }

        // ★ 九轮改四 (用户: seek/条件式预平移越改越乱, 回归最原始版本):
        //   跳完 → 左转 → 【固定右移 FINAL_PRE_RSHIFT_FIXED_SEC 秒 @ 0.15】→ 三段对齐。
        //   不再判线/判蓝, 就是老老实实右移一段, 把线移进视野, 剩下交给三段对齐。
        if (g_running) {
            std::cout << "[收尾] 固定右移 " << FINAL_PRE_RSHIFT_FIXED_SEC
                      << "s @ vy=" << FINAL_PRE_RSHIFT_FIXED_VY
                      << " (yaw 锁初始), 再进三段对齐" << std::endl;
            auto t_rs = std::chrono::steady_clock::now();
            while (g_running) {
                double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - t_rs).count() / 1000.0;
                if (el >= FINAL_PRE_RSHIFT_FIXED_SEC) break;
                float yaw_err = normalize_180(final_target_yaw_deg - g_yaw_deg.load());
                float omega = clampf(FINAL_YAW_HOLD_KP * yaw_err,
                                     -FINAL_YAW_HOLD_OMEGA_MAX, FINAL_YAW_HOLD_OMEGA_MAX);
                sport.Move(0.0f, FINAL_PRE_RSHIFT_FIXED_VY, omega);
                usleep(20 * 1000);
            }
            sport.Move(0.0f, 0.0f, 0.0f);
        }

        // 最后用相机闭环: 没看到线就持续右移找线,看到线后微调到画面中央并停车。
        if (g_running) {
            alignFinalLineCenter(sport, cap, final_target_yaw_deg);
        }

        std::cout << "[收尾] 收尾序列完成, 结束程序" << std::endl;
        addDogOnlyTiming(
            "收尾 红圆后双侧+最终停车",
            t_close,
            close_full_extra,
            close_full_extra > 0.0 ? "(full_est按双侧平台机械臂等待估算)" : "");
    }

    // ---------- 收尾 ----------
    if (hold_after_stair_failure && g_running) {
        std::cout << "\n[人工接管] 狗可能仍在台阶坡面，持续下发 FreeWalk 零速；"
                     "扶稳后按 Ctrl+C/ESC 再退出程序"
                  << std::endl;
        while (g_running) {
            sport.Move(0.0f, 0.0f, 0.0f);
            int key = guiWaitKey(1);
            if (key == 27) g_running = false;
            usleep(20 * 1000);
        }
    }
    std::cout << "\n[收尾] 停止运动..." << std::endl;
    sport.Move(0, 0, 0);
    usleep(300000);
    sport.StopMove();
    usleep(500000);
    g_running = false;
    stop_front_laser_thread();
    material_grab::armPortCloseAtExit();
    cap.release();
    if (g_gui_enabled) cv::destroyAllWindows();
    printDogOnlyTimingSummary();
    std::cout << "[完成] 整套流程结束" << std::endl;
    return 0;
}

// ============================================================
//  编译示例 (容器内无 OpenCV C++ 开发库,未做编译验证,
//            按你 line.cpp 的实际 build 环境调整路径):
//
//  g++ -O2 -std=c++17 raicom_run.cpp -o raicom_run \
//      -I/path/to/unitree_sdk2/include \
//      -L/path/to/unitree_sdk2/lib -lunitree_sdk2 -lddsc -lddscxx \
//      $(pkg-config --cflags --libs opencv4) -lpthread
//
//  —— 最稳的做法:直接套用你编 line.cpp 的那套 CMakeLists / 命令,
//     把源文件换成 raicom_run.cpp 即可 (line.cpp 本来就同时链
//     OpenCV + unitree_sdk2,依赖完全一致)。
// ============================================================
