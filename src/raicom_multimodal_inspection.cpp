
// RAICOM 2026 Multimodal Inspection System
//
// High-level onboard coordinator for a Unitree Go2 competition system. The
// program integrates visual line following, obstacle avoidance, stair
// traversal, platform alignment, command recognition, and serial coordination
// with an external manipulator into one hardware-oriented mission controller.
//
// Scope of this release:
//   - This file contains the Go2-side perception and mission-control program.
//   - Unitree SDK2 and OpenCV are external dependencies and are not vendored.
//   - Manipulator firmware, vendor examples, private calibration files, and
//     device-specific credentials are intentionally not included.
//
// Physical-robot software can cause injury or equipment damage. Review
// docs/SAFETY.md, test each stage independently, and keep an emergency stop
// available before running the integrated mission.

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
#include <string>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>

std::atomic<bool> g_running{true};

std::atomic<bool>  g_state_received{false};
std::atomic<float> g_pos_x{0.0f}, g_pos_y{0.0f};
std::atomic<float> g_roll{0.0f}, g_pitch{0.0f};
std::atomic<float> g_yaw_rad{0.0f};
std::atomic<float> g_yaw_deg{0.0f};

std::atomic<bool>  g_initial_yaw_ready{false};
std::atomic<float> g_initial_yaw_rad{0.0f};
std::atomic<float> g_initial_yaw_deg{0.0f};
std::atomic<float> g_global_yaw_prev_rad{0.0f};
std::atomic<float> g_global_yaw_cum_rad{0.0f};

std::atomic<bool> g_lowstate_received{false};
std::atomic<int>  g_foot_force[4];

std::atomic<bool>  g_lidar_received{false};
std::atomic<float> g_front_dist{5.0f};
std::atomic<float> g_left_dist{5.0f};
std::atomic<float> g_right_dist{5.0f};

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

bool g_gui_enabled = true;
inline int guiWaitKey(int delay_ms = 1) {
    if (!g_gui_enabled) return -1;
    return cv::waitKey(delay_ms);
}

bool openCameraByIdOrIndex(cv::VideoCapture& cap, const std::string& id, const char* tag) {
    bool is_idx = !id.empty()
                  && id.find_first_not_of("0123456789") == std::string::npos;

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

inline float normalize_180(float a) {
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

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

    float w = s->imu_state().quaternion()[0];
    float x = s->imu_state().quaternion()[1];
    float y = s->imu_state().quaternion()[2];
    float z = s->imu_state().quaternion()[3];
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    g_yaw_deg = std::atan2(siny_cosp, cosy_cosp) * 180.0f / 3.14159265f;

    g_state_received = true;
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

        static long last_warn_ms = 0;
        long now = nowMs();
        if (now - last_warn_ms > 2000) {
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

std::string g_line_cam_id = "0";
const std::string GO2_CAM_PIPELINE =
    "udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 "
    "! application/x-rtp, media=video, encoding-name=H264 "
    "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert "
    "! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1";
const bool USE_USB_CAMERA = true;

const bool CAM_LOCK_EXPOSURE = true;
int  g_cam_exposure_val      = 156;
const bool CAM_LOCK_WB       = false;

const int  CAM_WB_TEMP       = 4600;
const bool CAM_LOCK_GAIN     = false;
const int  CAM_GAIN_VAL      = 0;

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

const double MAX_FRAME_CONTOUR_AREA_RATIO = 0.90;
const double MAX_ROI_CONTOUR_AREA_RATIO   = 0.90;

const double CROSS_BAR_MIN_W_RATIO = 0.55;
const double CROSS_BAR_SIDE_RATIO  = 0.20;

int g_max_brightness    = LAB_MAX_BRIGHTNESS;
int g_real_line_min     = LAB_REAL_LINE_MIN;
int g_use_otsu          = LAB_USE_OTSU;
int g_min_area          = LAB_MIN_AREA;
int g_blur_size         = LAB_BLUR_SIZE;
int g_connect_threshold = LAB_CONNECT_THRESHOLD;

int g_use_adaptive  = 1;
int g_min_contrast  = 50;
int g_contrast_frac = 60;

int g_dbg_minL = 0, g_dbg_floorL = 0, g_dbg_contrast = 0, g_dbg_thr = 0;

int g_kp_x1000 = 2;
int g_kd_x1000 = 3;

const double MAX_SPEED         = 0.20;
const double MIN_SPEED_RATIO   = 0.30;
const double ROTATION_LIMIT    = 0.5;
const double SPEED_DECAY_PIVOT = 200.0;

const double TT_PHASE2_SPEED_MULT = 0.5;

const double ROI_NEAR_TOP    = 0.45;
const double ROI_NEAR_BOTTOM = 0.7;
const double ROI_FAR_TOP     = 0.20;
const double ROI_FAR_BOTTOM  = 0.50;

const int    LOST_SOFT_THRESHOLD = 8;
const int    LOST_HARD_THRESHOLD = 30;
const double SEARCH_YAW          = 0.3;

const float  LOST_BLIND_VX       = 0.15f;

const double STAGE1_LINE_KICK_TIME_SEC = 0.5;
const double STAGE1_LINE_KICK_SPEED    = 0.20;
const double STAGE1_LINE_SLOW_TIME_SEC = 6.0;
const double STAGE1_LINE_SLOW_SPEED    = 0.05;
const double STAGE1_LINE_ROT_MULT      = 4.0;

const double STAGE1_ROT_RAMP_SEC       = 1.5;
const double STAGE1_BLIND_TIME_SEC = 2.0;
const double BLIND_FORWARD_SPEED   = 0.18;

const double START_LINE_SEC     = 2.0;
const double START_PREJUMP_LINE_SPEED = 0.20;
const int    FRONTJUMP_WAIT_SEC = 3;

const double PREJUMP_DECEL_SEC     = 0.6;
const double PREJUMP_MIN_STAND_SEC = 1.0;
const double PREJUMP_MAX_STAND_SEC = 2.5;
const float  PREJUMP_ATT_TOL_DEG   = 2.5f;

const double PREJUMP_QUICK_DECEL_SEC     = 0.2;
const double PREJUMP_QUICK_MIN_STAND_SEC = 0.3;
const double PREJUMP_QUICK_MAX_STAND_SEC = 0.8;
const double POST_JUMP_SETTLE_SEC = 1.5;

const double POST_JUMP_GUARD_START_SEC    = 1.5;
const float  POST_JUMP_GUARD_PITCH_DEG    = 12.0f;
const float  POST_JUMP_GUARD_ROLL_DEG     = 8.0f;
const double POST_JUMP_GUARD_HOLD_SEC     = 0.3;
const double POST_JUMP_GUARD_ESCALATE_SEC = 0.8;
const double POST_JUMP_GUARD_RECOVERY_WAIT_SEC = 2.5;

bool         g_ob_detect_enabled  = true;

const double OB_APPROACH_KICK_SPEED = 0.20;
const double OB_APPROACH_KICK_SEC   = 0.5;
const double OB_APPROACH_SLOW_SPEED = 0.10;

const double OB_WIN_HALF_DIST     = 0.40;
const double OB_WIN_HALF_SEC      = OB_WIN_HALF_DIST / OB_APPROACH_SLOW_SPEED;
const int    OB_STRIP_HALF_W_PX   = 70;

const double OB_SCAN_TOP_FRAC     = 0.02;
const double OB_SCAN_BOTTOM_FRAC  = 0.97;
const int    OB_ROW_LINE_MIN_PX   = 25;
const int    OB_GAP_MIN_PX        = 12;
const int    OB_GAP_MAX_PX        = 260;
const int    OB_SEG_MIN_PX        = 10;
const int    OB_GAP_BRIGHT_MARGIN = 15;

const double OB_TRIGGER_ROW_FRAC  = 0.40;

const int    OB_STABLE_FRAMES     = 3;

inline double obNominalSecForDist(double dist_m) {
    double kick_dist = OB_APPROACH_KICK_SPEED * OB_APPROACH_KICK_SEC;
    if (dist_m <= kick_dist) return dist_m / OB_APPROACH_KICK_SPEED;
    return OB_APPROACH_KICK_SEC + (dist_m - kick_dist) / OB_APPROACH_SLOW_SPEED;
}
const double FINAL_LINE_SEC     = 5.0;
const double FINAL_PREJUMP_LINE_SPEED = 0.20;
const float  FINAL_TURN_TOL     = 1.0f;
const float  FINAL_YAW_EXTRA_LEFT_DEG = 0.0f;

const float  FINAL_SHIFT_VY     = -0.10f;
const int    FINAL_CENTER_TOL_PX = 40;
const int    FINAL_CENTER_FLUSH_FRAMES = 5;
const int    FINAL_CENTER_STABLE_FRAMES = 3;
const int    FINAL_CENTER_MIN_AREA = 600;
const int    FINAL_CENTER_LOST_HOLD_FRAMES = 15;

const float  FINAL_CENTER_VY_KP    = 0.0015f;
const float  FINAL_CENTER_VY_FLOOR = 0.10f;
const float  FINAL_CENTER_VY_MAX   = 0.18f;
const double FINAL_CENTER_TIMEOUT_SEC = 15.0;

const float  FINAL_YAW_HOLD_KP   = 0.02f;
const float  FINAL_YAW_HOLD_OMEGA_MAX = 0.30f;

const float  FINAL_PRE_RSHIFT_VY  = -0.20f;

const double FINAL_PRE_RSHIFT_MAX_SEC = 6.0;
const int    FINAL_PRE_RSHIFT_LOCK_FRAMES = 3;

const float  FINAL_FB_SEARCH_FWD_VX  = 0.15f;
const double FINAL_FB_SEARCH_FWD_SEC = 0.5;

bool         g_fb_align_enabled = true;
const int    FINAL_FB_H_MIN = 90;
const int    FINAL_FB_H_MAX = 135;
const int    FINAL_FB_S_MIN = 50;
const int    FINAL_FB_V_MIN = 40;
const double FINAL_FB_ROW_BLUE_FRAC = 0.40;
const int    FINAL_FB_MIN_BLUE_ROWS = 25;
const float  FINAL_FB_KP      = 0.0012f;
const float  FINAL_FB_VX_MAX  = 0.08f;
const float  FINAL_FB_VX_FLOOR = 0.04f;
const int    FINAL_FB_TOL_ROWS = 18;
const double FINAL_FB_NOBLUE_GIVEUP_SEC = 3.0;
const double FINAL_FB_TARGET_ROW_FRAC_DEFAULT = 0.55;
int          g_final_fb_target_row = -1;

const float  FINAL_FB_KICK_VX      = 0.20f;
const double FINAL_FB_KICK_SEC     = 0.35;
const double FINAL_FB_STALL_SEC    = 0.6;
const float  FINAL_FB_STALL_MOVE_M = 0.01f;

int          g_final_lat_target_x = -1;
const int    FINAL_PRE_RSHIFT_STOP_MARGIN_PX = 40;

const float  FINAL_YAW_VIS_KP          = 0.02f;
const float  FINAL_YAW_VIS_OMEGA_FLOOR = 0.05f;
const float  FINAL_YAW_VIS_TOL_DEG     = 1.2f;
const float  FINAL_YAW_VIS_TILT_MAX    = 25.0f;
const double FINAL_PHASE_LAT_MAX_SEC   = 6.0;
const double FINAL_PHASE_YAW_MAX_SEC   = 4.0;

const float  PRE_STAIR_SHIFT_VY  = 0.0f;
const double PRE_STAIR_SHIFT_SEC = 0.0;

const float  POST_STAIR_SHIFT_VY  = -0.20f;
const double POST_STAIR_SHIFT_SEC = 2.0;
const double ARC_START_LINE_CHECK_SEC = 0.0;

const double ARC_START_SHIFT_MAX_SEC    = 6.0;
const int    ARC_START_LINE_LOCK_FRAMES = 3;
const double POST_STAIR_LINE_REOPEN_WAIT_SEC = 1.0;
const double POST_STAIR_LINE_FLUSH_SEC = 2.0;
const int    POST_STAIR_LINE_CHECK_FRAMES = 8;

const double POST_STAIR_ARM0_WAIT_SEC = 10.0;

const double PLATFORM_WAIT_SEC      = 70.0;

const float  PLATFORM_LSHIFT_VY  = +0.14f;
const double PLATFORM_LSHIFT_SEC = 3.0;
const float  PLATFORM_AFTER_GRAB_RSHIFT_VY  = -0.20f;
const double PLATFORM_AFTER_GRAB_RSHIFT_SEC = 2.0;

const float POST_PLAT_PHASE0_THRESH_DEG = 45.0f;
const float POST_PLAT_PHASE1_THRESH_DEG = 30.0f;
const float POST_PLAT_FINAL_THRESH_DEG = 91.0f;
const int   POST_PLAT_STABLE_FRAMES    = 1;

const float  POST_TT_SHIFT_VY  = +0.20f;
const double POST_TT_SHIFT_SEC = 3.0;
const float  POST_TT_TURN_DEG  = 60.0f;
const float  POST_TT_TURN_TOL  = 1.0f;

const float  DUAL_PLAT_LIDAR_THRESH_M = 0.40f;

const double DUAL_PLAT_MIN_LINE_SEC   = 6.0;

const float  POST_DUAL_FWD_VX  = 0.15f;
const double POST_DUAL_FWD_SEC = 1.0;
const int    DUAL_PLAT_STABLE_FRAMES  = 5;
const double DUAL_PLAT_WAIT_SEC       = 5.0;
const double DUAL_PLAT_POST_BLIND_SEC = 1.0;
const double DUAL_ARM_AFTER_SEND_WAIT_SEC = 25.0;
const double DOG_ONLY_PLATFORM_WAIT_SEC = 5.0;

const float INITIAL_YAW_CUM_TRIGGER_DEG = 300.0f;
const float INITIAL_YAW_MATCH_TOL_DEG   = 1.0f;

const float ARC_YAW_TARGET_DELTA_DEG = INITIAL_YAW_CUM_TRIGGER_DEG;
const float ARC_YAW_TOL_DEG          = INITIAL_YAW_MATCH_TOL_DEG;
const float ARC_LIDAR_LEFT_THRESH_M  = 0.70f;
const int   ARC_STABLE_FRAMES        = 1;

const int LINE_LOST_TRIGGER_CONSEC = 1;
bool g_line_lost_avoidance_triggered = false;

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

    bool use_manual_exposure =
        (g_vision_profile == VisionProfile::SUN) && CAM_LOCK_EXPOSURE;
    if (use_manual_exposure) {
        std::cout << "[摄像头] sun 模式: 尝试锁定曝光 target="
                  << g_cam_exposure_val << std::endl;

        bool ok_auto = cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        usleep(100 * 1000);

        bool ok_exp  = cap.set(cv::CAP_PROP_EXPOSURE, g_cam_exposure_val);
        usleep(100 * 1000);
        std::cout << "  set AUTO_EXPOSURE=1 ret=" << ok_auto
                  << "  set EXPOSURE=" << g_cam_exposure_val << " ret=" << ok_exp
                  << std::endl;
        std::cout << "  回读 AUTO_EXPOSURE=" << cap.get(cv::CAP_PROP_AUTO_EXPOSURE)
                  << "  EXPOSURE=" << cap.get(cv::CAP_PROP_EXPOSURE) << std::endl;

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

    {
        cv::Mat tmp;
        for (int i = 0; i < 5; ++i) cap >> tmp;
    }
    return true;
}

const int CAM_EMPTY_FRAMES_REOPEN = 30;

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

    int  thr_used = 0;
    bool no_line  = false;
    if (g_use_adaptive) {

        int y0 = (int)(blurred.rows * ROI_FAR_TOP);
        int y1 = (int)(blurred.rows * ROI_NEAR_BOTTOM);
        y0 = std::max(0, std::min(y0, blurred.rows - 2));
        y1 = std::max(y0 + 1, std::min(y1, blurred.rows));
        cv::Mat band = blurred.rowRange(y0, y1);

        double minL_d, maxL_d;
        cv::minMaxLoc(band, &minL_d, &maxL_d);
        int minL = (int)minL_d;

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

            no_line = true; g_dbg_thr = -1;
        } else {
            int thr = minL + g_contrast_frac * contrast / 100;
            int lo  = minL + 15;
            int hi  = floorL - 20;
            if (hi < lo) { no_line = true; g_dbg_thr = -1; }
            else {
                thr_used = std::max(lo, std::min(thr, hi));
                g_dbg_thr = thr_used;
            }
        }
    } else {

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

    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, morph_kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, morph_kernel);

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

    cv::Rect bb = cv::boundingRect(contours[best->idx]);
    int cx_local  = best->cx - roi.x;
    int left_ext  = cx_local - bb.x;
    int right_ext = (bb.x + bb.width) - cx_local;
    bool wide_bar = (bb.width  > CROSS_BAR_MIN_W_RATIO * full_mask.cols)
                 && (left_ext  > CROSS_BAR_SIDE_RATIO  * full_mask.cols)
                 && (right_ext > CROSS_BAR_SIDE_RATIO  * full_mask.cols);
    if (is_wide_bar) *is_wide_bar = wide_bar;

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

int          g_red_s_min     = 80;
int          g_red_v_min     = 50;
const double RED_MIN_AREA    = 800;
const double RED_CIRCULARITY = 0.50;
const int    RED_SEEN_CONFIRM_FRAMES = 3;
const int    RED_LOST_TRIGGER_FRAMES = 5;

const float  RED_MIN_Y_FRAC  = 0.50f;

double       g_red_timeout_sec = 40.0;

const double ARC_WATCHDOG_SEC  = 180.0;
const double TT_WATCHDOG_SEC   = 120.0;
const double DUAL_WATCHDOG_SEC = 60.0;
const int    WARN_ACT_WAIT_SEC = 5;

const double WARN_ACT_MIN_STAND_SEC = 1.0;
const double WARN_ACT_MAX_STAND_SEC = 3.0;
const float  POST_RED_ACTION_LSHIFT_VY  = +0.20f;
const double POST_RED_ACTION_LSHIFT_SEC = 1.0;

const double RED_STOP_SEC     = 1.0;
const float  RED_TURN_DEG     = 90.0f;
const float  RED_TURN_OMEGA   = 0.6f;
const float  RED_TURN_TOL     = 1.0f;
const float  RED_TURN_MAX_SEC = 8.0f;
const float  RED_TURN_KP      = 0.04f;
const float  RED_TURN_OMEGA_MIN = 0.12f;

int  g_action_id        = 0;
int  g_dual_arm_cmd     = 0;
bool g_dog_only_mode    = false;
bool g_warn_action_done = false;
unitree::robot::go2::VuiClient* g_vui = nullptr;

bool detectRedCircle(cv::Mat& frame) {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat m1, m2, mask;
    cv::inRange(hsv, cv::Scalar(0,   g_red_s_min, g_red_v_min),
                     cv::Scalar(10,  255, 255), m1);
    cv::inRange(hsv, cv::Scalar(170, g_red_s_min, g_red_v_min),
                     cv::Scalar(180, 255, 255), m2);
    mask = m1 | m2;
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    if (g_gui_enabled) cv::imshow("Red Mask", mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < RED_MIN_AREA) continue;
        double peri = cv::arcLength(c, true);
        if (peri <= 1.0) continue;
        double circ = 4.0 * 3.14159265 * area / (peri * peri);
        if (circ >= RED_CIRCULARITY) {
            cv::Point2f ctr; float r;
            cv::minEnclosingCircle(c, ctr, r);

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
        action = 3;
    } else if (bottom_ratio > 0.16 && black_ratio > 0.08) {
        action = 2;
    } else if (largest_area > 0) {
        action = 1;
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

    int cmd = (diff > 0) ? 5 : 6;
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

}

void turnInPlace(unitree::robot::go2::SportClient& sport,
                 float delta_deg, float tol_deg = RED_TURN_TOL,
                 bool stop_at_end = true) {

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

        float omega = clampf(err * RED_TURN_KP, -RED_TURN_OMEGA, RED_TURN_OMEGA);
        if (omega > 0 && omega <  RED_TURN_OMEGA_MIN) omega =  RED_TURN_OMEGA_MIN;
        if (omega < 0 && omega > -RED_TURN_OMEGA_MIN) omega = -RED_TURN_OMEGA_MIN;
        sport.Move(0.0f, 0.0f, omega);
        usleep(20 * 1000);
    }
    if (stop_at_end) sport.StopMove();
}

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

void turnToYawDeg(unitree::robot::go2::SportClient& sport,
                  float target_yaw_deg,
                  float tol_deg = FINAL_TURN_TOL,
                  bool stop_at_end = true) {
    sport.StaticWalk();
    usleep(500 * 1000);
    auto t0 = std::chrono::steady_clock::now();
    while (g_running) {
        float err = normalize_180(target_yaw_deg - g_yaw_deg.load());
        if (std::abs(err) < tol_deg) break;
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
    if (stop_at_end) sport.StopMove();
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
            std::vector<cv::Point2f> keep;
            for (const auto& p : pts)
                if (std::abs(a + b * p.x - p.y) <= 18.0f) keep.push_back(p);
            if ((int)keep.size() >= 15) fitLS(keep, a, b);
            float tilt = rad2deg(std::atan(-b));
            if (std::abs(tilt) <= FINAL_YAW_VIS_TILT_MAX) {
                if (tilt_deg_out)   *tilt_deg_out = tilt;
                if (tilt_valid_out) *tilt_valid_out = true;
                int rc = (int)(a + b * (W * 0.5f));
                return std::max(0, std::min(H - 1, rc));
            }
        }
    }
    return row_scan;
}

const double FINAL_LINE_MIN_H_FRAC = 0.45;
static bool getFinalLineCenterXNoBlue(const cv::Mat& frame, int& center_x,
                                      cv::Mat& mask, cv::Mat& dbg_L,
                                      int min_area) {
    if (!extractBlackLine(frame, mask, dbg_L, min_area)) return false;
    cv::Mat hsv, blue;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
                cv::Scalar(FINAL_FB_H_MIN, FINAL_FB_S_MIN, FINAL_FB_V_MIN),
                cv::Scalar(FINAL_FB_H_MAX, 255, 255), blue);
    cv::dilate(blue, blue,
               cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9)));
    mask.setTo(0, blue);
    if (cv::countNonZero(mask) < min_area) return false;
    cv::Mat labels, stats, cents;
    int n = cv::connectedComponentsWithStats(mask, labels, stats, cents, 8);
    int best = -1; int best_area = 0;
    int min_h = (int)(mask.rows * FINAL_LINE_MIN_H_FRAC);
    for (int i = 1; i < n; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        int h    = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        int top  = stats.at<int>(i, cv::CC_STAT_TOP);
        int w    = stats.at<int>(i, cv::CC_STAT_WIDTH);
        if (area < min_area) continue;

        bool tall_ok = (h >= min_h);
        bool far_ok  = (top <= (int)(mask.rows * 0.15))
                       && (h >= (int)(mask.rows * 0.25))
                       && (w <= (int)(mask.cols * 0.45));
        if (!tall_ok && !far_ok) continue;
        if (area > best_area) { best_area = area; best = i; }
    }
    if (best < 0) return false;
    center_x = (int)cents.at<double>(best, 0);
    return true;
}

static int blueZoneCentroidX(const cv::Mat& frame_bgr) {
    if (frame_bgr.empty()) return -1;
    cv::Mat hsv, blue;
    cv::cvtColor(frame_bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
                cv::Scalar(FINAL_FB_H_MIN, FINAL_FB_S_MIN, FINAL_FB_V_MIN),
                cv::Scalar(FINAL_FB_H_MAX, 255, 255), blue);
    cv::Moments m = cv::moments(blue, true);
    if (m.m00 < 0.03 * blue.cols * blue.rows) return -1;
    return (int)(m.m10 / m.m00);
}

bool alignFinalLineCenter(unitree::robot::go2::SportClient& sport,
                          cv::VideoCapture& cap,
                          float target_yaw_deg) {
    std::cout << "[收尾] 相机居中微调: 目标 yaw=" << target_yaw_deg
              << "°, center_tol=" << FINAL_CENTER_TOL_PX << "px" << std::endl;

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
    bool centered = false;
    int centered_stable = 0;
    int empty_frames = 0;
    bool seen_line_once = false;
    int lost_after_seen = 0;
    auto t_center_start = std::chrono::steady_clock::now();

    enum AlignPhase { PH_LATERAL = 0, PH_YAW = 1, PH_FB = 2 };
    int   phase        = PH_LATERAL;
    int   phase_stable = 0;
    auto  t_phase      = std::chrono::steady_clock::now();
    float yaw_hold_deg = target_yaw_deg;
    bool  fb_active     = g_fb_align_enabled;
    int   fb_target_row = -1;
    bool  fb_seen_blue  = false;
    bool  fb_no_blue_step_done = false;
    bool  fb_no_blue_rshift_done = false;

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
    auto stepForwardForBlue = [&]() {
        std::cout << "[收尾] 未看到蓝区, 先向前探一步 vx="
                  << FINAL_FB_SEARCH_FWD_VX << " " << FINAL_FB_SEARCH_FWD_SEC
                  << "s, 然后继续识别蓝区" << std::endl;
        auto t_step = std::chrono::steady_clock::now();
        while (g_running) {
            double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - t_step).count() / 1000.0;
            if (el >= FINAL_FB_SEARCH_FWD_SEC) break;
            float step_yaw_err = normalize_180(yaw_hold_deg - g_yaw_deg.load());
            float step_omega = clampf(FINAL_YAW_HOLD_KP * step_yaw_err,
                                      -FINAL_YAW_HOLD_OMEGA_MAX,
                                       FINAL_YAW_HOLD_OMEGA_MAX);
            sport.Move(FINAL_FB_SEARCH_FWD_VX, 0.0f, step_omega);
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

            if (++empty_frames >= CAM_EMPTY_FRAMES_REOPEN) {
                std::cout << "[相机] 居中阶段连续 " << empty_frames
                          << " 空帧 → StopMove + 尝试重连..." << std::endl;
                sport.StopMove();
                cap.release();
                if (!openAndConfigLineCamera(cap)) usleep(500 * 1000);
                empty_frames = 0;
            }
            usleep(20 * 1000);
            continue;
        }
        empty_frames = 0;

        int line_cx = -1;

        bool have_line = getFinalLineCenterXNoBlue(
            frame, line_cx, mask, dbg_L, FINAL_CENTER_MIN_AREA);

        int lat_target_x = (g_final_lat_target_x >= 0) ? g_final_lat_target_x
                                                       : (frame.cols / 2);
        int err_px = have_line ? (line_cx - lat_target_x) : last_err_px;
        if (have_line) {
            last_err_px = err_px;
            seen_line_once = true;
            lost_after_seen = 0;
        } else if (seen_line_once) {
            lost_after_seen++;
        }

        float vy = 0.0f;
        const char* center_mode = "STOP";
        if (have_line) {
            if (std::abs(err_px) > FINAL_CENTER_TOL_PX) {

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
            vy = FINAL_SHIFT_VY;
            center_mode = "SEARCH_RIGHT";
        }

        int   fb_boundary   = -2;
        int   fb_err_rows   = 0;
        float fb_tilt_deg   = 0.0f;
        bool  fb_tilt_valid = false;
        if (fb_active) {
            if (fb_target_row < 0) {
                fb_target_row = (g_final_fb_target_row >= 0)
                    ? g_final_fb_target_row
                    : (int)(frame.rows * FINAL_FB_TARGET_ROW_FRAC_DEFAULT);
                std::cout << "[收尾] 前后目标: 蓝区边界行 " << fb_target_row
                          << (g_final_fb_target_row >= 0 ? " (起点参照)" : " (默认兜底)")
                          << " ±" << FINAL_FB_TOL_ROWS << "px" << std::endl;
            }
            fb_boundary = detectBlueZoneBoundary(frame, &fb_tilt_deg, &fb_tilt_valid);
            if (fb_boundary >= 0) fb_seen_blue = true;

            if (fb_boundary >= 0) {
                cv::line(frame, cv::Point(0, fb_boundary),
                         cv::Point(frame.cols, fb_boundary),
                         cv::Scalar(255, 0, 255), 2);
            }
            cv::line(frame, cv::Point(0, fb_target_row),
                     cv::Point(frame.cols, fb_target_row),
                     cv::Scalar(0, 255, 0), 1);
        }

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

            bool advance = false;
            if (fb_active && fb_tilt_valid) {
                if (std::abs(fb_tilt_deg) <= FINAL_YAW_VIS_TOL_DEG) {
                    omega = 0.0f;
                    if (++phase_stable >= FINAL_CENTER_STABLE_FRAMES) {
                        yaw_hold_deg = g_yaw_deg.load();
                        std::cout << "[收尾] ②朝向对平完成 (倾角 " << fb_tilt_deg
                                  << "°), 锁定 yaw=" << yaw_hold_deg
                                  << "° → ③前后对位" << std::endl;
                        advance = true;
                    }
                } else {
                    phase_stable = 0;
                    omega = FINAL_YAW_VIS_KP * fb_tilt_deg;
                    if (std::abs(omega) < FINAL_YAW_VIS_OMEGA_FLOOR) {
                        omega = (omega >= 0.0f) ? FINAL_YAW_VIS_OMEGA_FLOOR
                                                : -FINAL_YAW_VIS_OMEGA_FLOOR;
                    }
                    omega = clampf(omega, -FINAL_YAW_HOLD_OMEGA_MAX,
                                           FINAL_YAW_HOLD_OMEGA_MAX);
                }
            } else {

                if (fb_active && !fb_seen_blue
                    && phase_sec >= FINAL_FB_NOBLUE_GIVEUP_SEC) {
                    if (!fb_no_blue_step_done) {
                        stepForwardForBlue();
                        continue;
                    } else {

                        if (!fb_no_blue_rshift_done) {
                            fb_no_blue_rshift_done = true;
                            std::cout << "[收尾] ② 前探后仍未看到蓝区 → 有界右移找蓝区"
                                      << " (vy=" << FINAL_PRE_RSHIFT_VY << ", 上限 "
                                      << FINAL_PRE_RSHIFT_MAX_SEC << "s)" << std::endl;
                            auto t_bs = std::chrono::steady_clock::now();
                            bool blue_found = false;
                            cv::Mat bs_f;
                            while (g_running) {
                                double el_bs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - t_bs).count() / 1000.0;
                                if (el_bs >= FINAL_PRE_RSHIFT_MAX_SEC) break;
                                cap >> bs_f;
                                if (!bs_f.empty() && detectBlueZoneBoundary(bs_f) >= 0) {
                                    blue_found = true;
                                    std::cout << "[收尾] ② 右移中找到蓝区 ("
                                              << el_bs << "s) → 回①重新左右居中" << std::endl;
                                    break;
                                }
                                float bs_yaw_err = normalize_180(yaw_hold_deg - g_yaw_deg.load());
                                float bs_omega = clampf(FINAL_YAW_HOLD_KP * bs_yaw_err,
                                                        -FINAL_YAW_HOLD_OMEGA_MAX,
                                                         FINAL_YAW_HOLD_OMEGA_MAX);
                                sport.Move(0.0f, FINAL_PRE_RSHIFT_VY, bs_omega);
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
                        std::cout << "[收尾] ② 前探后仍未看到蓝区, 放弃视觉对平/前后闭环"
                                  << " (退回初始 yaw + 纯左右居中)" << std::endl;
                    }
                }
                if (!fb_active) {
                    if (yaw_ok) phase_stable++; else phase_stable = 0;
                    advance = (phase_stable >= FINAL_CENTER_STABLE_FRAMES);
                }

            }
            if (!advance && phase_sec >= FINAL_PHASE_YAW_MAX_SEC) {
                std::cout << "[收尾] ②朝向相位 " << FINAL_PHASE_YAW_MAX_SEC
                          << "s 超时兜底 (倾角" << (fb_tilt_valid ? "未收敛" : "不可用")
                          << ", 按当前朝向继续) → ③前后对位" << std::endl;
                yaw_hold_deg = g_yaw_deg.load();
                advance = true;
            }
            if (advance) {
                phase = PH_FB; phase_stable = 0;
                t_phase = std::chrono::steady_clock::now();

                fb_ref_px = g_pos_x.load(); fb_ref_py = g_pos_y.load();
                t_fb_stallref = std::chrono::steady_clock::now();
                fb_kicking = false;
            }
        } else {

            if (fb_active) {
                if (fb_seen_blue) {

                    int cur = (fb_boundary >= 0) ? fb_boundary : frame.rows;
                    fb_err_rows = fb_target_row - cur;
                    fb_ok = (fb_boundary >= 0)
                            && (std::abs(fb_err_rows) <= FINAL_FB_TOL_ROWS);
                    if (!fb_ok) {
                        vx = FINAL_FB_KP * (float)fb_err_rows;
                        if (std::abs(vx) < FINAL_FB_VX_FLOOR) {
                            vx = (vx >= 0.0f) ? FINAL_FB_VX_FLOOR : -FINAL_FB_VX_FLOOR;
                        }
                        vx = clampf(vx, -FINAL_FB_VX_MAX, FINAL_FB_VX_MAX);

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
                        stepForwardForBlue();
                        continue;
                    } else {
                        fb_active = false;
                        std::cout << "[收尾] ③前探后仍未见蓝区, 放弃前后闭环" << std::endl;
                    }
                } else {
                    fb_ok = false;
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
            sport.StopMove();
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

        double center_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - t_center_start).count() / 1000.0;
        if (center_elapsed >= FINAL_CENTER_TIMEOUT_SEC) {
            std::cout << "[收尾] 居中超时 " << FINAL_CENTER_TIMEOUT_SEC
                      << "s, 放弃微调, 停车结束 (err_px=" << err_px
                      << ", have_line=" << (have_line ? 1 : 0) << ")" << std::endl;
            break;
        }

        sport.Move(vx, vy, omega);
        usleep(20 * 1000);
    }

    if (g_running) {
        turnToYawDeg(sport, yaw_hold_deg, FINAL_TURN_TOL, true);
    }
    sport.StopMove();
    usleep(200 * 1000);
    return centered;
}

void postJumpAttitudeGuard(unitree::robot::go2::SportClient& sport,
                           double window_sec, double start_check_sec) {
    auto   t0 = std::chrono::steady_clock::now();
    double bad_since   = -1.0;
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
                    if (need > window_sec) window_sec = need;
                }
            }
        }
        usleep(20 * 1000);
    }
}

void doFrontJump(unitree::robot::go2::SportClient& sport,
                 float approach_vx = 0.20f,
                 bool quick = false) {

    const double decel_sec = quick ? PREJUMP_QUICK_DECEL_SEC     : PREJUMP_DECEL_SEC;
    const double min_stand = quick ? PREJUMP_QUICK_MIN_STAND_SEC : PREJUMP_MIN_STAND_SEC;
    const double max_stand = quick ? PREJUMP_QUICK_MAX_STAND_SEC : PREJUMP_MAX_STAND_SEC;
    if (quick) std::cout << "[前跳] 快速通道 (触发即跳): 减速 " << decel_sec
                         << "s, 站稳 " << min_stand << "~" << max_stand << "s" << std::endl;

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
    sport.StopMove();

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
    std::cout << "[前跳] FrontJump()..." << std::endl;
    sport.FrontJump();

    postJumpAttitudeGuard(sport, (double)FRONTJUMP_WAIT_SEC, POST_JUMP_GUARD_START_SEC);
}

void executeWarnAction(unitree::robot::go2::SportClient& sport, int action_id) {
    std::cout << "[警示动作] 执行动作 " << action_id << ": ";

    if (action_id == 1 || action_id == 2) {
        sport.BalanceStand();
        usleep(200 * 1000);
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

    std::vector<char> row_has_line(y1 - y0, 0);
    for (int y = y0; y < y1; ++y) {
        const unsigned char* mp = mask.ptr<unsigned char>(y);
        int cnt = 0;
        for (int x = x0; x < x1; ++x) if (mp[x]) cnt++;
        row_has_line[y - y0] = (cnt >= OB_ROW_LINE_MIN_PX) ? 1 : 0;
    }

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

    for (int i = (int)segs.size() - 1; i >= 1; --i) {
        const ObSeg& below = segs[i];
        const ObSeg& above = segs[i - 1];
        if ((below.b - below.a) < OB_SEG_MIN_PX) continue;
        if ((above.b - above.a) < OB_SEG_MIN_PX) continue;
        int ga = above.b;
        int gb = below.a;
        int gh = gb - ga;
        if (gh < OB_GAP_MIN_PX || gh > OB_GAP_MAX_PX) continue;

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

enum class LineMode   { TO_LINE_LOST, TIMED, FOREVER, PLATFORM_DETECT, THREE_TURN_DETECT, DUAL_PLATFORM_DETECT,
                        TO_OBSTACLE  };
enum class LineResult { AVOIDANCE_TRIGGER, TIMER_DONE, ABORTED, PLATFORM_REACHED, THREE_TURN_DONE, DUAL_PLATFORM_REACHED,
                        DETECT_TIMEOUT,
                        OBSTACLE_TRIGGER  };

LineResult execRedActionSequence(unitree::robot::go2::SportClient& sport,
                                 const cv::Mat* autoid_frame = nullptr) {
    sport.StopMove();
    usleep((unsigned int)(RED_STOP_SEC * 1000000));
    if (autoid_frame && !autoid_frame->empty()) {
        cv::Mat dbg;
        int auto_action = autoid::classifyWarningAction(
            *autoid_frame, g_gui_enabled ? &dbg : nullptr);
        if (auto_action >= 1 && auto_action <= 3) {
            std::cout << "[AUTOID] 警示标志识别成功: action_id "
                      << g_action_id << " -> " << auto_action << std::endl;
            g_action_id = auto_action;
            if (g_gui_enabled && !dbg.empty()) {
                cv::imshow("AutoID", dbg);
                guiWaitKey(1);
            }
        } else {
            std::cout << "[AUTOID] 警示标志单帧识别失败, 保持 action_id="
                      << g_action_id << std::endl;
        }
    }
    if (g_action_id > 0) {
        executeWarnAction(sport, g_action_id);
    } else {
        std::cout << "[红圆] 警示标志识别失败 (action_id=0), 无法确定对应动作, 跳过" << std::endl;
    }
    if (g_running) {
        std::cout << "[红圆] 动作完成后向左平移 vy="
                  << POST_RED_ACTION_LSHIFT_VY << " "
                  << POST_RED_ACTION_LSHIFT_SEC << "s" << std::endl;
        sport.StaticWalk();
        usleep(300 * 1000);
        auto t_red_shift = std::chrono::steady_clock::now();
        while (g_running) {
            double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - t_red_shift).count() / 1000.0;
            if (el >= POST_RED_ACTION_LSHIFT_SEC) break;
            sport.Move(0.0f, POST_RED_ACTION_LSHIFT_VY, 0.0f);
            usleep(20 * 1000);
        }
        sport.StopMove();
        usleep(200 * 1000);
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
                            double ob_window_min_sec = 0.0) {

    std::cout << "[步态] StaticWalk (常规步态)..." << std::endl;
    sport.StaticWalk();
    usleep(1000000);

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

    ResetTrajectory();

    int  no_line_consec = 0;
    bool line_lost_trigger = false;
    bool black_line_visible = true;

    int  ob_found_stable    = 0;
    int  ob_last_gap_bottom = -1;

    auto t_start = std::chrono::steady_clock::now();

    bool red_ever_seen   = false;
    int  red_seen_consec = 0;
    int  red_lost_consec = 0;

    float arc_yaw_prev           = yaw_baseline_rad;
    float arc_cum_delta          = 0.0f;

    int   plat_stable_cnt        = 0;

    int   arc_start_line_lock_cnt  = 0;
    bool  arc_start_shifting       = false;
    bool  arc_start_shift_done     = false;
    auto  arc_start_shift_t0       = t_start;
    float arc_start_shift_yaw_hold = 0.0f;

    int   tt_phase               = 0;
    int   tt_stable_cnt          = 0;

    float tt_stall_ref_delta_deg = -999.0f;
    auto  tt_stall_ref_time      = std::chrono::steady_clock::now();
    int   tt_stall_nudges        = 0;
    float tt_initial_yaw_rad     = yaw_baseline_rad;
    float tt_phase_ref_yaw_rad   = yaw_baseline_rad;
    float tt_phase0_thresh_rad   = deg2rad(POST_PLAT_PHASE0_THRESH_DEG);
    float tt_phase1_thresh_rad   = deg2rad(POST_PLAT_PHASE1_THRESH_DEG);
    float tt_final_thresh_rad    = deg2rad(POST_PLAT_FINAL_THRESH_DEG);

    int   dual_plat_stable_cnt   = 0;

    int   empty_frames           = 0;

    while (g_running) {
        cap >> frame;
        if (frame.empty()) {

            if (++empty_frames >= CAM_EMPTY_FRAMES_REOPEN) {
                std::cout << "[相机] 连续 " << empty_frames
                          << " 空帧 → StopMove + 尝试重连巡线相机..." << std::endl;
                sport.StopMove();
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

        } else {
            lost_count = 0;
            last_dir   = near_dir;

            double error;
            if (near_cross) {
                if (far_dir != INT_MIN && !far_cross) {
                    error = (double)far_dir;
                    cv::putText(frame, "CROSS BAR -> steer by FAR",
                                cv::Point(40, 70), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                                cv::Scalar(0, 165, 255), 2);
                } else {
                    error = prev_error;
                    cv::putText(frame, "CROSS BAR -> hold heading",
                                cv::Point(40, 70), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                                cv::Scalar(0, 165, 255), 2);
                }
            } else {
                error = (double)near_dir;
            }

            double d_error = error - prev_error;
            prev_error     = error;

            double Kp = g_kp_x1000 / 1000.0;
            double Kd = g_kd_x1000 / 1000.0;
            double rot_mult = (rotation_mult > 0.0) ? rotation_mult : 1.0;

            if (rot_mult_ramp_sec > 0.0 && elapsed < rot_mult_ramp_sec && rot_mult > 1.0) {
                rot_mult = 1.0 + (rot_mult - 1.0) * (elapsed / rot_mult_ramp_sec);
            }
            double rotation = (Kp * error + Kd * d_error) * rot_mult;
            double rotation_limit = ROTATION_LIMIT * rot_mult;
            if (rotation > rotation_limit)  rotation = rotation_limit;
            if (rotation < -rotation_limit) rotation = -rotation_limit;

            double abs_err = std::abs(error);
            double speed_scale = 1.0 - (1.0 - MIN_SPEED_RATIO) *
                                 std::min(1.0, abs_err / SPEED_DECAY_PIVOT);

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

            char info[200];
            snprintf(info, sizeof(info),
                     "near=%d far=%d  vx=%.2f yaw=%.2f  Kp=%.3f Kd=%.3f  bias_n=%d",
                     near_dir, far_dir, speed, -rotation, Kp, Kd, g_fork_bias_near);
            cv::putText(frame, info, cv::Point(20, H - 45),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55,
                        cv::Scalar(255, 255, 255), 2);
        }

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
                    return execRedActionSequence(sport, &frame);
                }
            }
        }

        if (mode == LineMode::TO_OBSTACLE) {
            bool ob_trigger = false;
            const char* ob_why = "";
            int trig_row = (int)(H * OB_TRIGGER_ROW_FRAC);
            if (g_ob_detect_enabled && elapsed >= ob_window_min_sec && have_real_line) {
                int strip_cx = (g_last_cx_near >= 0) ? g_last_cx_near
                             : (g_last_cx_far  >= 0) ? g_last_cx_far
                                                     : (W / 2);

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
                return LineResult::OBSTACLE_TRIGGER;
            }
        }

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
        if (key == 27) {
            g_running = false;
            sport.StopMove();
            return LineResult::ABORTED;
        }

        if (mode == LineMode::TO_LINE_LOST && line_lost_trigger) {
            std::cout << "\n[触发避障] 首次检测到视野中看不到黑线"
                      << " (lost=" << no_line_consec << "/" << LINE_LOST_TRIGGER_CONSEC
                      << ") → 退出巡线,进入避障" << std::endl;
            return LineResult::AVOIDANCE_TRIGGER;
        }
        if (mode == LineMode::TIMED && elapsed >= timer_sec) {
            std::cout << "\n[巡线计时] " << timer_sec
                      << "s 到 → 退出巡线" << std::endl;
            return LineResult::TIMER_DONE;
        }

        if (mode == LineMode::FOREVER && !g_warn_action_done
            && elapsed >= g_red_timeout_sec) {
            std::cout << "\n[红圆超时] FOREVER 巡线 " << g_red_timeout_sec
                      << "s 未触发红圆 → 降级: 原地执行动作序列后进收尾"
                      << " (若正常行程超过此值, 用 redtimeout= 调大)" << std::endl;
            return execRedActionSequence(sport, &frame);
        }

        if (mode == LineMode::PLATFORM_DETECT && elapsed >= ARC_WATCHDOG_SEC) {
            std::cout << "\n[看门狗] 弧形→平台检测 " << ARC_WATCHDOG_SEC
                      << "s 未触发 → StopMove, 返回 DETECT_TIMEOUT (主流程跳过第一抓取)"
                      << std::endl;
            sport.StopMove();
            return LineResult::DETECT_TIMEOUT;
        }
        if (mode == LineMode::THREE_TURN_DETECT && elapsed >= TT_WATCHDOG_SEC) {
            std::cout << "\n[看门狗] 三连转检测 " << TT_WATCHDOG_SEC
                      << "s 未完成 → StopMove, 返回 DETECT_TIMEOUT (主流程直接进红圈等待)"
                      << std::endl;
            sport.StopMove();
            return LineResult::DETECT_TIMEOUT;
        }
        if (mode == LineMode::DUAL_PLATFORM_DETECT && elapsed >= DUAL_WATCHDOG_SEC) {
            std::cout << "\n[看门狗] 双侧平台检测 " << DUAL_WATCHDOG_SEC
                      << "s 未触发 → 按已触发降级 (不 StopMove, 主流程接盲走)"
                      << std::endl;
            return LineResult::DETECT_TIMEOUT;
        }

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
                          << "m) → 退出巡线 (不 StopMove), 主流程接 "
                          << DUAL_PLAT_POST_BLIND_SEC << "s 盲走" << std::endl;
                return LineResult::DUAL_PLATFORM_REACHED;
            }
        }

        if (mode == LineMode::PLATFORM_DETECT || mode == LineMode::THREE_TURN_DETECT) {
            float yaw_now = g_yaw_rad.load();
            float dyaw    = yaw_now - arc_yaw_prev;
            while (dyaw >  3.14159265f) dyaw -= 2.0f * 3.14159265f;
            while (dyaw < -3.14159265f) dyaw += 2.0f * 3.14159265f;
            arc_cum_delta += dyaw;
            arc_yaw_prev   = yaw_now;

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
                              << (stop_on_exit ? " → StopMove" : " → 不 StopMove (外部续 Move)")
                              << std::endl;
                    if (stop_on_exit) sport.StopMove();
                    return LineResult::PLATFORM_REACHED;
                }
            }

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
                } else {
                    bool local_final_ok = (final_delta >= tt_final_thresh_rad);
                    phase_met = local_final_ok && global_cum_ok;
                    target_show = +POST_PLAT_FINAL_THRESH_DEG;
                    delta_show = rad2deg(final_delta);
                    target_ref = "yaw - tt_initial + global cum";
                }

                if (tt_phase == 2 && !phase_met) {
                    float fd_deg = rad2deg(final_delta);
                    bool near_target = (POST_PLAT_FINAL_THRESH_DEG - fd_deg) <= 3.0f
                                       && fd_deg < POST_PLAT_FINAL_THRESH_DEG;
                    if (!near_target
                        || std::fabs(fd_deg - tt_stall_ref_delta_deg) > 1.5f) {
                        tt_stall_ref_delta_deg = fd_deg;
                        tt_stall_ref_time = std::chrono::steady_clock::now();
                    } else {
                        double stall_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - tt_stall_ref_time).count() / 1000.0;
                        if (stall_sec >= 5.0 && tt_stall_nudges < 2) {
                            tt_stall_nudges++;
                            std::cout << "\n[三连转] ★ phase2 卡死保护: " << stall_sec
                                      << "s 角度停在 " << fd_deg << "° (目标 "
                                      << POST_PLAT_FINAL_THRESH_DEG
                                      << "°±3), 后退一步重找线 ("
                                      << tt_stall_nudges << "/2)" << std::endl;
                            auto t_bk = std::chrono::steady_clock::now();
                            while (g_running) {

                                double bel = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - t_bk).count() / 1000.0;
                                if (bel >= 1.0) break;
                                sport.Move(-0.15f, 0.0f, 0.0f);
                                usleep(20 * 1000);
                            }
                            sport.Move(0.0f, 0.0f, 0.0f);
                            tt_stall_ref_delta_deg = -999.0f;
                        }
                    }
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
                                  << "°) → StopMove, 中转平台前" << std::endl;
                        sport.StopMove();
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

    sport.StopMove();
    return LineResult::ABORTED;
}

LineResult runArcToPlatform(unitree::robot::go2::SportClient& sport,
                            cv::VideoCapture& cap,
                            float yaw_baseline_rad,
                            bool stop_on_exit = true) {
    std::cout << "\n[弧形] 进入 弧形→抓取平台 模式, baseline yaw="
              << rad2deg(yaw_baseline_rad) << "°"
              << (stop_on_exit ? "" : " (退出不 StopMove)") << std::endl;
    return runLineFollowing(sport, cap, LineMode::PLATFORM_DETECT, 0.0,
                            yaw_baseline_rad, stop_on_exit);
}

LineResult runThreeTurnDetect(unitree::robot::go2::SportClient& sport,
                              cv::VideoCapture& cap,
                              float yaw_baseline_rad) {
    std::cout << "\n[三连转] 进入 三连转检测 模式, baseline yaw="
              << rad2deg(yaw_baseline_rad) << "°" << std::endl;
    return runLineFollowing(sport, cap, LineMode::THREE_TURN_DETECT, 0.0,
                            yaw_baseline_rad);
}

LineResult runUntilDualPlatform(unitree::robot::go2::SportClient& sport,
                                cv::VideoCapture& cap) {
    std::cout << "\n[双侧] 进入 双侧平台检测 模式" << std::endl;
    return runLineFollowing(sport, cap, LineMode::DUAL_PLATFORM_DETECT, 0.0);
}

bool runBlindForward(unitree::robot::go2::SportClient& sport,
                     double duration_sec,
                     double vx) {

    const float BLIND_YAW_KP        = 0.02f;
    const float BLIND_YAW_OMEGA_MAX = 0.30f;
    float yaw_hold = g_yaw_deg.load();
    std::cout << "[盲走] 直行 " << duration_sec
              << "s, vx=" << vx << ", yaw 锁定 " << yaw_hold << "°" << std::endl;

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
            sport.StopMove();
            return false;
        }
        usleep(20 * 1000);
    }

    std::cout << "[盲走] 完成,进入避障" << std::endl;
    return true;
}

const std::vector<float> SEGMENT_YAW_DEG = { 0.0f, 90.0f, 180.0f, 90.0f, 0.0f };
const float EXIT_YAW_DEG = 90.0f;

const float TRIGGER_DIST     = 0.20f;
const float TRIGGER_DIST_24  = 0.25f;

const float EXIT_OPEN_DIST   = 2.0f;
const float AV_YAW_TOL       = 1.0f;

const float BASE_VX          = 0.13f;
const float SLOW_VX          = 0.07f;
const float NEAR_FRONT_DIST  = 0.40f;
const float AV_TURN_OMEGA    = 0.45f;
const float MAX_OMEGA        = 0.50f;
const float KP_YAW           = 0.025f;
const float VX_TURN          = 0.06f;
const float VX_TURN_FAST     = 0.08f;
const float VX_TURN_4        = 0.09f;

const float WALL_AVOID_DIST  = 0.22f;
const float WALL_AVOID_OMEGA = 0.15f;

const float AV_CORNER_LOCK_DIST  = 0.70f;
const float AV_TURN_RELATCH_DEG  = 30.0f;

const double AV_TURN_RAMP_SEC    = 0.4;
const float  AV_TURN_RAMP_MIN    = 0.30f;

const int   AV_WATCHDOG      = 90;

const int   LAT_PRE_TURN_IDX = 1;
const float LAT_PRE_VY       = +0.05f;
const float LAT_PRE_TIME_SEC = 1.0f;

const int   LAT_MID_TURN_IDX = 2;
const float LAT_MID_VY       = -0.05f;
const float LAT_MID_TIME_SEC = 2.0f;

const int   LAT_END_TURN_IDX = 4;
const float LAT_END_VY       = +0.05f;
const float LAT_END_TIME_SEC = 1.5f;

const float EXIT_SHIFT_VY       = -0.05f;
const float EXIT_SHIFT_TIME_SEC = 2.0f;

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
    float lateral_vy_active   = 0.0f;
    float lateral_time_active = 0.0f;
    int   loop_cnt = 0;
    int   turn_dir_latch = 0;
    auto  t_turn_entry   = std::chrono::steady_clock::now();

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

            if (front_dist >= AV_CORNER_LOCK_DIST) {
                if (g_right_dist < WALL_AVOID_DIST)      wall_corr = +WALL_AVOID_OMEGA;
                else if (g_left_dist < WALL_AVOID_DIST)  wall_corr = -WALL_AVOID_OMEGA;
            }
            float omega = clampf(omega_yaw + wall_corr, -MAX_OMEGA, MAX_OMEGA);
            float vx    = (front_dist < NEAR_FRONT_DIST) ? SLOW_VX : BASE_VX;

            sport.Move(vx, 0.0f, omega);

            int upcoming_turn = seg_idx + 1;
            float trig_dist_now = (upcoming_turn == 2 || upcoming_turn == 4)
                                      ? TRIGGER_DIST_24 : TRIGGER_DIST;
            if (front_dist < trig_dist_now) {
                if (upcoming_turn == LAT_PRE_TURN_IDX) {
                    std::cout << "\n[段 " << (seg_idx + 1)
                              << " 结束] 前激光 " << front_dist << " < " << trig_dist_now
                              << ",切换 LATERAL_PRE_TURN (" << LAT_PRE_TIME_SEC
                              << "s @ vy=" << LAT_PRE_VY << ")" << std::endl;
                    lateral_vy_active   = LAT_PRE_VY;
                    lateral_time_active = LAT_PRE_TIME_SEC;
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
            sport.Move(0.0f, lateral_vy_active, omega);

            float lat_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    now - lateral_start).count() / 1000.0f;
            if (lat_elapsed >= lateral_time_active) {
                std::cout << "[侧移完成] 切换 TURN (圆弧)  L=" << g_left_dist.load()
                          << " R=" << g_right_dist.load() << std::endl;
                state = State::TURN;
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
                turn_dir_latch = 0;
                seg_idx++;
                int just_finished_turn = seg_idx;
                if (seg_idx >= (int)SEGMENT_YAW_DEG.size()) {
                    std::cout << "[转向完成] 最后一个 90° 已完成,先向右平移 ("
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

                    std::cout << "[转向完成] 进入段 " << (seg_idx + 1)
                              << " (前进, 待前方太近再转下一个 90°)" << std::endl;
                    state = State::FORWARD;
                    seg_start = std::chrono::steady_clock::now();
                }
            } else {

                if (turn_dir_latch == 0) {
                    turn_dir_latch = (next_yaw_err > 0) ? +1 : -1;
                    t_turn_entry   = now;
                }
                int dir = (std::abs(next_yaw_err) > AV_TURN_RELATCH_DEG)
                              ? turn_dir_latch
                              : ((next_yaw_err > 0) ? +1 : -1);

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

        int key = guiWaitKey(1);
        if (key == 27) { g_running = false; break; }

        usleep(100 * 1000);
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

const float VX_UP   = 0.35f;

const int   ST_UP_FAST_CLIMBS = 1;

const double ST_UP_ASSAULT_SEC = 3.0;
const float VX_UP_LATE        = 0.30f;

const float VX_DOWN = 0.35f;

const float TURN_ANGLE_DEG  = 100.0f;
const float ST_TURN_OMEGA   = 1.57f;
const float TURN_VX         = 0.20f;
const float ST_YAW_TOL      = 7.0f;

const float YAW_KP        = 0.020f;
const float MAX_OMEGA_FWD = 0.25f;

const float PITCH_DEEP_DEG     = 24.5f;
const float PITCH_RECOVERY_DEG = 13.0f;

const float  ST_SUMMIT_FLAT_DEG     = 11.0f;

const float  ST_SUMMIT_EVAL_MIN_SEC = 0.8f;
const int    ST_SUMMIT_EVAL_FRAMES  = 3;
const float  ST_SUMMIT_NOSEDOWN_DEG = 4.0f;

const float  ST_EMERG_BACK_VX  = 0.10f;
const double ST_EMERG_BACK_SEC = 1.5;
const float  ST_SUMMIT_MIN_DIST     = 0.30f;

const float  ST_SUMMIT_MIN_DIST_STANCE = 0.65f;

const int    ST_DEEP_CONFIRM_FRAMES = 5;

const float DOWN_DESCEND_CONFIRM_DEG = 10.0f;
const float DOWN_FLAT_TOL_DEG        = 5.0f;

const float SUMMIT_STOP_SEC  = 2.0f;

const float SUMMIT_SHIFT_VY  = 0.25f;

const float SUMMIT_SHIFT_SEC = 1.2f;

const float SUMMIT_BACK_PITCH_TOL = 7.5f;
const float SUMMIT_BACK_VX        = 0.30f;
const float SUMMIT_BACK_SLOW_PITCH_MAX = 11.0f;
const float SUMMIT_BACK_SLOW_VX   = 0.20f;
const float SUMMIT_BACK_SEC       = 1.0f;

const float STUCK_CHECK_SEC   = 1.2f;
const float STUCK_MOVE_THRESH = 0.05f;
const float STUCK_VX_BOOST    = 0.05f;

const float ST_CYCLE_GAIN_MIN = 0.06f;
const float ST_CYCLE_BOOST_VX = 0.10f;

const int   UP_MIN_SEC   = 2;
const int   UP_MAX_SEC   = 40;

const int   TURN_MAX_SEC = 10;
const int   DOWN_MIN_SEC = 4;
const int   DOWN_MAX_SEC = 18;
const int   ST_WATCHDOG  = 90;

const double UP_CLIMB_SEC = 0.7;
const double UP_CLIMB_SEC_LATE = 0.7;

const double UP_STOP_SEC  = 1.2;

enum class Phase {
    UP, SUMMIT_STOP, SUMMIT_BACK, SUMMIT_STOP2, SUMMIT_SHIFT, SUMMIT_TURN, DOWN, EXIT, DONE
};
const char* phaseName(Phase p) {
    switch (p) {
        case Phase::UP:           return "UP         ";
        case Phase::SUMMIT_STOP:  return "SUMMIT_STOP";
        case Phase::SUMMIT_BACK:  return "SUMMIT_BACK ";
        case Phase::SUMMIT_STOP2: return "SUMMIT_STOP2";
        case Phase::SUMMIT_SHIFT: return "SUMMIT_SHIFT";
        case Phase::SUMMIT_TURN:  return "SUMMIT_TURN";
        case Phase::DOWN:         return "DOWN       ";
        case Phase::EXIT:         return "EXIT       ";
        case Phase::DONE:         return "DONE       ";
    }
    return "?";
}

enum class TurnDir { LEFT, RIGHT };

bool runStairs(unitree::robot::go2::SportClient& sport, TurnDir turn_dir) {

    std::cout << "[步态] StaticWalk → FreeWalk (AI 灵动,支持爬楼梯)..." << std::endl;
    int ret = sport.FreeWalk();
    std::cout << "  FreeWalk() ret=" << ret << std::endl;
    sleep(2);

    float init_x = g_pos_x.load();
    float init_y = g_pos_y.load();
    float entry_yaw = g_yaw_rad.load();

    float turn_sign = (turn_dir == TurnDir::LEFT) ? +1.0f : -1.0f;
    float target_yaw_after_turn = normalize_180_rad(
        entry_yaw + turn_sign * deg2rad(TURN_ANGLE_DEG));

    std::cout << "\n========== 台阶路径规划 ==========" << std::endl;
    std::cout << "入口 yaw:        " << rad2deg(entry_yaw) << "°" << std::endl;
    std::cout << "转身后目标 yaw:  " << rad2deg(target_yaw_after_turn)
              << "° (相对入口 "
              << (turn_dir == TurnDir::LEFT ? "+" : "-")
              << TURN_ANGLE_DEG << "°)" << std::endl;
    std::cout << "==================================" << std::endl;

    Phase phase = Phase::UP;
    auto phase_start = std::chrono::steady_clock::now();
    auto start_all   = std::chrono::steady_clock::now();

    float min_pitch_deg_seen = 0.0f;
    float max_pitch_deg_seen = 0.0f;

    float stuck_ref_x    = g_pos_x;
    float stuck_ref_y    = g_pos_y;
    auto  stuck_ref_time = std::chrono::steady_clock::now();
    bool  up_boosted     = false;
    float summit_back_vx_active = SUMMIT_BACK_VX;

    bool  up_climbing    = true;
    int   summit_eval_cnt = 0;
    int   deep_frames     = 0;
    float cycle_start_x   = g_pos_x;
    float cycle_start_y   = g_pos_y;
    bool  cycle_boost     = false;
    int   summit_fixups   = 0;
    int   climb_seg_idx   = 1;
    auto  up_sub_start   = std::chrono::steady_clock::now();
    long  n_loops        = 0;

    while (g_running && phase != Phase::DONE) {
        n_loops++;
        auto now = std::chrono::steady_clock::now();
        int total_sec = std::chrono::duration_cast<std::chrono::seconds>(
                            now - start_all).count();
        float phase_sec_f = std::chrono::duration_cast<std::chrono::milliseconds>(
                                now - phase_start).count() / 1000.0f;
        int phase_sec = (int)phase_sec_f;

        if (total_sec > ST_WATCHDOG) {
            std::cout << "\n[超时] " << ST_WATCHDOG << "s 强制停" << std::endl;
            break;
        }

        float dx = g_pos_x - init_x;
        float dy = g_pos_y - init_y;
        float pitch_deg = rad2deg(g_pitch);

        if (phase == Phase::UP) {
            if (pitch_deg < min_pitch_deg_seen) min_pitch_deg_seen = pitch_deg;
        } else if (phase == Phase::DOWN) {
            if (pitch_deg > max_pitch_deg_seen) max_pitch_deg_seen = pitch_deg;
        }

        float ref_yaw = (phase == Phase::DOWN) ? target_yaw_after_turn : entry_yaw;
        float yaw_err = normalize_180_rad(ref_yaw - g_yaw_rad);
        float omega_keep = clampf(YAW_KP * rad2deg(yaw_err),
                                  -MAX_OMEGA_FWD, MAX_OMEGA_FWD);

        if (phase == Phase::UP) {
            float up_sub_sec = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   now - up_sub_start).count() / 1000.0f;

            if (deep_frames < ST_DEEP_CONFIRM_FRAMES) {
                if (pitch_deg < -PITCH_DEEP_DEG) deep_frames++;
                else                             deep_frames = 0;
            }
            bool  deep_seen = (deep_frames >= ST_DEEP_CONFIRM_FRAMES);
            float recovery  = pitch_deg - min_pitch_deg_seen;
            bool  recovered = (recovery > PITCH_RECOVERY_DEG);
            float dist_climbed = std::sqrt(dx * dx + dy * dy);

            bool        summit     = false;
            bool        summit_emergency = false;
            const char* summit_why = "";
            if (phase_sec >= UP_MIN_SEC && deep_seen) {

                if (pitch_deg > ST_SUMMIT_NOSEDOWN_DEG
                    && dist_climbed >= ST_SUMMIT_MIN_DIST) {
                    summit     = true;
                    summit_emergency = true;
                    summit_why = "pitch 转正超过阈值 (前足已越过远端边缘), 立即紧急到顶";
                } else if (!up_climbing && up_sub_sec >= ST_SUMMIT_EVAL_MIN_SEC
                           && dist_climbed >= ST_SUMMIT_MIN_DIST_STANCE) {

                    bool stance_flat = (pitch_deg > -ST_SUMMIT_FLAT_DEG);
                    if (stance_flat || recovered) {
                        if (++summit_eval_cnt >= ST_SUMMIT_EVAL_FRAMES) {
                            summit     = true;
                            summit_why = stance_flat
                                ? "停段静止站姿指纹 (pitch 浅于 ST_SUMMIT_FLAT_DEG)"
                                : "停段静止回升判据";
                        }
                    } else {
                        summit_eval_cnt = 0;
                    }
                }
            }

            if (summit) {
                std::cout << "\n[到顶] " << summit_why
                          << ": min_pitch=" << min_pitch_deg_seen
                          << "° 当前 pitch=" << pitch_deg
                          << "° 回升=" << recovery
                          << "° dist=" << dist_climbed
                          << "m (子段=" << (up_climbing ? "CLIMB" : "STOP")
                          << " " << up_sub_sec << "s)"
                          << " → 零速保持静止 " << SUMMIT_STOP_SEC << "s" << std::endl;

                sport.Move(0.0f, 0.0f, omega_keep);

                if (summit_emergency && g_running) {
                    usleep(400 * 1000);
                    std::cout << "[到顶] 紧急触发 → 定向后撤 " << ST_EMERG_BACK_SEC
                              << "s @ -" << ST_EMERG_BACK_VX
                              << " m/s (脱离远端边缘)" << std::endl;
                    sport.FreeWalk();
                    usleep(200 * 1000);
                    auto tb = std::chrono::steady_clock::now();
                    while (g_running) {
                        double eb = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::steady_clock::now() - tb).count() / 1000.0;
                        if (eb >= ST_EMERG_BACK_SEC) break;
                        float berr = normalize_180_rad(entry_yaw - g_yaw_rad);
                        float bomega = clampf(YAW_KP * rad2deg(berr),
                                              -MAX_OMEGA_FWD, MAX_OMEGA_FWD);
                        sport.Move(-ST_EMERG_BACK_VX, 0.0f, bomega);
                        usleep(20 * 1000);
                    }
                    sport.Move(0.0f, 0.0f, omega_keep);
                }
                phase = Phase::SUMMIT_STOP;
                phase_start = std::chrono::steady_clock::now();
            } else if (phase_sec >= UP_MAX_SEC) {
                std::cout << "\n[超时] UP " << UP_MAX_SEC
                          << "s, min_pitch=" << min_pitch_deg_seen
                          << "°, 强制到顶, 零速保持静止 " << SUMMIT_STOP_SEC << "s"
                          << std::endl;
                sport.Move(0.0f, 0.0f, omega_keep);
                phase = Phase::SUMMIT_STOP;
                phase_start = now;
            }

            else if (up_climbing) {

                float moved = std::sqrt((g_pos_x - stuck_ref_x) * (g_pos_x - stuck_ref_x)
                                      + (g_pos_y - stuck_ref_y) * (g_pos_y - stuck_ref_y));
                float since_ref = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      now - stuck_ref_time).count() / 1000.0f;
                if (moved > STUCK_MOVE_THRESH) {
                    stuck_ref_x = g_pos_x;
                    stuck_ref_y = g_pos_y;
                    stuck_ref_time = now;
                    if (up_boosted) {
                        std::cout << "[卡住解除] 已移动, vx 恢复 " << VX_UP << std::endl;
                        up_boosted = false;
                    }
                } else if (since_ref > STUCK_CHECK_SEC && !up_boosted) {
                    up_boosted = true;
                    std::cout << "[卡住] " << STUCK_CHECK_SEC << "s 内位移 < "
                              << STUCK_MOVE_THRESH << "m, vx 提到 "
                              << (VX_UP + STUCK_VX_BOOST) << std::endl;
                }
                float vx_up_base = (climb_seg_idx <= ST_UP_FAST_CLIMBS)
                                       ? VX_UP : VX_UP_LATE;
                float vx_up_now = up_boosted ? (vx_up_base + STUCK_VX_BOOST) : vx_up_base;
                if (cycle_boost) vx_up_now += ST_CYCLE_BOOST_VX;
                sport.Move(vx_up_now, 0.0f, omega_keep);

                double climb_sec_now = (climb_seg_idx <= ST_UP_FAST_CLIMBS)
                                           ? ST_UP_ASSAULT_SEC
                                           : (deep_seen ? UP_CLIMB_SEC_LATE : UP_CLIMB_SEC);
                if (up_sub_sec >= climb_sec_now) {
                    std::cout << "[爬-停] CLIMB " << climb_sec_now
                              << (climb_seg_idx <= ST_UP_FAST_CLIMBS ? "s (冲锋段)"
                                  : (deep_seen ? "s (deep后爬段)" : "s"))
                              << " 完成 → 零速保持 " << UP_STOP_SEC
                              << "s 让 IMU 稳定" << std::endl;

                    sport.Move(0.0f, 0.0f, omega_keep);

                    {
                        float gain = std::sqrt(
                            (g_pos_x - cycle_start_x) * (g_pos_x - cycle_start_x)
                          + (g_pos_y - cycle_start_y) * (g_pos_y - cycle_start_y));
                        bool was_boost = cycle_boost;
                        cycle_boost = (gain < ST_CYCLE_GAIN_MIN);
                        if (cycle_boost && !was_boost)
                            std::cout << "[爬-停] ★ 本爬段仅推进 " << gain
                                      << "m (<" << ST_CYCLE_GAIN_MIN
                                      << "), 下一爬段加力 +" << ST_CYCLE_BOOST_VX << std::endl;
                    }
                    up_climbing  = false;
                    up_sub_start = now;
                    summit_eval_cnt = 0;

                    stuck_ref_x    = g_pos_x;
                    stuck_ref_y    = g_pos_y;
                    stuck_ref_time = now;
                }
            }
            else {
                sport.Move(0.0f, 0.0f, omega_keep);

                stuck_ref_x    = g_pos_x;
                stuck_ref_y    = g_pos_y;
                stuck_ref_time = now;

                if (up_sub_sec >= UP_STOP_SEC) {
                    climb_seg_idx++;
                    float next_vx = (climb_seg_idx <= ST_UP_FAST_CLIMBS) ? VX_UP : VX_UP_LATE;
                    std::cout << "[爬-停] STOP " << UP_STOP_SEC
                              << "s 完成 → 第 " << climb_seg_idx << " 个爬段 "
                              << (deep_seen ? UP_CLIMB_SEC_LATE : UP_CLIMB_SEC)
                              << "s @ vx=" << next_vx
                              << (climb_seg_idx > ST_UP_FAST_CLIMBS ? " (后期降速防冲顶)" : "")
                              << " (重新断言 FreeWalk)" << std::endl;

                    sport.FreeWalk();
                    usleep(200 * 1000);
                    cycle_start_x = g_pos_x;
                    cycle_start_y = g_pos_y;
                    up_climbing  = true;
                    up_sub_start = std::chrono::steady_clock::now();
                }
            }
        }
        else if (phase == Phase::SUMMIT_STOP) {

            sport.Move(0.0f, 0.0f, omega_keep);
            if (phase_sec_f >= SUMMIT_STOP_SEC) {
                float abs_pitch = std::abs(pitch_deg);
                if (abs_pitch <= SUMMIT_BACK_PITCH_TOL) {
                    summit_back_vx_active = SUMMIT_BACK_VX;
                    std::cout << "\n[静止结束] " << SUMMIT_STOP_SEC
                              << "s, pitch=" << pitch_deg << "° (|pitch|≤"
                              << SUMMIT_BACK_PITCH_TOL << "°), 回退 "
                              << SUMMIT_BACK_SEC << "s @ vx=-"
                              << summit_back_vx_active << std::endl;
                    phase = Phase::SUMMIT_BACK;
                } else if (abs_pitch < SUMMIT_BACK_SLOW_PITCH_MAX) {
                    summit_back_vx_active = SUMMIT_BACK_SLOW_VX;
                    std::cout << "\n[静止结束] " << SUMMIT_STOP_SEC
                              << "s, pitch=" << pitch_deg << "° ("
                              << SUMMIT_BACK_PITCH_TOL << "°<|pitch|<"
                              << SUMMIT_BACK_SLOW_PITCH_MAX << "°), 慢速回退 "
                              << SUMMIT_BACK_SEC << "s @ vx=-"
                              << summit_back_vx_active << std::endl;
                    phase = Phase::SUMMIT_BACK;
                } else if (summit_fixups < 2) {

                    summit_fixups++;
                    float fix_vx = (pitch_deg < 0.0f) ? +0.15f : -0.15f;
                    std::cout << "\n[静止结束] pitch=" << pitch_deg
                              << "° (|pitch|≥" << SUMMIT_BACK_SLOW_PITCH_MAX
                              << "°), ★ 姿态分诊补正 " << (fix_vx > 0 ? "补爬" : "补退")
                              << " 0.8s @ vx=" << fix_vx
                              << " (第 " << summit_fixups << "/2 次) 后重新静止判定" << std::endl;
                    auto t_fix = std::chrono::steady_clock::now();
                    while (g_running) {
                        double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::steady_clock::now() - t_fix).count() / 1000.0;
                        if (el >= 0.8) break;
                        sport.Move(fix_vx, 0.0f, omega_keep);
                        usleep(20 * 1000);
                    }
                    sport.Move(0.0f, 0.0f, omega_keep);

                } else {
                    std::cout << "\n[静止结束] " << SUMMIT_STOP_SEC
                              << "s, pitch=" << pitch_deg << "° (|pitch|≥"
                              << SUMMIT_BACK_SLOW_PITCH_MAX
                              << "°, 分诊已用完), 跳过回退" << std::endl;
                    phase = Phase::SUMMIT_SHIFT;
                }
                phase_start = now;
            }
        }
        else if (phase == Phase::SUMMIT_BACK) {

            sport.Move(-summit_back_vx_active, 0.0f, omega_keep);
            if (phase_sec_f >= SUMMIT_BACK_SEC) {
                std::cout << "\n[回退结束] " << SUMMIT_BACK_SEC
                          << "s, 零速保持再静止 " << SUMMIT_STOP_SEC << "s"
                          << std::endl;
                sport.Move(0.0f, 0.0f, omega_keep);
                phase = Phase::SUMMIT_STOP2;
                phase_start = now;
            }
        }
        else if (phase == Phase::SUMMIT_STOP2) {

            sport.Move(0.0f, 0.0f, omega_keep);
            if (phase_sec_f >= SUMMIT_STOP_SEC) {
                std::cout << "\n[二次静止结束] " << SUMMIT_STOP_SEC
                          << "s, 进入 SUMMIT_SHIFT" << std::endl;
                phase = Phase::SUMMIT_SHIFT;
                phase_start = now;
            }
        }
        else if (phase == Phase::SUMMIT_SHIFT) {

            sport.Move(0.0f, SUMMIT_SHIFT_VY, omega_keep);

            if (std::fabs(rad2deg(g_roll.load())) > 15.0f) {
                std::cout << "\n[平移中止] |roll| 超 15° (疑似踩边), 提前进入 SUMMIT_TURN"
                          << std::endl;
                phase = Phase::SUMMIT_TURN;
                phase_start = now;
            } else if (phase_sec_f >= SUMMIT_SHIFT_SEC) {
                std::cout << "\n[平移结束] 进入 SUMMIT_TURN" << std::endl;
                phase = Phase::SUMMIT_TURN;
                phase_start = now;
            }
        }
        else if (phase == Phase::SUMMIT_TURN) {
            float yaw_to_target = normalize_180_rad(target_yaw_after_turn - g_yaw_rad);

            if (std::abs(rad2deg(yaw_to_target)) < ST_YAW_TOL) {
                std::cout << "\n[转身完成] yaw=" << rad2deg(g_yaw_rad)
                          << "°  误差=" << rad2deg(yaw_to_target) << "°"
                          << ", 进入 DOWN" << std::endl;
                phase = Phase::DOWN;
                phase_start = now;
                min_pitch_deg_seen = 0.0f;
                max_pitch_deg_seen = 0.0f;
            }
            else if (phase_sec > TURN_MAX_SEC) {
                std::cout << "\n[转身超时] 误差还有 " << rad2deg(yaw_to_target)
                          << "°, 强制进 DOWN" << std::endl;
                phase = Phase::DOWN;
                phase_start = now;
                min_pitch_deg_seen = 0.0f;
                max_pitch_deg_seen = 0.0f;
            }
            else {

                float turn_sign_now = (yaw_to_target > 0) ? +1.0f : -1.0f;
                sport.Move(TURN_VX, 0.0f, turn_sign_now * ST_TURN_OMEGA);
            }
        }
        else if (phase == Phase::DOWN) {
            sport.Move(VX_DOWN, 0.0f, omega_keep);

            bool descended = (max_pitch_deg_seen > DOWN_DESCEND_CONFIRM_DEG);
            bool back_flat = (std::abs(pitch_deg) < DOWN_FLAT_TOL_DEG);

            if (phase_sec >= DOWN_MIN_SEC && descended && back_flat) {
                std::cout << "\n[到底] max_pitch=" << max_pitch_deg_seen
                          << "° 当前 pitch=" << pitch_deg
                          << "° 已回到平地附近, 进入 EXIT" << std::endl;
                phase = Phase::EXIT;
                phase_start = now;
            } else if (phase_sec >= DOWN_MAX_SEC) {
                std::cout << "\n[超时] DOWN " << DOWN_MAX_SEC
                          << "s, max_pitch=" << max_pitch_deg_seen
                          << "°, 强制 EXIT" << std::endl;
                phase = Phase::EXIT;
                phase_start = now;
            }
        }
        else if (phase == Phase::EXIT) {

            sport.Move(0, 0, 0);
            usleep(300000);
            std::cout << "\n[步态] FreeWalk → StaticWalk (台阶完成,切回常规步态)..."
                      << std::endl;
            ret = sport.StaticWalk();
            std::cout << "  StaticWalk() ret=" << ret << std::endl;
            sleep(2);
            phase = Phase::DONE;
        }

        {
            float yaw_to_tgt = (phase == Phase::SUMMIT_TURN)
                ? rad2deg(normalize_180_rad(target_yaw_after_turn - g_yaw_rad))
                : rad2deg(yaw_err);
            float track_extreme = (phase == Phase::DOWN) ? max_pitch_deg_seen
                                                          : min_pitch_deg_seen;
            printf("[V4][%s t=%2ds(%4.1fs)] pitch=%+6.1f° roll=%+6.1f° yaw=%+6.1f°"
                   "  yaw_err=%+6.1f°  extreme=%+5.1f°  dx=%+.2f dy=%+.2f\n",
                   phaseName(phase), total_sec, phase_sec_f,
                   pitch_deg, rad2deg(g_roll), rad2deg(g_yaw_rad),
                   yaw_to_tgt, track_extreme, dx, dy);
            fflush(stdout);
        }

        int key = guiWaitKey(1);
        if (key == 27) { g_running = false; break; }

        usleep(20 * 1000);
    }

    if (phase != Phase::DONE) {
        sport.Move(0, 0, 0);
        usleep(200000);
    }

    std::cout << "\n========== 台阶结束 ==========" << std::endl;
    std::cout << "最终阶段:    " << phaseName(phase) << std::endl;
    {
        float run_secs = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - start_all).count() / 1000.0f;
        std::cout << "★平均循环频率: " << (run_secs > 0 ? n_loops / run_secs : 0)
                  << " Hz  (整合系统爬楼实际工作点)" << std::endl;
    }
    std::cout << "净位移 dx:   " << (g_pos_x - init_x) << " m" << std::endl;
    std::cout << "净位移 dy:   " << (g_pos_y - init_y) << " m" << std::endl;
    std::cout << "最终 pitch:  " << rad2deg(g_pitch) << "°" << std::endl;
    std::cout << "上行 min_pitch: " << min_pitch_deg_seen << "°  (UP 极值)"
              << std::endl;
    std::cout << "下行 max_pitch: " << max_pitch_deg_seen << "°  (DOWN 极值)"
              << std::endl;
    std::cout << "最终 yaw:    " << rad2deg(g_yaw_rad) << "° (相对入口 "
              << rad2deg(g_yaw_rad - entry_yaw) << "°)" << std::endl;
    std::cout << "==============================" << std::endl;

    return (phase == Phase::DONE);
}

namespace material_grab {

int g_roi_left_pct = 8, g_roi_right_pct = 92, g_roi_top_pct = 8, g_roi_bottom_pct = 92;
int g_thr_mode = 0;
int g_mad_k_x10 = 35;
int g_diff_thr = 25;
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

std::string  g_arm_serial_dev = "/dev/ttyUSB1";
const double ARM_BOOT_WAIT_SEC = 6.0;

const double ARM_CMD1_MONITOR_SEC = 1.5;
const double ARM_SETTLE_SEC = 10.0;
const int    ARM_STABLE_FRAMES = 8;
const int    ARM_STABLE_TOL_PX = 12;
const int    ARM_GRAB_REPLY_TIMEOUT_MS = 10000;
const double ARM_AFTER_REPLY_WAIT_SEC = 15.0;
const double SECOND_GRAB_WAIT_SEC = 100.0;
const double ARM_SECOND_AFTER_REPLY_WAIT_SEC = 30.0;
const double GRAB_NO_TARGET_SEC = 8.0;

const int    GRAB_BLIND_FALLBACK_ENABLE = 1;
const double GRAB1_BLIND_WX = 0.0;
const double GRAB1_BLIND_WY = 340.0;
const double GRAB2_BLIND_WX = -20.0;
const double GRAB2_BLIND_WY = -350.0;
const int    GRAB_NO_TARGET_MAX_NUDGES = 2;

const float  GRAB_NO_TARGET_BACK_KICK_VX = -0.15f;
const double GRAB_NO_TARGET_BACK_KICK_SEC = 0.50;
const float  GRAB_NO_TARGET_BACK_VX = -0.15f;
const double GRAB_NO_TARGET_BACK_SEC = 0.50;

std::string g_material_cam_id = "1";
const bool   MAT_CAM_LOCK_WB = true;
const int    MAT_CAM_WB_TEMP = 4600;
const bool   MAT_CAM_LOCK_EXPOSURE = true;
const int    MAT_CAM_EXPOSURE_VAL = 156;
const bool   MAT_CAM_LOCK_GAIN = true;
const int    MAT_CAM_GAIN_VAL = 0;
int g_exposure = MAT_CAM_EXPOSURE_VAL;
int g_gain = MAT_CAM_GAIN_VAL;

int g_exposure_first  = MAT_CAM_EXPOSURE_VAL;
int g_exposure_second = MAT_CAM_EXPOSURE_VAL;

double g_last_dark_frac = 0.0;
const int    GRAB_FLUSH_FRAMES = 5;

static void backOffAfterNoMaterial(unitree::robot::go2::SportClient& sport,
                                   cv::VideoCapture& cap,
                                   const char* tag) {
    std::cout << "[" << tag << "] " << GRAB_NO_TARGET_SEC
              << "s 内未识别到物资, 常规步态后退: vx="
              << GRAB_NO_TARGET_BACK_KICK_VX << ", "
              << GRAB_NO_TARGET_BACK_SEC << "s 后继续识别" << std::endl;
    sport.StaticWalk();
    usleep(500 * 1000);

    auto t_back = std::chrono::steady_clock::now();
    while (::g_running) {
        double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t_back).count() / 1000.0;
        if (el >= GRAB_NO_TARGET_BACK_SEC) break;
        float vx = (el < GRAB_NO_TARGET_BACK_KICK_SEC)
            ? GRAB_NO_TARGET_BACK_KICK_VX
            : GRAB_NO_TARGET_BACK_VX;
        sport.Move(vx, 0.0f, 0.0f);
        usleep(20 * 1000);
    }

    sport.Move(0.0f, 0.0f, 0.0f);
    usleep(300 * 1000);
    for (int i = 0; i < GRAB_FLUSH_FRAMES; ++i) { cv::Mat f; cap >> f; }
}

const double HS_R1_TIMEOUT_SEC = 14.0;
const double HS_R3_TIMEOUT_SEC = 14.0;
const double HS_D2_TIMEOUT_SEC = 35.0;
const double HS_D7_TIMEOUT_SEC = 25.0;
const double HS_D4_TIMEOUT_SEC = 50.0;
const double HS_D0_TIMEOUT_SEC = 15.0;

const double ARM_ACK_TIMEOUT_SEC   = 2.0;
const int    ARM_SEND_MAX_TRIES    = 3;

const double ARM_STARTUP_DRAIN_SEC = 3.0;

double g_H1[9] = {1,0,0, 0,1,0, 0,0,1};  int g_H1_valid = 0;
double g_H2[9] = {1,0,0, 0,1,0, 0,0,1};  int g_H2_valid = 0;

const int CAL_PTS = 9;

double CAL_GRID_1[CAL_PTS][2] = {
    {-50, 310}, {0, 310}, {50, 310},
    {-50, 340}, {0, 340}, {50, 340},
    {-50, 380}, {0, 380}, {50, 380},
};

double CAL_GRID_2[CAL_PTS][2] = {
    {-20, -350}, {-20, -310}, {-20, -390},
    { 30, -390}, { 30, -350}, { 30, -310},
    {-60, -310}, {-60, -350}, {-60, -390},
};

const double GRAB1_SEND_X_OFF = 0.0;
const double GRAB2_SEND_Y_OFF = 0.0;

static const char* matCalFileName(bool second) {
    return second ? "mat_cal2.txt" : "mat_cal1.txt";
}

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

bool pixelToWorldMM(const double H[9], int valid, double px, double py,
                    double& wx, double& wy) {
    if (!valid) return false;
    double w = H[6]*px + H[7]*py + H[8];
    if (std::fabs(w) < 1e-9) return false;
    wx = (H[0]*px + H[1]*py + H[2]) / w;
    wy = (H[3]*px + H[4]*py + H[5]) / w;
    return true;
}

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
    bool board_found = false;
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

    cv::Mat dark_mask;
    cv::threshold(labch[0], dark_mask, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    double dark_frac = (double)cv::countNonZero(dark_mask) / (double)labch[0].total();
    g_last_dark_frac = dark_frac;
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
    out.board_found = !board_region.empty();

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

static void setupMaterialWindows() {
    if (!g_gui_enabled) return;
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
    g_last_key = key;
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

int g_arm_fd = -1;

void armPortMarkDead(const char* why) {
    if (g_arm_fd >= 0) { close(g_arm_fd); g_arm_fd = -1; }
    std::cout << "[ARM] ★ 串口失效 (" << why << "), 已关闭, 将自动扫描重连" << std::endl;
}

bool armPortAlive() {
    if (g_arm_fd < 0) return false;
    int m = 0;
    if (ioctl(g_arm_fd, TIOCMGET, &m) < 0
        && (errno == EIO || errno == ENXIO || errno == ENODEV)) return false;
    return true;
}

static std::string resolveReal(const std::string& p) {
    char buf[PATH_MAX];
    if (::realpath(p.c_str(), buf)) return std::string(buf);
    return p;
}

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
    if (miss_sec < 3.0) return "";

    std::string laser_real = resolveReal(::g_front_laser_dev);
    const char* cands[] = { "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
                            "/dev/ttyUSB3", "/dev/ttyUSB4",
                            "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2" };
    for (const char* c : cands) {
        if (access(c, F_OK) != 0) continue;
        if (resolveReal(c) == laser_real) continue;
        if (resolveReal(c) == resolveReal(g_arm_serial_dev)) continue;
        std::cout << "[ARM] ★ 配置路径消失已 " << miss_sec << "s, 兜底改用裸节点 "
                  << c << " (请确认这是臂不是别的设备!)" << std::endl;
        return std::string(c);
    }
    return "";
}

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

static bool readSerialLine(int fd, std::string& partial, std::string& line, int timeout_ms) {
    line.clear();
    auto t0 = std::chrono::steady_clock::now();
    while (true) {
        char c = 0; ssize_t n = read(fd, &c, 1);
        if (n > 0) {
            if (c == '\n' || c == '\r') {
                if (!partial.empty()) { line.swap(partial); return true; }

            } else {
                partial.push_back(c);
                if (partial.size() > 200) partial.clear();
            }
        }
        else if (n < 0 && errno != EINTR && errno != EAGAIN) { std::cerr << "[ARM] read: " << std::strerror(errno) << std::endl; return false; }
        int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - t0).count();
        if (elapsed >= timeout_ms) return false;
        if (n <= 0) usleep(10 * 1000);
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

static bool waitArmReply(int fd, const char* expect, double timeout_sec,
                         cv::VideoCapture* cap, const char* status) {
    auto t0 = std::chrono::steady_clock::now();
    DetectResult res; cv::Mat mask; cv::Rect roi;
    std::string partial;
    std::string token_window;

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

                        armPortMarkDead("等待回执期间 read 失败 (设备掉线?)");
                        cur_fd = -1;
                        break;
                    }
                    return false;
                }
            }
            break;
        }
        if (cap) updateMaterialPreview(*cap, status, res, mask, roi);
        else if (!got_char) usleep(20 * 1000);
    }
    return false;
}

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
        if (any  && quiet >= 3.5) break;
        if (!any && total >= 2.0) break;
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

    for (int attempt = 1; attempt <= ARM_SEND_MAX_TRIES && ::g_running; ++attempt) {

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
            continue;
        }

        if (waitArmReply(g_arm_fd, ack_tok, ARM_ACK_TIMEOUT_SEC, cap, status)) {
            return true;
        }
        if (!armPortAlive()) {

            armPortMarkDead("等 ACK 期间设备掉线 (命令可能已送达, 不重发)");
        } else {
            std::cout << "[ARM] " << ARM_ACK_TIMEOUT_SEC << "s 未收到 " << ack_tok
                      << ", ★ 不重发 (防重复动作), 按已发出继续, 由完成回执超时兜底"
                      << std::endl;
        }
        return true;
    }
    std::cout << "[ARM] ★ 命令始终未能发出 (串口多次重连失败): " << line;
    return false;
}

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
    return sendArmLineAcked(line, nullptr, "")
        && waitArmReply(g_arm_fd, expect, timeout_sec, nullptr, "");
}

bool runMaterialGrab(unitree::robot::go2::SportClient& sport,
                     double window_sec) {

    if (!g_H1_valid) {
        std::cout << "[抓取] ★ 第一抓取未标定 (无 mat_cal1.txt), 跳过抓取流程!"
                  << " 先按手册跑 calmat 采点标定" << std::endl;
        return false;
    }
    g_exposure = g_exposure_first;
    auto t_start = std::chrono::steady_clock::now();
    auto win_elapsed = [&]() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - t_start).count() / 1000.0;
    };

    cv::VideoCapture cap;
    setupMaterialWindows();
    auto cleanup = [&]() {

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
    if (!armPortEnsureOpen(&arm_fresh_open)) {
        std::cout << "[抓取] 串口打开失败 → 跳过抓取 (检查机械臂连线/设备名 armdev="
                  << g_arm_serial_dev << ")" << std::endl;
        cleanup();
        return false;
    }

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

    waitArmReply(g_arm_fd, "R1", HS_R1_TIMEOUT_SEC, &cap, "WAIT ARM R1");
    if (!::g_running || win_elapsed() >= window_sec) {
        if (::g_running) std::cout << "[抓取] 窗口耗尽(就位阶段), 跳过" << std::endl;
        cleanup();
        return false;
    }

    for (int i = 0; i < GRAB_FLUSH_FRAMES; ++i) { cv::Mat f; cap >> f; }

    std::cout << "[抓取] 开始检测物料, 连续 " << ARM_STABLE_FRAMES
              << " 帧稳定后发送抓取指令" << std::endl;
    bool last_valid = false; int stable = 0; cv::Point last_c{0, 0};
    bool sent = false; int notgt_log = 0;
    int  nudge_cnt   = 0;
    auto last_useful = std::chrono::steady_clock::now();
    while (::g_running && win_elapsed() < window_sec) {
        DetectResult res; cv::Mat mask; cv::Rect roi;
        bool found = updateMaterialPreview(cap, "TARGET SEARCH", res, mask, roi);

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

            if (nudge_cnt < GRAB_NO_TARGET_MAX_NUDGES && no_target_sec >= GRAB_NO_TARGET_SEC) {
                nudge_cnt++;
                std::cout << "[抓取] 无有效识别 " << no_target_sec << "s → 后退补救 ("
                          << nudge_cnt << "/" << GRAB_NO_TARGET_MAX_NUDGES << ")" << std::endl;
                backOffAfterNoMaterial(sport, cap, "抓取");
                last_useful = std::chrono::steady_clock::now();
                notgt_log = 0;
            }

            else if (GRAB_BLIND_FALLBACK_ENABLE
                     && nudge_cnt >= GRAB_NO_TARGET_MAX_NUDGES
                     && no_target_sec >= GRAB_NO_TARGET_SEC) {
                char cmd[64];
                std::snprintf(cmd, sizeof(cmd), "2,%.1f,%.1f\n",
                              GRAB1_BLIND_WX + GRAB1_SEND_X_OFF, GRAB1_BLIND_WY);
                std::cout << "[抓取] ★ 补救用尽后仍无有效识别 " << no_target_sec
                          << "s → 盲抓标定中心点, 发送: " << cmd;
                if (sendArmLineAcked(cmd, &cap, "SEND CMD 2 (BLIND)")) {
                    sent = true;
                    waitArmReply(g_arm_fd, "D2", HS_D2_TIMEOUT_SEC, &cap, "WAIT ARM D2");
                }
                break;
            }
            continue;
        }
        last_useful = std::chrono::steady_clock::now();
        int dx = res.bbox_center.x - last_c.x, dy = res.bbox_center.y - last_c.y;
        if (!last_valid || dx * dx + dy * dy > ARM_STABLE_TOL_PX * ARM_STABLE_TOL_PX) stable = 1;
        else stable++;
        last_c = res.bbox_center; last_valid = true;
        std::cout << "[抓取] center=(" << res.bbox_center.x << "," << res.bbox_center.y
                  << ") area=" << res.area << " 稳定 " << stable << "/" << ARM_STABLE_FRAMES << std::endl;
        if (stable >= ARM_STABLE_FRAMES) {

            double wx = 0, wy = 0;
            if (!pixelToWorldMM(g_H1, g_H1_valid,
                                res.bbox_center.x, res.bbox_center.y, wx, wy)) {
                std::cout << "[抓取] 像素→毫米换算失败, 不发坐标" << std::endl;
                break;
            }
            char cmd[64];
            std::snprintf(cmd, sizeof(cmd), "2,%.1f,%.1f\n",
                          wx + GRAB1_SEND_X_OFF, wy);
            std::cout << "[抓取] 发送: " << cmd
                      << "  (像素 " << res.bbox_center.x << ","
                      << res.bbox_center.y << " → 毫米)" << std::endl;
            if (sendArmLineAcked(cmd, &cap, "SEND CMD 2")) {
                sent = true;

                waitArmReply(g_arm_fd, "D2", HS_D2_TIMEOUT_SEC, &cap, "WAIT ARM D2");
            }
            break;
        }
    }
    if (!sent) std::cout << "[抓取] 窗口内未稳定锁定目标, 未发送抓取指令" << std::endl;
    cleanup();
    return sent;
}

bool runSecondMaterialGrab(unitree::robot::go2::SportClient& sport,
                           double window_sec) {
    if (!g_H2_valid) {
        std::cout << "[第二抓取] ★ 第二抓取未标定 (无 mat_cal2.txt), 跳过抓取流程!"
                  << " 先按手册跑 calmat2 采点标定" << std::endl;
        return false;
    }
    g_exposure = g_exposure_second;
    auto t_start = std::chrono::steady_clock::now();
    auto win_elapsed = [&]() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - t_start).count() / 1000.0;
    };

    cv::VideoCapture cap;
    bool windows_ready = false;
    auto cleanup = [&]() {

        cap.release();
        if (windows_ready) {
            destroyMaterialWindows();
            windows_ready = false;
        }
    };

    bool arm_fresh_open = false;
    if (!armPortEnsureOpen(&arm_fresh_open)) {
        std::cout << "[第二抓取] 串口打开失败 → 跳过第二抓取 (检查机械臂连线/设备名 armdev="
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

    std::cout << "[第二抓取] 发送: 3 (摆第二识别位, 握着物资1), 等待 R3 (超时 "
              << HS_R3_TIMEOUT_SEC << "s)" << std::endl;
    if (!sendArmLineAcked("3\n", nullptr, "")) {
        cleanup();
        return false;
    }
    {
        double t_left = window_sec - win_elapsed();
        waitArmReply(g_arm_fd, "R3", std::min(HS_R3_TIMEOUT_SEC, std::max(1.0, t_left)),
                     nullptr, "");
    }
    if (!::g_running || win_elapsed() >= window_sec) {
        cleanup();
        return false;
    }

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
    int  nudge_cnt   = 0;
    auto last_useful = std::chrono::steady_clock::now();
    while (::g_running && win_elapsed() < window_sec) {
        DetectResult res; cv::Mat mask; cv::Rect roi;
        bool found = updateMaterialPreview(cap, "SECOND TARGET SEARCH", res, mask, roi);

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
                std::cout << "[第二抓取] 无有效识别 " << no_target_sec << "s → 后退补救 ("
                          << nudge_cnt << "/" << GRAB_NO_TARGET_MAX_NUDGES << ")" << std::endl;
                backOffAfterNoMaterial(sport, cap, "第二抓取");
                last_useful = std::chrono::steady_clock::now();
                notgt_log = 0;
            }

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
                    waitArmReply(g_arm_fd, "D4", HS_D4_TIMEOUT_SEC, &cap, "WAIT ARM D4");
                }
                break;
            }
            continue;
        }
        last_useful = std::chrono::steady_clock::now();

        int dx = res.bbox_center.x - last_c.x, dy = res.bbox_center.y - last_c.y;
        if (!last_valid || dx * dx + dy * dy > ARM_STABLE_TOL_PX * ARM_STABLE_TOL_PX) stable = 1;
        else stable++;
        last_c = res.bbox_center; last_valid = true;

        std::cout << "[第二抓取] center=(" << res.bbox_center.x << "," << res.bbox_center.y
                  << ") area=" << res.area << " 稳定 "
                  << stable << "/" << ARM_STABLE_FRAMES << std::endl;

        if (stable >= ARM_STABLE_FRAMES) {

            double wx = 0, wy = 0;
            if (!pixelToWorldMM(g_H2, g_H2_valid,
                                res.bbox_center.x, res.bbox_center.y, wx, wy)) {
                std::cout << "[第二抓取] 像素→毫米换算失败, 不发坐标" << std::endl;
                break;
            }
            char cmd[64];
            std::snprintf(cmd, sizeof(cmd), "4,%.1f,%.1f\n",
                          wx, wy + GRAB2_SEND_Y_OFF);
            std::cout << "[第二抓取] 发送: " << cmd
                      << "  (像素 " << res.bbox_center.x << ","
                      << res.bbox_center.y << " → 毫米)" << std::endl;
            if (sendArmLineAcked(cmd, &cap, "SEND CMD 4")) {
                sent = true;

                waitArmReply(g_arm_fd, "D4", HS_D4_TIMEOUT_SEC, &cap, "WAIT ARM D4");
            }
            break;
        }
    }

    if (!sent) std::cout << "[第二抓取] 窗口内未稳定锁定目标, 未发送第二抓取坐标" << std::endl;
    cleanup();
    return sent;
}

}

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

    double (*grid)[2] = second_point ? material_grab::CAL_GRID_2
                                     : material_grab::CAL_GRID_1;
    std::vector<cv::Point2f> cap_px;
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
        if (key < 0) key = pollStdinKey();
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

enum class StartStage { FULL = 0, AVOID, STAIRS, ARC, TT, RED, CLOSE };
struct StartPreset { const char* name; float yaw_offset_deg; float cum_preload_deg; };
const StartPreset START_PRESETS[] = {
    { "full",   0.0f,    0.0f },
    { "avoid",  0.0f,    0.0f },
    { "stairs", 90.0f,   0.0f },
    { "arc",    180.0f, 180.0f },
    { "tt",     0.0f,   310.0f },
    { "red",    180.0f, 310.0f },
    { "close", -90.0f,  310.0f },
};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0]
                  << " <networkInterface> [left|right] [lab|sun] [nogui]"
                  << " [dogonly] [cal|calmat] [startat=X] [linecam=N] [key=value...]" << std::endl;
        std::cout << "  left|right     台阶顶层转向方向 (默认 left)" << std::endl;
        std::cout << "  lab|sun        视觉环境 profile (默认 lab; sun=强太阳光低曝光)"
                  << std::endl;
        std::cout << "  nogui          ★ 比赛模式: 不开任何窗口/滑动条 (省算力)"
                  << std::endl;
        std::cout << "  dogonly        ★ 只调机械狗: 禁用机械臂/抓取相机, 平台停位只停 5s"
                  << std::endl;
        std::cout << "  cal/calline/calred  静态巡线+红圈标定: 不初始化狗, 不下发运动"
                  << std::endl;
        std::cout << "  calmat/calmat1/calmat2  静态物资相机标定: 不发机械臂/狗命令"
                  << std::endl;
        std::cout << "  startat=X      ★ 续跑入口: avoid|stairs|arc|tt|red|close"
                  << " (1free=arc 别名)" << std::endl;
        std::cout << "  yawoff=D       覆盖续跑入口虚拟 yaw 偏移 (度)" << std::endl;
        std::cout << "  redtimeout=S   红圈等待超时秒数 (默认 40)" << std::endl;
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

    TurnDir turn_dir = TurnDir::LEFT;
    StartStage start_stage = StartStage::FULL;
    CalibrationMode cal_mode = CalibrationMode::NONE;
    bool  has_yawoff = false;
    float yawoff_val = 0.0f;

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
            else if (arg == "cal" || arg == "calline" || arg == "calred")
                cal_mode = CalibrationMode::LINE_RED;
            else if (arg == "calmat" || arg == "calmat1")
                cal_mode = CalibrationMode::MATERIAL1;
            else if (arg == "calmat2")
                cal_mode = CalibrationMode::MATERIAL2;
            else if (arg == "1free") start_stage = StartStage::ARC;
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

    ApplyVisionProfileDefaults();

    if (ov_maxbright >= 0) g_max_brightness = ov_maxbright;
    if (ov_realmin   >= 0) g_real_line_min  = ov_realmin;
    if (ov_minarea   >= 0) g_min_area       = ov_minarea;

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
    std::cout << "  阶段 3  巡线 (限时 3s)" << std::endl;
    std::cout << "  阶段 4  爬台阶 ["
              << (turn_dir == TurnDir::LEFT ? "left" : "right") << "]" << std::endl;
    std::cout << "  阶段 5  弧形巡线 → 抓取平台停位 (yaw+lidar) → "
              << (g_dog_only_mode ? "dogonly停位确认 5s" : "机械臂抓取流程")
              << std::endl;
    std::cout << "  阶段 6  三连转检测巡线 (+45/-30/+91) → 停在中转平台前" << std::endl;
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
    std::cout << "  警示标志识别: 启用 → 检测点红圆触发停车后, 视觉识别警示标志"
              << " 并执行对应动作" << std::endl;
    if (g_dog_only_mode) {
        std::cout << "  双侧平台机械臂指令: dogonly 下禁用, 只停 "
                  << DOG_ONLY_PLATFORM_WAIT_SEC << "s" << std::endl;
    } else {
        std::cout << "  放置平台选择: 由抓取平台正面识别标志视觉识别决定 (1号标识→一号平台,"
                  << " 2号标识→二号平台)" << std::endl;
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

    g_line_lost_avoidance_triggered = false;

    if (g_gui_enabled) {
        cv::namedWindow("Original", cv::WINDOW_NORMAL);
        cv::namedWindow("Mask", cv::WINDOW_NORMAL);
        cv::namedWindow("L channel", cv::WINDOW_NORMAL);
        cv::resizeWindow("Original", 800, 600);
        cv::resizeWindow("Mask", 600, 450);
        cv::resizeWindow("L channel", 600, 450);

        cv::namedWindow("Params", cv::WINDOW_NORMAL);
        cv::resizeWindow("Params", 520, 560);

        cv::createTrackbar("maxBright",   "Params", &g_max_brightness,    255);
        cv::createTrackbar("realLineMin", "Params", &g_real_line_min,     255);
        cv::createTrackbar("useOtsu",     "Params", &g_use_otsu,            1);
        cv::createTrackbar("minArea",     "Params", &g_min_area,        10000);
        cv::createTrackbar("blurSize",    "Params", &g_blur_size,          21);
        cv::createTrackbar("connectThr",  "Params", &g_connect_threshold, 600);
        cv::createTrackbar("Kp_x1000",    "Params", &g_kp_x1000,           20);
        cv::createTrackbar("Kd_x1000",    "Params", &g_kd_x1000,           30);
    }

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

    bool use_manual_exposure =
        (g_vision_profile == VisionProfile::SUN) && CAM_LOCK_EXPOSURE;
    if (use_manual_exposure) {
        std::cout << "[摄像头] sun 模式: 尝试锁定曝光 target="
                  << g_cam_exposure_val << std::endl;

        bool ok_auto = cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        usleep(100 * 1000);

        bool ok_exp  = cap.set(cv::CAP_PROP_EXPOSURE, g_cam_exposure_val);
        usleep(100 * 1000);
        std::cout << "  set AUTO_EXPOSURE=1 ret=" << ok_auto
                  << "  set EXPOSURE=" << g_cam_exposure_val << " ret=" << ok_exp
                  << std::endl;
        std::cout << "  回读 AUTO_EXPOSURE=" << cap.get(cv::CAP_PROP_AUTO_EXPOSURE)
                  << "  EXPOSURE=" << cap.get(cv::CAP_PROP_EXPOSURE) << std::endl;

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

    {
        cv::Mat tmp;
        for (int i = 0; i < 5; ++i) cap >> tmp;
    }

    if (needs_front_laser_for_avoidance) {
        g_front_laser_running = true;
        front_laser_thread = std::thread(frontLaserThread);
        std::cout << "[INFO] 等待前方激光数据..." << std::endl;
        for (int i = 0; i < 50 && g_running; ++i) {
            if (g_front_laser.ms.load() != 0) break;
            usleep(100 * 1000);
        }
        if (g_front_laser.ms.load() == 0) {

            std::cerr << "\n[警告] 5s 未收到前方激光数据 (laserdev="
                      << g_front_laser_dev << ")!\n"
                      << "[警告] 降级继续: 避障前距回退 Go2 雷达。"
                      << "若在调试, 建议 Ctrl+C 排查后再跑\n" << std::endl;
        } else {
            std::cout << "[前激光] front=" << frontLaserDistFresh() << "m" << std::endl;
        }
    }

    unitree::robot::go2::SportClient sport;
    sport.SetTimeout(10.0f);
    sport.Init();

    g_vui = new unitree::robot::go2::VuiClient();
    g_vui->Init();
    std::cout << "[INFO] VuiClient 已初始化 (前灯控制)" << std::endl;

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

    if (g_fb_align_enabled && start_stage == StartStage::FULL && g_running) {
        cv::Mat f, cap_m, cap_d;
        for (int i = 0; i < 8 && g_running; ++i) { cap >> f; usleep(30 * 1000); }
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
                x_sum += cx; x_cnt++;
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

    bool ok = true;
    float yaw_baseline_stair = g_yaw_rad.load();

    bool run_stage1      = (start_stage == StartStage::FULL);
    bool run_avoid_stage = (start_stage == StartStage::FULL || start_stage == StartStage::AVOID);
    bool run_pre_stairs  = (start_stage == StartStage::FULL || start_stage == StartStage::AVOID);
    bool run_stairs_stage= (start_stage == StartStage::FULL || start_stage == StartStage::AVOID
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
        if (dogonly_timing_rows.empty()) return;
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

    if (ok && g_running && run_stage1) {
        auto t_stage1 = timingNow();
        std::cout << "\n############ 阶段 1/6: 起步前跳 + 巡线 + 盲走 ############" << std::endl;

        double ob1_dist = START_LINE_SEC * START_PREJUMP_LINE_SPEED;
        double ob1_nom  = obNominalSecForDist(ob1_dist);
        double ob1_min  = std::max(OB_APPROACH_KICK_SEC, ob1_nom - OB_WIN_HALF_SEC);

        double ob1_max  = g_ob_detect_enabled ? (ob1_nom + OB_WIN_HALF_SEC) : ob1_nom;
        LineResult r0 = runLineFollowing(sport, cap, LineMode::TO_OBSTACLE, ob1_max,
                                         0.0f, true, OB_APPROACH_KICK_SPEED,
                                         OB_APPROACH_KICK_SEC, OB_APPROACH_SLOW_SPEED,
                                         1.0, 0.0, ob1_min);
        if (r0 == LineResult::ABORTED) ok = false;
        if (ok && g_running) {
            doFrontJump(sport, (float)OB_APPROACH_SLOW_SPEED, true);

            std::cout << "[阶段1] 前跳落地, 原地站稳 " << POST_JUMP_SETTLE_SEC
                      << "s (不切步态, 带落地姿态看门狗), 再进巡线锁常规步态..." << std::endl;

            postJumpAttitudeGuard(sport, POST_JUMP_SETTLE_SEC, 0.0);
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
                                             STAGE1_ROT_RAMP_SEC);
            if (r1 == LineResult::ABORTED) ok = false;
        }
        if (ok && g_running) {
            if (!runBlindForward(sport, STAGE1_BLIND_TIME_SEC, BLIND_FORWARD_SPEED)) {
                ok = false;
            }
        }
        addDogOnlyTiming("阶段1 起步前跳+巡线+盲走", t_stage1);
    }

    if (ok && g_running && run_avoid_stage) {
        auto t_stage2 = timingNow();
        std::cout << "\n############ 阶段 2/6: 避障 ############" << std::endl;
        bool avoid_ok = runAvoidance(sport);
        stop_front_laser_thread();
        std::cout << "[前激光] 避障结束, 已释放串口" << std::endl;
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

    if (ok && g_running && run_pre_stairs) {
        auto t_stage3 = timingNow();
        std::cout << "\n############ 阶段 3/6: 巡线 (限时 3s) ############" << std::endl;
        LineResult r3 = runLineFollowing(sport, cap, LineMode::TIMED, 3.0);
        if (r3 == LineResult::ABORTED) ok = false;
        addDogOnlyTiming("阶段3 台阶前巡线", t_stage3);
    }

    if (ok && g_running && run_pre_stairs) {
        auto t_stage35 = timingNow();
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
            usleep(20 * 1000);
        }
        sport.StopMove();
        usleep(200 * 1000);
        addDogOnlyTiming("阶段3.5 台阶前停稳/对位", t_stage35);
    }

    if (ok && g_running && run_stairs_stage) {
        auto t_stage4 = timingNow();
        yaw_baseline_stair = g_yaw_rad.load();
        std::cout << "\n[弧形] 快照 yaw baseline = "
                  << rad2deg(yaw_baseline_stair) << "° (台阶前)" << std::endl;
        std::cout << "\n############ 阶段 4/6: 爬台阶 (AI 步态) ############" << std::endl;
        if (!runStairs(sport, turn_dir)) {
            if (g_running) {
                std::cout << "[警告] 台阶未正常完成, 强制切回 StaticWalk 后继续后续阶段"
                          << std::endl;
                sport.Move(0, 0, 0);
                usleep(300 * 1000);
                sport.StaticWalk();
                sleep(2);
            } else {
                ok = false;
            }
        }
        addDogOnlyTiming("阶段4 爬台阶", t_stage4);
    }

    if (ok && g_running && run_arm0_pause) {
        auto t_arm0_pause = timingNow();
        if (g_dog_only_mode) {
            waitDogOnlyPlatform(sport, "阶段4.5 下台阶后停位");
        } else {
            std::cout << "\n[阶段4.5] 下台阶后向机械臂发送 0 (握手: 等 D0, 超时 "
                      << material_grab::HS_D0_TIMEOUT_SEC << "s)" << std::endl;
            sport.StopMove();

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

    if (ok && g_running && run_arc_stage) {
        auto t_stage5 = timingNow();
        double stage5_full_extra = 0.0;
        std::cout << "\n############ 阶段 5/6: 弧形巡线 → 抓取平台停位 ############"
                  << std::endl;

        LineResult r5 = runArcToPlatform(sport, cap, yaw_baseline_stair,
                                         false);
        if (r5 == LineResult::PLATFORM_REACHED) {
            std::cout << "[阶段5] 已停在抓取平台侧 (未 StopMove, 紧接左平移)" << std::endl;

            if (g_running) {
                std::cout << "[阶段5] 紧接左平移 vy=" << PLATFORM_LSHIFT_VY
                          << " " << PLATFORM_LSHIFT_SEC << "s (无中间停顿, 带 yaw 保持)"
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
                sport.StopMove();
                usleep(200 * 1000);
            }
            if (g_running) {
                std::cout << "[AUTOID] 识别抓取平台正面 1/2 号标志 (决定放置平台)"
                          << std::endl;
                int auto_cmd = autoid::recognizePlaceFromCamera(cap, 1.5);
                if (auto_cmd != 5 && auto_cmd != 6) {
                    std::cout << "[AUTOID] 首轮未识别, 延长识别窗口重试 (5s)" << std::endl;
                    auto_cmd = autoid::recognizePlaceFromCamera(cap, 5.0);
                }
                if (auto_cmd == 5 || auto_cmd == 6) {
                    g_dual_arm_cmd = auto_cmd;
                    std::cout << "[AUTOID] 放置平台识别成功: "
                              << (auto_cmd == 5 ? "1号标识 → 一号放置平台 (arm_cmd=5)"
                                                : "2号标识 → 二号放置平台 (arm_cmd=6)")
                              << std::endl;
                } else {
                    g_dual_arm_cmd = 5;
                    std::cout << "[AUTOID] 识别标志识别失败, 降级默认一号放置平台"
                              << " (arm_cmd=5)" << std::endl;
                }
            }
            if (g_dog_only_mode) {
                waitDogOnlyPlatform(sport, "阶段5 抓取平台停位");
                stage5_full_extra = PLATFORM_WAIT_SEC - DOG_ONLY_PLATFORM_WAIT_SEC;
                std::cout << "[阶段5] dogonly 停位确认结束, 准备第一平台后右移" << std::endl;
            } else {

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
                          << "s (yaw 保持)" << std::endl;
                sport.StaticWalk();
                usleep(300 * 1000);
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
                sport.StopMove();
                usleep(200 * 1000);
            }
            std::cout << "[阶段5] 第一平台后右移完成, 进入阶段6" << std::endl;
        } else if (r5 == LineResult::ABORTED) {
            ok = false;
        } else if (r5 == LineResult::DETECT_TIMEOUT) {
            std::cout << "[阶段5] 弧形→平台检测超时, 跳过第一抓取, 直接进入阶段6"
                      << std::endl;
        } else {

            ok = false;
        }
        addDogOnlyTiming(
            "阶段5 弧形到第一抓取平台",
            t_stage5,
            stage5_full_extra,
            stage5_full_extra > 0.0 ? "(full_est按第一抓取窗口估算)" : "");
    }

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
                sport.StopMove();
                usleep(200 * 1000);
            }

            if (g_running) {
                std::cout << "[阶段6] 平移后原地左转 " << POST_TT_TURN_DEG
                          << "° (容差±" << POST_TT_TURN_TOL << "°)" << std::endl;
                turnInPlace(sport, +POST_TT_TURN_DEG, POST_TT_TURN_TOL);
            }
            addDogOnlyTiming("阶段6B 中转后左移+转向", t_stage6_post);

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

    if (g_warn_action_done && g_running) {
        auto t_close = timingNow();
        double close_full_extra = 0.0;
        std::cout << "\n############ 红圆收尾序列 ############" << std::endl;

        if (g_running) {
            std::cout << "[收尾] 红圆后强制巡线 " << DUAL_PLAT_MIN_LINE_SEC
                      << "s 再允许触发双侧" << std::endl;
            runLineFollowing(sport, cap, LineMode::TIMED, DUAL_PLAT_MIN_LINE_SEC);
        }

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

                std::cout << "[收尾] 双侧触发后盲走 " << DUAL_PLAT_POST_BLIND_SEC
                          << "s (vx=" << POST_DUAL_FWD_VX << ")" << std::endl;
                runBlindForward(sport, DUAL_PLAT_POST_BLIND_SEC, POST_DUAL_FWD_VX);
                sport.StopMove();
                if (g_dog_only_mode) {
                    waitDogOnlyPlatform(sport, "收尾 双侧平台停位");
                    double full_dual_wait =
                        (g_dual_arm_cmd == 5 || g_dual_arm_cmd == 6)
                            ? DUAL_ARM_AFTER_SEND_WAIT_SEC
                            : DUAL_PLAT_WAIT_SEC;
                    close_full_extra += full_dual_wait - DOG_ONLY_PLATFORM_WAIT_SEC;
                } else {
                    if (g_dual_arm_cmd == 5 || g_dual_arm_cmd == 6) {

                        const char* expect = (g_dual_arm_cmd == 5) ? "D5" : "D6";
                        std::cout << "[收尾] 双侧检测到放置平台, 发送机械臂放置指令 "
                                  << g_dual_arm_cmd << " (握手: 等 " << expect
                                  << ", 超时 " << DUAL_ARM_AFTER_SEND_WAIT_SEC
                                  << "s)" << std::endl;
                        material_grab::sendSimpleArmCommandWaitReply(
                            g_dual_arm_cmd, expect, DUAL_ARM_AFTER_SEND_WAIT_SEC);
                    } else {
                        std::cout << "[收尾] 双侧检测到放置平台, 未配置机械臂 5/6 指令, 停留 "
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

        if (g_running) {
            double ob2_dist = FINAL_LINE_SEC * FINAL_PREJUMP_LINE_SPEED;
            double ob2_nom  = obNominalSecForDist(ob2_dist);
            double ob2_min  = std::max(OB_APPROACH_KICK_SEC, ob2_nom - OB_WIN_HALF_SEC);
            double ob2_max  = g_ob_detect_enabled ? (ob2_nom + OB_WIN_HALF_SEC) : ob2_nom;
            runLineFollowing(sport, cap, LineMode::TO_OBSTACLE, ob2_max,
                             0.0f, true, OB_APPROACH_KICK_SPEED,
                             OB_APPROACH_KICK_SEC, OB_APPROACH_SLOW_SPEED,
                             1.0, 0.0, ob2_min);
        }
        if (g_running) doFrontJump(sport, (float)OB_APPROACH_SLOW_SPEED, true);

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

        if (g_running) {
            std::cout << "[收尾] 常规步态转到初始 yaw+" << FINAL_YAW_EXTRA_LEFT_DEG
                      << "°: target=" << final_target_yaw_deg
                      << "° (严格容差 " << FINAL_TURN_TOL
                      << "°, 末尾不 StopMove)" << std::endl;
            turnToYawDeg(sport, final_target_yaw_deg, FINAL_TURN_TOL,
                         false);
        }

        if (g_running) {
            std::cout << "[收尾] 条件式预平移找线 (有线不移/无线找线, 上限 "
                      << FINAL_PRE_RSHIFT_MAX_SEC << "s; yaw 锁初始)" << std::endl;
            cv::Mat rs_f, rs_m, rs_d;
            int rs_lock_cnt = 0;
            const char* rs_last_why = "";
            auto t_rs = std::chrono::steady_clock::now();
            while (g_running) {
                double el = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - t_rs).count() / 1000.0;
                if (el >= FINAL_PRE_RSHIFT_MAX_SEC) {
                    std::cout << "[收尾] 预平移达上限 " << FINAL_PRE_RSHIFT_MAX_SEC
                              << "s 未锁到真线, 交给居中阶段继续搜" << std::endl;
                    break;
                }
                float vy_cmd = FINAL_PRE_RSHIFT_VY;
                const char* why = "无线, 默认右移找线";
                cap >> rs_f;
                if (!rs_f.empty()) {
                    int rs_target_x = (g_final_lat_target_x >= 0)
                                          ? g_final_lat_target_x : (rs_f.cols / 2);
                    int rs_cx = -1;
                    bool line_now = getFinalLineCenterXNoBlue(
                        rs_f, rs_cx, rs_m, rs_d, FINAL_CENTER_MIN_AREA);
                    rs_lock_cnt = line_now ? (rs_lock_cnt + 1) : 0;

                    if (rs_lock_cnt >= FINAL_PRE_RSHIFT_LOCK_FRAMES) {
                        if (std::abs(rs_cx - rs_target_x)
                                <= FINAL_PRE_RSHIFT_STOP_MARGIN_PX) {
                            std::cout << "[收尾] 已锁到真线且在目标x附近 (cx=" << rs_cx
                                      << ", 目标=" << rs_target_x << ", 蓝区"
                                      << (detectBlueZoneBoundary(rs_f) >= 0 ? "可见" : "未见")
                                      << "), 预平移结束 (" << el << "s)" << std::endl;
                            break;
                        }

                        vy_cmd = (rs_cx > rs_target_x) ? -std::fabs(FINAL_PRE_RSHIFT_VY)
                                                       : +std::fabs(FINAL_PRE_RSHIFT_VY);
                        why = (rs_cx > rs_target_x) ? "线在目标右侧, 右移" : "线在目标左侧, 左移";
                    } else {
                        int bx = blueZoneCentroidX(rs_f);
                        if (bx >= 0 && bx < (int)(rs_f.cols * 0.40)) {

                            vy_cmd = +std::fabs(FINAL_PRE_RSHIFT_VY);
                            why = "无线且蓝区偏左 (已越过右缘), 左移回找";
                        }
                    }
                }
                if (why != rs_last_why) {
                    std::cout << "[收尾] 预平移: " << why
                              << " (t=" << el << "s)" << std::endl;
                    rs_last_why = why;
                }
                float yaw_err = normalize_180(final_target_yaw_deg - g_yaw_deg.load());
                float omega = clampf(FINAL_YAW_HOLD_KP * yaw_err,
                                     -FINAL_YAW_HOLD_OMEGA_MAX, FINAL_YAW_HOLD_OMEGA_MAX);
                sport.Move(0.0f, vy_cmd, omega);
                usleep(5 * 1000);
            }
        }

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
