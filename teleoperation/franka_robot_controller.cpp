#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

using Clock = std::chrono::steady_clock;

std::atomic<bool> g_stop{false};

void signalHandler(int) {
  g_stop.store(true, std::memory_order_relaxed);
}

double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}

int64_t nowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             Clock::now().time_since_epoch())
      .count();
}

struct SharedCommand {
  std::array<std::atomic<double>, 6> twist;
  std::atomic<bool> enabled{false};
  std::atomic<bool> quit{false};
  std::atomic<int64_t> last_rx_ns{0};
  std::atomic<uint64_t> seq{0};

  SharedCommand() {
    for (auto& x : twist) x.store(0.0);
  }
};

class UdpReceiver {
 public:
  UdpReceiver(int port, SharedCommand& cmd) : port_(port), cmd_(cmd) {}

  void run() {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
      std::cerr << "[UDP] socket() failed\n";
      g_stop.store(true);
      return;
    }

    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 50000;  // 50 ms
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port_));

    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      std::cerr << "[UDP] bind() failed on port " << port_ << "\n";
      ::close(fd);
      g_stop.store(true);
      return;
    }

    std::cout << "[UDP] listening on 0.0.0.0:" << port_ << "\n";

    char buf[512];
    while (!g_stop.load(std::memory_order_relaxed)) {
      sockaddr_in peer{};
      socklen_t peer_len = sizeof(peer);
      ssize_t n = ::recvfrom(fd, buf, sizeof(buf) - 1, 0,
                             reinterpret_cast<sockaddr*>(&peer), &peer_len);
      if (n <= 0) continue;
      buf[n] = '\0';

      std::istringstream iss(buf);
      double v[6]{};
      int enabled = 0, quit = 0;
      uint64_t seq = 0;
      if (!(iss >> v[0] >> v[1] >> v[2] >> v[3] >> v[4] >> v[5]
                >> enabled >> quit >> seq)) {
        continue;
      }

      // Validate incoming commands against protocol velocity bounds.
      const double v_abs_max = 0.10;   // m/s
      const double w_abs_max = 0.60;   // rad/s
      bool valid = true;
      for (int i = 0; i < 3; ++i) valid &= std::isfinite(v[i]) && std::abs(v[i]) <= v_abs_max;
      for (int i = 3; i < 6; ++i) valid &= std::isfinite(v[i]) && std::abs(v[i]) <= w_abs_max;
      if (!valid) continue;

      for (int i = 0; i < 6; ++i) {
        cmd_.twist[i].store(v[i], std::memory_order_relaxed);
      }
      cmd_.enabled.store(enabled != 0, std::memory_order_relaxed);
      cmd_.quit.store(quit != 0, std::memory_order_relaxed);
      cmd_.seq.store(seq, std::memory_order_relaxed);
      // Store timestamp last so the RT loop sees a complete packet in normal operation.
      cmd_.last_rx_ns.store(nowNs(), std::memory_order_release);
    }

    ::close(fd);
  }

 private:
  int port_;
  SharedCommand& cmd_;
};



// -------- Non-real-time force telemetry / logging --------
// The 1 kHz robot callback NEVER performs file I/O or event recognition.
// It only publishes a small atomic snapshot every 5 cycles (~200 Hz).
// CSV logging, filtering, acceleration estimation and contact-state recognition
// run in the non-real-time worker thread.

struct SharedRobotTelemetry {
  std::array<std::atomic<double>, 6> wrench;
  std::array<std::atomic<double>, 6> cartesian_contact;
  std::array<std::atomic<double>, 3> linear_velocity_base;
  std::array<std::atomic<double>, 3> command_linear_base;
  std::atomic<double> robot_time_sec{0.0};
  std::atomic<bool> command_active{false};
  std::atomic<uint64_t> sequence{0};

  SharedRobotTelemetry() {
    for (auto& x : wrench) x.store(0.0);
    for (auto& x : cartesian_contact) x.store(0.0);
    for (auto& x : linear_velocity_base) x.store(0.0);
    for (auto& x : command_linear_base) x.store(0.0);
  }
};

void publishRobotTelemetry(SharedRobotTelemetry& dst,
                           const franka::RobotState& state,
                           franka::Model& model,
                           bool command_active,
                           const Eigen::Matrix<double, 6, 1>& command_twist) {
  // Actual configured end-effector/TCP Cartesian velocity in robot-base coordinates:
  // V_EE = J_EE(q) * dq.
  const std::array<double, 42> jac_array =
      model.zeroJacobian(franka::Frame::kEndEffector, state);
  Eigen::Map<const Eigen::Matrix<double, 6, 7, Eigen::ColMajor>> J(jac_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());
  const Eigen::Matrix<double, 6, 1> actual_twist = J * dq;

  // sequence odd -> writer active; sequence even -> snapshot complete.
  dst.sequence.fetch_add(1, std::memory_order_acq_rel);

  for (int i = 0; i < 6; ++i) {
    dst.wrench[i].store(state.O_F_ext_hat_K[i], std::memory_order_relaxed);
    dst.cartesian_contact[i].store(state.cartesian_contact[i],
                                   std::memory_order_relaxed);
  }
  for (int i = 0; i < 3; ++i) {
    dst.linear_velocity_base[i].store(actual_twist(i),
                                      std::memory_order_relaxed);
    // Translation commands are expressed in the robot base frame.
    dst.command_linear_base[i].store(command_twist(i),
                                     std::memory_order_relaxed);
  }

  dst.robot_time_sec.store(state.time.toSec(), std::memory_order_relaxed);
  dst.command_active.store(command_active, std::memory_order_relaxed);
  dst.sequence.fetch_add(1, std::memory_order_release);
}

struct TelemetrySnapshot {
  std::array<double, 6> wrench{};
  std::array<double, 6> cartesian_contact{};
  std::array<double, 3> linear_velocity_base{};
  std::array<double, 3> command_linear_base{};
  double robot_time_sec{0.0};
  bool command_active{false};
};

bool readRobotTelemetry(const SharedRobotTelemetry& src, TelemetrySnapshot& out) {
  for (int attempt = 0; attempt < 5; ++attempt) {
    const uint64_t s1 = src.sequence.load(std::memory_order_acquire);
    if (s1 == 0 || (s1 & 1U)) continue;

    for (int i = 0; i < 6; ++i) {
      out.wrench[i] = src.wrench[i].load(std::memory_order_relaxed);
      out.cartesian_contact[i] =
          src.cartesian_contact[i].load(std::memory_order_relaxed);
    }
    for (int i = 0; i < 3; ++i) {
      out.linear_velocity_base[i] =
          src.linear_velocity_base[i].load(std::memory_order_relaxed);
      out.command_linear_base[i] =
          src.command_linear_base[i].load(std::memory_order_relaxed);
    }
    out.robot_time_sec =
        src.robot_time_sec.load(std::memory_order_relaxed);
    out.command_active =
        src.command_active.load(std::memory_order_relaxed);

    const uint64_t s2 = src.sequence.load(std::memory_order_acquire);
    if (s1 == s2 && !(s2 & 1U)) return true;
  }
  return false;
}

std::string makeForceLogFilename() {
  std::time_t now = std::time(nullptr);
  std::tm tm{};
  localtime_r(&now, &tm);
  std::ostringstream oss;
  oss << "force_log_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
  return oss.str();
}

enum class MotionPhase : int {
  kCalibrating = 0,
  kIdle = 1,
  kAccel = 2,
  kFree = 3,
  kContact = 4,
  kRelease = 5,
};

const char* phaseName(MotionPhase p) {
  switch (p) {
    case MotionPhase::kCalibrating: return "CALIBRATING";
    case MotionPhase::kIdle: return "IDLE";
    case MotionPhase::kAccel: return "ACCEL";
    case MotionPhase::kFree: return "FREE";
    case MotionPhase::kContact: return "CONTACT";
    case MotionPhase::kRelease: return "RELEASE";
  }
  return "UNKNOWN";
}

class ForceTelemetryWorker {
 public:
  ForceTelemetryWorker(const SharedRobotTelemetry& telemetry,
                       std::string target_ip = "127.0.0.1",
                       int target_port = 10000)
      : telemetry_(telemetry),
        target_ip_(std::move(target_ip)),
        target_port_(target_port) {}

  void run() {
    // ---------------- Contact detection parameters ----------------
    constexpr int kBaselineSamples = 200;       // ~1.0 s at 200 Hz.

    // Minimum translational command speed for motion-state recognition.
    constexpr double kMinCmdSpeed = 0.004;      // m/s

    // Motion must be stable for this long before contact detection is enabled.
    constexpr double kStableConfirmSec = 0.18;  // s
    constexpr double kStableAccel = 0.040;      // m/s^2
    constexpr double kSpeedTrackingError = 0.008;  // m/s
    constexpr double kMinActualSpeed = 0.003;   // m/s

    // Contact recognition uses the magnitude of projected-force deviation
    // from the stable free-motion baseline.
    constexpr double kFreeBaselineTrackBand = 0.45; // N; only track baseline inside this band
    constexpr double kFreeBaselineAlpha = 0.008;    // very slow drift compensation

    constexpr double kContactResidual = 0.80;       // N above free-motion baseline
    constexpr double kContactHardResidual = 1.50;   // N; contact even if rise is gradual
    constexpr double kContactResidualRate = 5.0;    // N/s
    constexpr double kContactConfirmSec = 0.040;    // s

    // Contact-release thresholds and confirmation intervals.
    constexpr double kReleaseDrop = 0.90;           // N drop from residual peak
    constexpr double kReleaseRatio = 0.55;          // residual below 55% of peak
    constexpr double kReleaseResidualRate = -5.0;   // N/s
    constexpr double kReleaseConfirmSec = 0.025;    // s
    constexpr double kReleaseLatchSec = 0.80;   // UI latch

    // First-order filters at the ~200 Hz telemetry rate.
    constexpr double kForceAlpha = 0.22;
    constexpr double kVelocityAlpha = 0.25;
    constexpr double kAccelAlpha = 0.18;
    constexpr double kDfAlpha = 0.18;
    // ----------------------------------------------------------------

    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(static_cast<uint16_t>(target_port_));
    if (::inet_pton(AF_INET, target_ip_.c_str(), &dst.sin_addr) != 1) {
      std::cerr << "[FORCE] invalid telemetry IP: " << target_ip_ << "\n";
      if (fd >= 0) ::close(fd);
      return;
    }

    const std::string log_name = makeForceLogFilename();
    std::ofstream csv(log_name);
    if (!csv) {
      std::cerr << "[FORCE] cannot open log file: " << log_name << "\n";
    } else {
      csv
          << "t_s,"
          << "raw_Fx_N,raw_Fy_N,raw_Fz_N,raw_Mx_Nm,raw_My_Nm,raw_Mz_Nm,"
          << "Fx_N,Fy_N,Fz_N,Mx_Nm,My_Nm,Mz_Nm,F_norm_N,"
          << "vx_mps,vy_mps,vz_mps,speed_mps,"
          << "cmd_vx_mps,cmd_vy_mps,cmd_vz_mps,cmd_speed_mps,"
          << "v_parallel_mps,a_parallel_mps2,"
          << "F_parallel_N,dF_parallel_Nps,"
          << "F_free_baseline_N,F_contact_residual_N,dF_contact_residual_Nps,"
          << "phase,phase_name,motion_stable,contact_flag,release_flag,"
          << "franka_contact,command_active\n";
      std::cout << "[FORCE] logging: " << log_name << "\n";
    }

    std::cout << "[FORCE] telemetry UDP -> "
              << target_ip_ << ":" << target_port_ << "\n";
    std::cout << "[DETECT] state machine: IDLE -> ACCEL -> FREE -> CONTACT -> RELEASE\n";
    std::cout << "[DETECT] contact uses |F_projected - F_free_baseline|\n";
    std::cout << "[DETECT] residual >= " << kContactResidual
              << " N with rise >= " << kContactResidualRate
              << " N/s, or residual >= " << kContactHardResidual << " N\n";

    std::array<double, 6> baseline_sum{};
    std::array<double, 6> baseline{};
    int baseline_count = 0;
    bool calibrated = false;

    MotionPhase phase = MotionPhase::kCalibrating;
    bool release_latched = false;
    auto release_until = Clock::now();

    bool filters_initialized = false;
    std::array<double, 3> v_filtered{};
    double v_parallel_prev = 0.0;
    double a_parallel_filtered = 0.0;
    double f_parallel_filtered = 0.0;
    double f_parallel_prev = 0.0;
    double df_parallel_filtered = 0.0;

    bool prev_cmd_valid = false;
    std::array<double, 3> prev_cmd_dir{};
    double prev_cmd_speed = 0.0;

    double stable_time = 0.0;
    double contact_candidate_time = 0.0;
    double release_candidate_time = 0.0;

    // Dynamic force baseline learned during stable free motion.
    // Detection uses |projected_force - free_force_baseline|.
    double free_force_baseline = 0.0;
    bool free_force_baseline_valid = false;

    double force_residual = 0.0;
    double force_residual_prev = 0.0;
    double residual_rate_filtered = 0.0;
    double contact_peak_residual = 0.0;

    double t0 = -1.0;
    double previous_robot_time = -1.0;
    int flush_counter = 0;

    auto next = Clock::now();
    const auto worker_period = std::chrono::microseconds(5000);

    while (!g_stop.load(std::memory_order_relaxed)) {
      next += worker_period;

      TelemetrySnapshot snap;
      if (!readRobotTelemetry(telemetry_, snap)) {
        std::this_thread::sleep_until(next);
        continue;
      }

      if (t0 < 0.0) t0 = snap.robot_time_sec;
      const double t = snap.robot_time_sec - t0;

      double dt = 0.005;
      if (previous_robot_time >= 0.0) {
        dt = snap.robot_time_sec - previous_robot_time;
        if (!(dt > 0.001 && dt < 0.050)) dt = 0.005;
      }
      previous_robot_time = snap.robot_time_sec;

      // Build the force/torque baseline only while no operator motion is active.
      if (!calibrated && !snap.command_active) {
        for (int i = 0; i < 6; ++i) baseline_sum[i] += snap.wrench[i];
        ++baseline_count;
        if (baseline_count >= kBaselineSamples) {
          for (int i = 0; i < 6; ++i) {
            baseline[i] = baseline_sum[i] / static_cast<double>(baseline_count);
          }
          calibrated = true;
          phase = MotionPhase::kIdle;
          std::cout << "[FORCE] baseline calibrated. F0=["
                    << baseline[0] << ", "
                    << baseline[1] << ", "
                    << baseline[2] << "] N\n";
        }
      }

      std::array<double, 6> z{};
      if (calibrated) {
        for (int i = 0; i < 6; ++i) z[i] = snap.wrench[i] - baseline[i];
      }

      const double force_norm =
          std::sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]);

      // Low-pass actual translational velocity.
      if (!filters_initialized) {
        for (int i = 0; i < 3; ++i) {
          v_filtered[i] = snap.linear_velocity_base[i];
        }
      } else {
        for (int i = 0; i < 3; ++i) {
          v_filtered[i] =
              kVelocityAlpha * snap.linear_velocity_base[i] +
              (1.0 - kVelocityAlpha) * v_filtered[i];
        }
      }
      const double actual_speed =
          std::sqrt(v_filtered[0] * v_filtered[0] +
                    v_filtered[1] * v_filtered[1] +
                    v_filtered[2] * v_filtered[2]);

      // Current translational command and its unit direction.
      const auto& c = snap.command_linear_base;
      const double cmd_speed =
          std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
      const bool translational_motion =
          calibrated && snap.command_active && cmd_speed >= kMinCmdSpeed;

      std::array<double, 3> cmd_dir{};
      if (cmd_speed >= kMinCmdSpeed) {
        for (int i = 0; i < 3; ++i) cmd_dir[i] = c[i] / cmd_speed;
      }

      // Reset acceleration tracking when the commanded direction or speed changes.
      bool command_changed = false;
      if (translational_motion) {
        if (!prev_cmd_valid) {
          command_changed = true;
        } else {
          const double direction_dot =
              cmd_dir[0] * prev_cmd_dir[0] +
              cmd_dir[1] * prev_cmd_dir[1] +
              cmd_dir[2] * prev_cmd_dir[2];
          if (direction_dot < 0.985 ||
              std::abs(cmd_speed - prev_cmd_speed) > 0.006) {
            command_changed = true;
          }
        }
      }

      double v_parallel = 0.0;
      double f_parallel_raw = 0.0;
      if (translational_motion) {
        v_parallel =
            v_filtered[0] * cmd_dir[0] +
            v_filtered[1] * cmd_dir[1] +
            v_filtered[2] * cmd_dir[2];

        // Positive F_parallel means resistance opposing the commanded motion.
        f_parallel_raw =
            -(z[0] * cmd_dir[0] +
              z[1] * cmd_dir[1] +
              z[2] * cmd_dir[2]);
      }

      if (!filters_initialized || command_changed) {
        f_parallel_filtered = f_parallel_raw;
        f_parallel_prev = f_parallel_raw;
        v_parallel_prev = v_parallel;
        a_parallel_filtered = 0.0;
        df_parallel_filtered = 0.0;
        filters_initialized = true;
      } else {
        f_parallel_filtered =
            kForceAlpha * f_parallel_raw +
            (1.0 - kForceAlpha) * f_parallel_filtered;

        const double a_raw = (v_parallel - v_parallel_prev) / dt;
        a_parallel_filtered =
            kAccelAlpha * a_raw +
            (1.0 - kAccelAlpha) * a_parallel_filtered;

        const double df_raw =
            (f_parallel_filtered - f_parallel_prev) / dt;
        df_parallel_filtered =
            kDfAlpha * df_raw +
            (1.0 - kDfAlpha) * df_parallel_filtered;

        v_parallel_prev = v_parallel;
        f_parallel_prev = f_parallel_filtered;
      }

      // Contact feature: magnitude of deviation from the learned steady-motion
      // projected-force baseline. It is intentionally sign-independent.
      if (free_force_baseline_valid) {
        force_residual = std::abs(f_parallel_filtered - free_force_baseline);
      } else {
        force_residual = 0.0;
      }

      if (!filters_initialized || command_changed) {
        force_residual_prev = force_residual;
        residual_rate_filtered = 0.0;
      } else {
        const double residual_rate_raw =
            (force_residual - force_residual_prev) / dt;
        residual_rate_filtered =
            kDfAlpha * residual_rate_raw +
            (1.0 - kDfAlpha) * residual_rate_filtered;
        force_residual_prev = force_residual;
      }

      bool motion_stable = false;
      if (translational_motion) {
        const bool instant_stable =
            std::abs(a_parallel_filtered) <= kStableAccel &&
            std::abs(v_parallel - cmd_speed) <= kSpeedTrackingError &&
            actual_speed >= kMinActualSpeed;

        if (instant_stable) {
          stable_time += dt;
        } else {
          stable_time = 0.0;
        }
        motion_stable = stable_time >= kStableConfirmSec;
      } else {
        stable_time = 0.0;
      }

      if (!calibrated) {
        phase = MotionPhase::kCalibrating;
      } else if (!translational_motion) {
        // Return to IDLE when no translational motion is commanded.
        phase = MotionPhase::kIdle;
        contact_peak_residual = 0.0;
        contact_candidate_time = 0.0;
        release_candidate_time = 0.0;
        release_latched = false;
        free_force_baseline_valid = false;
        force_residual = 0.0;
        residual_rate_filtered = 0.0;
        prev_cmd_valid = false;
      } else {
        if (command_changed &&
            phase != MotionPhase::kCalibrating &&
            phase != MotionPhase::kIdle) {
          phase = MotionPhase::kAccel;
          stable_time = 0.0;
          contact_candidate_time = 0.0;
          release_candidate_time = 0.0;
          contact_peak_residual = 0.0;
          release_latched = false;
          free_force_baseline_valid = false;
          force_residual = 0.0;
          residual_rate_filtered = 0.0;
        }

        if (phase == MotionPhase::kIdle ||
            phase == MotionPhase::kCalibrating) {
          phase = MotionPhase::kAccel;
          stable_time = 0.0;
        }

        if (phase == MotionPhase::kAccel) {
          // Force transients during start-up are deliberately ignored.
          contact_candidate_time = 0.0;
          release_candidate_time = 0.0;
          if (motion_stable) {
            // At the end of the start-up transient, the current projected force
            // is the steady free-motion reference for this command/direction.
            free_force_baseline = f_parallel_filtered;
            free_force_baseline_valid = true;
            force_residual = 0.0;
            force_residual_prev = 0.0;
            residual_rate_filtered = 0.0;
            phase = MotionPhase::kFree;
          }
        } else if (phase == MotionPhase::kFree) {
          // Slowly track only tiny free-motion drift. As soon as the force moves
          // appreciably away from the baseline, freeze the baseline so a real
          // contact cannot be "learned away".
          if (free_force_baseline_valid &&
              force_residual < kFreeBaselineTrackBand) {
            free_force_baseline =
                kFreeBaselineAlpha * f_parallel_filtered +
                (1.0 - kFreeBaselineAlpha) * free_force_baseline;
            force_residual =
                std::abs(f_parallel_filtered - free_force_baseline);
          }

          // Persistent residual is the main criterion. A fast rise can trigger
          // at the lower threshold; a gradual contact is still recognized once
          // it reaches the hard residual threshold.
          const bool contact_candidate =
              force_residual >= kContactResidual &&
              (residual_rate_filtered >= kContactResidualRate ||
               force_residual >= kContactHardResidual);

          if (contact_candidate) {
            contact_candidate_time += dt;
          } else {
            contact_candidate_time = 0.0;
          }

          if (contact_candidate_time >= kContactConfirmSec) {
            phase = MotionPhase::kContact;
            contact_peak_residual = force_residual;
            contact_candidate_time = 0.0;
            release_candidate_time = 0.0;
          }
        } else if (phase == MotionPhase::kContact) {
          contact_peak_residual =
              std::max(contact_peak_residual, force_residual);

          const double residual_drop =
              contact_peak_residual - force_residual;

          const bool release_candidate =
              contact_peak_residual >= kContactResidual &&
              residual_drop >= kReleaseDrop &&
              force_residual <= kReleaseRatio * contact_peak_residual &&
              residual_rate_filtered <= kReleaseResidualRate;

          if (release_candidate) {
            release_candidate_time += dt;
          } else {
            release_candidate_time = 0.0;
          }

          if (release_candidate_time >= kReleaseConfirmSec) {
            phase = MotionPhase::kRelease;
            release_latched = true;
            release_until =
                Clock::now() +
                std::chrono::milliseconds(
                    static_cast<int>(kReleaseLatchSec * 1000.0));
            release_candidate_time = 0.0;
          }
        } else if (phase == MotionPhase::kRelease) {
          if (release_latched && Clock::now() >= release_until) {
            release_latched = false;
            phase = MotionPhase::kFree;
            contact_peak_residual = 0.0;

            // Update the free-motion force reference after the release interval.
            free_force_baseline = f_parallel_filtered;
            free_force_baseline_valid = true;
            force_residual = 0.0;
            force_residual_prev = 0.0;
            residual_rate_filtered = 0.0;
          }
        }

        prev_cmd_valid = true;
        prev_cmd_dir = cmd_dir;
        prev_cmd_speed = cmd_speed;
      }

      bool franka_contact = false;
      for (double cval : snap.cartesian_contact) {
        franka_contact |= (std::abs(cval) > 0.5);
      }

      const bool contact_flag = phase == MotionPhase::kContact;
      const bool release_flag = phase == MotionPhase::kRelease;

      // UDP telemetry fields:
      // t, zeroed wrench(6), |F|,
      // actual velocity(3), actual speed,
      // command velocity(3),
      // v_parallel, a_parallel, F_parallel, dF_parallel,
      // free-motion projected-force baseline, contact residual, residual rate,
      // phase, contact, release, franka_contact, command_active, motion_stable.
      char msg[1024];
      const int n = std::snprintf(
          msg, sizeof(msg),
          "%.6f "
          "%.6f %.6f %.6f %.6f %.6f %.6f %.6f "
          "%.6f %.6f %.6f %.6f "
          "%.6f %.6f %.6f "
          "%.6f %.6f %.6f %.6f "
          "%.6f %.6f %.6f "
          "%d %d %d %d %d %d\n",
          t,
          z[0], z[1], z[2], z[3], z[4], z[5], force_norm,
          v_filtered[0], v_filtered[1], v_filtered[2], actual_speed,
          c[0], c[1], c[2],
          v_parallel, a_parallel_filtered,
          f_parallel_filtered, df_parallel_filtered,
          free_force_baseline_valid ? free_force_baseline : 0.0,
          force_residual,
          residual_rate_filtered,
          static_cast<int>(phase),
          contact_flag ? 1 : 0,
          release_flag ? 1 : 0,
          franka_contact ? 1 : 0,
          snap.command_active ? 1 : 0,
          motion_stable ? 1 : 0);

      if (fd >= 0 && n > 0) {
        ::sendto(fd, msg, static_cast<size_t>(n), MSG_DONTWAIT,
                 reinterpret_cast<const sockaddr*>(&dst), sizeof(dst));
      }

      if (csv) {
        csv << std::fixed << std::setprecision(6) << t;
        for (double v : snap.wrench) csv << ',' << v;
        for (double v : z) csv << ',' << v;

        csv << ',' << force_norm
            << ',' << v_filtered[0]
            << ',' << v_filtered[1]
            << ',' << v_filtered[2]
            << ',' << actual_speed
            << ',' << c[0]
            << ',' << c[1]
            << ',' << c[2]
            << ',' << cmd_speed
            << ',' << v_parallel
            << ',' << a_parallel_filtered
            << ',' << f_parallel_filtered
            << ',' << df_parallel_filtered
            << ',' << (free_force_baseline_valid ? free_force_baseline : 0.0)
            << ',' << force_residual
            << ',' << residual_rate_filtered
            << ',' << static_cast<int>(phase)
            << ',' << phaseName(phase)
            << ',' << (motion_stable ? 1 : 0)
            << ',' << (contact_flag ? 1 : 0)
            << ',' << (release_flag ? 1 : 0)
            << ',' << (franka_contact ? 1 : 0)
            << ',' << (snap.command_active ? 1 : 0)
            << '\n';

        if (++flush_counter >= 50) {
          csv.flush();
          flush_counter = 0;
        }
      }

      std::this_thread::sleep_until(next);
    }

    if (csv) csv.flush();
    if (fd >= 0) ::close(fd);
  }

 private:
  const SharedRobotTelemetry& telemetry_;
  std::string target_ip_;
  int target_port_;
};

// -------- Joint limits and application workspace bounds --------

// Configured hard joint position limits (rad).
constexpr std::array<double, 7> Q_HARD_MIN{
    -2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508};
constexpr std::array<double, 7> Q_HARD_MAX{
     2.9007,  1.8361,  2.9007, -0.1169,  2.8763, 4.6216,  3.0508};

// Configured nominal joint velocity limits (rad/s).
constexpr std::array<double, 7> QD_HARD_MAX{
    2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26};

// Parameters for position-dependent joint velocity bounds.
constexpr std::array<double, 7> QD_OFFSET{
    0.6599, 0.2517, 0.2000, 0.3533, 0.5757, 0.4878, 0.4628};
constexpr std::array<double, 7> QDD_DEC{
    6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0};

// Software joint position limits with an inward margin.
constexpr std::array<double, 7> Q_SOFT_MIN{
    -2.2476, -1.4454, -2.3937, -2.6714, -2.4100, 0.8773, -2.6045};
constexpr std::array<double, 7> Q_SOFT_MAX{
     2.2476,  1.4454,  2.3937, -0.5226,  2.4100, 4.1841,  2.6045};

// Conservative application-level joint speed caps (rad/s).
constexpr std::array<double, 7> QD_USER_MAX{
    0.55, 0.55, 0.55, 0.55, 0.80, 0.70, 0.80};

// Conservative application-level acceleration caps (rad/s^2).
constexpr std::array<double, 7> QDD_USER_MAX{
    1.2, 1.2, 1.2, 1.2, 1.6, 1.4, 1.6};

std::pair<double, double> positionDependentVelocityBounds(int i, double q) {
  const double upper_term =
      std::max(0.0, 2.0 * QDD_DEC[i] * (Q_HARD_MAX[i] - q));
  const double lower_term =
      std::max(0.0, 2.0 * QDD_DEC[i] * (q - Q_HARD_MIN[i]));

  const double upper =
      std::min(QD_HARD_MAX[i],
               std::max(0.0, -QD_OFFSET[i] + std::sqrt(upper_term)));
  const double lower =
      std::max(-QD_HARD_MAX[i],
               std::min(0.0, QD_OFFSET[i] - std::sqrt(lower_term)));
  return {lower, upper};
}

Eigen::Matrix<double, 6, 1> loadToolTwist(
    const SharedCommand& shared, bool& active, bool& quit) {
  Eigen::Matrix<double, 6, 1> x;
  for (int i = 0; i < 6; ++i) {
    x(i) = shared.twist[i].load(std::memory_order_relaxed);
  }

  quit = shared.quit.load(std::memory_order_relaxed);

  const int64_t last = shared.last_rx_ns.load(std::memory_order_acquire);
  const double age =
      (last == 0) ? std::numeric_limits<double>::infinity()
                  : static_cast<double>(nowNs() - last) * 1e-9;

  constexpr double kWatchdogSec = 0.15;
  active = shared.enabled.load(std::memory_order_relaxed) &&
           age <= kWatchdogSec && !quit && !g_stop.load();

  if (!active) x.setZero();
  return x;
}

Eigen::Matrix3d rotationFromOTEE(const std::array<double, 16>& O_T_EE) {
  // libfranka homogeneous transforms are column-major.
  Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> T(O_T_EE.data());
  return T.block<3, 3>(0, 0);
}

Eigen::Vector3d positionFromOTEE(const std::array<double, 16>& O_T_EE) {
  Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> T(O_T_EE.data());
  return T.block<3, 1>(0, 3);
}


bool saveHomePoseFile(const std::string& path,
                      const std::array<double, 7>& q) {
  std::ofstream out(path);
  if (!out) return false;
  out << std::setprecision(17);
  for (size_t i = 0; i < q.size(); ++i) {
    if (i) out << ' ';
    out << q[i];
  }
  out << '\n';
  return static_cast<bool>(out);
}

bool loadHomePoseFile(const std::string& path,
                      std::array<double, 7>& q) {
  std::ifstream in(path);
  if (!in) return false;
  for (double& v : q) {
    if (!(in >> v) || !std::isfinite(v)) return false;
  }
  return true;
}

bool validateHomePose(const std::array<double, 7>& q) {
  for (int i = 0; i < 7; ++i) {
    if (q[i] < Q_SOFT_MIN[i] || q[i] > Q_SOFT_MAX[i]) {
      std::cerr << "[HOME] joint " << (i + 1)
                << " target " << q[i]
                << " rad is outside the software-safe range ["
                << Q_SOFT_MIN[i] << ", " << Q_SOFT_MAX[i] << "]\n";
      return false;
    }
  }
  return true;
}

void moveRobotToHome(franka::Robot& robot,
                     const std::array<double, 7>& q_home) {
  const franka::RobotState initial_state = robot.readOnce();
  const std::array<double, 7> q_start = initial_state.q;

  double max_delta = 0.0;
  for (int i = 0; i < 7; ++i) {
    max_delta = std::max(max_delta, std::abs(q_home[i] - q_start[i]));
  }

  if (max_delta < 1e-3) {
    std::cout << "[HOME] already at home pose.\n";
    return;
  }

  // Smooth quintic interpolation:
  // s(r) = 10 r^3 - 15 r^4 + 6 r^5
  //
  // The duration is deliberately conservative for an automatic startup return.
  // max(ds/dt) = 1.875 / T, so the first term limits joint velocity.
  constexpr double kHomeJointVelocity = 0.18;      // rad/s
  constexpr double kHomeJointAcceleration = 0.45;  // rad/s^2

  const double t_vel = 1.875 * max_delta / kHomeJointVelocity;
  const double t_acc =
      std::sqrt(5.8 * max_delta / kHomeJointAcceleration);
  const double duration = std::max({4.0, t_vel, t_acc});

  std::cout << "[HOME] returning to saved home pose, duration ~"
            << std::fixed << std::setprecision(1)
            << duration << " s\n";

  double elapsed = 0.0;
  robot.control(
      [&](const franka::RobotState&,
          franka::Duration period) -> franka::JointPositions {
        elapsed += period.toSec();

        const double r = clamp(elapsed / duration, 0.0, 1.0);
        const double r2 = r * r;
        const double r3 = r2 * r;
        const double r4 = r3 * r;
        const double r5 = r4 * r;
        const double s = 10.0 * r3 - 15.0 * r4 + 6.0 * r5;

        std::array<double, 7> q_cmd{};
        for (int i = 0; i < 7; ++i) {
          q_cmd[i] = q_start[i] + (q_home[i] - q_start[i]) * s;
        }

        if (elapsed >= duration) {
          return franka::MotionFinished(franka::JointPositions(q_home));
        }
        return franka::JointPositions(q_cmd);
      });

  std::cout << "[HOME] home pose reached.\n";
}

void clearSharedCommand(SharedCommand& shared) {
  for (auto& x : shared.twist) {
    x.store(0.0, std::memory_order_relaxed);
  }
  shared.enabled.store(false, std::memory_order_relaxed);
  shared.quit.store(false, std::memory_order_relaxed);
  shared.last_rx_ns.store(0, std::memory_order_relaxed);
}

bool waitForNeutralInput(SharedCommand& shared,
                         double stable_seconds = 0.5) {
  std::cout << "[HOME] return the handle to neutral before teleoperation...\n";
  auto neutral_since = Clock::now();

  while (!g_stop.load(std::memory_order_relaxed)) {
    const int64_t last = shared.last_rx_ns.load(std::memory_order_acquire);
    const double age =
        (last == 0)
            ? std::numeric_limits<double>::infinity()
            : static_cast<double>(nowNs() - last) * 1e-9;

    const bool active =
        shared.enabled.load(std::memory_order_relaxed) && age <= 0.15;

    if (active) {
      neutral_since = Clock::now();
    } else {
      const double neutral_time =
          std::chrono::duration<double>(Clock::now() - neutral_since).count();
      if (neutral_time >= stable_seconds) {
        std::cout << "[HOME] input neutral confirmed. Teleoperation enabled.\n";
        return true;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

}  // namespace


int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr
        << "Usage:\n"
        << "  " << argv[0]
        << " <robot-hostname-or-ip> [control-udp-port] [telemetry-udp-port]"
        << " [--home-file PATH]\n"
        << "  " << argv[0]
        << " <robot-hostname-or-ip> --save-home [--home-file PATH]\n";
    return 1;
  }

  const std::string robot_host = argv[1];
  int udp_port = 9999;
  int telemetry_port = 10000;
  std::string home_file = "home_pose.txt";
  bool save_home = false;

  int positional_index = 0;
  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--save-home") {
      save_home = true;
    } else if (arg == "--home-file") {
      if (i + 1 >= argc) {
        std::cerr << "--home-file requires a path\n";
        return 1;
      }
      home_file = argv[++i];
    } else if (!arg.empty() && arg[0] == '-') {
      std::cerr << "Unknown option: " << arg << "\n";
      return 1;
    } else {
      if (positional_index == 0) {
        udp_port = std::stoi(arg);
      } else if (positional_index == 1) {
        telemetry_port = std::stoi(arg);
      } else {
        std::cerr << "Too many positional arguments.\n";
        return 1;
      }
      ++positional_index;
    }
  }

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // --save-home is intentionally a separate, non-teleoperation mode.
  // Manually place the robot at a verified collision-free startup pose once,
  // then save that exact joint configuration.
  if (save_home) {
    try {
      franka::Robot robot(robot_host);
      const franka::RobotState state = robot.readOnce();

      if (!validateHomePose(state.q)) {
        std::cerr << "[HOME] current pose is outside the configured software-safe range; not saved.\n";
        return 2;
      }

      if (!saveHomePoseFile(home_file, state.q)) {
        std::cerr << "[HOME] failed to write " << home_file << "\n";
        return 2;
      }

      std::cout << "[HOME] saved current joint pose to " << home_file << "\n";
      std::cout << "[HOME] q = [";
      for (int i = 0; i < 7; ++i) {
        if (i) std::cout << ", ";
        std::cout << state.q[i];
      }
      std::cout << "]\n";
      return 0;
    } catch (const franka::Exception& e) {
      std::cerr << "[FRANKA] exception while saving home: "
                << e.what() << "\n";
      return 2;
    }
  }

  std::array<double, 7> q_home{};
  if (!loadHomePoseFile(home_file, q_home)) {
    std::cerr << "[HOME] cannot read " << home_file << "\n"
              << "[HOME] First move the robot manually to your verified safe origin,\n"
              << "       then run:\n"
              << "       " << argv[0] << " " << robot_host
              << " --save-home --home-file " << home_file << "\n";
    return 2;
  }
  if (!validateHomePose(q_home)) {
    return 2;
  }

  SharedCommand shared;
  UdpReceiver receiver(udp_port, shared);
  std::thread udp_thread(&UdpReceiver::run, &receiver);

  SharedRobotTelemetry robot_telemetry;
  ForceTelemetryWorker force_worker(robot_telemetry, "127.0.0.1", telemetry_port);
  std::thread force_thread(&ForceTelemetryWorker::run, &force_worker);

  try {
    franka::Robot robot(robot_host);
    franka::Model model = robot.loadModel();

    std::cout << "[FRANKA] connected: " << robot_host << "\n";
    std::cout << "[FRAME] control point = configured EndEffector/TCP.\n";
    std::cout << "[INPUT] Active direction commands enable motion.\n";
    std::cout << "[SAFE] UDP watchdog = 150 ms; CTRL+C also stops.\n";

    // Every normal startup first returns to the user-saved, verified home pose.
    // Keep the physical emergency stop accessible during this automatic motion.
    moveRobotToHome(robot, q_home);

    // Ignore any stale command accumulated while homing. The operator/handle
    // must return to neutral before the Cartesian teleoperation loop starts.
    clearSharedCommand(shared);
    if (!waitForNeutralInput(shared)) {
      throw std::runtime_error("Stopped while waiting for neutral input.");
    }

    std::array<double, 7> dq_cmd{};
    dq_cmd.fill(0.0);

    bool initialized = false;
    Eigen::Vector3d p_start = Eigen::Vector3d::Zero();
    uint32_t telemetry_decimation = 0;

    // Relative Cartesian safety box around the TCP pose after automatic homing.
    // Bounds are expressed relative to the TCP position after homing.
    constexpr double WS_X = 0.20;  // +/- m in robot base X
    constexpr double WS_Y = 0.20;  // +/- m in robot base Y
    constexpr double WS_Z_DOWN = 0.12;
    constexpr double WS_Z_UP = 0.18;

    // If a commanded velocity would exit the workspace, remove the outward component.
    constexpr double WS_MARGIN = 0.015;  // m

    // DLS / redundancy parameters.
    constexpr double LAMBDA_MIN = 0.015;
    constexpr double LAMBDA_MAX = 0.12;
    constexpr double SIGMA_WARN = 0.12;
    constexpr double SIGMA_STOP = 0.035;
    constexpr double NULL_GAIN = 0.10;
    constexpr double SOFT_LIMIT_HORIZON = 0.60;  // s

    robot.control(
        [&](const franka::RobotState& state,
            franka::Duration period) -> franka::JointVelocities {
          const double dt = std::max(period.toSec(), 1e-6);

          if (!initialized) {
            const auto O_T_EE = model.pose(franka::Frame::kEndEffector, state);
            p_start = positionFromOTEE(O_T_EE);
            dq_cmd.fill(0.0);
            initialized = true;
          }

          bool command_active = false;
          bool quit_requested = false;
          Eigen::Matrix<double, 6, 1> V_E =
              loadToolTwist(shared, command_active, quit_requested);

          // Publish a tiny atomic snapshot at 200 Hz (every 5th 1 kHz cycle).
          // File I/O and contact detection stay outside the real-time callback.
          if (++telemetry_decimation >= 5) {
            telemetry_decimation = 0;
            publishRobotTelemetry(robot_telemetry, state, model, command_active, V_E);
          }

          // Input semantics are device-independent:
          // active=true  -> a direction command is active.
          // active=false -> all directions released OR UDP watchdog expired.
          //
          // When inactive, do not run Cartesian control or null-space centering.
          // Only decelerate the previous joint command smoothly to zero.
          if (!command_active) {
            double max_abs = 0.0;
            for (int i = 0; i < 7; ++i) {
              const double step = QDD_USER_MAX[i] * dt;
              dq_cmd[i] += clamp(-dq_cmd[i], -step, step);
              max_abs = std::max(max_abs, std::abs(dq_cmd[i]));
            }

            if ((quit_requested || g_stop.load(std::memory_order_relaxed)) &&
                max_abs < 1e-3) {
              std::array<double, 7> zero{};
              return franka::MotionFinished(franka::JointVelocities(zero));
            }
            return franka::JointVelocities(dq_cmd);
          }

          // Command coordinate conventions:
          // translation is defined directly in the robot base frame;
          // rotation is around the CURRENT configured end-effector/TCP axes.
          const auto O_T_EE = model.pose(franka::Frame::kEndEffector, state);
          const Eigen::Matrix3d R_BEE = rotationFromOTEE(O_T_EE);
          Eigen::Matrix<double, 6, 1> V_B;
          V_B.head<3>() = V_E.head<3>();
          V_B.tail<3>() = R_BEE * V_E.tail<3>();

          // Cartesian workspace guard in base frame, referenced to the configured end-effector/TCP.
          const Eigen::Vector3d p = positionFromOTEE(O_T_EE);
          const Eigen::Vector3d d = p - p_start;

          auto suppress_outward = [&](int axis, double low, double high) {
            const double rel = d(axis);
            double& v = V_B(axis);
            if (rel > high - WS_MARGIN && v > 0.0) {
              const double s = clamp((high - rel) / WS_MARGIN, 0.0, 1.0);
              v *= s;
            }
            if (rel < low + WS_MARGIN && v < 0.0) {
              const double s = clamp((rel - low) / WS_MARGIN, 0.0, 1.0);
              v *= s;
            }
            if ((rel >= high && v > 0.0) || (rel <= low && v < 0.0)) v = 0.0;
          };
          suppress_outward(0, -WS_X, WS_X);
          suppress_outward(1, -WS_Y, WS_Y);
          suppress_outward(2, -WS_Z_DOWN, WS_Z_UP);

          // Configured end-effector/TCP zero Jacobian, expressed in base frame.
          const std::array<double, 42> jac_array =
              model.zeroJacobian(franka::Frame::kEndEffector, state);
          Eigen::Map<const Eigen::Matrix<double, 6, 7, Eigen::ColMajor>> J(
              jac_array.data());

          // SVD for singularity metric.
          Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>> svd(J);
          const double sigma_min = svd.singularValues().minCoeff();

          // As singularity approaches, reduce requested Cartesian twist and
          // increase damping. At SIGMA_STOP, only null-space retreat remains.
          double singular_scale = 1.0;
          if (sigma_min < SIGMA_WARN) {
            singular_scale =
                clamp((sigma_min - SIGMA_STOP) / (SIGMA_WARN - SIGMA_STOP),
                      0.0, 1.0);
            V_B *= singular_scale;
          }

          const double lambda =
              LAMBDA_MIN +
              (1.0 - singular_scale) * (LAMBDA_MAX - LAMBDA_MIN);

          const Eigen::Matrix<double, 6, 6> A =
              J * J.transpose() +
              (lambda * lambda) * Eigen::Matrix<double, 6, 6>::Identity();

          const Eigen::Matrix<double, 7, 6> J_dls =
              J.transpose() * A.ldlt().solve(
                                  Eigen::Matrix<double, 6, 6>::Identity());

          Eigen::Matrix<double, 7, 1> dq_task = J_dls * V_B;

          // Null-space centering inside the conservative software range.
          Eigen::Matrix<double, 7, 1> dq_null;
          for (int i = 0; i < 7; ++i) {
            const double mid = 0.5 * (Q_SOFT_MIN[i] + Q_SOFT_MAX[i]);
            const double half =
                std::max(0.1, 0.5 * (Q_SOFT_MAX[i] - Q_SOFT_MIN[i]));
            const double e = (state.q[i] - mid) / half;
            dq_null(i) = clamp(-NULL_GAIN * e, -0.22, 0.22);
          }

          const Eigen::Matrix<double, 7, 7> N =
              Eigen::Matrix<double, 7, 7>::Identity() - J_dls * J;
          Eigen::Matrix<double, 7, 1> dq_des = dq_task + N * dq_null;

          // Joint safety bounds:
          // 1) hardware position-dependent velocity bounds
          // 2) conservative user speed cap
          // 3) soft-limit braking bounds
          for (int i = 0; i < 7; ++i) {
            auto [lo_hw, hi_hw] = positionDependentVelocityBounds(i, state.q[i]);
            double lo = std::max(lo_hw, -QD_USER_MAX[i]);
            double hi = std::min(hi_hw,  QD_USER_MAX[i]);

            const double lo_soft =
                (Q_SOFT_MIN[i] - state.q[i]) / SOFT_LIMIT_HORIZON;
            const double hi_soft =
                (Q_SOFT_MAX[i] - state.q[i]) / SOFT_LIMIT_HORIZON;

            lo = std::max(lo, lo_soft);
            hi = std::min(hi, hi_soft);

            // If already slightly outside our software range, only permit return motion.
            if (state.q[i] <= Q_SOFT_MIN[i]) lo = std::max(lo, 0.0);
            if (state.q[i] >= Q_SOFT_MAX[i]) hi = std::min(hi, 0.0);

            if (lo > hi) {
              // Degenerate safety state: command zero for this joint.
              dq_des(i) = 0.0;
            } else {
              dq_des(i) = clamp(dq_des(i), lo, hi);
            }

            // Application-level acceleration limiter.
            const double step = QDD_USER_MAX[i] * dt;
            dq_cmd[i] += clamp(dq_des(i) - dq_cmd[i], -step, step);
          }

          // Final finite check.
          for (double v : dq_cmd) {
            if (!std::isfinite(v)) {
              std::array<double, 7> zero{};
              return franka::MotionFinished(franka::JointVelocities(zero));
            }
          }

          if (quit_requested || g_stop.load(std::memory_order_relaxed)) {
            // First ramp toward zero. Finish only when nearly stopped.
            double max_abs = 0.0;
            for (int i = 0; i < 7; ++i) {
              const double step = QDD_USER_MAX[i] * dt;
              dq_cmd[i] += clamp(-dq_cmd[i], -step, step);
              max_abs = std::max(max_abs, std::abs(dq_cmd[i]));
            }
            if (max_abs < 1e-3) {
              std::array<double, 7> zero{};
              return franka::MotionFinished(franka::JointVelocities(zero));
            }
          }

          return franka::JointVelocities(dq_cmd);
        });

    std::cout << "[FRANKA] control loop finished safely.\n";

  } catch (const franka::Exception& e) {
    std::cerr << "[FRANKA] exception: " << e.what() << "\n";
    g_stop.store(true);
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] " << e.what() << "\n";
    g_stop.store(true);
  }

  g_stop.store(true);
  if (udp_thread.joinable()) udp_thread.join();
  if (force_thread.joinable()) force_thread.join();
  return 0;
}
