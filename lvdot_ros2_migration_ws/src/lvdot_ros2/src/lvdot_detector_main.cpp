#include <csignal>
#include <cstring>
#include <execinfo.h>
#include <memory>
#include <unistd.h>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lvdot_ros2/lvdot_detector_node.hpp"

namespace
{
void crash_handler(int sig)
{
  void * frames[128];
  int n = backtrace(frames, 128);
  char header[160];
  int hlen = std::snprintf(
    header, sizeof(header),
    "\n=== FATAL SIGNAL %d received, backtrace (depth=%d) ===\n", sig, n);
  if (hlen > 0) {
    (void)write(STDERR_FILENO, header, static_cast<size_t>(hlen));
  }
  backtrace_symbols_fd(frames, n, STDERR_FILENO);
  static const char tail[] = "=== end backtrace ===\n";
  (void)write(STDERR_FILENO, tail, sizeof(tail) - 1);
  std::signal(sig, SIG_DFL);
  std::raise(sig);
}

void install_crash_handlers()
{
  std::signal(SIGSEGV, crash_handler);
  std::signal(SIGABRT, crash_handler);
  std::signal(SIGFPE, crash_handler);
  std::signal(SIGILL, crash_handler);
  std::signal(SIGBUS, crash_handler);
}
}  // namespace

int main(int argc, char ** argv)
{
  install_crash_handlers();

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<lvdot_ros2::LVdotDetectorNode>(options);
  const auto executor_threads = node->executor_threads();
  RCLCPP_INFO(
    node->get_logger(),
    "Starting executor with executor_threads=%d",
    executor_threads);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), executor_threads);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
