#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <ncurses.h>
#include <locale.h>
#include <csignal>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <string>
#include <cstring>
#include <chrono>
#include <sstream>
#include <cctype>

using namespace std::chrono_literals;

/* ========= 設定 ========= */
static const size_t MAX_LOG_LINES = 1000;
static const size_t MAX_INPUT_LEN = 512;

/* ========= グローバル状態 ========= */
static std::atomic_bool running{true};
static std::atomic_bool ui_dirty{true};

// ユーザー設定（CLI引数やコマンドで更新）
static std::string g_username = "anon";
static std::string g_topic    = "chat_room";
static bool   g_qos_reliable        = true;
static bool   g_qos_transient_local = false;
static size_t g_qos_depth           = 50;

/* ========= ログ ========= */
static std::vector<std::string> log_lines;
static std::mutex log_mtx;

static void add_log(const std::string &line) {
  std::lock_guard<std::mutex> lk(log_mtx);
  if (log_lines.size() >= MAX_LOG_LINES) {
    log_lines.erase(log_lines.begin());
  }
  log_lines.push_back(line);
}

/* ========= ncurses UI ========= */
static WINDOW *win_status=nullptr, *win_log=nullptr, *win_input=nullptr;

static void ui_destroy() {
  if (win_status) { delwin(win_status); win_status=nullptr; }
  if (win_log)    { delwin(win_log);    win_log=nullptr; }
  if (win_input)  { delwin(win_input);  win_input=nullptr; }
}

static void ui_layout() {
  int rows, cols; getmaxyx(stdscr, rows, cols);
  int status_h = 1;
  int input_h  = 3;
  int log_h    = rows - status_h - input_h;
  if (log_h < 3) log_h = 3;

  ui_destroy();
  win_status = newwin(status_h, cols, 0, 0);
  win_log    = newwin(log_h,   cols, status_h, 0);
  win_input  = newwin(input_h, cols, status_h + log_h, 0);

  keypad(win_input, TRUE);
  nodelay(win_input, TRUE);
}

static void ui_draw_status() {
  werase(win_status);
  wattron(win_status, A_REVERSE);
  mvwprintw(
    win_status, 0, 0,
    " Chat | user:%s | topic:/%s | QoS:%s%s depth=%zu | Ctrl+C to quit ",
    g_username.c_str(), g_topic.c_str(),
    g_qos_reliable ? "Reliable" : "BestEffort",
    g_qos_transient_local ? "+TransientLocal" : "",
    g_qos_depth
  );
  wattroff(win_status, A_REVERSE);
  wrefresh(win_status);
}

static void ui_draw_log() {
  werase(win_log);
  box(win_log, 0, 0);

  int maxy = getmaxy(win_log);
  int maxx = getmaxx(win_log);
  int inner_lines = maxy - 2;
  if (inner_lines < 1) inner_lines = 1;

  std::lock_guard<std::mutex> lk(log_mtx);
  int start = 0;
  if ((int)log_lines.size() > inner_lines)
    start = (int)log_lines.size() - inner_lines;

  int y = 1;
  for (int i = start; i < (int)log_lines.size(); ++i) {
    std::string s = log_lines[i];
    if ((int)s.size() > maxx - 2) s.resize(maxx - 3);
    mvwprintw(win_log, y++, 1, "%s", s.c_str());
    if (y >= maxy-1) break;
  }
  wrefresh(win_log);
}

static void ui_draw_input(const std::string &line) {
  werase(win_input);
  box(win_input, 0, 0);
  mvwprintw(win_input, 1, 1, "> %s", line.c_str());
  int cursor_x = 3 + (int)line.size();
  if (cursor_x >= getmaxx(win_input)-1) cursor_x = getmaxx(win_input)-2;
  wmove(win_input, 1, cursor_x);
  wrefresh(win_input);
}

static void ui_redraw_all(const std::string &line) {
  ui_layout();
  ui_draw_status();
  ui_draw_log();
  ui_draw_input(line);
}

/* ========= ユーティリティ ========= */
static std::string now_hms() {
  auto t = std::time(nullptr);
  char ts[16];
  std::strftime(ts, sizeof(ts), "%H:%M:%S", std::localtime(&t));
  return std::string(ts);
}

/* ========= シグナル ========= */
static void sigint_handler(int) { running = false; }

/* ========= QoS 構築 ========= */
static rmw_qos_profile_t build_qos_raw() {
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos.depth = g_qos_depth;
  qos.reliability = g_qos_reliable
    ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
    : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  qos.durability = g_qos_transient_local
    ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
    : RMW_QOS_POLICY_DURABILITY_VOLATILE;
  return qos;
}

/* ========= メイン ========= */
int main(int argc, char **argv) {
  setlocale(LC_ALL, "");

  // 引数処理
  for (int i=1; i<argc; ++i) {
    if      (!std::strcmp(argv[i], "--user") && i+1<argc) g_username = argv[++i];
    else if (!std::strcmp(argv[i], "--topic") && i+1<argc) g_topic = argv[++i];
    else if (!std::strcmp(argv[i], "--reliable"))          g_qos_reliable = true;
    else if (!std::strcmp(argv[i], "--best-effort"))       g_qos_reliable = false;
    else if (!std::strcmp(argv[i], "--transient-local"))   g_qos_transient_local = true;
    else if (!std::strcmp(argv[i], "--volatile"))          g_qos_transient_local = false;
    else if (!std::strcmp(argv[i], "--depth") && i+1<argc) g_qos_depth = std::stoul(argv[++i]);
  }

  // ncurses 初期化（RAIIガード）
  struct NcursesGuard {
    NcursesGuard() {
      initscr();
      cbreak();
      noecho();
      keypad(stdscr, TRUE);
      start_color();
      use_default_colors();
      curs_set(1);
      ui_layout();
    }
    ~NcursesGuard() {
      ui_destroy();
      endwin();
      echo();
      curs_set(1);
    }
  } guard;

  ui_redraw_all("");
  add_log("[" + now_hms() + "] Welcome to ROS2 FastDDS Chat (rclcpp). Type /help");
  ui_draw_log();

  std::signal(SIGINT, sigint_handler);

  // ROS2 init
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("chat_cli");

  // QoS
  auto qos_raw = build_qos_raw();
  rclcpp::QoS qos_cpp(rclcpp::QoSInitialization::from_rmw(qos_raw));
  qos_cpp.reliability(
    g_qos_reliable ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                   : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_cpp.durability(
    g_qos_transient_local ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
                          : RMW_QOS_POLICY_DURABILITY_VOLATILE);

  // Pub/Sub
  auto pub = node->create_publisher<std_msgs::msg::String>(g_topic, qos_cpp);

  auto sub = node->create_subscription<std_msgs::msg::String>(
    g_topic, qos_cpp,
    [&](std_msgs::msg::String::SharedPtr msg){
      const std::string prefix = g_username + ": ";
      if (msg->data.rfind(prefix, 0) == 0) {
        ui_dirty.store(true);
        return;
      }
      add_log("[" + now_hms() + "] " + msg->data);
      ui_dirty.store(true);
    }
  );

  // Executor は別スレッド
  std::thread spin_th([&]{
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    while (running && rclcpp::ok()) {
      exec.spin_some(50ms);
      std::this_thread::sleep_for(50ms);
    }
  });

  /* ===== 入力ループ（非ブロッキング） ===== */
  std::string line;
  int ch = 0;

#ifdef KEY_RESIZE
  bool need_relayout = false;
#endif

  while (running) {
    ch = wgetch(win_input);
    if (ch != ERR) {
#ifdef KEY_RESIZE
      if (ch == KEY_RESIZE) {
        need_relayout = true;
        ui_dirty.store(true);
        continue;
      }
#endif
      if (ch == '\n' || ch == '\r') {
        std::string in = line;
        line.clear();

        if (!in.empty() && in[0] == '/') {
          // コマンド処理
          if (in == "/help") {
            add_log("[sys] Commands: /help, /whoami NAME, /topic NAME (restart), /qos rel|best (restart), /dur trans|volatile (restart), /depth N (restart)");
          } else if (in.rfind("/whoami ", 0) == 0) {
            g_username = in.substr(8);
            add_log("[sys] username updated: " + g_username);
          } else if (in.rfind("/topic ", 0) == 0) {
            g_topic = in.substr(7);
            add_log("[sys] topic updated (restart to apply): " + g_topic);
          } else if (in.rfind("/qos ", 0) == 0) {
            if (in.find("rel") != std::string::npos)  { g_qos_reliable = true;  add_log("[sys] QoS set: Reliable (restart)"); }
            if (in.find("best")!= std::string::npos)  { g_qos_reliable = false; add_log("[sys] QoS set: BestEffort (restart)"); }
          } else if (in.rfind("/dur ", 0) == 0) {
            if (in.find("trans") != std::string::npos){ g_qos_transient_local = true;  add_log("[sys] Durability: TransientLocal (restart)"); }
            if (in.find("volatile")!=std::string::npos){ g_qos_transient_local = false; add_log("[sys] Durability: Volatile (restart)"); }
          } else if (in.rfind("/depth ", 0) == 0) {
            try {
              g_qos_depth = std::stoul(in.substr(7));
              add_log("[sys] depth set (restart): " + std::to_string(g_qos_depth));
            } catch (...) {
              add_log("[sys] invalid depth");
            }
          } else {
            add_log("[sys] unknown command. /help");
          }
          ui_dirty.store(true);
        }
        else if (!in.empty()) {
          // Publish（送信側も即ログ表示する）
          std_msgs::msg::String m;
          m.data = g_username + ": " + in;
          try {
            pub->publish(m);
            add_log("[" + now_hms() + "] " + m.data);
          } catch (const std::exception &e) {
            add_log(std::string("[sys] publish failed: ") + e.what());
          }
          ui_dirty.store(true);
        }
      }
      else if (ch == KEY_BACKSPACE || ch == 127 || ch == 8) {
        if (!line.empty()) line.pop_back();
        ui_dirty.store(true);
      }
      else if (isprint(ch)) {
        if (line.size() < MAX_INPUT_LEN-1) line.push_back((char)ch);
        ui_dirty.store(true);
      }
    }

    // 再描画
    if (ui_dirty.exchange(false)) {
#ifdef KEY_RESIZE
      if (need_relayout) {
        ui_redraw_all(line);
        need_relayout = false;
      } else {
        ui_draw_status();
        ui_draw_log();
        ui_draw_input(line);
      }
#else
      ui_draw_status();
      ui_draw_log();
      ui_draw_input(line);
#endif
    }

    std::this_thread::sleep_for(20ms);
  }

  rclcpp::shutdown();
  if (spin_th.joinable()) spin_th.join();

  return 0;
}
