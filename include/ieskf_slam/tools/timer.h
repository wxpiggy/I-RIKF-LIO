#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

// 程序启动时清空一次日志文件
struct TimerLoggerInit {
    TimerLoggerInit(const std::string& log_file) {
        std::ofstream(log_file).close();  // 覆盖模式打开再关 → 清空文件
    }
};

// 定时器模板
template <typename Func>
void Timer(Func&& func, const std::string& task_name = "Task", const std::string& log_file = "time_log.txt") {
    auto start = std::chrono::steady_clock::now();
    func();
    auto end = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // 控制台输出
    std::cout << task_name << " took " << elapsed_ms << " ms\n";

    // 文件追加
    std::ofstream ofs(log_file, std::ios::app);
    if (ofs.is_open()) {
        ofs << task_name << " took " << elapsed_ms << " ms\n";
    }
}