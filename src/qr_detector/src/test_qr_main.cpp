#include <chrono>
#include <thread>
#include <mutex>
#include <signal.h>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <zlib.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <functional>

using json = nlohmann::json;

// 全局 flag，Ctrl-C 退出
static volatile sig_atomic_t g_running = 1;

void signal_handler(int) { g_running = 0; }

// ---------- 时间戳生成 ----------
static std::string timestamp_filename() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    std::tm tm_buf;
    localtime_r(&t, &tm_buf);
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
        << "_" << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

// ---------- 保存 JSON ----------
static bool save_json(const std::string& path, const std::string& qr,
                      const std::string& camera_topic, const std::string& file) {
    json j;
    j["timestamp"]    = timestamp_filename();
    j["qr_data"]      = qr;
    j["camera_topic"] = camera_topic;
    j["saved_file"]   = file;
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;
    ofs << j.dump(2);
    return ofs.good();
}

// ---------- 主函数 ----------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_qr_scanner");

    // ---- 参数 ----
    node->declare_parameter("save_path",    "/home/sunrise/wheel_S200_src/src/qr_detector/qr_results");
    node->declare_parameter("camera_topic", "/camera_dcw2/color/image_raw");
    node->declare_parameter("timeout_sec",  30.0);          // 扫描窗口（秒）
    node->declare_parameter("retry_rate",   4.0);            // 每秒识别几次

    std::string save_path    = node->get_parameter("save_path").as_string();
    std::string camera_topic = node->get_parameter("camera_topic").as_string();
    double timeout_sec       = node->get_parameter("timeout_sec").as_double();
    double retry_rate        = node->get_parameter("retry_rate").as_double();

    // 确保目录存在
    std::error_code ec;
    std::filesystem::create_directories(save_path, ec);

    // ---- 共享数据 ----
    cv::Mat latest_image;
    bool image_received = false;
    std::mutex image_mutex;

    // ---- 图像订阅 (QoS depth=1) ----
    int frame_total = 0;
    auto sub = node->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, 1,
        [&](const sensor_msgs::msg::Image::SharedPtr msg) {
            try {
                frame_total++;
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
                std::lock_guard<std::mutex> lock(image_mutex);
                // 统一转灰度，适配 rgb8/bgr8/mono8 等任意编码
                if (cv_ptr->image.channels() == 3) {
                    cv::cvtColor(cv_ptr->image, latest_image, cv::COLOR_BGR2GRAY);
                } else if (cv_ptr->image.channels() == 4) {
                    cv::cvtColor(cv_ptr->image, latest_image, cv::COLOR_BGRA2GRAY);
                } else if (cv_ptr->image.channels() == 1) {
                    latest_image = cv_ptr->image.clone();
                } else {
                    return;  // 不支持的通道数
                }
                image_received = true;

                // ---- 调试步骤 1：确认图像真的到了 ----
                if (frame_total % 30 == 1) {  // 每 30 帧保存一次预览
                    std::string debug_path = save_path + "/debug_last_frame.jpg";
                    cv::imwrite(debug_path, latest_image);
                    RCLCPP_INFO(node->get_logger(),
                        "📷 收到第 %d 帧, 尺寸 %dx%d, 已存 %s",
                        frame_total, latest_image.cols, latest_image.rows,
                        debug_path.c_str());
                }
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_ERROR(node->get_logger(), "cv_bridge 转换失败: %s", e.what());
            }
        });

    // ---- 发布 /qr_detected 供 inspection_master 订阅 ----
    auto qr_pub = node->create_publisher<std_msgs::msg::String>("/qr_detected", 10);

    // ---- ZBar 扫描器（初始化一次，复用） ----
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    signal(SIGINT, signal_handler);

    auto start = node->now();
    auto loop_duration = std::chrono::milliseconds(
        static_cast<int>(1000.0 / retry_rate));

    RCLCPP_INFO(node->get_logger(),
        "开始扫描 [topic=%s, timeout=%.0fs, rate=%.1fHz]",
        camera_topic.c_str(), timeout_sec, retry_rate);
    RCLCPP_INFO(node->get_logger(), "保存目录: %s", save_path.c_str());

    bool success = false;
    std::string found_qr;
    std::string json_path;
    int frame_processed = 0;

    while (g_running && rclcpp::ok()) {
        rclcpp::spin_some(node);

        // 超时检查
        if ((node->now() - start).seconds() > timeout_sec) {
            RCLCPP_WARN(node->get_logger(),
                "⏰ 扫描超时 (%.0fs), 共收到 %d 帧, 处理 %d 帧",
                timeout_sec, frame_total, frame_processed);
            break;
        }

        // 取图
        cv::Mat img;
        bool got = false;
        {
            std::lock_guard<std::mutex> lock(image_mutex);
            if (image_received && !latest_image.empty()) {
                img = latest_image.clone();
                image_received = false;
                got = true;
            }
        }

        if (!got) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        frame_processed++;

        // ---- 调试步骤 2：保存即将送给 ZBar 的图像 ----
        if (frame_processed <= 3 || frame_processed % 10 == 0) {
            std::string zbar_dbg = save_path + "/debug_for_zbar_" +
                                   std::to_string(frame_processed) + ".jpg";
            cv::imwrite(zbar_dbg, img);
            RCLCPP_INFO(node->get_logger(),
                "🔍 尝试识别第 %d 帧, 尺寸 %dx%d, 已存 %s",
                frame_processed, img.cols, img.rows, zbar_dbg.c_str());
        }

        // ZBar 识别
        zbar::Image zbar_img(img.cols, img.rows, "Y800",
                             static_cast<const void*>(img.data),
                             img.cols * img.rows);
        scanner.scan(zbar_img);

        std::string result;
        int symbol_count = 0;
        for (auto s = zbar_img.symbol_begin(); s != zbar_img.symbol_end(); ++s) {
            result = s->get_data();
            symbol_count++;
            break;
        }
        zbar_img.set_data(nullptr, 0);

        // ---- 调试步骤 3：输出 ZBar 扫描统计 ----
        RCLCPP_INFO(node->get_logger(),
            "    → ZBar 发现 %d 个符号, 结果=\"%s\"",
            symbol_count, result.empty() ? "(空)" : result.c_str());

        // 新增：判断是否为文件路径模式（以 '/' 开头）
        if (!result.empty() && result.front() == '/') {
            found_qr = result;
            // 直接发布路径，跳过 base64/gzip 解码和中间文件保存
            auto qr_msg = std_msgs::msg::String();
            qr_msg.data = found_qr;
            qr_pub->publish(qr_msg);
            RCLCPP_INFO(node->get_logger(), "✅ 识别到文件路径: [%s]", found_qr.c_str());
            RCLCPP_INFO(node->get_logger(), "    📤 已发布 /qr_detected: %s", found_qr.c_str());
            success = true;
            break;
        }

        if (result.empty()) {
            std::this_thread::sleep_for(loop_duration);
            continue;
        }

        // ---- 识别成功 ----
        found_qr = result;
        std::string ts = timestamp_filename();  // 统一时间戳，消除毫秒级差异

        // 尝试 base64 + gzip 解码（仅当内容含 base64 特征字符时才尝试）
        {
            // ---- 预检：base64 有效性判断 ----
            // 纯 JSON 以 { 或 [ 开头，短数据或可读文本直接跳过
            bool looks_like_b64 = (result.size() > 20 && !result.empty()
                && result.front() != '{' && result.front() != '[');
            if (looks_like_b64) {
                // ---- base64 解码 ----
                static const std::string b64 =
                    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
                std::string compressed;
                int bi = 0, bits = 0;
                for (char c : result) {
                    auto p = b64.find(c);
                    if (p == std::string::npos) { if (c == '=') break; continue; }
                    bits = (bits << 6) | static_cast<int>(p);
                    bi += 6;
                    if (bi >= 8) { bi -= 8; compressed += static_cast<char>((bits >> bi) & 0xFF); }
                }
                // ---- gzip 解压 ----
                if (!compressed.empty()) {
                    z_stream strm = {};
                    if (inflateInit2(&strm, 16 + MAX_WBITS) == Z_OK) {
                        // const 安全：数据仅用于只读解压
                        strm.next_in = reinterpret_cast<Bytef*>(
                            const_cast<char*>(compressed.data()));
                        strm.avail_in = compressed.size();
                        char out[4096];
                        std::string decompressed;
                        int ret;
                        do {
                            strm.next_out = reinterpret_cast<Bytef*>(out);
                            strm.avail_out = sizeof(out);
                            ret = inflate(&strm, Z_NO_FLUSH);
                            if (ret == Z_STREAM_ERROR || ret == Z_DATA_ERROR) break;
                            decompressed.append(out, sizeof(out) - strm.avail_out);
                        } while (ret != Z_STREAM_END);
                        inflateEnd(&strm);
                        // 验证解压结果是有效 JSON，防止随机字节误触发
                        if (ret == Z_STREAM_END && !decompressed.empty()
                            && (decompressed.front() == '{' || decompressed.front() == '[')) {
                            RCLCPP_INFO(node->get_logger(),
                                "🔓 gzip 解压成功: %zu → %zu 字节",
                                result.size(), decompressed.size());
                            found_qr = decompressed;
                        }
                    }
                }
            }
        }

        json_path = save_path + "/qr_" + ts + ".json";
        if (save_json(json_path, found_qr, camera_topic, json_path)) {
            RCLCPP_INFO(node->get_logger(), "✅ 识别成功: [%s]", found_qr.c_str());
            RCLCPP_INFO(node->get_logger(), "    JSON: %s", json_path.c_str());
            // 发布到 /qr_detected，供 inspection_master 订阅
            auto qr_msg = std_msgs::msg::String();
            qr_msg.data = json_path;
            qr_pub->publish(qr_msg);
            RCLCPP_INFO(node->get_logger(), "    📤 已发布 /qr_detected: %s", json_path.c_str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "JSON 保存失败: %s", json_path.c_str());
        }
        success = true;
        break;
    }

    if (success) {
        RCLCPP_INFO(node->get_logger(), "最终状态: SUCCESS");
    } else {
        RCLCPP_WARN(node->get_logger(), "最终状态: FAILURE (未识别到二维码)");
    }

    rclcpp::shutdown();
    return success ? 0 : 1;
}