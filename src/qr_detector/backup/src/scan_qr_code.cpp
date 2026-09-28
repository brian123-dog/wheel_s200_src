#include "qr_detector/scan_qr_code.hpp"

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>
#include <ctime>

#include <cv_bridge/cv_bridge.h>
#include <nlohmann/json.hpp>
#include <zbar.h>
#include <zlib.h>

using json = nlohmann::json;

namespace qr_detector {

// ==================== 构造 / 端口 ====================

ScanQRCode::ScanQRCode(const std::string& name, const BT::NodeConfiguration& cfg)
    : BT::StatefulActionNode(name, cfg)
{
    if (!config().blackboard->get("node", node_)) {
        throw BT::RuntimeError("ScanQRCode: 无法从 Blackboard 获取 ROS2 节点句柄");
    }
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    image_received_ = false;
    retry_count_ = 0;
}

BT::PortsList ScanQRCode::providedPorts() {
    return {
        BT::InputPort<std::string>("save_path",    "/home/sunrise/wheel_S200_src/src/qr_detector/qr_results", "二维码 JSON 保存目录"),
        BT::InputPort<std::string>("camera_topic", "/camera_dcw2/color/image_raw",  "相机图像话题"),
        BT::InputPort<double>("timeout_sec",       5.0,                        "等待图像超时秒数"),
        BT::OutputPort<std::string>("qr_data",                                   "识别到的二维码内容"),
        BT::OutputPort<std::string>("qr_file",                                   "保存的 JSON 绝对路径"),
    };
}

// ==================== 生命周期 ====================

BT::NodeStatus ScanQRCode::onStart() {
    // 1. 读取输入端口 (BT.CPP 4.x: getInput<T> 返回 Expected<T>)
    auto res_save = getInput<std::string>("save_path");
    if (!res_save) throw BT::RuntimeError("ScanQRCode: 缺少必要输入 [save_path]");
    save_path_ = res_save.value();

    auto res_topic = getInput<std::string>("camera_topic");
    if (!res_topic) throw BT::RuntimeError("ScanQRCode: 缺少必要输入 [camera_topic]");
    camera_topic_ = res_topic.value();

    auto res_timeout = getInput<double>("timeout_sec");
    if (!res_timeout) throw BT::RuntimeError("ScanQRCode: 缺少必要输入 [timeout_sec]");
    timeout_sec_ = res_timeout.value();

    // 2. 确保保存目录存在
    std::error_code ec;
    std::filesystem::create_directories(save_path_, ec);
    if (ec) {
        RCLCPP_ERROR(node_->get_logger(),
            "ScanQRCode: 无法创建保存目录 %s: %s",
            save_path_.c_str(), ec.message().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // 3. 创建图像订阅 (QoS depth=1，防止图像队列堆积)
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        camera_topic_,
        1,
        std::bind(&ScanQRCode::imageCallback, this, std::placeholders::_1)
    );

    // 4. 创建 /qr_detected 发布者（供 Python 主控节点订阅）
    qr_pub_ = node_->create_publisher<std_msgs::msg::String>("/qr_detected", 10);

    // 4. 初始化运行状态
    start_time_     = node_->now();
    image_received_ = false;
    retry_count_    = 0;

    RCLCPP_INFO(node_->get_logger(),
        "ScanQRCode: 开始扫描 [topic=%s, timeout=%.1fs]",
        camera_topic_.c_str(), timeout_sec_);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ScanQRCode::onRunning() {
    // 1. 超时检查
    double elapsed = (node_->now() - start_time_).seconds();
    if (elapsed > timeout_sec_) {
        image_sub_.reset();
        RCLCPP_WARN(node_->get_logger(),
            "ScanQRCode: 超时 (%.2fs)，扫描失败", elapsed);
        return BT::NodeStatus::FAILURE;
    }

    // 2. 等待图像到达
    if (!image_received_) {
        return BT::NodeStatus::RUNNING;
    }

    // 3. 拷贝图像 (缩锁区间)
    cv::Mat local_image;
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        local_image = latest_image_.clone();
        image_received_ = false;
    }

    if (local_image.empty()) {
        return BT::NodeStatus::RUNNING;
    }

    // 4. ZBar 识别 (在锁外执行，避免阻塞图像回调)
    detected_qr_ = detectQRCode(local_image);

    if (!detected_qr_.empty()) {
        // ---- 尝试 base64 + gzip 解码 ----
        std::string decoded = decodeBase64Gzip(detected_qr_);
        if (!decoded.empty()) {
            RCLCPP_INFO(node_->get_logger(),
                "ScanQRCode: gzip 解压成功, 原始长度=%zu → 解压长度=%zu",
                detected_qr_.size(), decoded.size());
            detected_qr_ = decoded;
        }

        // ---- 识别成功 ----
        // 统一生成一次时间戳，保证文件名与 JSON 内容一致
        std::string ts = generateTimestampFilename();
        saved_file_ = save_path_ + "/qr_" + ts + ".json";
        if (!saveJsonToFile()) {
            RCLCPP_ERROR(node_->get_logger(),
                "ScanQRCode: JSON 保存失败: %s", saved_file_.c_str());
            saved_file_.clear();  // 清空路径，防止下游节点读到不存在的文件
        }
        setOutput("qr_data", detected_qr_);
        setOutput("qr_file", saved_file_);
        image_sub_.reset();

        // 新增：发布 /qr_detected Topic，供 Python 主控节点订阅
        auto msg = std_msgs::msg::String();
        msg.data = saved_file_;  // 消息内容为 JSON 文件的绝对路径
        qr_pub_->publish(msg);

        RCLCPP_INFO(node_->get_logger(),
            "ScanQRCode: 识别成功 [%s] → %s, 已发布到 /qr_detected",
            detected_qr_.c_str(), saved_file_.c_str());

        return BT::NodeStatus::SUCCESS;
    }

    // 5. 识别失败：重试
    retry_count_++;
    RCLCPP_DEBUG(node_->get_logger(),
        "ScanQRCode: 未识别到二维码 (重试 %d/%d)",
        retry_count_, MAX_RETRY);

    if (retry_count_ >= MAX_RETRY) {
        image_sub_.reset();
        RCLCPP_WARN(node_->get_logger(),
            "ScanQRCode: 超过最大重试次数，扫描失败");
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void ScanQRCode::onHalted() {
    image_sub_.reset();
    RCLCPP_INFO(node_->get_logger(), "ScanQRCode: 被中断");
}

// ==================== 图像回调 ====================

void ScanQRCode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
        std::lock_guard<std::mutex> lock(image_mutex_);
        // 自动匹配原始编码转灰度，适配 rgb8/bgr8/mono8/RGBA 等任意编码
        if (cv_ptr->image.channels() == 3) {
            cv::cvtColor(cv_ptr->image, latest_image_, cv::COLOR_BGR2GRAY);
        } else if (cv_ptr->image.channels() == 4) {
            cv::cvtColor(cv_ptr->image, latest_image_, cv::COLOR_BGRA2GRAY);
        } else if (cv_ptr->image.channels() == 1) {
            latest_image_ = cv_ptr->image.clone();
        } else {
            // 不支持的通道数，不更新 image_received_
            RCLCPP_WARN_ONCE(node_->get_logger(),
                "ScanQRCode: 不支持的图像通道数: %d", cv_ptr->image.channels());
            return;
        }
        image_received_ = true;
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
            "ScanQRCode: cv_bridge 转换失败: %s", e.what());
    }
}

// ==================== 二维码识别 ====================

std::string ScanQRCode::detectQRCode(const cv::Mat& image) {
    if (image.empty()) return "";

    // ZBar Image 不会拷贝数据，仅包装指针，image 必须在此期间有效
    zbar::Image zbar_image(
        image.cols, image.rows, "Y800",
        static_cast<const void*>(image.data),
        image.cols * image.rows
    );

    scanner_.scan(zbar_image);

    std::string result;
    for (auto symbol = zbar_image.symbol_begin();
         symbol != zbar_image.symbol_end(); ++symbol) {
        result = symbol->get_data();
        break;  // 只取第一个二维码
    }

    // 断开 ZBar 对 Mat data 的引用，避免双重释放
    zbar_image.set_data(nullptr, 0);

    return result;
}

// ==================== JSON 保存 ====================

bool ScanQRCode::saveJsonToFile() {
    json j;
    j["timestamp"]    = generateTimestampFilename();
    j["qr_data"]      = detected_qr_;
    j["camera_topic"] = camera_topic_;
    j["saved_file"]   = saved_file_;
    j["robot_pose"]   = nullptr;  // 可选扩展：从 Blackboard 获取 pose

    std::ofstream ofs(saved_file_);
    if (!ofs.is_open()) return false;

    ofs << j.dump(2);
    return ofs.good();
}

std::string ScanQRCode::generateTimestampFilename() const {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::ostringstream oss;
    std::tm tm_buf;
    localtime_r(&time_t_now, &tm_buf);
    oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
        << "_" << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

// ==================== base64 + gzip 解码 ====================

// ---- base64 解码表 ----
static const std::string BASE64_CHARS =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

static bool is_base64(unsigned char c) {
    return (isalnum(c) || (c == '+') || (c == '/'));
}

static std::string base64_decode(const std::string& encoded) {
    int in_len = static_cast<int>(encoded.size());
    int i = 0, j = 0, in_ = 0;
    unsigned char char_array_4[4] = {}, char_array_3[3] = {};
    std::string ret;

    while (in_len-- && (encoded[in_] != '=') && is_base64(encoded[in_])) {
        char_array_4[i++] = encoded[in_];
        in_++;
        if (i == 4) {
            for (i = 0; i < 4; i++) {
                char_array_4[i] = static_cast<unsigned char>(
                    BASE64_CHARS.find(char_array_4[i]));
            }
            char_array_3[0] = (char_array_4[0] << 2)
                + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0x0F) << 4)
                + ((char_array_4[2] & 0x3C) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x03) << 6)
                + char_array_4[3];
            for (i = 0; i < 3; i++) {
                ret += char_array_3[i];
            }
            i = 0;
        }
    }
    if (i) {
        for (j = 0; j < i; j++) {
            char_array_4[j] = static_cast<unsigned char>(
                BASE64_CHARS.find(char_array_4[j]));
        }
        for (j = i; j < 4; j++) {
            char_array_4[j] = 0;  // 填充零，消除 UB
        }
        char_array_3[0] = (char_array_4[0] << 2)
            + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0x0F) << 4)
            + ((char_array_4[2] & 0x3C) >> 2);
        for (j = 0; j < i - 1; j++) {
            ret += char_array_3[j];
        }
    }
    return ret;
}

static std::string gzip_decompress(const std::string& compressed) {
    if (compressed.empty()) return "";

    z_stream strm = {};
    // 16 + MAX_WBITS → gzip 格式; 32 + MAX_WBITS → 自动检测 zlib/gzip
    if (inflateInit2(&strm, 16 + MAX_WBITS) != Z_OK) {
        return "";
    }

    strm.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(compressed.data()));
    strm.avail_in = static_cast<uInt>(compressed.size());

    constexpr size_t CHUNK = 4096;
    char out[CHUNK];
    std::string result;

    int ret;
    do {
        strm.next_out = reinterpret_cast<Bytef*>(out);
        strm.avail_out = CHUNK;
        ret = inflate(&strm, Z_NO_FLUSH);
        if (ret == Z_STREAM_ERROR || ret == Z_DATA_ERROR || ret == Z_MEM_ERROR) {
            inflateEnd(&strm);
            return "";
        }
        result.append(out, CHUNK - strm.avail_out);
    } while (ret != Z_STREAM_END);

    inflateEnd(&strm);
    return result;
}

std::string ScanQRCode::decodeBase64Gzip(const std::string& encoded) {
    // 步骤 0: 预检——纯 JSON 或以可读字符开头的内容不尝试解码
    if (encoded.empty() || encoded.front() == '{' || encoded.front() == '[') {
        return "";
    }

    // 步骤 1: base64 解码
    std::string compressed = base64_decode(encoded);
    if (compressed.empty()) {
        return "";
    }

    // 步骤 2: gzip 解压
    std::string decompressed = gzip_decompress(compressed);
    // 验证解压结果是有效 JSON
    if (!decompressed.empty()
        && (decompressed.front() == '{' || decompressed.front() == '[')) {
        return decompressed;
    }
    return "";
}

}  // namespace qr_detector
