#pragma once
#include <nlohmann/json.hpp>
#include <string>

inline std::string cabinet_log_text(std::string value, size_t limit = 600)
{
  for (char &c : value) {
    if (c == '\n' || c == '\r' || c == '\t') c = ' ';
  }
  if (value.size() > limit) {
    size_t end = limit;
    while (end > 0 && (static_cast<unsigned char>(value[end]) & 0xc0) == 0x80) --end;
    value = value.substr(0, end) + "…";
  }
  return value;
}

inline std::string cabinet_response_summary(const std::string &message)
{
  const auto data = nlohmann::json::parse(message, nullptr, false);
  if (!data.is_object()) return cabinet_log_text(message);
  const auto field = [&data](const char *key) -> std::string {
    const auto it = data.find(key);
    return it != data.end() && it->is_string() ? it->get<std::string>() : "";
  };
  std::string result = field("summary");
  if (result.empty()) {
    const std::string step = field("step");
    if (!step.empty()) result = "步骤=" + cabinet_log_text(step, 100) + " | ";
    const std::string error = field("error");
    result += error.empty() ? field("event") : error;
    if (result.empty()) result = "服务已返回，完整信息见 /rg_cabinet/status";
  }
  result = cabinet_log_text(result);
  const std::string path = field("diagnostic_file");
  if (!path.empty()) result += "\n诊断文件: " + cabinet_log_text(path, 600);
  return result;
}
