// 从 SDF 文件中解析锥桶位置
// 扫描所有 <include> 标签，提取 uri/name 推断颜色，提取 pose 获取位置

#include "right_angle_stack_cpp/track_model.hpp"

#include <fstream>
#include <sstream>
#include <cctype>

namespace right_angle_stack_cpp
{

std::vector<ConeInfo> load_cones_from_sdf(const std::string & path)
{
  std::vector<ConeInfo> cones;
  std::ifstream file(path);
  if (!file.is_open()) {
    return cones;
  }

  // 读取整个文件内容
  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string content = buffer.str();

  size_t pos = 0;
  // 搜索所有 <include> 块
  while ((pos = content.find("<include>", pos)) != std::string::npos) {
    size_t end = content.find("</include>", pos);
    if (end == std::string::npos) break;
    std::string include_block = content.substr(pos, end - pos + 10);
    pos = end + 10;

    // 提取 XML 标签中的文本内容
    auto extract = [&](const std::string & tag) -> std::string {
      std::string open = "<" + tag + ">";
      std::string close = "</" + tag + ">";
      size_t s = include_block.find(open);
      if (s == std::string::npos) return "";
      s += open.size();
      size_t e = include_block.find(close, s);
      if (e == std::string::npos) return "";
      return include_block.substr(s, e - s);
    };

    std::string uri = extract("uri");
    std::string name = extract("name");
    std::string pose_text = extract("pose");

    // 解析 pose: "x y z roll pitch yaw"，取前三个
    double x = 0.0, y = 0.0, z = 0.0;
    std::istringstream pose_stream(pose_text);
    pose_stream >> x >> y >> z;

    // 从 uri 和 name 中推断锥桶颜色
    std::string label = uri + " " + name;
    for (auto & c : label) c = std::tolower(static_cast<unsigned char>(c));

    std::string color = "unknown";
    if (label.find("blue") != std::string::npos) color = "blue";
    else if (label.find("yellow") != std::string::npos) color = "yellow";
    else if (label.find("red") != std::string::npos) color = "red";

    cones.push_back({color, x, y, z});
  }

  return cones;
}

}  // namespace right_angle_stack_cpp