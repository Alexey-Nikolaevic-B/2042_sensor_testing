#include <ros/ros.h>
#include <ros/package.h>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/SystemPaths.hh>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/GetModelState.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>

#include <thread>
#include <atomic>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <random>
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <unistd.h>

using std::string;

namespace {

struct Fixture {
  string name;
  int id = 0;
  double x = 0, y = 0, z = 0;
};

struct Tag {
  string name;
  double x = 0, y = 0, z = 0;
};

static std::string g_plugin_sdf_file;
static std::string g_plugin_sdf_dir;

static inline bool starts_with(const std::string& s, const std::string& pref) {
  return s.size() >= pref.size() && s.compare(0, pref.size(), pref) == 0;
}

static inline std::string dirname_of(const std::string& p) {
  const auto pos = p.find_last_of("/\\");
  if (pos == std::string::npos) return {};
  if (pos == 0) return "/";
  return p.substr(0, pos);
}

static inline std::string join_path(const std::string& a, const std::string& b) {
  if (a.empty()) return b;
  if (b.empty()) return a;
  if (a.back() == '/' || a.back() == '\\') return a + b;
  return a + "/" + b;
}

static inline bool is_abs_path(const std::string& p) {
  return !p.empty() && p[0] == '/';
}

static inline bool readable_file(const std::string& p) {
  return !p.empty() && ::access(p.c_str(), R_OK) == 0;
}

static inline std::string trim_copy(std::string s) {
  auto not_space = [](unsigned char c){ return !std::isspace(c); };
  while (!s.empty() && !not_space(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
  while (!s.empty() && !not_space(static_cast<unsigned char>(s.back()))) s.pop_back();
  return s;
}

static std::string expand_tilde(const std::string& p) {
  if (!starts_with(p, "~/")) return p;
  const char* home = std::getenv("HOME");
  if (!home || !*home) return p;
  return join_path(std::string(home), p.substr(2));
}

static std::string expand_package_uri(const std::string& p) {
  const std::string prefix = "package://";
  if (!starts_with(p, prefix)) return p;

  const std::string rest = p.substr(prefix.size());
  const auto slash = rest.find('/');
  const std::string pkg = (slash == std::string::npos) ? rest : rest.substr(0, slash);
  const std::string sub = (slash == std::string::npos) ? ""   : rest.substr(slash + 1);

  const std::string pkgPath = ros::package::getPath(pkg);
  if (pkgPath.empty()) return p;

  return sub.empty() ? pkgPath : join_path(pkgPath, sub);
}

static std::string resolve_custom_path(std::string raw) {
  raw = trim_copy(raw);
  if (raw.empty()) return raw;

  raw = expand_package_uri(raw);

  if (raw.find("://") != std::string::npos) {
    const std::string found = gazebo::common::SystemPaths::Instance()->FindFileURI(raw);
    if (!found.empty()) return found;

    const std::string filePrefix = "file://";
    if (starts_with(raw, filePrefix)) {
      const std::string noScheme = raw.substr(filePrefix.size());
      if (is_abs_path(noScheme) && readable_file(noScheme)) return noScheme;
      raw = noScheme;
    }
  }

  if (is_abs_path(raw)) return raw;

  raw = expand_tilde(raw);
  if (is_abs_path(raw)) return raw;

  if (!g_plugin_sdf_dir.empty()) {
    const std::string candidate = join_path(g_plugin_sdf_dir, raw);
    if (readable_file(candidate)) return candidate;
  }

  {
    const std::string found = gazebo::common::SystemPaths::Instance()->FindFile(raw, true);
    if (!found.empty()) return found;
  }

  if (!g_plugin_sdf_dir.empty()) return join_path(g_plugin_sdf_dir, raw);
  return raw;
}

static inline bool ros_ready() {
  if (!ros::isInitialized()) {
    std::cerr << "[RFID_tag_plugin] ROS is not initialized. Use gazebo_ros.\n";
    return false;
  }
  return true;
}

static inline void ltrim_inplace(std::string& s) {
  s.erase(s.begin(),
          std::find_if(s.begin(), s.end(), [](unsigned char c) { return !std::isspace(c); }));
}

static inline void rtrim_inplace(std::string& s) {
  s.erase(std::find_if(s.rbegin(), s.rend(),
                       [](unsigned char c) { return !std::isspace(c); })
              .base(),
          s.end());
}

static inline void strip_bom_inplace(std::string& s) {
  if (s.size() >= 3 &&
      static_cast<unsigned char>(s[0]) == 0xEF &&
      static_cast<unsigned char>(s[1]) == 0xBB &&
      static_cast<unsigned char>(s[2]) == 0xBF) {
    s.erase(0, 3);
  }
}

static inline std::vector<Fixture> read_map_robust(const string& raw_path) {
  std::vector<Fixture> out;

  const std::string path = resolve_custom_path(raw_path);
  std::ifstream f(path);
  if (!f.is_open()) {
    ROS_ERROR_STREAM("[RFID_tag_plugin] cannot open map: " << path << " (raw: " << raw_path << ")");
    return out;
  }

  string line;
  int line_no = 0;
  while (std::getline(f, line)) {
    line_no++;

    strip_bom_inplace(line);
    ltrim_inplace(line);
    rtrim_inplace(line);

    if (line.empty()) continue;
    if (!line.empty() && line[0] == '#') continue;

    for (char& c : line) {
      if (c == ',' || c == ';') c = ' ';
    }

    std::istringstream iss(line);
    Fixture fx;
    if (!(iss >> fx.name >> fx.id >> fx.x >> fx.y >> fx.z)) {
      ROS_WARN_STREAM("[RFID_tag_plugin] bad map line " << line_no << ": '" << line << "'");
      continue;
    }

    out.push_back(fx);
  }

  return out;
}

class Spawner {
public:
  string parent_frame = "base_link_sim";
  string tag_prefix   = "rfid_tag";
  int tags_per_fix    = 1;

  string fix_sdf_path;
  string tag_sdf_path;
  string map_path;

  bool randomize = false;
  double rand_xy = 0.0;
  double rand_z  = 0.0;

  bool dynamic_tf = false;
  double tf_rate  = 20.0;

  std::vector<Tag> tags;

  ros::NodeHandle nh;
  ros::ServiceClient spawn_cli;
  ros::ServiceClient get_state_cli;

  std::atomic<bool> stop{false};
  std::thread tf_thread;

  explicit Spawner(const ros::NodeHandle& n)
      : nh(n)
  {
    spawn_cli = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    get_state_cli = nh.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
  }

  ~Spawner() {
    stop.store(true);
    if (tf_thread.joinable()) tf_thread.join();
  }

  static string slurp_file(const string& raw_path) {
    const std::string path = resolve_custom_path(raw_path);
    std::ifstream f(path);
    if (!f.is_open()) {
      ROS_ERROR_STREAM("[RFID_tag_plugin] cannot open SDF file: " << path << " (raw: " << raw_path << ")");
      return {};
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
  }

  gazebo_msgs::SpawnModel make_spawn(const string& name, double x, double y, double z, const string& sdf_xml) {
    gazebo_msgs::SpawnModel req;
    req.request.model_name = name;
    req.request.reference_frame = "world";
    req.request.initial_pose.position.x = x;
    req.request.initial_pose.position.y = y;
    req.request.initial_pose.position.z = z;
    req.request.initial_pose.orientation.w = 1.0;
    req.request.model_xml = sdf_xml;
    return req;
  }

  void wait_services_or_stop() {
    while (!stop.load() && ros::ok()) {
      if (ros::service::exists("gazebo/spawn_sdf_model", false) &&
          ros::service::exists("gazebo/get_model_state", false)) {
        return;
      }
      ros::Duration(0.1).sleep();
    }
  }

  void spawn_call(gazebo_msgs::SpawnModel& m) {
    if (!spawn_cli.call(m)) {
      ROS_ERROR_STREAM("[RFID_tag_plugin] spawn_sdf_model RPC failed for " << m.request.model_name);
      return;
    }
    if (!m.response.success) {
      ROS_WARN_STREAM("[RFID_tag_plugin] spawn failed for " << m.request.model_name
                      << ": " << m.response.status_message);
    }
  }

  bool get_model_pose_world(const string& model, geometry_msgs::Pose& pose_out) {
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = model;
    srv.request.relative_entity_name = "world";
    if (!get_state_cli.call(srv)) return false;
    if (!srv.response.success) return false;
    pose_out = srv.response.pose;
    return true;
  }

  void publish_static_tf_once() {
    static tf2_ros::StaticTransformBroadcaster sbr;

    std::vector<geometry_msgs::TransformStamped> v;
    v.reserve(tags.size());

    const ros::Time stamp = ros::Time::now();
    for (const auto& t : tags) {
      geometry_msgs::TransformStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = parent_frame;
      msg.child_frame_id = t.name;
      msg.transform.translation.x = t.x;
      msg.transform.translation.y = t.y;
      msg.transform.translation.z = t.z;
      msg.transform.rotation.w = 1.0;
      v.push_back(msg);
    }

    sbr.sendTransform(v);
    ROS_INFO_STREAM("[RFID_tag_plugin] published " << v.size() << " static TF frames");
  }

  void tf_loop_dynamic() {
    tf2_ros::TransformBroadcaster br;
    ros::Rate rate(tf_rate);
    ros::Time last_stamp(0);

    while (ros::ok() && !stop.load()) {
      ros::Time stamp = ros::Time::now();

      if (stamp == last_stamp) {
        rate.sleep();
        continue;
      }
      last_stamp = stamp;

      for (auto& t : tags) {
        geometry_msgs::Pose p;
        if (get_model_pose_world(t.name, p)) {
          t.x = p.position.x;
          t.y = p.position.y;
          t.z = p.position.z;
        }

        geometry_msgs::TransformStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = parent_frame;
        msg.child_frame_id = t.name;
        msg.transform.translation.x = t.x;
        msg.transform.translation.y = t.y;
        msg.transform.translation.z = t.z;
        msg.transform.rotation.w = 1.0;
        br.sendTransform(msg);
      }

      rate.sleep();
    }
  }

  void start_tf() {
    if (!dynamic_tf) {
      publish_static_tf_once();
      return;
    }
    tf_thread = std::thread([this]() { tf_loop_dynamic(); });
  }

  void spawn_from_map() {
    ROS_INFO_STREAM("[RFID_tag_plugin] map_path=" << map_path);
    ROS_INFO_STREAM("[RFID_tag_plugin] fix_sdf_path=" << fix_sdf_path);
    ROS_INFO_STREAM("[RFID_tag_plugin] tag_sdf_path=" << tag_sdf_path);
    ROS_INFO_STREAM("[RFID_tag_plugin] parent_frame=" << parent_frame
                    << " tag_prefix=" << tag_prefix
                    << " tags_per_fix=" << tags_per_fix
                    << " dynamic_tf=" << (dynamic_tf ? "true" : "false")
                    << " tf_rate=" << tf_rate);

    if (fix_sdf_path.empty() || tag_sdf_path.empty()) {
      ROS_ERROR("[RFID_tag_plugin] fix_sdf_path/tag_sdf_path is empty.");
      return;
    }

    const auto fixtures = read_map_robust(map_path);
    if (fixtures.empty()) {
      ROS_ERROR_STREAM("[RFID_tag_plugin] empty/invalid map: " << map_path);
      return;
    }

    const string fix_xml = slurp_file(fix_sdf_path);
    const string tag_xml = slurp_file(tag_sdf_path);
    if (fix_xml.empty() || tag_xml.empty()) {
      ROS_ERROR("[RFID_tag_plugin] SDF xml is empty (file read failed).");
      return;
    }

    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> dxy(-rand_xy, rand_xy);
    std::uniform_real_distribution<double> dz(0.0, rand_z);

    int tag_counter = 0;
    tags.clear();

    for (const auto& fx : fixtures) {
      if (stop.load()) return;

      {
        auto req = make_spawn(fx.name, fx.x, fx.y, fx.z, fix_xml);
        spawn_call(req);
      }

      for (int k = 0; k < tags_per_fix; ++k) {
        ++tag_counter;
        Tag t;
        t.name = tag_prefix + std::to_string(tag_counter);

        const double ox = randomize ? dxy(rng) : 0.0;
        const double oy = randomize ? dxy(rng) : 0.0;
        const double oz = randomize ? dz(rng)  : 0.0;

        t.x = fx.x + ox;
        t.y = fx.y + oy;
        t.z = fx.z + oz;

        auto req = make_spawn(t.name, t.x, t.y, t.z, tag_xml);
        spawn_call(req);

        tags.push_back(t);
        ROS_INFO_STREAM("[RFID_tag_plugin] spawned tag " << t.name
                        << " at " << t.x << " " << t.y << " " << t.z);
      }
    }

    start_tf();
  }
};

} // namespace

namespace gazebo {

class RFID_TAG : public SensorPlugin {
public:
  RFID_TAG() = default;

  ~RFID_TAG() override {
    stop_.store(true);
    if (worker_.joinable()) worker_.join();
    spawner_.reset();
  }

  void Load(sensors::SensorPtr, sdf::ElementPtr _sdf) override {
    if (!ros_ready()) return;

    g_plugin_sdf_file = _sdf ? _sdf->FilePath() : "";
    g_plugin_sdf_dir  = dirname_of(g_plugin_sdf_file);
    if (g_plugin_sdf_dir.empty()) g_plugin_sdf_dir = ".";

    spawner_ = std::make_shared<Spawner>(ros::NodeHandle());

    spawner_->parent_frame = _sdf->HasElement("parent_frame")
                               ? _sdf->Get<string>("parent_frame")
                               : "base_link_sim";

    spawner_->tag_prefix   = _sdf->HasElement("tag_name")
                               ? _sdf->Get<string>("tag_name")
                               : "rfid_tag";

    spawner_->tags_per_fix = _sdf->HasElement("tags_per_fix")
                               ? _sdf->Get<int>("tags_per_fix")
                               : 1;

    spawner_->fix_sdf_path = _sdf->HasElement("fix_sdf_path")
                               ? _sdf->Get<string>("fix_sdf_path")
                               : "";

    spawner_->tag_sdf_path = _sdf->HasElement("tag_sdf_path")
                               ? _sdf->Get<string>("tag_sdf_path")
                               : "";

    if (_sdf->HasElement("map_path")) {
      spawner_->map_path = _sdf->Get<string>("map_path");
    } else {
      const string pkg = ros::package::getPath("RFID_tag_plugin");
      spawner_->map_path = pkg + "/map_layouts/map.txt";
    }

    spawner_->randomize = _sdf->HasElement("randomize") ? _sdf->Get<bool>("randomize") : false;
    spawner_->rand_xy   = _sdf->HasElement("rand_xy")   ? _sdf->Get<double>("rand_xy") : 0.0;
    spawner_->rand_z    = _sdf->HasElement("rand_z")    ? _sdf->Get<double>("rand_z")  : 0.0;

    spawner_->dynamic_tf = _sdf->HasElement("dynamic_tf") ? _sdf->Get<bool>("dynamic_tf") : false;
    spawner_->tf_rate    = _sdf->HasElement("tf_rate")    ? _sdf->Get<double>("tf_rate")  : 20.0;

    worker_ = std::thread([this]() {
      spawner_->wait_services_or_stop();
      if (stop_.load() || !ros::ok()) return;
      ros::Duration(0.5).sleep();
      spawner_->spawn_from_map();
    });
  }

private:
  std::shared_ptr<Spawner> spawner_;
  std::thread worker_;
  std::atomic<bool> stop_{false};
};

GZ_REGISTER_SENSOR_PLUGIN(RFID_TAG)

} // namespace gazebo
