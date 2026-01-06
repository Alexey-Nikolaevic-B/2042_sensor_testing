// RFID_antenna_plugin.cpp
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/SystemPaths.hh>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/GetModelState.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <cmath>
#include <string>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstdlib>
#include <cctype>
#include <unistd.h>
#include <ros/package.h>

#define MAX_TAGS 1000
#define MAX_DETECTION_RANGE 10
#define PI 3.1415

using namespace std;

static string detected_tag_name;
static string detected_tag_colour;
static int antenna_id;

static float beam_width_h;
static float beam_width_v;
static float r_zero;

static string path_yellow_tag;
static string path_blue_tag;
static string path_red_tag;
static string path_purple_tag;

static string target_frame; // antenna prefix e.g. "antenna_"
static string source_frame; // tag prefix e.g. "rfid_tag"

static int max_tags = 50;
static int stop_after_misses = 5;

static string antenna_parent_frame = "base_link_sim";
static double ant_x=0, ant_y=0, ant_z=0, ant_roll=0, ant_pitch=0, ant_yaw=0;

static bool spawn_visual_markers = true;

static string detected_topic = "/detected_tags";
static bool event_only = true;

static bool deterministic = true;
static double deterministic_threshold = 0.5; // pdf >= threshold

static std::string g_plugin_sdf_file;
static std::string g_plugin_sdf_dir;

// --- Path helpers ---
static inline bool starts_with(const std::string &s, const std::string &p) {
  return s.size() >= p.size() && s.compare(0, p.size(), p) == 0;
}

static inline std::string trim_copy(std::string s) {
  auto not_space = [](unsigned char c){ return !std::isspace(c); };
  while (!s.empty() && !not_space((unsigned char)s.front())) s.erase(s.begin());
  while (!s.empty() && !not_space((unsigned char)s.back())) s.pop_back();
  return s;
}

static inline bool is_abs_path(const std::string &p) {
  return !p.empty() && p[0] == '/';
}

static inline std::string dirname_of(const std::string &p) {
  if (p.empty()) return "";
  const auto pos = p.find_last_of("/\\");
  if (pos == std::string::npos) return "";
  if (pos == 0) return "/";
  return p.substr(0, pos);
}

static inline std::string join_path(const std::string &a, const std::string &b) {
  if (a.empty()) return b;
  if (b.empty()) return a;
  if (a.back() == '/' || a.back() == '\\') return a + b;
  return a + "/" + b;
}

static inline bool readable_file(const std::string &p) {
  return !p.empty() && ::access(p.c_str(), R_OK) == 0;
}

static std::string expand_tilde(const std::string &p) {
  if (!starts_with(p, "~/")) return p;
  const char *home = std::getenv("HOME");
  if (!home || !*home) return p;
  return join_path(std::string(home), p.substr(2));
}

static std::string expand_package_uri(const std::string &p) {
  // package://my_pkg/path/to/file.sdf
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

enum Tag_colors { white, yellow, blue, red, purple };

class Tag {
public:
  string name;
  float x=0, y=0, z=0;
  bool spawned=false;   // visual marker spawned
  bool detected=false;  // last detection state
};

class Antenna {
public:
  string get_color_path_from_sdf(Tag_colors c) {
    switch (c) {
      case yellow: return path_yellow_tag;
      case blue: return path_blue_tag;
      case red: return path_red_tag;
      case purple: return path_purple_tag;
      default: return path_blue_tag;
    }
  }

  double normalized_antenna_pattern(float eh, float ev){
    return pow(cos((PI/2)*(eh/beam_width_h)),2.0) * pow(cos((PI/2)*(ev/beam_width_v)),2.0);
  }

  double x_argument(float d, float eh, float ev){
    double d0 = normalized_antenna_pattern(0,0);
    return (pow((d0*d),2.0) / pow((normalized_antenna_pattern(eh,ev))*r_zero,2.0));
  }

  double pdf(float d, float eh, float ev){
    if (abs(eh) > PI/2) return 0;
    return 2.0 / (1.0 + pow(3.0, sqrt(x_argument(d,eh,ev))));
  }

  bool accept(double p){
    if (deterministic) return p >= deterministic_threshold;
    float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float reduced = pow(1.0f - (1.0f - static_cast<float>(p)), 1.0f);
    return r < reduced;
  }

  double elev_h(float x, float y){
    return acos(x / sqrt(x*x + y*y));
  }
  double elev_v(float x, float z){
    return acos(x / sqrt(x*x + z*z));
  }

  float dist_xy(float x, float y){
    return sqrt(x*x + y*y);
  }
};

class Spawner {
public:
  ros::NodeHandle nh;
  ros::ServiceClient spawn_cli;
  ros::ServiceClient get_state_cli;
  ros::Publisher det_pub;

  tf2_ros::StaticTransformBroadcaster static_br;

  explicit Spawner(const ros::NodeHandle& n): nh(n) {
    spawn_cli = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    get_state_cli = nh.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
    det_pub = nh.advertise<geometry_msgs::PoseStamped>(detected_topic, 50);
  }

  void publish_antenna_static_tf_once() {
    // parent -> antenna_1
    geometry_msgs::TransformStamped t;
    t.header.stamp = ros::Time::now();
    t.header.frame_id = antenna_parent_frame;
    t.child_frame_id = target_frame + to_string(antenna_id);
    t.transform.translation.x = ant_x;
    t.transform.translation.y = ant_y;
    t.transform.translation.z = ant_z;

    t.transform.rotation.w = 1.0;
    static_br.sendTransform(t);
  }

  static string slurp(const string& raw_path){
    const std::string path = resolve_custom_path(raw_path);

    std::ifstream f(path);
    if (!f.is_open()) {
      std::ostringstream ss;
      ss << "[RFID_antenna_plugin] Can't open model SDF file: '" << path
         << "' (raw: '" << raw_path << "')";
      throw std::runtime_error(ss.str());
    }

    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
  }

  gazebo_msgs::SpawnModel make_spawn(const string& name, float x, float y, float z, const string& xml){
    gazebo_msgs::SpawnModel m;
    m.request.model_name = name;
    m.request.reference_frame = "world";
    m.request.initial_pose.position.x = x;
    m.request.initial_pose.position.y = y;
    m.request.initial_pose.position.z = z;
    m.request.initial_pose.orientation.w = 1.0;
    m.request.model_xml = xml;
    return m;
  }

  void spawn_detected_model(const string& model_name, float x, float y, float z, Tag_colors c){
    Antenna a;

    string xml;
    try {
      xml = slurp(a.get_color_path_from_sdf(c));
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM(e.what());
      return;
    }

    auto req = make_spawn(model_name, x, y, z, xml);
    ros::service::waitForService("gazebo/spawn_sdf_model");
    spawn_cli.call(req);
  }

  bool get_model_pose_world(const string& model, geometry_msgs::Pose& pose_out){
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = model;
    srv.request.relative_entity_name = "world";
    if (!get_state_cli.call(srv)) return false;
    if (!srv.response.success) return false;
    pose_out = srv.response.pose;
    return true;
  }

  void publish_detection_pose(const string& tag_name){
    geometry_msgs::Pose p;
    geometry_msgs::PoseStamped out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = tag_name;

    if (get_model_pose_world(tag_name, p)) {
      out.pose = p;
    } else {
      out.pose.orientation.w = 1.0;
    }
    det_pub.publish(out);
  }

  Tag_colors colour_from_sdf(){
    if (detected_tag_colour == "blue") return blue;
    if (detected_tag_colour == "yellow") return yellow;
    if (detected_tag_colour == "red") return red;
    if (detected_tag_colour == "purple") return purple;
    return blue;
  }

  string detected_name_for_idx(int idx){
    return detected_tag_name + detected_tag_colour + to_string(idx + 1);
  }

  void antenna_loop(){
    sleep(1);
    publish_antenna_static_tf_once();

    Antenna ant;
    Tag tags[MAX_TAGS];

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listen(tfBuffer);
    ros::Rate rate(20.0);

    const int scan_limit = std::max(1, std::min(max_tags, MAX_TAGS));
    const string target = target_frame + to_string(antenna_id);

    while (ros::ok()){
      int misses = 0;

      for (int idx = 0; idx < scan_limit; ++idx){
        const string src = source_frame + to_string(idx + 1);

        if (!tfBuffer.canTransform(target, src, ros::Time(0), ros::Duration(0.0))){
          misses++;
          if (misses >= stop_after_misses) break;
          continue;
        }
        misses = 0;

        auto tfm = tfBuffer.lookupTransform(target, src, ros::Time(0));

        tags[idx].x = tfm.transform.translation.x;
        tags[idx].y = tfm.transform.translation.y;
        tags[idx].z = tfm.transform.translation.z;

        float d = ant.dist_xy(tags[idx].x, tags[idx].y);
        bool in_range = (d < MAX_DETECTION_RANGE);

        bool detected_now = false;
        if (in_range){
          double eh = ant.elev_h(tags[idx].x, tags[idx].y);
          double ev = ant.elev_v(tags[idx].x, tags[idx].z);
          double p = ant.pdf(d, eh, ev);
          detected_now = ant.accept(p);
        }

        if (detected_now && (!event_only || !tags[idx].detected)){
          publish_detection_pose(src);
        }
        tags[idx].detected = detected_now;

        if (spawn_visual_markers && detected_now && !tags[idx].spawned){
          spawn_detected_model(detected_name_for_idx(idx), tags[idx].x, tags[idx].y, tags[idx].z, colour_from_sdf());
          tags[idx].spawned = true;
        }
        if (!detected_now){
          tags[idx].spawned = false;
        }
      }

      rate.sleep();
    }
  }
};

namespace gazebo {

class RFID_ANTENNA : public SensorPlugin {
public:
  RFID_ANTENNA() : SensorPlugin() {}

  void Load(sensors::SensorPtr /*_sensor*/, sdf::ElementPtr _sdf) override {
    if (!ros::isInitialized()) {
      std::cerr << "[RFID_antenna_plugin] ROS is not initialized. Use gazebo_ros.\n";
      return;
    }

    g_plugin_sdf_file = _sdf ? _sdf->FilePath() : "";
    g_plugin_sdf_dir  = dirname_of(g_plugin_sdf_file);
    if (g_plugin_sdf_dir.empty()) g_plugin_sdf_dir = ".";

    target_frame = _sdf->HasElement("antenna_name") ? _sdf->Get<string>("antenna_name") : "antenna_";
    source_frame = _sdf->HasElement("tag_name") ? _sdf->Get<string>("tag_name") : "rfid_tag";

    detected_tag_name = _sdf->HasElement("detected_tag_name") ? _sdf->Get<string>("detected_tag_name") : "rfid_tag_";
    detected_tag_colour = _sdf->HasElement("detected_tag_colour") ? _sdf->Get<string>("detected_tag_colour") : "blue";

    antenna_id = _sdf->HasElement("antenna_id") ? _sdf->Get<int>("antenna_id") : 1;
    beam_width_h = _sdf->HasElement("azimuth_beamwidth") ? _sdf->Get<float>("azimuth_beamwidth") : 1.0f;
    beam_width_v = _sdf->HasElement("elevation_beamwidth") ? _sdf->Get<float>("elevation_beamwidth") : 1.0f;
    r_zero = _sdf->HasElement("rzero") ? _sdf->Get<float>("rzero") : 10.0f;

    // Paths from SDF (raw)
    path_yellow_tag = _sdf->HasElement("tag_yellow_sdf_path") ? _sdf->Get<string>("tag_yellow_sdf_path") : "";
    path_blue_tag   = _sdf->HasElement("tag_blue_sdf_path")   ? _sdf->Get<string>("tag_blue_sdf_path")   : "";
    path_red_tag    = _sdf->HasElement("tag_red_sdf_path")    ? _sdf->Get<string>("tag_red_sdf_path")    : "";
    path_purple_tag = _sdf->HasElement("tag_purple_sdf_path") ? _sdf->Get<string>("tag_purple_sdf_path") : "";

    if (!path_yellow_tag.empty()) path_yellow_tag = resolve_custom_path(path_yellow_tag);
    if (!path_blue_tag.empty())   path_blue_tag   = resolve_custom_path(path_blue_tag);
    if (!path_red_tag.empty())    path_red_tag    = resolve_custom_path(path_red_tag);
    if (!path_purple_tag.empty()) path_purple_tag = resolve_custom_path(path_purple_tag);

    // scan limits
    if (_sdf->HasElement("max_tags")) max_tags = _sdf->Get<int>("max_tags");
    if (_sdf->HasElement("stop_after_misses")) stop_after_misses = _sdf->Get<int>("stop_after_misses");

    // self-contained antenna TF
    antenna_parent_frame = _sdf->HasElement("parent_frame") ? _sdf->Get<string>("parent_frame") : "base_link_sim";
    if (_sdf->HasElement("antenna_xyz")) {
      auto v = _sdf->Get<ignition::math::Vector3d>("antenna_xyz");
      ant_x = v.X(); ant_y = v.Y(); ant_z = v.Z();
    }

    // publish detections
    detected_topic = _sdf->HasElement("detected_topic") ? _sdf->Get<string>("detected_topic") : "/detected_tags";
    event_only = _sdf->HasElement("event_only") ? _sdf->Get<bool>("event_only") : true;

    spawn_visual_markers = _sdf->HasElement("spawn_visual_markers") ? _sdf->Get<bool>("spawn_visual_markers") : true;

    deterministic = _sdf->HasElement("deterministic") ? _sdf->Get<bool>("deterministic") : true;
    deterministic_threshold = _sdf->HasElement("deterministic_threshold") ? _sdf->Get<double>("deterministic_threshold") : 0.5;

    ROS_INFO_STREAM("[RFID_antenna_plugin] plugin SDF: " << g_plugin_sdf_file);
    ROS_INFO_STREAM("[RFID_antenna_plugin] base dir : " << g_plugin_sdf_dir);

    auto sp = std::make_shared<Spawner>(ros::NodeHandle());
    std::thread([sp]() { sp->antenna_loop(); }).detach();
  }
};

GZ_REGISTER_SENSOR_PLUGIN(RFID_ANTENNA)

} // namespace gazebo
