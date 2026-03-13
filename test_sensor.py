#!/usr/bin/env python3
"""
Tactile Sensor Diagnostic Test for ROS Noetic
Tests contact sensor functionality using gazebo_ros_bumper plugin
"""

import os
import cv2
import numpy as np
import rospy
import subprocess
import time
import sys
import xml.etree.ElementTree as ET
import rostopic
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ContactsState

# ── config ────────────────────────────────────────────────────────────────────

CATKIN_SETUP  = "catkin_ws/devel/setup.bash"
SENSOR_PKG    = "scenario_test_pkg"
LAUNCH_FILE   = "scenario.launch"
BASE_WORLD    = "catkin_ws/src/scenario_test_pkg/worlds/base_world.world"

# Update these paths to match your system
CAMERA_SDF    = "/home/alexey/Documents/projects/2042/github/2042_sensor_testing/resources/sensors/tactile/tactile.sdf"
WORLD_PATH    = "resources/worlds/rfid/change_distance.world"  # any existing world

TOPIC         = "/tactile_sensor/bumper_states"  # or "/bumper_states" if no namespace
TIMEOUT       = 15   # seconds to wait for first contact
SAVE_PATH     = "/tmp/tactile_contact.jpg"  # not used for contacts, but kept for compatibility

# ── diagnostic helpers ────────────────────────────────────────────────────────

def check_plugin_availability():
    """Check if the required Gazebo plugin exists"""
    print("\n=== Checking Plugin Availability ===")
    
    # Check for bumper plugin
    plugin_paths = [
        "/opt/ros/noetic/lib/libgazebo_ros_bumper.so",
        "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libgazebo_ros_bumper.so"
    ]
    
    found = False
    for path in plugin_paths:
        if os.path.exists(path):
            print(f"✓ Plugin found: {path}")
            found = True
    
    if not found:
        print("✗ Bumper plugin not found in standard locations")
        print("  Try: sudo apt install ros-noetic-gazebo-plugins")
    
    # Check if gazebo_msgs is available
    try:
        from gazebo_msgs.msg import ContactsState
        print("✓ gazebo_msgs/ContactsState available")
    except ImportError as e:
        print(f"✗ gazebo_msgs not available: {e}")

def check_sdf_file():
    """Validate the SDF file structure"""
    print("\n=== Checking SDF File ===")
    
    if not os.path.exists(CAMERA_SDF):
        print(f"✗ SDF file not found: {CAMERA_SDF}")
        return False
    
    try:
        tree = ET.parse(CAMERA_SDF)
        root = tree.getroot()
        
        # Check for sensor element
        sensors = root.findall(".//sensor")
        if not sensors:
            print("✗ No <sensor> element found in SDF")
            return False
        
        for sensor in sensors:
            sensor_type = sensor.get('type')
            sensor_name = sensor.get('name')
            print(f"  Found sensor: '{sensor_name}' (type='{sensor_type}')")
            
            # Check for plugin
            plugins = sensor.findall(".//plugin")
            if plugins:
                for plugin in plugins:
                    plugin_name = plugin.get('name')
                    plugin_file = plugin.get('filename')
                    print(f"    Plugin: '{plugin_name}' ({plugin_file})")
                    
                    # Check for topic configuration
                    topic = plugin.find(".//topicName")
                    if topic is not None and topic.text:
                        print(f"    Configured topic: {topic.text}")
                    else:
                        print(f"    ⚠️ No explicit topicName in plugin")
            else:
                print(f"    ⚠️ No plugin found in sensor")
        
        return True
    except ET.ParseError as e:
        print(f"✗ Invalid XML in SDF: {e}")
        return False
    except Exception as e:
        print(f"✗ Error parsing SDF: {e}")
        return False

def check_gazebo_models():
    """Check what models are loaded in Gazebo"""
    print("\n=== Checking Gazebo Models ===")
    
    try:
        # Try to get model properties via service
        from rospy import ServiceProxy
        from gazebo_msgs.srv import GetModelProperties
        
        rospy.wait_for_service('/gazebo/get_model_properties', timeout=2.0)
        get_model = ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
        
        # Try to get tactile sensor model
        try:
            resp = get_model("tactile_sensor")
            print(f"✓ Model 'tactile_sensor' found")
            print(f"  Status: {resp.status_message}")
        except rospy.ServiceException:
            print(f"✗ Model 'tactile_sensor' not found")
            
    except rospy.ROSException:
        print("  Service /gazebo/get_model_properties not available")

def verify_message_type(topic_name):
    """Verify the message type for the topic"""
    print(f"\n=== Verifying Message Type for {topic_name} ===")
    
    try:
        # Get topic type from ROS master
        topics = rospy.get_published_topics()
        found = False
        for t, t_type in topics:
            if t == topic_name:
                print(f"✓ Topic found: {t}")
                print(f"  Type: {t_type}")
                found = True
                
                # Try to get message class
                try:
                    msg_class, real_topic, _ = rostopic.get_topic_class(topic_name)
                    if msg_class:
                        print(f"  ✓ Message class: {msg_class.__name__}")
                        if msg_class.__name__ == 'ContactsState':
                            print(f"  ✓ Correct type for bumper sensor")
                        else:
                            print(f"  ⚠️ Expected ContactsState, got {msg_class.__name__}")
                    else:
                        print(f"  ✗ get_topic_class returned None")
                except Exception as e:
                    print(f"  ✗ Error getting message class: {e}")
                break
        
        if not found:
            print(f"✗ Topic {topic_name} not found in published topics")
            
            # Show available topics for debugging
            print("\nAvailable topics:")
            for t, t_type in topics[:10]:  # Show first 10
                if not t.startswith('/rosout'):
                    print(f"  {t} -> {t_type}")
                    
    except Exception as e:
        print(f"✗ Error checking message type: {e}")

def spawn_touch_object():
    """Spawn a sphere to touch the tactile sensor"""
    print("\n=== Spawning Touch Object ===")
    
    # Create a simple sphere SDF - XML declaration on FIRST line!
    sphere_sdf = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="touch_sphere">
    <pose>0.02 0 0.1 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <sphere><radius>0.01</radius></sphere>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere><radius>0.01</radius></sphere>
        </geometry>
        <material>
          <ambient>0.2 0.8 0.2 1</ambient>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
    
    sphere_path = "/tmp/touch_sphere.sdf"
    with open(sphere_path, 'w') as f:
        f.write(sphere_sdf)
    
    # Spawn the sphere
    cmd = f"source {CATKIN_SETUP} && rosrun gazebo_ros spawn_model -file {sphere_path} -sdf -model touch_sphere"
    result = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✓ Touch sphere spawned")
        return True
    else:
        print(f"✗ Failed to spawn touch sphere: {result.stderr}")
        return False

def print_contact_details(msg):
    """Pretty print contact message details"""
    if not msg:
        print("No message to display")
        return
    
    print(f"\n{'='*50}")
    print("CONTACT DATA RECEIVED")
    print(f"{'='*50}")
    
    # Message header
    print(f"Timestamp: {msg.header.stamp.to_sec():.3f} seconds")
    print(f"Frame ID: {msg.header.frame_id}")
    print(f"Number of contacts: {len(msg.states)}")
    
    # Individual contacts
    for i, state in enumerate(msg.states):
        print(f"\n--- Contact {i+1} ---")
        print(f"  Collision1: {state.collision1_name}")
        print(f"  Collision2: {state.collision2_name}")
        
        # Contact positions (if available)
        if state.contact_positions:
            print(f"  Contact positions: {len(state.contact_positions)} point(s)")
            for j, pos in enumerate(state.contact_positions[:3]):  # Show first 3
                print(f"    Point {j+1}: ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f})")
        
        # Contact normals (if available)
        if state.contact_normals:
            print(f"  Contact normals: {len(state.contact_normals)} normal(s)")
        
        # Depths (if available)
        if state.depths:
            print(f"  Depth: {state.depths}")
        
        # Forces
        if state.total_wrench.force:
            f = state.total_wrench.force
            print(f"  Total force: ({f.x:.3f}, {f.y:.3f}, {f.z:.3f}) N")
        
        # Torques
        if state.total_wrench.torque:
            t = state.total_wrench.torque
            print(f"  Total torque: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) Nm")
    
    print(f"\n{'='*50}")

# ── original helpers ──────────────────────────────────────────────────────────

def kill_existing():
    for name in ["gzserver", "gzclient", "roscore", "rosmaster"]:
        subprocess.run(["pkill", "-f", name], capture_output=True)
    time.sleep(2)

def build_world():
    """Merge camera SDF into world file and write to BASE_WORLD."""
    tree = ET.parse(WORLD_PATH)
    root = tree.getroot()
    world = root.find('world')

    cam_tree = ET.parse(CAMERA_SDF)
    cam_root = cam_tree.getroot()
    for model in cam_root.findall('model'):
        world.append(model)

    os.makedirs(os.path.dirname(BASE_WORLD), exist_ok=True)
    tree.write(BASE_WORLD, encoding='utf-8', xml_declaration=True)
    print(f"[test] world written to {BASE_WORLD}")

def start_gazebo():
    cmd = f"source {CATKIN_SETUP} && roslaunch {SENSOR_PKG} {LAUNCH_FILE}"
    proc = subprocess.Popen(["bash", "-c", cmd],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    print(f"[test] gazebo pid={proc.pid}")
    return proc

def wait_gazebo_ready(timeout=30):
    """Wait until /gazebo/get_world_properties service is up."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            rospy.wait_for_service('/gazebo/get_world_properties', timeout=1.0)
            print("[test] Gazebo ready")
            return True
        except Exception:
            pass
    print("[test] Gazebo did NOT come up in time")
    return False

def capture_contact(topic, timeout):
    """Capture a single contact message from the topic"""
    print(f"[test] waiting for contact on {topic} (timeout={timeout}s)")
    
    # First check if topic exists
    topics = rospy.get_published_topics()
    topic_exists = any(t == topic for t, _ in topics)
    
    if not topic_exists:
        print(f"[test] ⚠️ Topic {topic} not found in published topics")
        print(f"[test] Available topics:")
        for t, typ in topics[:15]:
            if not t.startswith('/rosout'):
                print(f"      {t} -> {typ}")
        return None
    
    # Get message class
    msg_class, _, _ = rostopic.get_topic_class(topic)
    print(f"[test] Message class: {msg_class}")
    
    if msg_class is None:
        print("[test] ✗ Could not determine message class")
        # Try with explicit ContactsState
        try:
            print("[test] Trying with explicit ContactsState")
            msg = rospy.wait_for_message(topic, ContactsState, timeout=timeout)
            return msg
        except Exception as e:
            print(f"[test] ✗ Failed with ContactsState: {e}")
            return None
    
    try:
        msg = rospy.wait_for_message(topic, msg_class, timeout=timeout)
        print(f"[test] ✓ Got message of type {type(msg).__name__}")
        return msg
    except Exception as e:
        print(f"[test] ✗ Failed: {e}")
        return None

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    print("="*50)
    print("TACTILE SENSOR DIAGNOSTIC TEST")
    print("="*50)
    
    print("[test] killing any existing ROS/Gazebo processes")
    kill_existing()

    print("[test] starting roscore")
    roscore = subprocess.Popen(["roscore"],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
    time.sleep(2)

    print("[test] init rospy node")
    rospy.init_node('tactile_test', anonymous=True)
    
    # Run pre-flight checks
    check_plugin_availability()
    check_sdf_file()
    
    print("\n[test] building world")
    build_world()

    print("[test] starting gazebo")
    gz_proc = start_gazebo()

    if not wait_gazebo_ready(timeout=30):
        print("[test] ✗ Gazebo failed to start")
        gz_proc.terminate()
        roscore.terminate()
        return

    # Wait for plugins to initialize
    print("[test] waiting 5s for plugins to initialize")
    time.sleep(5)

    # Show what topics are available
    r = subprocess.run(["rostopic", "list"], capture_output=True, text=True, timeout=5)
    print(f"[test] active topics:\n{r.stdout}")
    
    # Check if our topic exists
    verify_message_type(TOPIC)
    
    # Check what models are loaded
    check_gazebo_models()
    
    # Ask user if they want to spawn a touch object
    response = input("\nSpawn a sphere to touch the sensor? (y/n): ")
    if response.lower() == 'y':
        spawn_touch_object()
        time.sleep(2)
    
    # Monitor the topic for contacts
    print("\n" + "="*50)
    print("ATTEMPTING TO CAPTURE CONTACT DATA")
    print("="*50)
    
    try:
        msg = capture_contact(TOPIC, timeout=TIMEOUT)
        
        if msg:
            print("\n[test] ✓ SUCCESS - Got message!")
            
            # Print detailed contact information
            print_contact_details(msg)
            
            # Show summary
            if msg.states:
                print(f"\n✅ Sensor is working! Detected {len(msg.states)} contact(s)")
            else:
                print("⚠️ Message received but no contacts (empty states)")
        else:
            print("\n[test] ✗ FAILED - No message received")
            
            # Suggest next steps
            print("\n=== Troubleshooting Suggestions ===")
            print("1. Check if the tactile sensor is properly placed in the world")
            print("2. Ensure something is touching the sensor surface")
            print("3. Verify the plugin loaded in Gazebo (check Gazebo terminal)")
            print("4. Try the topic /bumper_states if using no namespace")
            
    except Exception as e:
        print(f"\n[test] ✗ FAILED: {e}")
        import traceback
        traceback.print_exc()
    finally:
        response = input("\nKeep Gazebo running for inspection? (y/n): ")
        if response.lower() != 'y':
            gz_proc.terminate()
            roscore.terminate()
            print("[test] Cleanup complete")

if __name__ == "__main__":
    main()