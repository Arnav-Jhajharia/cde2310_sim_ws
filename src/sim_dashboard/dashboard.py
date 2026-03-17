#!/usr/bin/env python3
"""
ROS 2 Web Dashboard - streams camera feeds, robot status, and teleop controls
over HTTP. Access from any browser, tunnel with ngrok for remote access.
"""

import threading
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from flask import Flask, Response, render_template, request, jsonify

# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class DashboardNode(Node):
    def __init__(self):
        super().__init__('web_dashboard')
        self.bridge = CvBridge()

        # Latest data stores
        self.frames = {
            'front': None,
            'left': None,
            'right': None,
        }
        self.odom = None
        self.scan = None
        self.locks = {
            'front': threading.Lock(),
            'left': threading.Lock(),
            'right': threading.Lock(),
            'odom': threading.Lock(),
            'scan': threading.Lock(),
        }

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Camera subscribers
        self.create_subscription(Image, '/camera_front/image_raw', self._cb_front, qos)
        self.create_subscription(Image, '/camera_left/image_raw', self._cb_left, qos)
        self.create_subscription(Image, '/camera_right/image_raw', self._cb_right, qos)

        # Odom subscriber
        self.create_subscription(Odometry, '/odom', self._cb_odom, qos)

        # LiDAR subscriber
        self.create_subscription(LaserScan, '/scan', self._cb_scan, qos)

        # Teleop publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Dashboard node started')

    # -- callbacks ----------------------------------------------------------

    def _img_cb(self, msg, name):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            _, jpeg = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            with self.locks[name]:
                self.frames[name] = jpeg.tobytes()
        except Exception as e:
            self.get_logger().warn(f'Frame error ({name}): {e}')

    def _cb_front(self, msg): self._img_cb(msg, 'front')
    def _cb_left(self, msg): self._img_cb(msg, 'left')
    def _cb_right(self, msg): self._img_cb(msg, 'right')

    def _cb_odom(self, msg):
        with self.locks['odom']:
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            v = msg.twist.twist
            self.odom = {
                'x': round(p.x, 3),
                'y': round(p.y, 3),
                'z': round(p.z, 3),
                'ox': round(o.x, 3),
                'oy': round(o.y, 3),
                'oz': round(o.z, 3),
                'ow': round(o.w, 3),
                'vx': round(v.linear.x, 3),
                'vy': round(v.linear.y, 3),
                'wz': round(v.angular.z, 3),
            }

    def _cb_scan(self, msg):
        with self.locks['scan']:
            self.scan = {
                'ranges': list(msg.ranges),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
            }

    # -- public API ---------------------------------------------------------

    def get_frame(self, name):
        with self.locks[name]:
            return self.frames.get(name)

    def get_odom(self):
        with self.locks['odom']:
            return self.odom

    def get_scan(self):
        with self.locks['scan']:
            return self.scan

    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    # -- navigation presets -------------------------------------------------

    active_nav = None  # current running nav thread
    nav_cancel = threading.Event()

    def cancel_nav(self):
        """Stop any running navigation preset."""
        self.nav_cancel.set()
        self.send_cmd(0, 0)

    def run_nav(self, steps):
        """
        Execute a sequence of (linear, angular, duration) steps.
        Each step drives at the given velocity for `duration` seconds.
        """
        self.nav_cancel.clear()
        for linear, angular, duration in steps:
            if self.nav_cancel.is_set():
                break
            end = time.time() + duration
            while time.time() < end and not self.nav_cancel.is_set():
                self.send_cmd(linear, angular)
                time.sleep(0.05)
        self.send_cmd(0, 0)

    def start_nav(self, preset_name):
        """Start a navigation preset in a background thread."""
        presets = {
            # (linear_vel, angular_vel, duration_sec)
            'forward_1m': [
                (0.15, 0, 6.5),     # ~1m at 0.15 m/s
            ],
            'backward_1m': [
                (-0.15, 0, 6.5),
            ],
            'turn_left_90': [
                (0, 0.5, 3.14),     # ~90 deg at 0.5 rad/s
            ],
            'turn_right_90': [
                (0, -0.5, 3.14),
            ],
            'turn_around': [
                (0, 0.5, 6.28),     # ~180 deg
            ],
            'explore_square': [
                (0.15, 0, 6.5),     # forward 1m
                (0, 0.5, 3.14),     # turn left 90
                (0.15, 0, 6.5),     # forward 1m
                (0, 0.5, 3.14),     # turn left 90
                (0.15, 0, 6.5),     # forward 1m
                (0, 0.5, 3.14),     # turn left 90
                (0.15, 0, 6.5),     # forward 1m
                (0, 0.5, 3.14),     # turn left 90
            ],
            'explore_room': [
                (0.15, 0, 10),      # forward ~1.5m
                (0, 0.5, 3.14),     # turn left 90
                (0.15, 0, 6.5),     # forward 1m
                (0, 0.5, 3.14),     # turn left 90
                (0.15, 0, 10),      # forward ~1.5m
                (0, -0.5, 3.14),    # turn right 90
                (0.15, 0, 6.5),     # forward 1m
                (0, -0.5, 3.14),    # turn right 90
                (0.15, 0, 10),      # forward ~1.5m
            ],
            'wander': [
                (0.12, 0, 5),
                (0, 0.4, 2),
                (0.12, 0, 8),
                (0, -0.6, 3),
                (0.12, 0, 5),
                (0, 0.3, 4),
                (0.12, 0, 6),
                (0, -0.4, 2.5),
                (0.12, 0, 7),
                (0, 0.5, 3),
            ],
        }
        steps = presets.get(preset_name)
        if not steps:
            return False
        # Cancel any running nav
        self.cancel_nav()
        time.sleep(0.1)
        self.nav_cancel.clear()
        t = threading.Thread(target=self.run_nav, args=(steps,), daemon=True)
        t.start()
        self.active_nav = t
        return True


# ---------------------------------------------------------------------------
# Flask App
# ---------------------------------------------------------------------------

ros_node = None
app = Flask(__name__, template_folder='templates')


@app.route('/')
def index():
    return render_template('index.html')


def mjpeg_stream(camera_name):
    """Generator that yields MJPEG frames."""
    while True:
        frame = ros_node.get_frame(camera_name)
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            # 1x1 gray pixel placeholder
            blank = np.zeros((120, 160, 3), dtype=np.uint8)
            cv2.putText(blank, 'No Signal', (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
            _, jpeg = cv2.imencode('.jpg', blank)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        time.sleep(0.033)


@app.route('/stream/<camera>')
def stream(camera):
    if camera not in ('front', 'left', 'right'):
        return 'Unknown camera', 404
    return Response(mjpeg_stream(camera),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/odom')
def api_odom():
    data = ros_node.get_odom()
    return jsonify(data or {})


@app.route('/api/scan')
def api_scan():
    data = ros_node.get_scan()
    if data:
        return jsonify({
            'angle_min': data['angle_min'],
            'angle_max': data['angle_max'],
            'range_min': data['range_min'],
            'range_max': data['range_max'],
            'num_ranges': len(data['ranges']),
            'ranges': data['ranges'],
        })
    return jsonify({})


@app.route('/api/cmd_vel', methods=['POST'])
def api_cmd_vel():
    body = request.get_json(force=True)
    linear = body.get('linear', 0.0)
    angular = body.get('angular', 0.0)
    ros_node.send_cmd(linear, angular)
    return jsonify({'ok': True})


@app.route('/api/nav', methods=['POST'])
def api_nav():
    body = request.get_json(force=True)
    preset = body.get('preset', '')
    ok = ros_node.start_nav(preset)
    if ok:
        return jsonify({'ok': True, 'preset': preset})
    return jsonify({'ok': False, 'error': 'Unknown preset'}), 400


@app.route('/api/nav/custom', methods=['POST'])
def api_nav_custom():
    body = request.get_json(force=True)
    linear = body.get('linear', 0.0)
    angular = body.get('angular', 0.0)
    duration = body.get('duration', 0.0)
    if duration <= 0 or duration > 120:
        return jsonify({'ok': False, 'error': 'Invalid duration'}), 400
    ros_node.cancel_nav()
    import time as _t; _t.sleep(0.1)
    ros_node.nav_cancel.clear()
    t = threading.Thread(
        target=ros_node.run_nav,
        args=([(linear, angular, duration)],),
        daemon=True,
    )
    t.start()
    ros_node.active_nav = t
    return jsonify({'ok': True, 'linear': linear, 'angular': angular, 'duration': round(duration, 2)})


@app.route('/api/nav/cancel', methods=['POST'])
def api_nav_cancel():
    ros_node.cancel_nav()
    return jsonify({'ok': True})


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    global ros_node

    rclpy.init()
    ros_node = DashboardNode()

    # Spin ROS in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    print('\n' + '=' * 50)
    print('  Dashboard running at http://0.0.0.0:5000')
    print('  Use ngrok: ngrok http 5000')
    print('=' * 50 + '\n')

    app.run(host='0.0.0.0', port=5000, threaded=True)


if __name__ == '__main__':
    main()
