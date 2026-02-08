import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from flask import Flask, request, jsonify
import threading

app = Flask(__name__)
ros_node = None

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Robot Arm Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; text-align: center; padding: 20px; background: #1a1a2e; color: #eee; }
        h1 { color: #e94560; }
        .slider-container { margin: 30px auto; max-width: 400px; }
        .slider-label { font-size: 24px; margin: 10px 0; }
        input[type=range] { width: 100%; height: 30px; }
        .angle-display { font-size: 48px; color: #e94560; font-weight: bold; }
        .status { margin-top: 20px; color: #0f3460; background: #16213e; 
                  padding: 10px; border-radius: 8px; }
        .preset-btn { padding: 15px 30px; margin: 5px; font-size: 18px; 
                      border: none; border-radius: 8px; cursor: pointer;
                      background: #0f3460; color: #eee; }
        .preset-btn:hover { background: #e94560; }
    </style>
</head>
<body>
    <h1>Robot Arm Control</h1>
    
    <div class="slider-container">
        <div class="slider-label">Servo Angle</div>
        <div class="angle-display" id="angleVal">90</div>
        <input type="range" id="slider" min="0" max="180" value="90"
               oninput="sendAngle(this.value)">
    </div>
    
    <div>
        <button class="preset-btn" onclick="sendAngle(0)">0°</button>
        <button class="preset-btn" onclick="sendAngle(45)">45°</button>
        <button class="preset-btn" onclick="sendAngle(90)">90°</button>
        <button class="preset-btn" onclick="sendAngle(135)">135°</button>
        <button class="preset-btn" onclick="sendAngle(180)">180°</button>
    </div>
    
    <div class="status" id="status">Ready</div>

    <script>
        let lastSend = 0;
        function sendAngle(angle) {
            document.getElementById('angleVal').innerText = angle;
            document.getElementById('slider').value = angle;
            
            const now = Date.now();
            if (now - lastSend < 50) return;
            lastSend = now;
            
            fetch('/set_angle?angle=' + angle)
                .then(r => r.json())
                .then(d => document.getElementById('status').innerText = 'Sent: ' + angle + '°')
                .catch(e => document.getElementById('status').innerText = 'Error: ' + e);
        }
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return HTML_PAGE

@app.route('/set_angle')
def set_angle():
    angle = request.args.get('angle', type=int)
    if angle is not None and 0 <= angle <= 180:
        msg = Int32()
        msg.data = angle
        ros_node.publisher.publish(msg)
        ros_node.get_logger().info(f'Published angle: {angle}')
        return jsonify({'status': 'ok', 'angle': angle})
    return jsonify({'status': 'error'}), 400

class WebGUI(Node):
    def __init__(self):
        super().__init__('web_gui')
        self.publisher = self.create_publisher(Int32, 'servo_angle', 10)
        self.get_logger().info('Web GUI node started on port 5000')

def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = WebGUI()
    
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000), daemon=True)
    flask_thread.start()
    
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
