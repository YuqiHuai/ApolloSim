import json
import os.path

import threading

from loguru import logger
from flask import Flask, render_template
from flask_socketio import SocketIO

class SimRender:
    def __init__(self, data_queue, map_file, host='0.0.0.0', port=18888, debug=False):
        self.data_queue = data_queue
        self.map_file = map_file
        self.map_name = os.path.basename(os.path.dirname(self.map_file))
        self.host = host
        self.port = port
        self.debug = debug
        self.running = True

        async_mode = "eventlet" #None
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'secret!'
        # async_mode='eventlet', cors_allowed_origins="*"
        self.socketio = SocketIO(self.app, async_mode=async_mode, logger=False, engineio_logger=False, cors_allowed_origins="*")
        self._setup_routes()
        self._setup_socketio_events()

    def _setup_routes(self):


        @self.app.route('/')
        def index():
            """Load map data and pass it to the index template."""
            # Find the global min and max values for x and y
            min_x = float('inf')
            min_y = float('inf')
            max_x = float('-inf')
            max_y = float('-inf')

            try:
                with open(self.map_file, 'r') as f:
                    map_data = json.load(f)

                lanes = map_data['lanes']
                for lane in lanes:
                    for point in lane['polygon']:
                        x, y = point
                        min_x = min(min_x, x)
                        min_y = min(min_y, y)
                        max_x = max(max_x, x)
                        max_y = max(max_y, y)

            except FileNotFoundError:
                map_data = {"error": "Map file not found"}
            except json.JSONDecodeError:
                map_data = {"error": "Error decoding JSON"}

            # Pass the map data to the template
            return render_template(
                "index.html",
                map_data=map_data,
                min_x=min_x,
                min_y=min_y,
                max_x=max_x,
                max_y=max_y
            )

    def _background_thread(self):
        """Background task to read data from the pipe and emit to the client."""
        while self.running:
            try:
                if not self.data_queue.empty():
                    data = self.data_queue.get()  # Retrieve data from the queue
                    self.socketio.emit('update', data)
            except Exception as e:
                print("Error in background_task:", e)  # Log any potential errors
            finally:
                self.socketio.sleep(0.01)  # Sleep briefly to prevent high CPU usage

    def _setup_socketio_events(self):
        @self.socketio.event
        def connect():
            _ = self.socketio.start_background_task(self._background_thread)

    def run(self):
        """Run the Flask-SocketIO server in a background thread."""
        def start_server():
            logger.info(f"Server starting on {self.host}:{self.port}")
            self.running = True
            self.socketio.run(self.app, host=self.host, port=self.port, debug=self.debug, use_reloader=False, log_output=False)
            logger.info(f"Server stopped on {self.host}:{self.port}")
        # Start the server in a separate thread
        server_thread = threading.Thread(target=start_server, daemon=True)
        server_thread.start()

    def is_running(self):
        """Check if the server is running."""
        return self.running

    def close(self):
        self.running = False
        self.socketio.stop()

def mock_data_sender(pipe):
    import time
    """Simulate sending data to the server at regular intervals."""
    while True:
        # logger.debug("Generate mock data")
        # Mock data representing a vehicle
        # x = float(random.random() * 10)
        # mock_data = {
        #     'frame': 1,
        #     'game_time': 0,
        #     'real_time': 0,
        #     'map_name': 'borregas_ave',
        #     'actors': [
        #         {
        #             "id": "vehicle_1",
        #             "category": "vehicle",
        #             "polygon": [
        #                 [587021.4181594849 + x, 4141590.2582378387],
        #                 [587021.4181594849 + x, 4141590.2582378387 + 2],
        #                 [587021.4181594849 + 4 + x, 4141590.2582378387 + 2],
        #                 [587021.4181594849 + 4 + x, 4141590.2582378387]
        #             ],
        #             "speed": 5.5,
        #             "acceleration": 0.2,
        #             "role": "agent"
        #         }
        #     ],
        #     'traffic_lights': []
        # }
        mock_data = {
            'map_name': 'borregas_ave',
            'frame': 32,
            'game_time': 1.28,
            'real_time': 1.7057383060455322,
            'actors': [
                {
                    'id': 0,
                    'category': 'vehicle.lincoln.mkz',
                    'bbox': {'length': 4.933, 'width': 2.11, 'height': 1.48},
                    'location': {'x': 587039.4818971977, 'y': 4141520.44803457, 'z': 0.0, 'pitch': 0.0,
                                 'yaw': 1.4881686950747754, 'roll': 0.0},
                    'speed': 0.0,
                    'angular_speed': 0.0,
                    'acceleration': 0.0,
                    'control': {'throttle': 0.0, 'brake': 0.145, 'steering': 0.0},
                    'role': 'ads',
                    'last_location': {'x': 587039.4818971977, 'y': 4141520.44803457, 'z': 0.0, 'pitch': 0.0,
                                      'yaw': 1.4881686950747754, 'roll': 0.0},
                    'polygon': [
                        [587038.751552434, 4141524.4118359685],
                        [587038.3444139739, 4141519.4956659884],
                        [587040.4472152425, 4141519.321520002],
                        [587040.8543537027, 4141524.237689982]
                    ],
                },
                # Add other actors following the same structure
            ],
            'traffic_lights': [
                {
                    'id': 'signal_0',
                    'category': 'signal.traffic_light',
                    'state': 'red',
                    'role': 'traffic_light',
                    'stop_line': [
                        [587063.8222551346, 4141576.7195044756],
                        [587066.028377533, 4141584.8203845024],
                        [587068.7553367615, 4141595.1673812866]
                    ],
                    'conflicts': ['signal_1', 'signal_2', 'signal_3', 'signal_4', 'signal_5', 'signal_6', 'signal_7',
                                  'signal_8', 'signal_10', 'signal_11', 'signal_12'],
                    'equals': ['signal_9', 'signal_13', 'signal_14']
                },
                # Add other traffic lights following the same structure
            ]
        }

        # print("Sending mock data")
        # Send the mock data through the pipe
        pipe.send(mock_data)
        time.sleep(1)  # Wait 1 second before sending the next update


if __name__ == '__main__':
    import multiprocessing

    # Set up the data pipe
    p, c = multiprocessing.Pipe()

    # Path to your map file
    # mf = "/Users/mingfeicheng/Desktop/PhD/Github/dev/ApolloSim_private/drive_test/scenario_runner/ApolloSim/data/borregas_ave/map.json"
    mf = "/data/c/mingfeicheng/dev/ApolloSim_private/drive_test/scenario_runner/ApolloSim/data/borregas_ave/map.json"
    # Start the server
    r = SimRender(p, mf, debug=True)

    # # Start the mock data sender in a separate process
    sender_process = multiprocessing.Process(target=mock_data_sender, args=(c,))
    sender_process.start()

    # Run the Flask-SocketIO server
    r.run()

    # Ensure cleanup on exit
    sender_process.join()
