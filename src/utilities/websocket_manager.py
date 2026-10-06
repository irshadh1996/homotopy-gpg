import json
import threading
import websocket
import numpy as np
from utilities.websocket_quarternion import quaternion_to_euler_yaw

class RoboWebsocketManager:
    def __init__(self, robot_configs, n_players, cmd_vel_type, odom_type):
        self.configs = robot_configs
        self.n_players = n_players
        self.cmd_vel_type = cmd_vel_type
        self.odom_type = odom_type
        
        self.odom_data = {}
        self.ws_clients = {}
        self.data_lock = threading.Lock()
        
        self.connected_events = {c['id']: threading.Event() for c in self.configs}
        self.all_initial_received = threading.Event()
        self.shutdown_event = threading.Event()

    def start(self):
        self.threads = []
        for config in self.configs:
            t = threading.Thread(target=self._run_client, args=(config,), daemon=True)
            t.start()
            self.threads.append(t)

    def _run_client(self, config):
        r_id = config['id']
        ws = websocket.WebSocketApp(
            config['websocket_url'],
            on_open=lambda ws: self._on_open(ws, config),
            on_message=lambda ws, msg: self._on_message(ws, msg, r_id),
            on_error=lambda ws, err: print(f"WS Error Robot {r_id}: {err}"),
            on_close=lambda ws, code, msg: self.shutdown_event.set()
        )
        self.ws_clients[r_id] = ws
        ws.run_forever()

    def _on_open(self, ws, config):
        ws.send(json.dumps({"op": "advertise", "topic": config['cmd_vel_topic'], "type": self.cmd_vel_type}))
        ws.send(json.dumps({"op": "subscribe", "topic": config['odom_topic'], "type": self.odom_type}))
        self.connected_events[config['id']].set()

    def _on_message(self, ws, message, r_id):
        try:
            data = json.loads(message)
            if data.get('op') == 'publish':
                msg = data.get('msg')
                with self.data_lock:
                    self.odom_data[r_id] = msg
                    if not self.all_initial_received.is_set():
                        if len(self.odom_data) == self.n_players:
                            self.all_initial_received.set()
        except Exception as e:
            print(f"Error in on_message for Robot {r_id}: {e}")

    def get_x0(self):
        with self.data_lock:
            if not all(i in self.odom_data for i in range(1, self.n_players + 1)):
                return None
            current_x0 = []
            for i in range(1, self.n_players + 1):
                odom_msg = self.odom_data[i]
                pos = odom_msg['pose']['pose']['position']
                ori = odom_msg['pose']['pose']['orientation']
                theta = quaternion_to_euler_yaw(ori['x'], ori['y'], ori['z'], ori['w'])
                current_x0.extend([pos['x'], pos['y'], theta])
            return np.array(current_x0)

    def publish_velocities(self, u_to_publish):
        sorted_configs = sorted(self.configs, key=lambda x: x['id'])
        for i, config in enumerate(sorted_configs):
            ws = self.ws_clients.get(config['id'])
            if ws and ws.sock and ws.sock.connected:
                msg = {
                    "op": "publish",
                    "topic": config['cmd_vel_topic'],
                    "msg": {
                        "linear": {"x": float(u_to_publish[i, 0]), "y": 0.0, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": float(u_to_publish[i, 1])}
                    }
                }
                ws.send(json.dumps(msg))

    def stop(self):
        self.shutdown_event.set()
        for r_id, ws in self.ws_clients.items():
            if ws and ws.sock and ws.sock.connected:
                ws.close()