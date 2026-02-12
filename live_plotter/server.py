import http.server
import webbrowser
import threading
import time
import json
from pathlib import Path
import paho.mqtt.client as mqtt
from functools import partial
import ast

BASE_DIR = Path(__file__).parent
SERIAL_PATH = Path(__file__).parent / "serial_replay" / "serial.txt"
USE_MQTT = False        # Enables live MQTT mode, otherwise utilizes serial replay
MQTT_HOST = "mqtt"      # Broker hostname
MQTT_PORT = 1884        # Broker port

class SSEBroker:
    def __init__(self):
        self.clients = set()
        self.lock = threading.Lock()

    def add(self, handler):
        with self.lock:
            self.clients.add(handler)

    def remove(self, handler):
        with self.lock:
            self.clients.discard(handler)

    def broadcast(self, data):
        payload = f"data: {json.dumps(data)}\n\n".encode()
        with self.lock:
            dead_clients = set()

            for client in self.clients:
                try:
                    client.wfile.write(payload)
                    client.wfile.flush()
                except Exception as e:
                    print(f"SSE broadcast error: {e}")
                    dead_clients.add(client)

            self.clients -= dead_clients

broker = SSEBroker()

class RequestHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        pass
    
    def do_GET(self): 
        # SSE endpoint       
        if self.path == "/stream":
            self.handle_SSE()
            return
        
        super().do_GET()

    def handle_SSE(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        broker.add(self)

        try:
            self.wfile.write(b": connected\n\n")
            self.wfile.flush()

            while True:
                self.wfile.write(b": keepalive\n\n")
                self.wfile.flush()
                time.sleep(15)
        except Exception:
            pass
        finally:
            broker.remove(self)

def filter_telemetry(values):
    return {
        "stage": values["is_sustainer"],
        "lat": values["latitude"],
        "lon": values["longitude"],
        "alt": values["barometer_altitude"],
        "fsm": values["FSM_State"]
    }

def start_mqtt_listener(broker):
    def on_message(client, userdata, msg):
        raw = json.loads(msg.payload)
        values = raw["data"]["value"]
        data = filter_telemetry(values)
        broker.broadcast(data)

    client = mqtt.Client()
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe("FlightData-Sustainer")
    client.subscribe("FlightData-Booster")
    client.loop_forever()

def start_serial_replay(broker, path, rate_hz=10, verbose=False):
    delay = 1.0 / rate_hz

    with open(path) as f:
        for lineno, line in enumerate(f, start=1):
            line = line.strip()
            if not line.startswith("data:"):
                continue

            payload_str = line[len("data:"):].strip()

            try:
                payload = ast.literal_eval(payload_str)
                if not (isinstance(payload, dict) and payload.get("type") == "data"):
                    continue

                values = payload.get("value")
                if not values:
                    continue

                data = filter_telemetry(values)
                broker.broadcast(data)

            except Exception as e:
                if verbose:
                    print(f"Line {lineno} skipped: parse error ({e})")

            time.sleep(delay)

if __name__ == "__main__":
    handler = partial(RequestHandler, directory=str(BASE_DIR))
    server = http.server.ThreadingHTTPServer(("localhost", 0), handler)
    host, port = server.server_address
    print(f"Server running on http://{host}:{port}")
    webbrowser.open(f"http://localhost:{port}")

    server_thread = threading.Thread(
        target=server.serve_forever, 
        daemon=True).start()

    if USE_MQTT:
        data_thread = threading.Thread(
            target=start_mqtt_listener,
            args=(broker,),
            daemon=True
        )
    else:
        data_thread = threading.Thread(
            target=start_serial_replay,
            args=(broker, SERIAL_PATH,),
            daemon=True
        )

    data_thread.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping server")
    finally:
        server.shutdown()
        server.server_close()