import time
import foxglove

# Start WebSocket server
foxglove.start_server(host="0.0.0.0", port=8765)

counter = 0
while True:
    foxglove.log("/counter", {"count": counter, "timestamp": time.time()})
    print(f"Published count: {counter}", flush=True)
    counter += 1
    time.sleep(1.0)
