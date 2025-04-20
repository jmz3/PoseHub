import time
import zmq
import json
import os
from sksurgerynditracker.nditracker import NDITracker
import numpy as np

# === Configuration ===
SETTINGS = {
    "tracker type": "polaris",
    "romfiles" : [
        "SceneGraphMarkers/artool.rom",
        "SceneGraphMarkers/HoloRef.rom",
        "SceneGraphMarkers/phantom.rom",
    ],
    "use quaternions": "true"
}

TRACKING_HZ = 50  # Adjust frequency here
ZMQ_ADDRESS = "tcp://*:5603"  # Or "tcp://127.0.0.1:5555" for local only

# === Initialize Tracker ===
tracker = NDITracker(SETTINGS)
tracker.start_tracking()

# === Prepare ZMQ Publisher ===
context = zmq.Context()
publisher = context.socket(zmq.PUB)
publisher.bind(ZMQ_ADDRESS)
publisher.setsockopt(zmq.SNDHWM, 50)

# === Get ROM names from paths (for topic names) ===
rom_topics = ["Probe","StaticRef","Anatomy"]

print(f"Publishing tracking data at {TRACKING_HZ} Hz on {ZMQ_ADDRESS}...\nTopics: {rom_topics}")

try:
    while True:
        start_time = time.time()

        _, timestamps, framenumbers, tracking, quality = tracker.get_frame()

        for i, t in enumerate(tracking):
            topic = rom_topics[i]
            has_nan_or_none = np.isnan(t).any() or np.any(t == None)
            t = np.append(t,int(has_nan_or_none))
            s = ','.join(map(str, t.flatten()))
            publisher.send_multipart([topic.encode(), s.encode()])
        elapsed = time.time() - start_time
        sleep_time = max(0.0, 1.0 / TRACKING_HZ - elapsed)
        time.sleep(sleep_time)

except KeyboardInterrupt:
    print("\nStopping tracking...")

finally:
    tracker.stop_tracking()
    tracker.close()
    publisher.close()
    context.term()
