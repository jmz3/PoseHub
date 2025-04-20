import zmq
import json

# === ZMQ Setup ===
ZMQ_ADDRESS = "tcp://localhost:5601"  # Match the publisher's address
TOPICS = ["Probe", "StaticRef", "Anatomy"]  # Subscribe to specific topics, or use [""] for all

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect(ZMQ_ADDRESS)

# Subscribe to each topic
for topic in TOPICS:
    socket.setsockopt_string(zmq.SUBSCRIBE, topic)

print(f"Subscribed to topics: {TOPICS} on {ZMQ_ADDRESS}")

try:
    while True:
        try:
            topic, message = socket.recv_multipart(zmq.NOBLOCK)
            print("Topic:" + topic.decode() + "MSG:" + message.decode())
        except zmq.Again:
            pass

except KeyboardInterrupt:
    print("Subscriber interrupted.")

finally:
    socket.close()
    context.term()