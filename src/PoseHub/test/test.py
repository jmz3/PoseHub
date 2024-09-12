import zmq
import time

def main():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5588")

    topic = "example"
    message = "Hello, World!"

    # Ensure the socket is ready
    time.sleep(1)

    while True:
        socket.send_string(topic, zmq.SNDMORE)
        socket.send_string(message)
        time.sleep(1)

if __name__ == "__main__":
    main()