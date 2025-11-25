import zmq

class ZmqInterface:
    def __init__(self, tx_port, rx_port):
        robot_localhost = "0.0.0.0"
        context = zmq.Context()
        self.tx_socket = context.socket(zmq.PUB)
        self.tx_socket.setsockopt(zmq.SNDHWM, 500)
        self.tx_socket.bind(f"tcp://{robot_localhost}:{tx_port}")

        self.rx_socket = context.socket(zmq.PULL)
        self.rx_socket.setsockopt(zmq.RCVHWM, 500)
        self.rx_socket.bind(f"tcp://{robot_localhost}:{rx_port}")

    def send(self, msg: str):
        self.tx_socket.send_string(msg)

    def receive(self):
        try:
            return self.rx_socket.recv_string(flags=zmq.NOBLOCK)
        except zmq.Again:
            return None
