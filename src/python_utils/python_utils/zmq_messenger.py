import numpy as np
import zmq
import threading


class ZMQSubscriber:
    """
    Creates a thread that subscribes to a ZMQ publisher
    """

    def __init__(self, ip_address="tcp://192.168.1.3:2096", verbose=False):
        context = zmq.Context()
        self._sub_socket = context.socket(zmq.SUB)
        self._sub_socket.setsockopt(zmq.CONFLATE, False)
        self._sub_socket.connect(ip_address)
        self._sub_socket.setsockopt(zmq.SUBSCRIBE, b'')

        self._subscriber_thread = threading.Thread(target=self._update_value)
        self._subscriber_thread.start()

        self._value = None
        self.verbose = verbose
        self.last_message = None

    @property
    def message(self):
        if self._value is None and self.verbose:
            print("The subscriber has not received a message")
        self.last_message = self._value
        return self._value
    
    def _update_value(self):
        while(True):
            message = self._sub_socket.recv()
            self._value = np.frombuffer(message).astype(np.float32)


class ZMQPublisher:
    """
    Creates a thread that publishes to a ZMQ subscriber
    """

    def __init__(self, ip_address="tcp://192.168.1.3:2096"):
        context = zmq.Context()
        self._pub_socket = context.socket(zmq.PUB)
        self._pub_socket.bind(ip_address)

        self.last_message = None

    def send_message(self, message):
        self.last_message = message
        self._pub_socket.send(message.astype(np.float64).tobytes())

    
