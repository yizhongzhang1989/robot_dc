from abc import ABC

class ModbusDevice(ABC):
    def __init__(self, device_id, send_request_fn, recv_request_fn):
        self.device_id = device_id
        self.send = send_request_fn
        self.recv = recv_request_fn

