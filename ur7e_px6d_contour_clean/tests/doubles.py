"""Device doubles used beneath the production RTDE owner and runtimes."""
import time
import numpy as np
from core.models import Wrench
from robot.rtde_controller import URRTDEController


class Receive:
    def __init__(self, pose, tcp):
        self.pose, self.tcp = np.array(pose, dtype=float), list(tcp)
        self.speed = np.zeros(6)
        self.connected = True
        self.frozen_stamp = None
        self.emergency = False
        self.protective = False
    def getActualTCPPose(self): return self.pose.copy()
    def getActualTCPSpeed(self): return self.speed.copy()
    def getTimestamp(self): return time.monotonic() if self.frozen_stamp is None else self.frozen_stamp
    def isConnected(self): return self.connected
    def isEmergencyStopped(self): return self.emergency
    def isProtectiveStopped(self): return self.protective
    def getRobotMode(self): return 7
    def getSafetyMode(self): return 1
    def disconnect(self): self.connected = False


class Control:
    def __init__(self, receive):
        self.receive, self.connected = receive, True
        self.calls = []
        self.stop_value = True
        self.stop_exception = None
        self.keep_moving = False
    def getTCPOffset(self): return self.receive.tcp
    def isConnected(self): return self.connected
    def speedL(self, velocity, acceleration, duration):
        self.calls.append(('speedL', list(velocity)))
        self.receive.speed = np.array(velocity)
        self.receive.pose += self.receive.speed*duration
        return True
    def moveL(self, target, speed, acceleration, asynchronous):
        self.calls.append(('moveL', list(target)))
        self.receive.pose = np.array(target)
        self.receive.speed[:] = 0
        return True
    def speedStop(self, deceleration):
        self.calls.append(('speedStop', deceleration))
        if not self.keep_moving: self.receive.speed[:] = 0
        if self.stop_exception: raise self.stop_exception
        return self.stop_value
    def stopL(self, deceleration, asynchronous):
        self.calls.append(('stopL', asynchronous))
        if not self.keep_moving: self.receive.speed[:] = 0
        return None
    def setWatchdog(self, hz): self.calls.append(('setWatchdog', hz)); return True
    def kickWatchdog(self): return True
    def stopScript(self): self.calls.append(('stopScript',)); return True
    def disconnect(self): self.connected = False


class Devices:
    def __init__(self, config, pose):
        self.receive = Receive(pose, config['tcp']['offset'])
        self.control = Control(self.receive)
        self.control_count = 0
        self.receive_count = 0
        self.owner = None
    def receive_factory(self, ip):
        self.receive_count += 1
        return self.receive
    def control_factory(self, ip):
        self.control_count += 1
        return self.control
    def controller(self, config):
        self.owner = URRTDEController(config, receive_factory=self.receive_factory, control_factory=self.control_factory)
        return self.owner


class Sensor:
    firmware = 'fake PX6D'
    def __init__(self, *args): self.connected = False; self.count = 0
    def connect(self): self.connected = True
    def close(self): self.connected = False
    def read_wrench(self):
        self.count += 1
        return Wrench(0., 0., 0., 0., 0., 0.)


class Keyboard:
    answer = 'START'
    key = None
    def __enter__(self): return self
    def __exit__(self, *args): pass
    def poll(self): return self.key
    def read_line(self, prompt): return self.answer
