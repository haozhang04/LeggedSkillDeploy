import socket
from threading import Thread, Event
from .common import pretty_print_obj
from .highState import highState

listenPort = 8090
sendPort_low = 8007
sendPort_high = 8082

local_ip_wifi = '192.168.12.222'
local_ip_eth = '192.168.123.12'  #my pc
addr_wifi = '192.168.12.1'
addr_low = '192.168.123.10'  #nano can not connect
addr_high = '192.168.123.161' #pi


LOW_WIRED_DEFAULTS = (listenPort, addr_low, sendPort_low, local_ip_eth)
# LOW_WIFI_DEFAULTS = (listenPort, addr_low, sendPort_low, local_ip_wifi)
# HIGH_WIRED_DEFAULTS = (listenPort, addr_high, sendPort_high, local_ip_eth)
# HIGH_WIFI_DEFAULTS = (listenPort, addr_wifi, sendPort_high, local_ip_wifi)

class unitreeConnection:
    def __init__(self, settings=LOW_WIRED_DEFAULTS):
        self.listenPort = settings[0]
        self.addr = settings[1]
        self.sendPort = settings[2]
        self.localIP = settings[3]
        self.sock = self.connect()
        self.runRecv = Event()
        self.recvThreadID = None
        self.data = []
        self.data_callback = None  # 数据接收回调函数

    def set_data_callback(self, callback):
        """设置数据接收回调函数"""
        self.data_callback = callback

    def startRecv(self):
        self.recvThreadID = Thread(target=self.recvThread, args=(self.runRecv,))
        self.recvThreadID.daemon = True
        self.recvThreadID.start()

    def stopRecv(self):
        self.runRecv.set()
        self.recvThreadID.join()

    def connect(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 端口复用
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)  # 接收缓冲区
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1048576)  # 发送缓冲区
        sock.bind((self.localIP, self.listenPort))
        sock.settimeout(1)
        return sock

    def send(self, cmd):
        self.sock.sendto(cmd, (self.addr, self.sendPort))

    def recvThread(self, event):
        """UDP接收线程：持续接收数据并触发回调
        """
        # print('[*] Start receive Thread ...\n')
        while not event.isSet():
            try:
                recv_data = self.sock.recv(2048)
                
                # 如果设置了回调函数，直接调用处理
                if self.data_callback is not None:
                    self.data_callback(recv_data)
                else:
                    # 否则存入缓冲区（兼容旧的getData方式）
                    self.data.append(recv_data)
                    
            except Exception as e:
                print(f"[ERROR] unitreeConnection receive exception: {e}")
                pass
        # print('[*] Exited receive Thread ...')

    def getData(self):
        ret = self.data.copy()
        # Clear data buffer after handing it out
        self.data.clear()
        return ret
