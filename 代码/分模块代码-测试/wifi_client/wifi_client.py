import socket
import time

# 服务器配置
HOST = '192.168.4.1'  # ESP32的IP地址
PORT = 8080           # 端口号

# 创建socket连接
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print(f"正在连接到 {HOST}:{PORT}...")
client.connect((HOST, PORT))
print("连接成功！")

try:
    while True:
        # 接收数据
        data = client.recv(1024).decode('utf-8')
        if data:
            print(f"收到数据: {data}")
except KeyboardInterrupt:
    print("\n正在断开连接...")
    client.close()