# import socket

# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# s.connect(("172.22.22.2", 30002))

# command = "movep(p[-0.6566, 0.2770, 0.0550, 0.6423, -3.0742, 0.0293], a=1.2, v=0.25, r=0.025)".encode()

# s.send(command)

# data = s.recv(1024)
# s.close()
# print(data.decode())

import socket
import time

HOST = "172.22.22.2"   # The remote host
PORT = 30002              # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.send ("set_analog_inputrange(0, 0)".encode() + "\n".encode())
data = s.recv(1024)
s.close()
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.send ("set_analog_inputrange(1, 0)".encode() + "\n".encode())
data = s.recv(1024)
s.close()
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.send ("set_analog_outputdomain(0, 0)".encode() + "\n".encode())
data = s.recv(1024)
s.close()
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.send ("set_analog_outputdomain(1, 0)".encode() + "\n".encode())
data = s.recv(1024)
s.close()
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.send ("set_tool_voltage(24)".encode() + "\n".encode())
data = s.recv(1024)
s.close()
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.send ("set_runstate_outputs([])".encode() + "\n".encode())
data = s.recv(1024)
s.close()
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.send ("set_payload(0.0)".encode() + "\n".encode())
data = s.recv(1024)
s.close()
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

s.send ("set_gravity([0.0, 0.0, 9.82])".encode() + "\n".encode())

data = s.recv(1024)
s.close()
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))


s.send ("movej([0.4166615605354309, -1.6050642172442835, 2.164191246032715, 3.720980167388916, -4.021045986806051, -0.6397879759417933], a=1.3962634015954636, v=1.0471975511965976)".encode() + "\n".encode())

# time.sleep(10)
# s.send ("movej(p[0.0000000000000000, 0.4300000000000000, 0.4000000000000000, 0.0000000000000000, 2.0000000000000000, 0.0000000000000000], a=1.3962634015954636, v=1.0471975511965976)".encode() + "\n".encode())

# time.sleep(10)
# s.send ("movej(p[0.0000000000000000, 0.6300000000000000, 0.4000000000000000, 0.0000000000000000, 2.0000000000000000, 0.0000000000000000], a=1.3962634015954636, v=1.0471975511965976)".encode() + "\n".encode())

data = s.recv(1024)
s.close()
print ("Received", repr(data))