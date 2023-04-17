import os, struct
import can

os.system('sudo ip link set can0 type can bitrate 1000000')
os.system('sudo ifconfig can0 up')

can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan')# socketcan_native

#msg = can.Message(arbitration_id=0x123, data=[0, 1, 2, 3, 4, 5, 6, 7], is_extended_id=False)
while(True):
    msg = can0.recv(10.0)
    if (msg.arbitration_id == 256):
        # print(bytes(msg.data))
        print ("Air temp:", int.from_bytes(bytes(msg.data), "big") / 10)
        print(msg)
    if msg is None:
        print('Timeout occurred, no message.')

os.system('sudo ifconfig can0 down')