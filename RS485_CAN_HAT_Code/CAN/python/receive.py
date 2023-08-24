import os, struct
import can

# os.system('sudo ip link set can0 type can bitrate 1000000')
# os.system('sudo ifconfig can0 up')

can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan')# socketcan_native

#msg = can.Message(arbitration_id=0x123, data=[0, 1, 2, 3, 4, 5, 6, 7], is_extended_id=False)
while(True):
    msg = can0.recv(10.0)
    print(msg)
    if (msg.arbitration_id == 323):
        print(bytes(msg.data))
        print (struct.unpack('<L', bytes(msg.data[0:2]))[0]  / 100000)
        print (struct.unpack('<', bytes(msg.data[0:2]))[0]  / 100000)
        # print(msg)
        pass
    if msg is None:
        print('Timeout occurred, no message.')

os.system('sudo ifconfig can0 down')