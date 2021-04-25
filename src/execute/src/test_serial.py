from serial import Serial
import time

sp = Serial('COM2',115200,timeout=0.5)

def talker(sp):
    
    hello_str = b'\x0a\xff\x12'
    sp.write(hello_str.encode('utf-8'))
    sp.read
    time.sleep(0.5)

if __name__ == '__main__':
    talker(sp)
    sp.close()