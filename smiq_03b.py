import serial
import io

if __name__ == '__main__':
  PORT = 'COM1'
  BAUDRATE = 115200
  ser = serial.Serial(  port=PORT,
                        baudrate=BAUDRATE,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        rtscts = True,
                        timeout=1)
  ser_io = io.TextIOWrapper(io.BufferedRWPair(ser, ser, 1),  
                            newline = '\n',
                            line_buffering = True)
  # switch SMIQ 03B to remote RS232
  ser_io.write(unicode('\n'))
                            
  # identify SMIQ03B
  ser_io.write(unicode('*IDN?\n'))
  self_id = ser_io.readline()

  print(self_id)
