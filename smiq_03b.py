import sys
import serial
import io
import time
import math

ARB_MEM_ADDR = 0  # the start in ARB memory where the waveform is going to be written

class SMIQ3B:
  def __init__(self, port, baudrate=115200, eol=b'\n'):
    self.ser = serial.Serial( port = port,
                              baudrate = baudrate,
                              bytesize = serial.EIGHTBITS,
                              parity = serial.PARITY_NONE,
                              stopbits = serial.STOPBITS_ONE,
                              rtscts = True,
                              timeout = 1)
    self.port = port
    self.eol = eol
    self.write(self.eol)  # switch SMIQ03B to remote RS232 mode
    time.sleep(0.1)       # wait for SMIQ03B to switch over before issuing other commands
    self.identify()
    

  def write(self, s):
    self.ser.write(s)

  # function reads one line from SMIQ03B until it encounters EOL character
  def readline(self):
    leneol = len(self.eol)
    line = bytearray()
    while True:
        c = self.ser.read(1)
        if c:
            line += c
            if line[-leneol:] == self.eol:
                break
        else:
            break
    return bytes(line)

  # function checks if we are actually talking to SMIQ03B device
  def identify(self):
    self.write('*IDN?' + self.eol)
    
    if 'SMIQ03B' in self.readline():
      print("Successfully connected to SMIQ03B on port {:s}".format(self.port))
    else:
      print("Can't connect to SMIQ03B")
      sys.exit(-1)

  def to_dac(self, s_float):
    if (s_float < -1.0) or (s_float > 1.0):
      raise ValueError, "Sample value {:f} out of <-1.0, 1.0> range".format(s_float)
    # +1.0 -> 64768
    # 0.0 -> 32768
    # -1.0 -> 768
    return int(32000.0*s_float + 32768.0)

  def write_waveform(self, name, clock_hz, samples):
    # convert samples to integer form as expected by DAC in AWG in SMIQ03B
    samples_dac = []
    for i,q in samples:
      samples_dac.append((self.to_dac(i), self.to_dac(q)))

    # convert DAC samples to waveform (two bytes value, little endian format)
    waveform = ''
    waveform = waveform + '{:d},#'.format(ARB_MEM_ADDR)
    for i,q in samples_dac:
      waveform = waveform + chr(i & 0xFF) + chr((i >> 8) & 0xFF)
      waveform = waveform + chr(q & 0xFF) + chr((q >> 8) & 0xFF)
  
    # write waveform data to SMIQ03B (this command is actually described in AMIQ operating manual)
    waveform_block = "{{TYPE: WV,0}}{{CLOCK: {:.4f}}}{{WAVEFORM-{:d}: {:s}}}".format(clock_hz, len(waveform), waveform)
    block_len = len(waveform_block)
    digits = int(math.log10(block_len)) + 1
    cmmd = ":MMEM:DATA '{:s}',#{:d}{:d}{:s}".format(name, digits, block_len, waveform_block)
    self.write(cmmd)
    self.write(';*opc?' + self.eol)
    if '1' in self.readline():
      print("Waveform successfully written")
    else:
      raise ValueError, "Error writing waveform"

if __name__ == '__main__':
  Smiq03b = SMIQ3B('COM1')
  # create I/Q samples for the waveform (first in floating point in range <-1.0, 1.0>
  samples = [ (-1.0, -1.0),
              (-0.5, -0.5),
              (0.0, 0.0),
              (0.5, 0.5),
              (1.0, 1.0)]
  # and write samples to SMIQ03B
  Smiq03b.write_waveform('TEST_2', 25000.0, samples)
