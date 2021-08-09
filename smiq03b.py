import sys
import serial
import io
import time
import math
import cmath
import argparse

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
    self.identify()
    self.min_clock_rate_Hz = 5e6  # minimum clock rate that will make sure that analog antialiasing filter is working fine
    self.max_clock_rate_Hz = 40e6 # maximum possible clock rate

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

  def calc_clock_rate(self, base_clock_Hz):
    oversampling = 10 # initial oversampling
    clock_Hz = oversampling * base_clock_Hz
    if (clock_Hz < self.min_clock_rate_Hz):
      # clock rate less that minimum, so incrase oversampling
      oversampling = int(math.ceil(self.min_clock_rate_Hz/base_clock_Hz))
      clock_Hz = oversampling * base_clock_Hz
    elif (clock_Hz > self.max_clock_rate_Hz):
      print("WARNING !!! Required clock rate is {:f}Hz, but maximum allowed is {:f}Hz".format(clock_Hz, self.max_clock_rate_Hz))
      oversampling = int(math.floor(self.max_clock_rate_Hz/base_clock_Hz))
      clock_Hz = oversampling * base_clock_Hz
    return clock_Hz

  def calc_fm(self, Fm_Hz, Fd_Hz):
    # first calculate the frequency of I/Q samples clock
    base_clock_Hz = max(Fm_Hz, Fd_Hz) # minimal clock rate that would do the job
    clock_Hz = self.calc_clock_rate(base_clock_Hz)
    samples_in_period = int(clock_Hz / Fm_Hz)
    mod_index = Fd_Hz / Fm_Hz
    samples = []
    for sample in range(samples_in_period):
      phi = -mod_index * math.cos(2.0 * math.pi * (float(sample)/float(samples_in_period))) # amplitude is constant in FM, calculate phi
      # convert to I/Q pair
      i = math.cos(phi)
      q = math.sin(phi)
      samples.append((i, q))
    return samples, clock_Hz

  '''
  Function calculates samples for N tones with given separation
  '''
  def calc_n_tone(self, N, separation_Hz):
    base_clock_Hz = separation_Hz * N * 2 # to satisfy Nyquist condition, sampling clock rate needs to be
                                          # at least twice the highest frequency of the tone
    clock_Hz = self.calc_clock_rate(base_clock_Hz)
    ampl = 1.0/N  # the tones will get add up, so make sure that the amplitude of the sum will not exceed 1.0
    samples_in_period = int(clock_Hz / separation_Hz)
    samples = []
    j = complex(0.0, 1.0)
    for sample in range(samples_in_period):
      val = complex(0.0, 0.0)
      for f in range(1, N+1):
        phi = 2.0 * cmath.pi * float(f) * (float(sample)/float(samples_in_period))
        val = val + cmath.exp(-j * phi)
      # scale amplitude so we don't exceed 1.0
      val = val / float(N)
      samples.append((val.real, val.imag)) # create I/Q pair
    return samples, clock_Hz

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
  parser = argparse.ArgumentParser()
  parser.add_argument('--port', default = 'COM1', help = 'Serial port name to use to connect to SMIQ03B')
  parser.add_argument('--wav_name', default = 'TEST_1', help = 'Waveform name')
  subparser = parser.add_subparsers(dest = 'mod')
  # subparser for modulation 'FM'
  fm = subparser.add_parser('FM')
  fm.add_argument('--Fm_Hz', type = float, help = 'FM modulation frequency in Hz')
  fm.add_argument('--Fd_Hz', type = float, help = 'FM deviation frequency in Hz')

  # subparser for modulation 'N tone'
  n_tone = subparser.add_parser('N_TONE')
  n_tone.add_argument('--N', type = int, help = 'Number of tones to generate')
  n_tone.add_argument('--separation_Hz', type = float, help = 'Separation between tones in Hz')

  # parse the arguments
  args = parser.parse_args()

  # connect to SMIQ03B
  Smiq03b = SMIQ3B(args.port)

  # create I/Q samples for the waveform depending on what kind of modulation type is required
  if (args.mod == 'FM'):
    samples, clock = Smiq03b.calc_fm(args.Fm_Hz, args.Fd_Hz)
  elif (args.mod == 'N_TONE'):
    samples, clock = Smiq03b.calc_n_tone(args.N, args.separation_Hz)
  else:
    raise ValueError, "Invalid modulation type {:s}".format(args.mod)

  # and write samples to SMIQ03B
  Smiq03b.write_waveform(args.wav_name, clock, samples)
