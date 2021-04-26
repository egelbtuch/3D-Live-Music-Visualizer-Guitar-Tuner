import time
import sys
import pyaudio
from struct import unpack
import numpy as np
import math
import os
from bluetooth import *
#import matplotlib.pyplot as plt

# don't need rgbmatrix if we use samplebase (i.e. command line args)
from rgbmatrix import RGBMatrix, RGBMatrixOptions
from samplebase import SampleBase

#options = RGBMatrixOptions()
#options.rows = 64
#options.cols = 64
#options.hardware_mapping = 'adafruit-hat'  # If you have an Adafruit HAT: 'adafruit-hat'


class RunVisual(SampleBase):

    # Audio setup
    global no_channels
    no_channels = 1
    # Sample rate must satisfy nyquist criterion; 44100 typical for A/D audio (BW = 20kHz)
    global sample_rate
    sample_rate = 44100

    # Chunk must be a multiple of 8
    # NOTE: If chunk size is too small the program will crash
    # with error message: [Errno Input overflowed]
    global chunk
    chunk = 8192 

    global base
    global rows
    global buf_size
    buf_size = 2048;
    rows = 64

    # Use results from list_devices() to determine your microphone index
    
    global p
    p = pyaudio.PyAudio()
    global stream
    stream = p.open(format = pyaudio.paInt16,
                    channels = no_channels,
                    rate = sample_rate,
                    input = True,
                    frames_per_buffer = chunk,
                    input_device_index = 2)


    
    # command line arguments
    def __init__(self, *args, **kwargs):
        super(RunVisual, self).__init__(*args, **kwargs)

    def run(self):
        
        # Return power array index corresponding to a particular frequency
        def piff(val):
            return 2*4096*val/sample_rate
        
        # Initialize canvas
        offscreen_canvas = self.matrix.CreateFrameCanvas()
        
        # Create the client socket
        sock = BluetoothSocket(RFCOMM)
        sock.connect(('24:62:AB:FC:14:66', 1))
        bt_data = sock.recv(buf_size)
        rounds = 1
        '''
        sock.shutdown(socket.SHUT_RDWR)
        sock.close()
        '''
        while True:
            try:
                # counter used to prevent constant retrieval (leads to heavy lag)
                if (rounds % 50) == 0:
                    rounds = 1
                    bt_data = sock.recv(buf_size)
                    bt_data = bt_data.decode('utf-8')
                    bt_data = bt_data[len(bt_data)-1]
                
                # initialize bt_data for non bluetooth purposes
                #bt_data = 0
                
                freqs = {0:500, 1:2500, 2:5000, 3:10000, 4:15000, 5:20000}
                #print(freqs[int(bt_data)])
                
                # Get microphone data
                data = stream.read(chunk)
                #mapping = calculate_levels(data, chunk, sample_rate, 64)
                
                # Convert raw data (ASCII string) to numpy array
                data = unpack("%dh"%(len(data)/2),data)
                data = np.array(data, dtype='h')
                # Apply FFT - real data
                fourier = np.fft.rfft(data)
                # Remove last element in array to make it the same size as chunk
                fourier=np.delete(fourier, len(fourier)-1)
                # calculate power of spectrum
                power = np.abs(fourier)
                
                # tuning frequencies and their harmonics
                tuning_freqs = [164, 220, 146.8, 146.8*2, 196, 196*2, 246.9, 246.9*2, 329.6]
                
                # cut-off frequency for HPF
                offset = 20
                
                start_frequency = freqs[int(bt_data)]*offset/(offset+rows)
                #print(start_frequency)
                
                # Find average 'amplitude' for specific frequency ranges in Hz
                # Ignore frequencies below start_frequency to filter out motor noise
                amp = np.zeros(64).tolist()
                for i in range(0, rows):
                    amp[i] = int( np.mean( power[ int(math.floor(chunk*((i+offset)*freqs[int(bt_data)]/(rows+offset))/sample_rate)) :
                                                  int(math.ceil(chunk*((i+1+offset)*freqs[int(bt_data)]/(rows+offset))/sample_rate)) ] ) )       
                
                # label each tuning freuqency by its 'bin' (i.e., row number)
                bins = []
                for idx in range(0,len(tuning_freqs)):
                    bins.append( round((tuning_freqs[idx]/500) * (offset + rows) - offset) )
                #print(bins)

                # map amplitude for frequency ranges to columns 31-0, 32-63 on the board
                left_board = np.zeros(64, dtype=int).tolist()
                right_board = np.zeros(64, dtype=int).tolist()
                for j in range(0, rows):
                    left_board[j] = 31 - int(amp[j] * 31 // np.max(amp) ) 
                    right_board[j] = 32 + int(amp[j] * 31 // np.max(amp) )
                    
                    # for testing purposes, see which row has the most power
                    if (left_board[j] <= 4):
                        print('Frequency: {:.2f} Hz'.format( ((offset+j)/(rows+offset))*500  ) )

                # make sure canvas is reset
                offscreen_canvas.Clear()
                
                # turn on pixels
                color = { 500: (128, 128, 128), 2500: (128, 128, 0), 5000: (0, 0, 128), 10000: (128, 0, 0), 15000: (128, 0, 128), 20000: (0, 128, 128)  }
                for y in range(0, rows):
                    color_tuple = color[ freqs[int(bt_data)] ]
                    for x in range(32, left_board[y], -1):               
                        
                        if freqs[int(bt_data)] == 500:
                            for idx in range(0, len(bins)):
                                if left_board[bins[idx]] <= 4:
                                    offscreen_canvas.SetPixel(y, x, 0, 128, 0)
                                    break
                                else:
                                    offscreen_canvas.SetPixel(y, x, color_tuple[0], color_tuple[1], color_tuple[2])     
                        else:
                            offscreen_canvas.SetPixel(y, x, color_tuple[0], color_tuple[1], color_tuple[2])
                            
                    for z in range(32, right_board[y]):
                        
                        if freqs[int(bt_data)] == 500:
                            for idx in range(0, len(bins)):
                                if right_board[bins[idx]] >= 59:
                                    offscreen_canvas.SetPixel(y, z, 0, 128, 0)
                                    break
                                else:
                                    offscreen_canvas.SetPixel(y, z, color_tuple[0], color_tuple[1], color_tuple[2])                         
                        else:
                            offscreen_canvas.SetPixel(y, z, color_tuple[0], color_tuple[1], color_tuple[2])
                
                # update rounds
                rounds = rounds + 1
                #print(rounds)
                time.sleep(0.1)
                offscreen_canvas = self.matrix.SwapOnVSync(offscreen_canvas)             
            except KeyboardInterrupt:
                print("Ctrl-C Terminating...")
                stream.stop_stream()
                stream.close()
                p.terminate()
                sys.exit(1)
            except Exception as e:
                print("Error on line {}".format(sys.exc_info()[-1].tb_lineno))
                print(e)
                print("ERROR Terminating...")
                stream.stop_stream()
                stream.close()
                p.terminate()
                sys.exit(1)
                

# Main function
if __name__ == "__main__":
    run_visual = RunVisual()
    if (not run_visual.process()):
        run_visual.print_help()