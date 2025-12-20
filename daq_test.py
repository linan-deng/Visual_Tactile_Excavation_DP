import time

import nidaqmx
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import numpy as np

def daq_read():
    # Check if task needs to update the graph
    samplesAvailable = daqTask._in_stream.avail_samp_per_chan
    vals = daqTask.read(samplesAvailable)
    vals = np.array(vals)
    print(vals)


daqTask = nidaqmx.Task()
maxVoltage = 10
minVoltage = -10
sampleRate = 5000
samplesPerChannel = 100000
dev_name = "PXI2Slot4/"
for i in range(21):
    daqTask.ai_channels.add_ai_voltage_chan(dev_name + 'ai' + str(i), min_val=minVoltage,
                                            max_val=maxVoltage,
                                            terminal_config=nidaqmx.constants.TerminalConfiguration.DIFFERENTIAL)
# 每个通道读取多组，提高读取速度
daqTask.timing.cfg_samp_clk_timing(sampleRate, sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS,
                                   samps_per_chan=samplesPerChannel)

# daq_read_Timer = QTimer()
# daq_read_Timer.timeout.connect(daq_read)
# daq_read_Timer.start(1)

# spin off call to check
while True:
    daqTask.start()
    time.sleep(1)
    daq_read()
    daqTask.stop()
    time.sleep(1)

