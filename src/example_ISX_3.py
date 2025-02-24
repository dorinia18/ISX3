from ISX_3 import ISX_3
import sciopy_dataclasses as sdc
from sciopy_dataclasses import FreqList
import numpy as np
import struct
from com_util import (clTbt_sp, uintTbt)



eis = sdc.EisMeasurementSetup(
    freq_list= FreqList(
        #freq = None,
        start_freq=1000,
        stop_freq=10000000,
        steps=10,
        scale=1,
        precision=1.0,
        volt_amp=None,
        current_amp=0.01,
        point_delay=1000,
        phase_sync=0,
        exc_type=2
    ),
    repeat=200

)
Sciospec = ISX_3()
Sciospec.hex_counter = 3

Sciospec.connect_device_FS("COM3")
Sciospec.GetDeviceID()

Sciospec.SetSetup(eis)
Sciospec.StartMeasure(eis)
