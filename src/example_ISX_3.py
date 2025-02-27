from ISX_3 import ISX_3
import sciopy_dataclasses as sdc
from sciopy_dataclasses import FreqList
import numpy as np
import struct
from com_util import (clTbt_sp, uintTbt,bytesarray_to_float)



eis = sdc.EisMeasurementSetup(
    freq_list= FreqList(
        #freq = None,
        start_freq=100,
        stop_freq=100,
        steps=1,
        scale=1,
        precision=1.0,
        volt_amp=None,
        current_amp=0.01,
        point_delay=1000,
        phase_sync=0,
        exc_type=2
    ),
    repeat=1,
    time_stamp_ms=1

)
Sciospec = ISX_3()
Sciospec.hex_counter = 3


Sciospec.connect_device_FS("COM3")
Sciospec.GetDeviceID()
Sciospec.SetOptions(eis)

Sciospec.SetSetup(eis)
Sciospec.GetSetup()
# print(bytesarray_to_float(b'\xb6%\x03B\xc8\x00\x00CH\x00\x00?\x80\x00\x00\x01?\x80\x00\x00<#\xd7\n\x01\x00\x00\x03\xe8\x02\x00\x00\x00\x00\x03\x00\x00\x00\x02\xb6'))

Sciospec.StartMeasure(eis)
