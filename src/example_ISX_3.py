from ISX_3 import ISX_3
import sciopy_dataclasses as sdc
from sciopy_dataclasses import FreqList
import numpy as np
import struct
from com_util import (clTbt_sp, uintTbt,bytesarray_to_float)



eis = sdc.EisMeasurementSetup(
    freq_list= FreqList(
        #freq = None,
        start_freq=500, #ok
        stop_freq=10000, #ok
        steps=100, #ok
        scale=1, #TODO find out why always linear
        precision=3.0, #ok # low: 0.0; medium: 1.0; high: 2.0; very high: 3.0
        volt_amp=None,
        current_amp=0.0001,
        point_delay=1000,
        phase_sync=0,
        exc_type=2 #Excitation type not changed in GUI

    ),
    repeat=1, #nbr. of repeat is not changed in GUI
    time_stamp_ms=0

)
Sciospec = ISX_3()
Sciospec.hex_counter = 3


Sciospec.connect_device_FS("COM3")
Sciospec.GetDeviceID()
Sciospec.SetOptions(eis)

Sciospec.GetOptions()

Sciospec.SetSetup(eis)
Sciospec.GetSetup()


#Sciospec.StartMeasure(eis)
