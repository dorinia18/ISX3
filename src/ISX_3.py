try:
    import serial
except ImportError:
    print("Could not import module: serial")

from dataclasses import dataclass
from pyftdi.ftdi import Ftdi
from sciopy_dataclasses import FreqList, EisMeasurementSetup
from com_util import(
    clTbt_dp,
    clTbt_sp,
    del_hex_in_list,
    reshape_full_message_in_bursts,
    split_bursts_in_frames,
    uintTbt,
)
import numpy as np
import itertools
import struct
import csv
from datetime import datetime as dt


msg_dict = {
    "0x01": "Frame-Not-Acknowledge: Incorrect syntax",
    "0x02": "Timeout: Communication-timeout (less data than expected)",
    "0x04": "Wake-Up Message: System boot ready",
    "0x11": "TCP-Socket: Valid TCP client-socket connection",
    "0x81": "Not-Acknowledge: Command has not been executed",
    "0x82": "Not-Acknowledge: Command could not be recognized",
    "0x83": "Command-Acknowledge: Command has been executed successfully",
    "0x84": "System-Ready Message: System is operational and ready to receive data",
    "0x90": "Overcurrent Detected Value of DC current on W-ports exceeds capability of configured current range",
    "0x91": "Overvoltage Detected Value of DC voltage difference between R and WS port exceeds capability of configured voltage range",
    "0x92": "Data holdup: Measurement data could not be sent via the master interface",
}



class ISX_3:
    def __init__(self) -> None:

        self.serial_protocol = 'FS'
        self.device: Ftdi = None
        self.channel_group = None
        self.print_msg = True
        self.ret_hex_int = None
        self.sync_time: float = None  # sync time


        # HEX FE Setting
        self.hex_measurement_mode: str = None  # 4 point configuration : 0x02, 3 point configuration: 0x03, 2 point configuration: 0x01
        self.hex_measurement_chanel: str = None  # BNC Port (ISX-3mini: Port 1) : 0x01, ExtensionPort: 0x02, ExtensionPort2 (ISX-3mini: Port 2, ISX-3: optional, InternalMux): 0x03
        self.hex_range_setting: str = None  # autoranging : 0x00, +- 10mA (100) : 0x01, +- 100uA (10k) : 0x02, +- 1uA (1M) : 0x04, +- 10nA (100M) : 0x06


        # ExtensionPort Channel
        self.hex_counter: str = None  # (C-Port): Counter - Port Selection
        self.hex_reference: str = None  # (R-Port): Reference - Port Selection
        self.hex_working_sense: str = None  # (WS-Port): Working Sense - Port Selection
        self.hex_work: str = None  # (W-Port): Work - Port Selection


    def connect_device_FS(self, port: str, baudrate: int = 9600, timeout: int = 1):
        """
        Connect to full speed
        """
        if hasattr(self, "serial_protocol"):
            print(
                f"Serial connection 'self.serial_protocol' already defined as {self.serial_protocol}."
            )
        else:
            self.serial_protocol = "FS"
        self.device = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        print("Connection to", self.device.name, "is established.")

    def SystemMessageCallback(self):
        return self.SystemMessageCallback_usb_fs()

    def SystemMessageCallback_usb_fs(self):
        """
        !Only used if a full-speed connection is established!

        Reads the message buffer of a serial connection. Also prints out the general system message.
        """
        timeout_count = 0
        received = []
        received_hex = []
        data_count = 0
      
        while True:
            buffer = self.device.read()
            if buffer:
                received.extend(buffer)
                data_count += len(buffer)
                timeout_count = 0
                continue
            timeout_count += 1
            if timeout_count >= 1:
                # Break if we haven't received any data
                break

            received = "".join(str(received))  # If you need all the data
        received_hex = [hex(receive) for receive in received]
        print(received_hex)
        try:
            found_start_ack = False
            found_end_ack = False
            found_len_ack = False
            found_msg_ack = None
            for i in range(len(received_hex)):
                found_end_ack = received_hex[i] == "0x18" and found_msg_ack is not None
                if found_end_ack:
                    print(msg_dict[found_msg_ack])
                found_msg_ack = received_hex[i] if found_len_ack else None
                found_len_ack = received_hex[i] == "0x1" and found_start_ack
                found_start_ack = received_hex[i] == "0x18"      

        except BaseException:
            if self.print_msg:
                print(msg_dict["0x01"])
        if self.print_msg:
            return received_hex

        # Changed code above - to process the correct ACK-Frame   
        # try:
        #     msg_idx = received_hex.index("0x18")
        #     print(msg_idx)
        #     if self.print_msg:
        #         print(msg_dict[received_hex[msg_idx + 2]])
        # except BaseException:
        #     if self.print_msg:
        #         print(msg_dict["0x01"])
        #     # self.print_msg = False
        # if self.print_msg:
        #     print("message buffer:\n", received_hex)
        #     print("message length:\t", data_count)

        if self.ret_hex_int is None:
            return
        elif self.ret_hex_int == "hex":
            return received_hex
        elif self.ret_hex_int == "int":
            return received
        elif self.ret_hex_int == "both":
            return received, received_hex

    def SaveSettings(self):
        """
        Saves the following parameters permanently into the flash memory of the EIT system:
        - Ethernet parameters (DHCP, static IP)
        - WLAN parameters (SSID, DHCP, autoConnect, StaticIpSettings)
        - battery mode parameter (MeasureOnBat, minCapacity)
        - LED automode


        This command can only be used if no measurement is currently running.

        [CT] 00 [CT]

        Returns
        -------
        ACK : str

        """
        self.print_msg = True
        self.write_command_string(bytearray([0x90, 0x00, 0x90]))
        self.print_msg = False


    def SetOptions(self):
        """
        This command causes a reboot of the Sciospec EIS-Device.

        Returns
        -------
        ACK
        """

        self.print_msg = True
        self.write_command_string(bytearray([0x97, 0x00, 0x97]))
        self.print_msg = False

    def GetOptions(self):
        # 0x98
        self.print_msg = True
        self.write_command_string(bytearray([0x98, 0x00, 0x98]))
        self.print_msg = False

    def ResetSystem(self):
        # 0xA1
        self.print_msg = True
        self.write_command_string(bytearray([0x90, 0x00, 0x90]))
        self.print_msg = False

    def SetFE_Settings(self):
        self.print_msg = True
        self.write_command_string(bytearray(
            [0xB0,
             self.hex_measurement_mode,
             self.hex_measurement_chanel,
             self.hex_range_setting,
             0xB0]))
        self.print_msg = False

    def GetFE_Settings(self):
        self.print_msg = True
        self.write_command_string(bytearray([0xB1, 0x00, 0xB1]))
        self.print_msg = False

    def SetExtensionPortChannel(self):
        """
        This command causes a reboot of the Sciospec EIS-Device.

        See documentation of the connected extension module for detailed information.

        Returns
        -------
        ACK
        """
        self.print_msg = True
        self.write_command_string(bytearray([
            0xB2,
            self.hex_counter,
            self.hex_reference,
            self.hex_working_sense,
            self.hex_work,
            0xB2
        ]))
        self.print_msg = False

    def GetExtensionPortChannel(self):
        """
        0xB3 - Get ExtensionPort Channel
        Read the currently set ExtensionPort configuration.

        CP : counter port
        RP: reference port
        WS: working sense
        WP: work port

        Returns
        -------
        [CT] 04 [CP] [RP] [WS] [WP] [CT]
        ACK
        """
        self.print_msg = True
        self.write_command_string(bytearray([0xB3, 0x00, 0xB3]))
        self.print_msg = False

    def GetExtensionPortModule(self):
        """
        0xB5 - Get ExtensionPort Module
        Read the type of the currently connected extension module.


        LE  =   - 2 byte, unsigned integer
                - 4 byte, unsigned integer if ExtensionModule = Mux32any2any2202
                - 4 byte, unsigned integer if InternalModule = Mux32any2any2202
                - 6 byte, unsigned integer if ExtensionModule = Mux32any2any2202 and InternalModule = Mux32any2any2202

        ExtensionModule (This code represents the connected extension module.)
                no module connected : 0x00
                MEArack : 0x01
                MuxModule32 : 0x02
                ECIS Adapter : 0x03
                ExtensionPortAdapter : 0x05
                SlideChipAdapter: 0x06
                Mux32any2any (external) : 0x07
                DaQEisMux : 0x08
                Mux32any2any2202 (external) : 0x09

        InternalModule (This code represents the internal module.)
                no module connected : 0x00
                MuxModule16x4 : 0x01
                MuxModule32x2 : 0x02
                Mux32any2any (internal) : 0x07
                Mux32any2any2202 : 0x09

        optional ChannelCount* (2byte unsigned integer)


        Returns
        -------
        Optional channelCount is only valid in case of module = Mux32any2any2202
        [CT] [LE] [ExtensionModule] [InternalModule] [optional ChannelCount Ext*] [optional ChannelCount Int*] [CT]
        ACK
        """
        self.print_msg = True
        self.write_command_string(bytearray([0xB5, 0x00, 0xB5]))
        self.print_msg = False


    def SetSetup(self, EisSetup: EisMeasurementSetup):
        """
        0xB6 - Set Setup

        General syntax
        [CT] [LE] [OB] [CD] [CT]

        Returns
        -------
        ACK

        """
        def Init_Setup():
            """
            0x01 - Init
            This option resets the currently configured setup and an empty setup is initialized.

            Syntax
            [CT] 01 01 [CT]

            Returns
            -------
            ACK

            """
            return bytearray([0xB6, 0x01, 0x01, 0xB6])
           

        def Add_Sing_Freq(self):
            # not implemented
            """
            0x02 - Add single frequency point
            This command is used to add a single frequency point to the currently configured setup.

            Syntax
            [CT] [LE] 02 [CD] [CT]

            [CD]
            Frequency in Hz
                Length: 4Byte
                Data Format: float
            Precision
                Length: 4Byte
                Data Format: float
            Amplitude
                Voltage Excitation
                    Length: 4Byte
                    Data Format: float 
                Current Excitation
                    Length: 4Byte
                    Data Format: float  
            Point delay (EOP=01)
                Length: 4Byte
                Data Format: unsigned integer
            Use phase sync (EOP=02)
                Use: 0x0000001
                Do not use: 0x00000000
            Excitation type (EOP=03)
                voltage excitation: 0x00000001 (default)
                current excitation: 0x00000002
            
            Returns
            -------
            ACK

            """
            # self.print_msg = True
            # self.write_command_string(bytearray([
            #     0xB7,
            #     0x1C,
            #     0x02,
            #     self.FreqBlock.freq,
            #     self.FreqBlock.precision,
            #     self.FreqBlock.amplitude,
            #     0x01,
            #     self.FreqBlock.point_delay,
            #     0x02,
            #     self.FreqBlock.phase_sync,
            #     0x03,
            #     self.FreqBlock.exc_type,
            #     0xB7]))
            # self.print_msg = False
            pass
        
        def Add_Freq_List(freq_list: FreqList):
            """
            0x03 - Add frequency list
             This command is used to add multiple frequencies to the currently configured setup.

            Syntax
            [CT] [LE] 03 [CD] [CT]

            [CD]
            Start Frequency in Hz
                Length: 4Byte
                Data Format: float
            Stop Frequency in Hz
                Length: 4Byte
                Data Format: float
            Count
                Length: 4Byte
                Data Format: float rounded to the next smaller integer
            Scale
                Length: 1Byte
                Data Format: integer
                    linear: 0
                    logarithmic: 1
            Precision
                Length: 4Byte
                Data Format: float
            Amplitude
                Voltage Excitation in V
                    Length: 4Byte
                    Data Format: float 
                Current Excitation in A
                    Length: 4Byte
                    Data Format: float  
            Point delay (EOP=01)
                Length: 4Byte
                Data Format: unsigned integer
            Use phase sync (EOP=02)
                Use: 0x0000001
                Do not use: 0x00000000
            Excitation type (EOP=03)
                voltage excitation: 0x00000001 (default)
                current excitation: 0x00000002
            
            Returns
            -------
            ACK

            """
            print("hallo:", clTbt_sp(freq_list.stop_freq))
            print(clTbt_sp(freq_list.start_freq))

            print( bytearray(list(itertools.chain([
                0xB6,
                0x25,
                0x03],
                clTbt_sp(freq_list.start_freq),
                clTbt_sp(freq_list.stop_freq),
                clTbt_sp(freq_list.steps),
                [freq_list.scale],
                clTbt_sp(freq_list.precision),
                clTbt_sp(freq_list.current_amp),
                [0x01],
                uintTbt(freq_list.point_delay),
                [0x02],
                clTbt_sp(freq_list.phase_sync), #TODO find out what changes are needed to enable phase sync, error: Command could not be recognized
                [0x03],
                uintTbt(freq_list.exc_type),
                [0xB6]))))

            return bytearray(list(itertools.chain([
                0xB6,
                0x25,
                0x03],
                clTbt_sp(freq_list.start_freq),
                clTbt_sp(freq_list.stop_freq),
                clTbt_sp(freq_list.steps),
                [freq_list.scale],
                clTbt_sp(freq_list.precision),
                clTbt_sp(freq_list.current_amp),
                [0x01],
                uintTbt(freq_list.point_delay),
                [0x02],
                clTbt_sp(freq_list.phase_sync), #TODO find out what changes are needed to enable phase sync, error: Command could not be recognized
                [0x03],
                uintTbt(freq_list.exc_type),
                [0xB6])))
           
            

        def Set_All_Amp(self):
            # not implemented
            """
            0x05 -  Set Amplitude
            Syntax set all amplitudes with one command.

            Syntax
            [CT] 06 05 [ExcitationType] [Amplitude] [CT]

            [CD]
            Excitation Type
                Length: 1Byte
                Voltage Excitation: 0x01
                Current Excitation: 0x02
            Amplitude
                Length: 4Byte
                Data Format: float 
            
            Returns
            -------
            ACK

            """
            # self.print_msg = True
            # self.write_command_string(bytearray([
            #     0xB7,
            #     0x06,
            #     0x05,
            #     # excitation Type
            #     # Amplitude
            #     0xB7]))
            # self.print_msg = False
            pass

        def Set_Row_Amp(self):
            # not implemented
            """
            0x05 -  Set Amplitude row-wise
            Syntax set amplitudes row-wise.

            Syntax
            [CT] 08 05 [Row] [ExcitationType] [Amplitude] [CT]

            [CD]
            Row
                Length: 2Byte
                Data Format: probably int 
                0 <= row <= maximum row count
            Excitation Type
                Length: 1Byte
                Voltage Excitation: 0x01
                Current Excitation: 0x02
            Amplitude
                Length: 4Byte
                Data Format: float 
            
            Returns
            -------
            ACK

            """
            # self.print_msg = True
            # self.write_command_string(bytearray([
            #     0xB7,
            #     0x08,
            #     0x05,
            #     # Row
            #     # excitation Type
            #     # Amplitude
            #     0xB7]))
            # self.print_msg = False
            pass

        def Start_Compensation(self):
            # not implemented
            """
            0x10 -  Start Compensation
            This command starts the compensation of the setup.

            Syntax
            [CT] 01 10 [CT]
            
            Returns
            -------
            ACK

            """
            # self.print_msg = True
            # self.write_command_string(bytearray([
            #     0xB7,
            #     0x01,
            #     0x10,
            #     0xB7]))
            # self.print_msg = False
            pass

        def Compensation_Acknowledge(self):
            # not implemented
            """
            0x11 - Compensation Acknowledge
            This command starts the compensation of the setup.

            Syntax
            [CT] 01 10 [CT]
            
            Returns
            -------
            ACK

            """
            # self.print_msg = True
            # self.write_command_string(bytearray([
            #     0xB7,
            #     0x01,
            #     0x10,
            #     0xB7]))
            # self.print_msg = False
            pass

        # self.setup = EisSetup

        self.print_msg = True
        self.write_command_string(Init_Setup())
        self.write_command_string(Add_Freq_List(EisSetup.freq_list))
        # self.write_command_string([0xB6 , 0x20 , 0x03 , 0x44 , 0x7A , 0x00 , 0x00 , 0x4B , 0x18 , 0x96 , 0x80 , 0x41 , 0x20 , 0x00 , 0x00 , 0x01 , 0x3F , 0x80 , 0x00 , 0x00 , 0x3E , 0x80 , 0x00 , 0x00 , 0x01 , 0x00 , 0x00 , 0x03 , 0xE8 , 0x02 , 0x00 , 0x00 , 0x00 , 0x00 , 0xB6])
        self.print_msg = False        
        
    def GetSetup(self):
        """
        0xB7 - Get Setup
        This function is split into multiple functions/ commands to get information of the different settings of the setup.
        
        General syntax
        [CT] [LE] [OB] [CD] [CT]

        [OB]
            Get total number of frequencies: 0x01 
            TODO: add the rest
        Return
        [CT] [LE] [OB] [CD] [CT]
        -------
        ACK
        """
        def GetNbrFreq():
           """
           0x01 - Get total number of frequencies
           This command reads the total number of frequencies configured in the setup.

           Syntax
           [CT] 01 01 [CT]

           Return
           [CT] 03 01 [CD] [CT]

           [CD]
           number of rows configured
           Length: 2 byte
           Data format: unsigned integer 
           """
           return(bytearray([0xB7, 0x01, 0x01, 0xB7]))

        def GetFreqPoint():
            """
            0x02 -  Get information of frequency point
            This command gets information of a configured point of the setup.

            Syntax
            [CT] 03 02 [CD] [CT]

            [CD]
                Row number
                Length: 2 byte
                Data format: unsigned integer

            Return
            [CT] 0D 02 [Frequency] [Precision] [Signal amplitude] [CT]

            [Frequency]
                Length: 4 byte
                Data format: float
                Unit: Hz

            [Precision]
                Length: 4 byte
                Data format: float

            [Signal amplitude]
                Length: 4 byte
                Data format: float
                Voltage amplitude in V
                Current amplitude in A
            """
            #TODO: Add variable to define row -> here just row 1 chosen
            return(bytearray([0xB7, 0x03, 0x02,0x00, 0x01, 0xB7]))
        
        def GetFreqList():
            """
            0x02 - Get frequency list
            This command gets a list of frequencies configured in this setup.

            Syntax
            [CT] 01 04 [CT]

            Return
            [CT] [LE] 04 [4 Byte float frequency1] […] [4 Byte float frequencyN] [CT]

            [4 Byte float frequency1] […] [4 Byte float frequencyN]
                In the case of "get frequency list" cmd one data frame is limited to a total of 252 bytes of data.
                The returning command will be split into multiple separate frames, if this number is exceeded.
                For example if the setup contains 64 frequency points the first 62 will be transmitted in a frame containing 249 bytes (=62*4+1)
                  and in a separate frame containing 9 bytes (=2*4+1) bytes of data.

            """
            return(bytearray([0xB7, 0x01, 0x04, 0xB7]))
  
        self.print_msg = True
        self.write_command_string(GetNbrFreq())
        #self.write_command_string(GetFreqPoint())
        self.write_command_string(GetFreqList())
        self.print_msg = False

    def SetSyncTime(self):
        """
        Set the synchronization time in us. (Time between the measurement of two spectra)

        Note
        ----
        This is also the time the SyncOut signal is in its low state between two measurements. Use save settings command (0x90) to save this parameter persistent.

        Sync time
            - Length: 4 byte
            - Data format: integer value
            - Unit: us
            - Default: 0us
            - Min: 0us
            - Max 180s = 180E6 us (0x0ABA9500)


        Returns
        -------
        ACK
        """

        # TODO: Format sync time to hex as descript above


        raise NotImplemented
        hex_st = format_sync_time_to_hex(self.sync_time)

        self.print_msg = True
        self.write_command_string(bytearray([
            0xB9,
            hex_st,
            0xB9]))
        self.print_msg = False


    def GetSyncTime(self):
        """
        Reads the currently configured synchronization time.

        - Length: 4 byte
        - Data format: integer value
        - Unit: us

        Returns
        -------
        [CT] 04 [Sync time] [CT]
        ACK
        """
        #TODO ?
        self.print_msg = True
        self.write_command_string(bytearray([0xBA, 0x00, 0xBA]))
        self.print_msg = False


    def SetEthernetConfiguration(self):
        #0xBD
        raise NotImplemented

    def GetEthernetConfiguration(self):
        # 0xBE
        raise NotImplemented

    def write_command_string(self, command):
        """
        Function for writing a command 'bytearray(...)' to the serial port
        """
        self.device.write(command)
        return self.SystemMessageCallback()

    def GetDeviceID(self):
        """
        0xD1 - Get Device ID
        Reads out device information.
        Includes all information necessary to identify a Sciospec device.

        Syntax
        [CT] 00 [CT]    

        Returns
        -------
        [CT] [LE] [general information] [developer information] [CT]
        ACK

        [general information]
        This information can also be found on the serial number label on the case of the device.
            version of the general information part
                Length: 1 Byte
                Data Format: unsigned Integer
            device identifier
                Length: 2 Byte
                Data Format: unsigned Integer   
            serial number
                Length: 2 Byte
                Data Format: unsigned Integer   
            date of delivery
                Length: 2 Byte
                Byte 6: Year since 2010 (0x00 = 2010, ..., 0xFF = 2265)
                Byte 7: Month (0x01 = january, ..., 0x0C = december)   

        [developer information]
        This information is for internal development purposes only.   
        """
        self.print_msg = True
        message = self.write_command_string(bytearray([0xD1, 0x00, 0xD1]))
        self.print_msg = False
        message = message[:-4]
        int_list = [int(hex_str, 16) for hex_str in message[2:9]]
        hex_strings = "".join([f"{i:02X}" for i in int_list])
        hex_strings= str(hex_strings)[::-1]
        groups = [hex_strings[i:i+4] for i in range(0, len(hex_strings), 4)]
        formatted_string = "-".join(groups)[::-1]
        print(f"Device ID: {formatted_string}")

    def GetFPGAFirmwareID(self):
        """
        0xD2 - Get FPGA firmware ID
        Reads out version number of ARM firmware.

        [developer information]
            This information is for internal development purposes only.
            Length: 5 Byte
        [revision number]
            Length: 2 Byte
            Data format: unsigned integer
        [build number]
            Length: 2 Byte
            Data format: unsigned integer

        Returns
        -------
        D2 09 [developer information] [revision number] [build number] D2
        ACK

        """
        # 0xD2
        self.print_msg = True
        self.write_command_string(bytearray([0xD2, 0x00, 0xD2]))
        self.print_msg = False

    def GetExtensionPortChannel(self):
        # 0xD3 TODO: WHERE DEFINED ?
        pass

    def StartMeasure(self, EisSetup: EisMeasurementSetup):
        
        """
        # 0xB8 - Start Measure
        Starts the measurement. 

        Syntax
        [CT] [LE] [OP] [CD] [CT]

        Return
        If time stamp and current range are disabled (see command 0x97 and 0x98)
            [CT] 0A [ID] [Real part] [Imaginary part] [CT]
        Else if time stamp in ms is enabled (see command 0x97 and 0x98)
            [CT] 0E [ID] [Time stamp] [Real part] [Imaginary part] [CT]
        Else if time stamp in µs is enabled (see command 0x97 and 0x98)
            [CT] 0F [ID] [Time stamp] [Real part] [Imaginary part] [CT]
        Else if current range is enabled (see command 0x97 and 0x98)
            [CT] 0B [ID] [Current Range] [Real part] [Imaginary part] [CT]
        Else if time stamp and current range are enabled (see command 0x97 and 0x98)
            [CT] 0F [ID] [Time stamp] [Current Range] [Real part] [Imaginary part] [CT]

        [ID]
        ID number of the frequency point
            Length: 2 byte
            Data format: unsigned integer

        [Time stamp]
        time stamp of the frequency point
            Length: 4 byte or 5 byte (depending on configured time stamp)
            Data format: unsigned integer
            Unit: ms or us (depending on configured time stamp)
 
        [Current Range]
        the current range of that frequency point; in case of auto ranging this could change from point to point
            Length 1 byte
            Data format: unsigned integer

        [Real part]
        real part of impedance
            Length: 4 byte
            Data format: float

        [Imaginary part]
        imaginary part of impedance
            Length: 4 byte
            Data format: float

        """
        def StopMeasurement():
        
            """"
            0x00 - Stop Measurement
            Stops the measurement

            Syntax
            [CT] [01] [01] [00] [CT]
            """
            pass

        def StartMeasurement(EisSetup: EisMeasurementSetup):

            """"
            0x01 - Start Measurement
            Starts Measurement 

            Syntax
            [CT] [03] [01] [CD] [CT]

            [CD] - Repeat
            Number of spectra to be measured
                Length: 2byte
                Data format: integer value
                The setting 0 starts a continuous measurement. 
                    Send the command (B8 01 00 B8) to stop the continuous run.  
            
            Example: B8 03 01 00 01 B8 - to start a measurement and stop it automatically after one measurement spectra per channel configuration.

            """
            repeat = clTbt_sp(EisSetup.repeat)
            print(repeat)

            return (bytearray(list(itertools.chain(
                    [0xB8,
                    0x03,
                    0x01,
                    0x00],
                    repeat[:2],
                    [0xB8]))))
        
        def parse_data(data):
            #not implemented yet

            pass
        
        def store_data(parsed_data, path):
            """
            store measurement data in .csv at location defined in path
            """
            # open the CSV-file
            with open(path, "w", newline = '') as file: # parameter "w" to write a new file or "a" to append to an existing file

                print(f"CSV-file opened:{path}") 

                writer = csv.writer(file, delimiter=';') # start of the CSV-writers
                writer.writerow(['Measurement '+(dt.now().strftime("%d.%m.%Y"))+" "+(dt.now().strftime("%H:%M:%S"))]) # this row marks the start of a new measurement
                writer.writerow(['TBD']) # header for the CSV file


        
        self.print_msg = True
        # self.write_command_string(StopMeasurement())
        print('Measurement started.')
        print(StartMeasurement(EisSetup))
        data = self.write_command_string(StartMeasurement(EisSetup))
        
        self.print_msg = False  

# 0xBD - Set Ethernet Configuration
# 0xBE - Get Ethernet Configuration
# 0xCF - TCP connection watchdog
# 0xD0 - Get ARM firmware ID




