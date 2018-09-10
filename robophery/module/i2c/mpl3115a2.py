import math
from robophery.interface.i2c import I2cModule



# MPL3115A2 Regster Map
MPL3115A2_REG_STATUS				= 0x00 # Sensor status Register
MPL3115A2_REG_PRESSURE_MSB			= 0x01 # Pressure data out MSB
MPL3115A2_REG_PRESSURE_CSB			= 0x02 # Pressure data out CSB
MPL3115A2_REG_PRESSURE_LSB			= 0x03 # Pressure data out LSB
MPL3115A2_REG_TEMP_MSB				= 0x04 # Temperature data out MSB
MPL3115A2_REG_TEMP_LSB				= 0x05 # Temperature data out LSB
MPL3115A2_REG_DR_STATUS				= 0x06 # Data Ready status registe
MPL3115A2_OUT_P_DELTA_MSB			= 0x07 # Pressure data out delta MSB
MPL3115A2_OUT_P_DELTA_CSB			= 0x08 # Pressure data out delta CSB
MPL3115A2_OUT_P_DELTA_LSB			= 0x09 # Pressure data out delta LSB
MPL3115A2_OUT_T_DELTA_MSB			= 0x0A # Temperature data out delta MSB
MPL3115A2_OUT_T_DELTA_LSB			= 0x0B # Temperature data out delta LSB
MPL3115A2_REG_WHO_AM_I				= 0x0C # Device Identification Register
MPL3115A2_PT_DATA_CFG				= 0x13 # PT Data Configuration Register
MPL3115A2_CTRL_REG1					= 0x26 # Control Register-1
MPL3115A2_CTRL_REG2					= 0x27 # Control Register-2
MPL3115A2_CTRL_REG3					= 0x28 # Control Register-3
MPL3115A2_CTRL_REG4					= 0x29 # Control Register-4
MPL3115A2_CTRL_REG5					= 0x2A # Control Register-5

# MPL3115A2 PT Data Configuration Register
MPL3115A2_PT_DATA_CFG_TDEFE			= 0x01 # Raise event flag on new temperature data
MPL3115A2_PT_DATA_CFG_PDEFE			= 0x02 # Raise event flag on new pressure/altitude data
MPL3115A2_PT_DATA_CFG_DREM			= 0x04 # Generate data ready event flag on new pressure/altitude or temperature data

# MPL3115A2 Control Register-1 Configuration
MPL3115A2_CTRL_REG1_SBYB			= 0x01 # Part is ACTIVE
MPL3115A2_CTRL_REG1_OST				= 0x02 # OST Bit ACTIVE
MPL3115A2_CTRL_REG1_RST				= 0x04 # Device reset enabled
MPL3115A2_CTRL_REG1_OS1				= 0x00 # Oversample ratio = 1
MPL3115A2_CTRL_REG1_OS2				= 0x08 # Oversample ratio = 2
MPL3115A2_CTRL_REG1_OS4				= 0x10 # Oversample ratio = 4
MPL3115A2_CTRL_REG1_OS8				= 0x18 # Oversample ratio = 8
MPL3115A2_CTRL_REG1_OS16			= 0x20 # Oversample ratio = 16
MPL3115A2_CTRL_REG1_OS32			= 0x28 # Oversample ratio = 32
MPL3115A2_CTRL_REG1_OS64			= 0x30 # Oversample ratio = 64
MPL3115A2_CTRL_REG1_OS128			= 0x38 # Oversample ratio = 128
MPL3115A2_CTRL_REG1_RAW				= 0x40 # RAW output mode
MPL3115A2_CTRL_REG1_ALT				= 0x80 # Part is in altimeter mod
MPL3115A2_CTRL_REG1_BAR				= 0x00 # Part is in barometer mode


class Mpl3115a2Module(I2cModule):
    """
    Module for MPL3115A2 temperature and pressure sensor.

    derived from https://github.com/ControlEverythingCommunity/MPL3115A2/tree/master/Python

    """
    DEVICE_NAME = 'mpl3115a2'
    # MPL3115A2 default address
    DEVICE_ADDR = 0x60





    # # Operating modes
    # HOLD_MASTER = 0x00
    # NOHOLD_MASTER = 0x10


    def __init__(self, *args, **kwargs):
        super(Mpl3115a2Module, self).__init__(*args, **kwargs)
        self._data = self._setup_i2c_iface(kwargs.get('data'))

        #TODO: confirm we don't require this , from original htu21d.py
        # # Check that mode is valid.
        # self._mode = kwargs.get('mode', self.HOLD_MASTER)
        # if self._mode not in [self.HOLD_MASTER, self.NOHOLD_MASTER]:
        #     raise ValueError('Unexpected mode value {0}.'.format(self._mode))
        



        """
        Setup for Absolute pressure measurement
        Select the Control Register-1 Configuration from the given provided value"""
        CONTROL_CONFIG = (MPL3115A2_CTRL_REG1_SBYB | MPL3115A2_CTRL_REG1_OS128)
        self._data.write8( MPL3115A2_CTRL_REG1, CONTROL_CONFIG)

        #TODO: Implement ALtitude measurement switch 
        # CONTROL_CONFIG = (MPL3115A2_CTRL_REG1_SBYB | MPL3115A2_CTRL_REG1_OS128 | MPL3115A2_CTRL_REG1_ALT)
        # self._data.write8( MPL3115A2_CTRL_REG1, CONTROL_CONFIG)


    def crc_check(self, msb, lsb, crc):
        remainder = ((msb << 8) | lsb) << 8
        remainder |= crc
        divsor = 0x988000
        for i in range(0, 16):
            if remainder & 1 << (23 - i):
                remainder ^= divsor
            divsor >>= 1
        if remainder == 0:
            return True
        else:
            return False


    def get_raw_pres_temp(self):
        """Read data back from MPL3115A2_REG_STATUS(0x00), 4 bytes
        status, pres MSB, pres CSB, pres LSB"""
        status = 0x08

        # Is Data Ready */
        # if status & 0x08:
        #     raise RuntimeError("Data not ready")
       
        pollingExit = False
        retries = 10
        while (status & 0x08) or pollingExit:
            status, pmsb, pcsb, plsb  = self._data.readList(MPL3115A2_REG_STATUS, 4)
            retries -= 1
            pollingExit = (retries==0)  #TODO: do some form or continous polling prevention

        if pollingExit:
            raise RuntimeError('i2c: mpl3115a2 retried polling pres_temp')

        #TODO: is there any CRC check possible        
        # if self.crc_check(msb, lsb, chsum) is False:
        #     raise RuntimeError("CRC Exception")
        # raw = (msb << 8) + lsb
        # raw &= 0xFFFC

        # Convert the data to 20-bits
        rawPres = ((pmsb * 65536) + (pcsb * 256) + (plsb & 0xF0)) / 16
        rawTemp = 0xFF #TODO: adjust readList to 6 and calculate rawTemp
        #rawTemp = ((tmsb * 256) + (tlsb & 0xF0)) / 16

        return rawPres, rawTemp


    def get_pressure(self):
        """
        Gets the pressure  in unit: Pa .
        """
        rawPres, rawTemp = self.get_raw_pres_temp()

        pressure = (rawPres / 4.0) / 1000.0
        
        return  pressure

    def get_raw_alt_temp(self):
        """Read data back from MPL3115A2_REG_STATUS(0x00), 6 bytes
        status, tHeight MSB, tHeight CSB, tHeight LSB, temp MSB, temp LSB"""
    
        status = 0x08
        # Is Data Ready */
        # if status & 0x08:
        #     raise RuntimeError("Data not ready")
        pollingExit = False
        retries = 10
        while (status & 0x08) or pollingExit:
            status, pmsb, pcsb, plsb, tmsb, tlsb  = self._data.readList(MPL3115A2_REG_STATUS, 6)
            retries -= 1
            pollingExit = (retries==0)  #TODO: do some form or continous polling prevention

        if pollingExit:
            raise RuntimeError('i2c: mpl3115a2 retried polling alt_temp')


        
        # Convert the data to 20-bits
        rawtHeight = ((plsb * 65536) + (pcsb * 256) + (plsb & 0xF0)) / 16
        rawtemp = ((tmsb * 256) + (tlsb & 0xF0)) / 16
        
        altitude = rawtHeight / 16.0
        cTemp = rawtemp / 16.0
        fTemp = cTemp * 1.8 + 32
        
        return {'a' : altitude, 'c' : cTemp, 'f' : fTemp}
   

    def get_temperature(self):
        """
        Gets the temperature in degrees celsius.
        """
        result = self.get_raw_alt_temp()
        temp = result['c']
        
        return temp

    def get_altitude(self):
        """
        Gets the altitude in m.
        """
        result = self.get_raw_alt_temp()
        altitude = result['a']
        return altitude


    def read_data(self):
        """
        Get all sensor readings.
        """
        temp_time_start = self._get_time()
        try:
            temp = self.get_temperature()
        except IOError:
            temp = None
        temp_time_stop = self._get_time()
        temp_time_delta = temp_time_stop - temp_time_start

        pressure_time_start = self._get_time()
        try:
            pressure = self.get_pressure()
        except IOError:
            pressure = None
        pressure_time_stop = self._get_time()
        pressure_time_delta = pressure_time_stop - pressure_time_start
        data = [
            (self._name, 'temperature', temp, temp_time_delta),
            (self._name, 'pressureAbsolute', pressure, pressure_time_delta),
        ]
        self._log_data(data)
        return data

    def meta_data(self):
        """
        Get the readings meta-data.
        """
        return {
            'temperature': {
                'type': 'gauge',
                'unit': 'C',
                'precision': 1,
                'range_low': -40,
                'range_high': 85,
                'sensor': self.DEVICE_NAME
            },
            'pressureAbsolute': {
                'type': 'gauge',
                'unit': 'kPa',
                'precision': 0.4,
                'range_low': 50,
                'range_high': 110,
                'sensor': self.DEVICE_NAME
            },
            'pressureAltitude': {
                'type': 'gauge',
                'unit': 'm',
                'precision': 0.3,
                'range_low': -100,
                'range_high': 5000,
                'sensor': self.DEVICE_NAME
            },
        }
