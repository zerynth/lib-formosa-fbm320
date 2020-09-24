# Zerynth - libs - formosa-fbm320/fbm320.py
#
# Zerynth library for FBM320 digital barometer sensor
#
# @Author: Stefano Torneo
#
# @Date: 2020-08-26
# @Last Modified by: 
# @Last Modified time:

"""
.. module:: FBM320

**************
FBM320 Module
**************

.. _datasheet: http://www.fmti.com.tw/fmti689/program_download/good/201702101732113548.pdf

This module contains the Zerynth driver for FBM320 digital barometer. 
The FBM320 is a digital pressure sensor which consists of a MEMS piezoresistive pressure sensor and a signal
conditioning ASIC. The ASIC include a 24bits sigma-delta ADC, OTP memory for calibration data,
and serial interface circuits. The FBM320 features I2C and SPI digital interfaces, the present library enables I2C only.
"""

import i2c

# Define some constants from the datasheet

# Registers
CONFIG_REG = 0xF4
READ_MEAS_REG_U = 0xF6
READ_MEAS_REG_L = 0xF7
READ_MEAS_REG_XL = 0xF8
SOFTRESET_REG = 0xE0
CHIP_ID_REG = 0x6B
VERSION_REG = 0xA5

# CMD list
MEAS_TEMP = 0x2E # temperature measurement value
MEAS_PRESS_OVERSAMP_0 = 0x34  # OSR1024
MEAS_PRESS_OVERSAMP_1 = 0x74  # OSR2048
MEAS_PRESS_OVERSAMP_2 = 0xB4 # OSR4096
MEAS_PRESS_OVERSAMP_3 = 0xF4  # OSR8192
SOFTRESET_CMD = 0xB6

# HW Versions
HW_VER_B1 = 0x0
HW_VER_B2 = 0x1
HW_VER_B3 = 0x3
HW_VER_B4 = 0x5
HW_VER_unknown = 0xFF

# Conversion time in ms
CONV_TIME_TEMP = 3
CONV_TIME_MEAS_0 = 3
CONV_TIME_MEAS_1 = 4
CONV_TIME_MEAS_2 = 6
CONV_TIME_MEAS_3 = 11
TEMP_RESOLUTION = 0.01
    
# Calibration registers
CALIBRATION_DATA_START0 = 0xAA # Calibraton data address {0xf1, 0xd0, 0xbb:0xaa}
CALIBRATION_DATA_START1 = 0xD0
CALIBRATION_DATA_START2 = 0xF1
CALIBRATION_DATA_LENGTH = 20 # bytes

class FBM320(i2c.I2C):
    """
    
===============
 FBM320 class
===============

.. class:: FBM320(drvname, addr=0x6D, clk=400000)

    Creates an intance of the FBM320 class.

    :param drvname: I2C Bus used '( I2C0, ... )'
    :param addr: Slave address, default 0x6D.
                 If SDO pin is pulled low, I2C address is 6C.
                 If SDO pin is pulled high, I2C address is 6D.
    :param clk: Clock speed, default 400kHz
    
    Barometer values can be easily obtained from the sensor: ::

        from formosa.fbm320 import fbm320

        ...

        fbm = fbm320.FBM320(I2C0)

        temp, press, altitude = fbm.get_values()

    """

    # list of pressures and corrisponding settings [press, P0, h0, hs0, hs1] 
    pressure_setting = [
        [824000, 103, -138507, -5252, 311],
        [784000, 98, 280531, -5468, 338],
        [744000, 93, 717253, -5704, 370],
        [704000, 88, 1173421, -5964, 407],
        [664000, 83, 1651084, -6252, 450],
        [624000, 78, 2152645, -6573, 501],
        [584000, 73, 2680954, -6934, 560],
        [544000, 68, 3239426, -7342, 632],
        [504000, 63, 3832204, -7808, 719],
        [464000, 58, 4464387, -8345, 826],
        [424000, 53, 5142359, -8972, 960],
        [384000, 48, 5874268, -9714, 1131],
        [344000, 43, 6670762, -10609, 1354],
        [304000, 38, 7546157, -11711, 1654],
        [264000, 33, 8520395, -13103, 2072]
    ]

    # list of versions 
    version = [HW_VER_B1, HW_VER_B2, HW_VER_B3, HW_VER_B4]

    # dictionary of osr settings
    osr_setting = {
        '1024': [MEAS_PRESS_OVERSAMP_0, CONV_TIME_MEAS_0],
        '2048': [MEAS_PRESS_OVERSAMP_1, CONV_TIME_MEAS_1],
        '4096': [MEAS_PRESS_OVERSAMP_2, CONV_TIME_MEAS_2],
        '8192': [MEAS_PRESS_OVERSAMP_3, CONV_TIME_MEAS_3]
    }

    def __init__(self, drvname, addr=0x6D, clk=400000):
        
        if (addr != 0x6D and addr != 0x6C):
            raise ValueError

        i2c.I2C.__init__(self,drvname,addr,clk)
        try:
            self.start()
        except PeripheralError as e:
            print(e)

        if (self.write_read(CHIP_ID_REG, n=1)[0] != 0x42):
            raise ValueError
        
        self.get_version_id()
        self.get_otp_data()
        self.set_osr(8192)
    
    ##
    ## @brief      Get the hardware version.
    ##
    ## @param      self
    ## @return     nothing
    ##
    def get_version_id(self):
        
        # Reset the sensor
        self.write_bytes(SOFTRESET_REG, SOFTRESET_CMD)
        sleep(15) # The minimum start up time of fbm320 is 15ms

        buf = [0] * 2
        buf[0] = self.write_read(CONFIG_REG, n=1)[0]
        buf[1] = self.write_read(VERSION_REG, n=1)[0]
        
        version = ((buf[0] & 0xC0) >> 6) | ((buf[1] & 0x70) >> 2)

        flag = 0
        for i in range(4):
            if (version == self.version[i]):
                self.hw_ver = self.version[i]
                flag = 1

        if (flag == 0):
            self.hw_ver = HW_VER_unknown
    
    ##
    ## @brief      Get calibration data saved in OTP memory.
    ##
    ## @param      self
    ## @return     nothing
    ##
    def get_otp_data(self):

        R = [0] * 10

        tmp = self.write_read(CALIBRATION_DATA_START0, n=CALIBRATION_DATA_LENGTH)
        
	    # Read OTP data
        R[0] = (tmp[0] << 8 | tmp[1])
        R[1] = (tmp[2] << 8 | tmp[3])
        R[2] = (tmp[4] << 8 | tmp[5])
        R[3] = (tmp[6] << 8 | tmp[7])
        R[4] = (tmp[8] << 8 | tmp[9])
        R[5] = (tmp[10] << 8 | tmp[11])
        R[6] = (tmp[12] << 8 | tmp[13])
        R[7] = (tmp[14] << 8 | tmp[15])
        R[8] = (tmp[16] << 8 | tmp[17])
        R[9] = (tmp[18] << 8 | tmp[19])

        # Coefficient reconstruction
        self.C0 = R[0] >> 4
        self.C1 = ((R[1] & 0xFF00) >> 5) | (R[2] & 7)
        self.C2 = ((R[1] & 0xFF) << 1) | (R[4] & 1)
        self.C3 = R[2] >> 3
        self.C5 = R[4] >> 1
        self.C6 = R[5] >> 3
        self.C8 = R[7] >> 3
        self.C9 = R[8] >> 2
        self.C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 3)
        self.C11 = R[9] & 0xFF
        
        if (self.hw_ver == HW_VER_B1):
            self.C4 = (R[3] << 1) | (R[5] & 1)
            self.C7 = (R[6] << 2) | ((R[0] >> 2) & 3)
            self.C12 = ((R[5] & 6) << 2) | (R[7] & 7)
        else:
            self.C4 = (R[3] << 2) | (R[0] & 3)
            self.C7 = (R[6] << 3) | (R[5] & 7)
            self.C12 = ((R[0] & 0x0C) << 1) | (R[7] & 7)
    
    def set_osr(self, osr):
        """
    .. method:: set_osr(osr)

        :param osr: is the oversampling rate to set. Values accepted: 1024, 2048, 4096 or 8192.
        
        Set oversampling rate.
        
        """   

        if (osr not in [1024, 2048, 4096, 8192]):
            raise ValueError
        
        # get osr setting from dictionary
        osr_setting = self.osr_setting[str(osr)]
        # set measurement value
        self.meas_value = osr_setting[0]
        # set conversion time
        self.conv_time = osr_setting[1]

    def get_temp(self):
        """
    .. method:: get_temp()
        
        Return the temperature in degrees Celsius.
        
        """    
        return self.get_values()[0]
    
    def get_press(self):
        """
    .. method:: get_press()
        
        Return the pressure in hPa.
        
        """    
        return self.get_values()[1]

    ##
    ## @brief      Calculate the real value of temperature and pressure.
    ##             The temperature returned is in unit of Celsius.
    ##             The pressure returned is in unit of Pa.
    ##
    ## @param      self
    ## @param      raw_temp     the raw value of temperature.
    ## @param      raw_press    the raw value of pressure.
    ## @return     real values of temperature and pressure
    ##
    def calculation_real_value(self, raw_temp, raw_press):
        # calculation for real temperature value
        UT = raw_temp
        DT = ((UT - 8388608) >> 4) + (self.C0 << 4)
        X01 = 1
        RT = 0
        if (self.hw_ver == HW_VER_B1):
            X01 = (self.C1 + 4418) * DT >> 1
        else:
            X01 = (self.C1 + 4459) * DT >> 1

        X02 = ((((self.C2 - 256) * DT) >> 14) * DT) >> 4
        X03 = (((((self.C3 * DT) >> 18) * DT) >> 18) * DT)
        RT =  ((2500 << 15) - X01 - X02 - X03) >> 15

        #calculation for real pressure value
        DT2 = (X01 + X02 + X03) >> 12
        UP = raw_press
        if (self.hw_ver == HW_VER_B1):
            X11 = (self.C5 * DT2)
        else:
            X11 = ((self.C5 - 4443) * DT2)
        
        X12 = (((self.C6 * DT2) >> 16) * DT2) >> 2
        if (self.hw_ver == HW_VER_B1):
            X13 = ((X11 + X12) >> 10) + ((self.C4 + 211288) << 4)
            X21 = ((self.C8 + 7209) * DT2) >> 10
        else:
            X13 = ((X11 + X12) >> 10) + ((self.C4 + 120586) << 4)
            X21 = ((self.C8 + 7180) * DT2) >> 10

        X22 = (((self.C9 * DT2) >> 17) * DT2) >> 12
        if (X22 >= X21):
            X23 = (X22 - X21)
        else:
            X23 = (X21 - X22)
        
        if (self.hw_ver == HW_VER_B1):
            X24 = (X23 >> 11) * (self.C7 + 285594)
            X25 = ((X23 & 0x7FF) * (self.C7 + 285594)) >> 11
            if ((X22 - X21) < 0):
                X26 = ((0 - X24 - X25) >> 11) + self.C7 + 285594
            else:
                X26 = ((X24 + X25) >> 11) + self.C7 + 285594
        else:
            X24 = (X23 >> 11) * (self.C7 + 166426)
            X25 = ((X23 & 0x7FF) * (self.C7 + 166426)) >> 11
            if ((X22 - X21) < 0):
                X26 = ((0 - X24 - X25) >> 11) + self.C7 + 166426
            else:
                X26 = ((X24 + X25) >> 11) + self.C7 + 166426

        PP1 = ((UP - 8388608) - X13) >> 3
        PP2 = (X26 >> 11) * PP1
        PP3 = ((X26 & 0x7FF) * PP1) >> 11
        PP4 = (PP2 + PP3) >> 10
        CF = (2097152 + self.C12 * DT2) >> 3
        X31 = (((CF * self.C10) >> 17) * PP4) >> 2
        X32 = (((((CF * self.C11) >> 15) * PP4) >> 18) * PP4)
        RP = ((X31 + X32) >> 15) + PP4 + 100000

        return RT, RP
 
    def get_altitude(self, pressure):
        """
    .. method:: get_altitude(pressure)
        
        :param pressure: pressure value in hPa.

        Return the altitude in metres.
        
        """   
        # get altitude value from pressure (pressure is in 0.125 Pa)
        alt = self.pressure_to_altitude(pressure * 8)
        # convert altitude value from mm to m
        alt /= 1000
        return alt
    

    ##
    ## @brief      Convert pressure value to altitude.
    ##
    ## @param      self
    ## @param      real_pressure    real pressure in unit of 0.125 Pa
    ## @return     altitude
    ##
    def pressure_to_altitude(self, real_pressure):

        RP = real_pressure
        nrp = len(self.pressure_setting)
        flag = 0
        for i in range(nrp):
            value = self.pressure_setting[i]
            press_value = value[0]
            if (RP >= press_value):
                P0, h0, hs0, hs1 = value[1:5]
                flag = 1
                break

        if (flag == 0):
            P0 = 28
            h0 = 9622536
            hs0	= -14926
            hs1	= 2682

        dP0	= RP - P0 * 8000
        HP1	= (hs0 * dP0 ) >> 1
        HP2	= (((hs1 * dP0) >> 14) * dP0) >> 4
        RH = ((HP1 + HP2) >> 8) + h0

        return RH

    def get_values(self):
        """
    .. method:: get_values()
        
        Return the temperature (°C), pressure (hPa) and altitude (m) in a list [temperature, pressure, altitude].
        
        """    
        # get raw_press
        self.write_bytes(CONFIG_REG, self.meas_value) # set pressure conversion
        value = self.write_read(CONFIG_REG, n=1)[0]
        sleep(self.conv_time) # wait conversion time
        data = self.write_read(READ_MEAS_REG_U, n=3) # read raw data from registers
        raw_press = (data[0] << 16) + (data[1] << 8) + data[2] # compose raw data together

        # get raw_temp
        self.write_bytes(CONFIG_REG, MEAS_TEMP) # set temperature conversion
        sleep(CONV_TIME_TEMP) # wait conversion time
        data = self.write_read(READ_MEAS_REG_U, n=3) # read raw data from registers
        raw_temp = (data[0] << 16) + (data[1] << 8) + data[2] # compose raw data together

        # get real data
        # convert temperature in degrees Celsius and pressure in Pa
        real_temp, real_press = self.calculation_real_value(raw_temp, raw_press) 
        
        alt = self.get_altitude(real_press) # get altitude value from pressure
        press = real_press / 100 # convert pressure from Pa to hPa
        temp = real_temp * TEMP_RESOLUTION # convert temperature

        return temp, press, alt
