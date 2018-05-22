########################################################
########################################################
## DAQuiri Analog to Digital Converter Library        ##
##                                                    ##
## This library will handle interfacing with the ADC  ##
##   using the Raspberry Pi 3 through I2C protocol    ##
##                                                    ##
## ADC Chip: Texas Instruments ADS122C04              ##
##   http://www.ti.com/product/ADS122C04              ##
##                                                    ##
## Darren Tan, 2018                                   ##
########################################################
########################################################


import smbus2
import time
import os

# GLOBAL VARIABLES
RESET     = 0x06
START     = 0x08
POWERDOWN = 0x02
RDATA     = 0x10
RREG_0    = 0x20
RREG_1    = 0x24
RREG_2    = 0x28
RREG_3    = 0x2C
WREG_0    = 0x40
WREG_1    = 0x44
WREG_2    = 0x48
WREG_3    = 0x4C

WREG_ARR = [WREG_0, WREG_1, WREG_2, WREG_3]
RREG_ARR = [RREG_0, RREG_1, RREG_2, RREG_3]


class ADS122C04_i2c():
    """ Class to contain all parameters
        and configs for interacting with
        this chip 
    """

    def __init__(self, address, **kwargs):
        # init SMBus2 on the rasp-pi
        # use <sudo i2cdetect -y 1> to check device addresses
        self.bus = smbus2.SMBus(1)

        self.address = address
        self.config = []
        self.full_scale = 2.048         # Volts

        # initialize default configurations
        self.mux        = 0b0000        # AIN0/AIN1
        self.gain       = 0b000         # Gain = 1
        self.pga_bypass = 0b1           # Bypass PGA
        self.dr         = 0b000         # Datarate = 20hz
        self.mode       = 0b0           # Turbo Mode (better noise performance)
        self.cm         = 0b1           # Continuous Conversion Mode
        self.vref       = 0b00          # Use internal 2.048V ref
        self.ts         = 0b0           # Disable internal temp sensor
        self.drdy       = 0b1           # Enable conversion ready flag
        self.dcnt       = 0b0           # Disable conversion counter
        self.crc        = 0b00          # CRC 16 checksum disabled
        self.bcs        = 0b0           # Disable burnout current 
        self.idac       = 0b000         # Disable excitation current source
        self.i1mux      = 0b000         # Disable IDAC1
        self.i2mux      = 0b000         # Disable IDAC2


        # reset the device
        self.reset()

        # assemble configs into bytes to write
        self.assemble_config_bytes()
        
        # write config to adc
        self.write_full_config()

        # begin conversions
        self.start_conversion()



    def assemble_config_bytes(self, *args):
        """ assemble config bytes """
        if len(self.config) == 0 or len(args)==0:
            # write all config bytes
            self.config = []
            self.config.append(self.mux<<4   | self.gain<<1  | self.pga_bypass)
            self.config.append(self.dr<<5    | self.mode<<4  | self.cm<<3       | self.vref<<1 | self.ts)
            self.config.append(self.drdy<<7  | self.dcnt<<6  | self.crc<<4      | self.bcs<<3  | self.idac)
            self.config.append(self.i1mux<<5 | self.i2mux<<2 | 0b00)

        for i in args:
            # write specific bytes
            if i == 0:
                self.config[0] = (self.mux<<4   | self.gain<<1  | self.pga_bypass)
            elif i == 1:
                self.config[1] = (self.dr<<5    | self.mode<<4  | self.cm<<3       | self.vref<<1 | self.ts)
            elif i == 2:
                self.config[2] = (self.drdy<<7  | self.dcnt<<6  | self.crc<<4      | self.bcs<<3  | self.idac)
            elif i == 3:
                self.config[3] = (self.i1mux<<5 | self.i2mux<<2 | 0b00)




    def assign_setting(self, **kwargs):
        for key in kwargs:
            key = key.lower()
            if type(kwargs[key]) is int and kwargs[key] < 0b1111:
                if key in ['mux']:
                    self.mux = kwargs[key]
                elif key in ['gain']:
                    self.gain = kwargs[key]
                elif key in ['pga_bypass']:
                    self.pga_bypass = kwargs[key]
                elif key in ['dr']:
                    self.dr = kwargs[key]
                elif key in ['mode']:
                    self.mode = kwargs[key]
                elif key in ['cm']:
                    self.cm = kwargs[key]
                elif key in ['vref']:
                    self.vref = kwargs[key]
                elif key in ['ts']:
                    self.ts = kwargs[key]
                elif key in ['drdy']:
                    self.drdy = kwargs[key]
                elif key in ['dcnt']:
                    self.dcnt = kwargs[key]
                elif key in ['crc']:
                    self.crc = kwargs[key]
                elif key in ['bcs']:
                    self.bcs = kwargs[key]
                elif key in ['idac']:
                    self.idac = kwargs[key]
                elif key in ['i1mux']:
                    self.i1mux = kwargs[key]
                elif key in ['i2mux']:
                    self.i2mux = kwargs[key]
                else:
                    print("CONFIG ERROR: Invalid config keywords")
            else:
                print("CONFIG ERROR: Invalid config values")


    def write_full_config(self):
        """ write full config to ADC """
        for i, reg in enumerate(WREG_ARR):
            self.bus.write_byte_data(self.address, reg, self.config[i])

        self.verify_config()


    def verify_config(self):
        """ verify that configs were received correctly """
        read_config = self.read_full_config()

        for i, reg in enumerate(self.config):
            # DRDY (0x07) is a read only bit that changes when conversion is ready
            # ignore that bit in this check (force high)
            if i == 2:
                reg | 0x07 
                read_config[i] | 0x07
            
            if reg != read_config[i]:
                print "CONFIG ERROR: Read config does not match sent config"


    def read_full_config(self):
        """ read full configs """
        config_arr = []
        for reg in RREG_ARR:
            config_arr.append(self.bus.read_byte_data(self.address, reg))

        return config_arr


    def read_byte(self, reg):
        """ read single config from ADC """
        single_config = self.bus.read_byte_data(self.address, reg)


    def start_conversion(self):
        self.bus.write_byte(self.address, START)


    def reset(self):
        self.bus.write_byte(self.address, RESET)


    def update_config(self, **kwargs):
        """ assign and update to specific configs
            **kwargs : a setting keyword and binary value
            refer to datasheet for settings
            e.g. update_config(mux=0b01100, gain=0b100))
        """
        # assign specified settings to object properties
        self.assign_setting(**kwargs)

        if len(self.config) == 4:
            for key in kwargs:
                key = key.lower()

                if key in ['mux', 'gain', 'pga_bypass']:
                    self.assemble_config_bytes(0)
                    self.bus.write_byte_data(self.address, WREG_ARR[0], self.config[0])

                elif key in ['dr', 'mode', 'cm', 'vref', 'ts']:
                    self.assemble_config_bytes(1)
                    self.bus.write_byte_data(self.address, WREG_ARR[1], self.config[1])

                elif key in ['drdy', 'dcnt', 'crc', 'bcs', 'idac']:
                    self.assemble_config_bytes(2)
                    self.bus.write_byte_data(self.address, WREG_ARR[2], self.config[2])

                elif key in ['i1mux', 'i2mux']:
                    self.assemble_config_bytes(3)
                    self.bus.write_byte_data(self.address, WREG_ARR[3], self.config[3])

                else:
                    print("CONFIG ERROR: Invalid keyword")

        else:
            print("CONFIG ERROR: Config is not been initiated correctly")

        # verify that config was sent correctly
        self.verify_config()

    
    def read_conversion(self):
        raw_data_arr = self.bus.read_i2c_block_data(self.address, RDATA, 3)
        data = ((raw_data_arr[0]<< 16) | (raw_data_arr[1]<<8) | raw_data_arr[2])

        # account for negative scale (2's complement)
        if (data & 0x800000) == 0x800000:
            data = ~(data) - 1

        # convert to voltage
        raw_voltage = data * self.full_scale/2**23.0

        return raw_voltage

    def read_temp(self):
        # temporaily set ts bit to 1 to enable temp sensor
        self.update_config(ts=0b1)
        time.sleep(0.3)

        # read data
        raw_data_arr = self.bus.read_i2c_block_data(self.address, RDATA, 3)
        data = ((raw_data_arr[0]<< 16) | (raw_data_arr[1]<<8) | raw_data_arr[2])
        data = data >> 10


        # account for negative scale (2's complement)
        if (data & 0x800000) == 0x800000:
            data = ~(data) - 1

        # convert to C
        temp_celcius = data * 0.03125

        # return ts bit to 0
        time.sleep(0.3)
        self.update_config(ts=0b0)
        time.sleep(0.3)

        return temp_celcius


def main():
    # create object called adc with 0x40 address
    adc = ADS122C04_i2c(0x40)
    delay = 0.1
    while(1):
        voltage = adc.read_conversion()
        os.system('clear')
        print("DAQuiri is reading: " + "{0:.5f}".format(voltage) + " V")
        print('----------\n\n')
        time.sleep(delay)


        # voltage = adc.read_conversion()
        # # time.sleep(0.3)
        # temp = adc.read_temp()
        # os.system('clear')
        # print("DAQuiri is reading: " + "{0:.5f}".format(voltage) + " V")
        # print('----------\n\n')

        # # time.sleep(0.1)
        # print("DAQuiri temperature is: " + "{0:.5f}".format(temp) + " C")
        # print('----------\n\n')
        # # time.sleep(0.3)


if __name__ == '__main__':
    main()


