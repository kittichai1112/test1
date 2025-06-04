import struct
import time
import spidev
import RPi.GPIO as GPIO
from typing import Optional, Callable, Dict, Any

# You'll need to create these modules or import equivalents
import BGT60TRXX_define as BGT60TR_CONST
import dftclass as DFT

class BGT60TRxxModule:
    """Implementation of an BGT60-Radar sensor using one antenna"""

    def __init__(self, word_size: int, interrupt_handler: Optional[Callable] = None, 
                 spi_bus: int = 0, spi_device: int = 0):
        # Create SPI Interface with 50 MHz
        #=========================================
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = 50_000_000
        self.spi.mode = 0  # CPOL=0, CPHA=0
        self.spi.bits_per_word = 8
        
        # GPIO Pin setup (adjust pin numbers for your hardware)
        GPIO.setmode(GPIO.BCM)
        self.cs_pin = 18  # Chip Select pin
        self.reset_pin = 24  # Reset pin
        self.irq_pin = 25  # Interrupt pin
        
        GPIO.setup(self.cs_pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.reset_pin, GPIO.OUT, initial=GPIO.HIGH)
        
        # Transfer of SPI Interface
        #============================
        self.word_size = word_size
        self.frame_size = int(word_size * 1.5 + 0.5)

        self.skipFirstValues = 6  # Because of misaligned address.
                                  # Address of FIFO = 0x60.
                                  # But when reading, sensor returns
                                  # error!
                                  # 4 words (or 6 bytes) seems to
                                  # be the minimum value where
                                  # an SPI-Read works.
        
        # Check if fifo overflow would happen
        if (self.frame_size > (BGT60TR_CONST.FIFO_SIZE_BYTE - self.skipFirstValues)):
            raise Exception("Error! Max Word-Size allowed: {}".format(BGT60TR_CONST.FIFO_SIZE))
        
        self.frame_size += self.skipFirstValues

        # Init Byte Arrays to optimize read_fifo function
        self.headerGSR0 = bytearray(BGT60TR_CONST.BYTE_SIZE)
        self.data = bytearray(self.frame_size)

        self.reg_values = BGT60TR_CONST.init_register_list

        # Init FFT
        #============
        self.fft = DFT.DFT(self.word_size)

        # Enable Interrupt Request when a Function is given
        #===================================================
        if interrupt_handler is not None:
            GPIO.setup(self.irq_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(self.irq_pin, GPIO.RISING, callback=interrupt_handler)
            print("Register IRQ-Event on Function: {}".format(interrupt_handler))

        # Chirp Configuration
        #=======================
        self.start_freq = (self.reg_values[BGT60TR_CONST.PLL1_0_ADDR] 
                          & BGT60TR_CONST.PLL1_0_FSU_MASK)
        
        self.Clk_Per_Chirp = (self.reg_values[BGT60TR_CONST.PLL1_2_ADDR] 
                              & BGT60TR_CONST.PLL1_2_RTU_MASK)
        
        self.step_freq_chirp = (self.reg_values[BGT60TR_CONST.PLL1_1_ADDR] 
                                & BGT60TR_CONST.PLL1_1_RSU_MASK)

    def __del__(self):
        """Cleanup GPIO and SPI on destruction"""
        try:
            self.spi.close()
            GPIO.cleanup()
        except:
            pass

    def get_range_resolution(self) -> float:
        """Returns meter per index.
        Is based from parameters of chirp config
        """
        f_adc_clk = 80e6  # 80MHz
        step_chirp_divider = 8

        RSU = self.step_freq_chirp
        RTU = self.Clk_Per_Chirp

        delta_f_RF = step_chirp_divider * f_adc_clk * RSU / (2**20)

        c0 = 3e8  # speed of light in m/s
        bandwidth = (RTU * 8) * delta_f_RF  # Multiply with 8! -> See Datasheet BGT60TRXX P.56 RTU
        range_resolution = c0 / (2 * bandwidth)

        return range_resolution

    def read_reg(self, reg_addr: int) -> bytes:      
        """Read register of BGT60TR-Sensor"""
        # Only Address needs to be send
        # LSB = R/W = 0
        addr = (reg_addr << 1) & 0xFE

        # SPI Read
        GPIO.output(self.cs_pin, GPIO.LOW)
        data = self.spi.xfer2([addr] + [0] * (BGT60TR_CONST.BYTE_SIZE - 1))
        GPIO.output(self.cs_pin, GPIO.HIGH)

        # Check if Error Occurred
        if (data[0] & 0x0F) not in [0x0, 0x4]:
            raise Exception("Status Register Error! GSR0 = {}".format(hex(data[0])))
        
        # Only use the lower bytes (Bit 23:0) -> Data
        # Rest is Header-Data
        return bytes(data[1:])
        
    def write_reg(self, reg_addr: int, data: int):
        """Write register of BGT60TR-Sensor with given Data"""
        # SPI Transfer Format: Addr - R/W - Data 
        dataToSend = ((reg_addr << BGT60TR_CONST.ADDR_OFFSET) 
                      & BGT60TR_CONST.ADDR_MASK)
        
        dataToSend |= BGT60TR_CONST.WRITE_EN

        dataToSend |= ((data << BGT60TR_CONST.DATA_OFFSET) 
                       & BGT60TR_CONST.DATA_MASK)
        
        # Convert addr to bytes
        data_bytes = dataToSend.to_bytes(BGT60TR_CONST.BYTE_SIZE, "big")

        # SPI Write
        GPIO.output(self.cs_pin, GPIO.LOW)
        self.spi.writebytes(list(data_bytes))
        GPIO.output(self.cs_pin, GPIO.HIGH)

    def __set_bits(self, reg_addr: int, bits: int):
        """sets data bits inside register to '1' using a read-modify-write"""
        data = self.read_reg(reg_addr)
        data = int.from_bytes(data, "big") | bits
        self.write_reg(reg_addr, data)

    def set_adc_div(self, data: int):
        """sets frequency divider of adc. Only enabled with InitSensor-Method"""
        self.set_init_value(data, 
                            BGT60TR_CONST.ADC0_ADDR, 
                            BGT60TR_CONST.ADC0_DIV_MASK, 
                            BGT60TR_CONST.ADC0_DIV_OFFSET)
        
    def set_chirp_len(self, chirp_len: int):
        """sets chirp length of sensor. Only enabled with InitSensor-Method"""
        self.set_init_value(chirp_len, 
                            BGT60TR_CONST.PLL1_3_ADDR, 
                            BGT60TR_CONST.APU0_MASK, 
                            BGT60TR_CONST.APU0_OFFSET)
        
    def configure_chirp(self, N_FSU: int, N_RTU: int, N_RSU: int):
        """Configures chirp parameters. 
        Only enabled with InitSensor-Method.

        N_FSU = Starting Frequency
        N_RTU = Clock cycles per chirp
        N_RSU = Frequency step per clock cycle

        for calculation of those values see Datasheet
        """
        self.set_init_value(N_FSU, 
                            BGT60TR_CONST.PLL1_0_ADDR, 
                            BGT60TR_CONST.PLL1_0_FSU_MASK, 
                            BGT60TR_CONST.PLL1_0_FSU_OFFSET)
        self.set_init_value(N_RSU, 
                            BGT60TR_CONST.PLL1_1_ADDR, 
                            BGT60TR_CONST.PLL1_1_RSU_MASK, 
                            BGT60TR_CONST.PLL1_1_RSU_OFFSET)
        self.set_init_value(N_RTU, 
                            BGT60TR_CONST.PLL1_2_ADDR, 
                            BGT60TR_CONST.PLL1_2_RTU_MASK, 
                            BGT60TR_CONST.PLL1_2_RTU_OFFSET)

        # set values for range resolution
        self.start_freq = N_FSU
        self.step_freq_chirp = N_RSU
        self.Clk_Per_Chirp = N_RTU

    def set_vga_gain_ch1(self, gain: int):
        """Amplifier for received signal (Channel 1) from sensor. 
        Values from 0 to 5 are allowed (see Datasheet).
        Only enabled with InitSensor-Method.
        """
        self.set_init_value(gain, 
                            BGT60TR_CONST.CSU1_2_ADDR, 
                            BGT60TR_CONST.CSU1_2_VGA_GAIN1_MASK, 
                            BGT60TR_CONST.CSU1_2_VGA_GAIN1_OFFSET)
        
    def set_vga_gain_ch2(self, gain: int):
        """Amplifier for received signal (Channel 2) from sensor. 
        Values from 0 to 5 are allowed (see Datasheet).
        Only enabled with InitSensor-Method.
        """
        self.set_init_value(gain, 
                            BGT60TR_CONST.CSU1_2_ADDR, 
                            BGT60TR_CONST.CSU1_2_VGA_GAIN2_MASK, 
                            BGT60TR_CONST.CSU1_2_VGA_GAIN2_OFFSET)
        
    def set_vga_gain_ch3(self, gain: int):
        """Amplifier for received signal (Channel 3) from sensor. 
        Values from 0 to 5 are allowed (see Datasheet).
        Only enabled with InitSensor-Method.
        """
        self.set_init_value(gain, 
                            BGT60TR_CONST.CSU1_2_ADDR, 
                            BGT60TR_CONST.CSU1_2_VGA_GAIN3_MASK, 
                            BGT60TR_CONST.CSU1_2_VGA_GAIN3_OFFSET)
        
    def set_init_value(self, data: int, address: int, 
                       reset_mask: int, offset: int):
        """Sets register values for sensor
        Only enabled with InitSensor-Method.
        """
        oldValue = self.reg_values[address]    # read
        newValue = (oldValue & ~reset_mask) | (data << offset)    # modify
        self.reg_values[address] = newValue    # write

        
    def setCompareValue(self, compare_value: int):
        """sets compare value using a read-modify-write
        Only enabled with a InitSensor call
        """
        if compare_value >= 100:
            self.set_init_value(BGT60TR_CONST.FIFO_SIZE - 1, 
                              BGT60TR_CONST.SFCTL_ADDR, 
                              BGT60TR_CONST.SFCTL_FIFO_CREF_MASK, 
                              BGT60TR_CONST.SFCTL_FIFO_CREF_OFFSET)
        else:
            self.set_init_value(compare_value, 
                              BGT60TR_CONST.SFCTL_ADDR, 
                              BGT60TR_CONST.SFCTL_FIFO_CREF_MASK, 
                              BGT60TR_CONST.SFCTL_FIFO_CREF_OFFSET)

    def enableTestMode(self):
        """Enables Test-Mode (LFSR Enable) for Sensor"""
        # Enables TestMode
        self.__set_bits(BGT60TR_CONST.SFCTL_ADDR, BGT60TR_CONST.TEST_MODE_EN)

        # Init RFT0 Register
        self.__set_bits(BGT60TR_CONST.RFT0_ADDR, BGT60TR_CONST.TEST_IF_ENABLE)

        self.resetFSM()

    def startFrame(self):
        """Start frame generation and leave Deep-Sleep-Mode"""
        self.__set_bits(BGT60TR_CONST.MAIN_ADDR, BGT60TR_CONST.START_FRAME)
        
    def readFifo(self):    
        """Read n-words from FIFO-Stack.
        Checks Header-Information for Error (like Overflow/Underflow).
        Data is stored inside self.data object
        """
        # Enable Burst Mode and read GSR0 Register
        GPIO.output(self.cs_pin, GPIO.LOW)
        header_response = self.spi.xfer2(list(BGT60TR_CONST.ENABLE_BURST_MODE))
        self.headerGSR0[:len(header_response)] = header_response

        if self.headerGSR0[3] not in [0x04, 0x00]:
            raise Exception(
                "Error detected in FIFO-Read. GSR0 = {}".format(hex(self.headerGSR0[3])))

        # Read from SPI
        data_response = self.spi.readbytes(len(self.data))
        self.data[:] = data_response
        GPIO.output(self.cs_pin, GPIO.HIGH)

    def unpackRecData(self):
        """Unpacks recorded adc data
        into real-part of fft.    
        
        words (adc) are represented in a byte array:
        a2 a1;  a0 b2;  b1 b0
        """
        byte_index = self.skipFirstValues
        fft_index = 0
        
        while byte_index < self.frame_size - 2:
            # Extract 3 bytes from the byte array
            byte1 = self.data[byte_index]
            byte2 = self.data[byte_index + 1]
            byte3 = self.data[byte_index + 2]
            
            a = (((byte1 & 0xFF) << 4) 
                 | ((byte2 & 0xF0) >> 4))
            
            b = (((byte2 & 0x0F) << 8) 
                 | ((byte3 & 0xFF) >> 0))

            # Assign the 12-bit values to the self.fft.re register
            self.fft.re[fft_index] = a
            self.fft.re[fft_index + 1] = b

            # self.fft.im is reset by running a FORWARD-FFT

            fft_index += 2
            byte_index += 3

    def runHighPassFilter(self):
        """High-Pass Filter Chebysheff 2nd Order
        over real-part data before fft.
        """
        # High-Pass Filter
        x0, x1, x2 = 0.0, 0.0, 0.0
        y0, y1, y2 = 0.0, 0.0, 0.0
        
        for i in range(self.word_size):
            x2 = x1
            x1 = x0
            x0 = float(self.fft.re[i])
            y2 = y1
            y1 = y0
            y0 = x0*0.943 - x1*1.885 + x2*0.943 + y1*1.881 - y2*0.890  # Chebyshev 2nd Order
            self.fft.re[i] = y0
        
    def readDistance(self):
        """Reads FIFO and
        save word-wise inside self.fft.re.
        Then runs High-Pass-Filter.
        Using the FFT, the range-profile is calculated and
        stored inside self.fft.re and self.fft.im
        """
        self.readFifo()
        self.unpackRecData()
        self.runHighPassFilter()
        self.fft.run(DFT.FORWARD)    

    def initSensor(self):
        """inits sensor
        and writes all registers anew.
        """
        for reg_addr, reg_data in self.reg_values.items():
            reg_data = (reg_data & BGT60TR_CONST.DATA_MASK)  # filter out address
            self.write_reg(reg_addr, reg_data)

    def resetFIFO(self):
        """Resets fifo and fsm"""
        self.__set_bits(BGT60TR_CONST.MAIN_ADDR, BGT60TR_CONST.FIFO_RESET)

    def resetFSM(self):
        """Resets fsm"""
        self.__set_bits(BGT60TR_CONST.MAIN_ADDR, BGT60TR_CONST.FSM_RESET)

    def reset(self):
        """Resets software, fifo and fsm"""
        self.write_reg(BGT60TR_CONST.MAIN_ADDR, BGT60TR_CONST.SOFT_RESET)


def calculateRTU(adc_div: int, samples_per_chirp: int) -> int:
    """Calculates Clock cycles per chirp.
    Needs division factor for sample rate (adc) and
    how many samples per chirp are needed.
    """
    return (adc_div * samples_per_chirp) // 8 + BGT60TR_CONST.T_SETUP
  

def calculateFSU(start_freq: int) -> int:
    """Calculates value for Starting Frequency. 
    start_freq needs to be in kHz.
    """
    return int(2**20 * ((start_freq/640_000) - 96)) & 0xFFFFFF  # 24 bit two complement needed for sensor


def calculateRSU(bandwidth: int, RTU: int) -> int:
    """Calculates Frequency step per clock cycle.
    Needs Starting Frequency as well as wanted bandwidth
    """
    dRF = bandwidth / (8 * RTU)
    return int(2**20 * dRF / 640_000)


def checkData(data: list, length: int):
    """Check Data for overflow and underflow"""
    for i in range(length):
        if data[i] == 0x00:
            print("Underflow detected! At index {}".format(i))
            return
        elif data[i] == ~0x00:
            print("Overflow detected! At index {}".format(i))
            return