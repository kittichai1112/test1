import spidev
import RPi.GPIO as GPIO
import time
import numpy as np
import logging

class BGT60TR13C:
    def __init__(self, spi_bus=0, spi_device=0, cs_pin=8, rst_pin=7, irq_pin=25):
        self.spi = spidev.SpiDev()
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.cs_pin = cs_pin
        self.rst_pin = rst_pin
        self.irq_pin = irq_pin

        # REGISTER MAP WITH DESCRIPTIONS
        self.registers = {
            'MAIN': {
                'address': 0x00,
                'description': 'Main control register - Start/stop operations',
                'bits': {
                    0: 'START_CHIRP - Start chirp sequence'
                }
            },
            'CHIP_ID': {
                'address': 0x02,
                'description': 'Chip identification register',
                'readonly': True
            },
            'PACR1': {
                'address': 0x04,
                'description': 'Power amplifier control register 1',
                'bits': {
                    0: 'PA_EN - Power amplifier enable'
                }
            },
            'PACR2': {
                'address': 0x05,
                'description': 'Power amplifier control register 2',
                'bits': {
                    '2:0': 'PA_GAIN - Power amplifier gain control'
                }
            },
            'SFCTL': {
                'address': 0x06,
                'description': 'Sampling frequency control',
                'bits': {
                    0: 'SF_EN - Sampling frequency enable'
                }
            },
            'PLL1_0': {
                'address': 0x30,
                'description': 'PLL1 configuration register 0 - Base frequency control',
                'bits': {
                    '7:0': 'PLL frequency control bits [7:0]'
                }
            },
            'PLL1_1': {
                'address': 0x31,
                'description': 'PLL1 configuration register 1 - Frequency control',
                'bits': {
                    '7:0': 'PLL frequency control bits [15:8]'
                }
            },
            'PLL1_2': {
                'address': 0x32,
                'description': 'PLL1 configuration register 2 - Bandwidth control',
                'bits': {
                    '7:0': 'PLL bandwidth and modulation control'
                }
            },
            'PLL1_3': {
                'address': 0x33,
                'description': 'PLL1 configuration register 3 - Enable and fine tuning',
                'bits': {
                    7: 'PLL_EN - PLL enable',
                    '6:0': 'Fine frequency adjustment'
                }
            },
            'STAT0': {
                'address': 0x5D,
                'description': 'Status register 0 - FIFO and system status',
                'readonly': True,
                'bits': {
                    0: 'FIFO_RDY - FIFO ready for read',
                    1: 'FIFO_FULL - FIFO full flag',
                    2: 'FIFO_EMPTY - FIFO empty flag',
                    3: 'PLL_LOCK - PLL lock status'
                }
            },
            'FIFO_ACCESS': {
                'address': 0x60,
                'description': 'FIFO data access register - Read ADC samples',
                'readonly': True
            }
        }

        # Quick access to register addresses (for backward compatibility)
        self.REG_MAIN        = self.registers['MAIN']['address']
        self.REG_CHIP_ID     = self.registers['CHIP_ID']['address']
        self.REG_PACR1       = self.registers['PACR1']['address']
        self.REG_PACR2       = self.registers['PACR2']['address']
        self.REG_SFCTL       = self.registers['SFCTL']['address']
        self.REG_PLL1_0      = self.registers['PLL1_0']['address']
        self.REG_PLL1_1      = self.registers['PLL1_1']['address']
        self.REG_PLL1_2      = self.registers['PLL1_2']['address']
        self.REG_PLL1_3      = self.registers['PLL1_3']['address']
        self.REG_STAT0       = self.registers['STAT0']['address']
        self.REG_FIFO_ACCESS = self.registers['FIFO_ACCESS']['address']

        # Configuration values with descriptions
        self.config_values = {
            'PLL1_0': {
                'value': 0xC4,
                'description': 'Base frequency: 60GHz + offset'
            },
            'PLL1_1': {
                'value': 0x09,
                'description': 'Frequency fine control'
            },
            'PLL1_2': {
                'value': 0x00,
                'description': 'Bandwidth: 1GHz typical'
            },
            'PLL1_3': {
                'value': 0x80,
                'description': 'PLL enable + frequency adjustment'
            },
            'PACR1': {
                'value': 0x01,
                'description': 'PA enable'
            },
            'PACR2': {
                'value': 0x07,
                'description': 'PA gain: maximum'
            },
            'SFCTL': {
                'value': 0x01,
                'description': 'Sampling frequency enable'
            }
        }

        # Legacy config values for backward compatibility
        self.PLL1_0_CONFIG = self.config_values['PLL1_0']['value']
        self.PLL1_1_CONFIG = self.config_values['PLL1_1']['value']
        self.PLL1_2_CONFIG = self.config_values['PLL1_2']['value']
        self.PLL1_3_CONFIG = self.config_values['PLL1_3']['value']
        self.PACR1_CONFIG  = self.config_values['PACR1']['value']
        self.PACR2_CONFIG  = self.config_values['PACR2']['value']
        self.SFCTL_CONFIG  = self.config_values['SFCTL']['value']

        # Processing parameters
        self.samples_per_chirp = 64
        self.motion_threshold = 100
        self.breathing_filter_alpha = 0.1
        self.fft_threshold_db = 20

        # State variables
        self.last_samples = []
        self.motion_detected = False
        self.breathing_rate = 0
        self.distance = 0
        self.motion_history = []
        self.spectrum_data = []

        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

    def print_register_map(self):
        """Print complete register map with descriptions"""
        print("\n=== BGT60TR13C Register Map ===")
        for name, reg_info in self.registers.items():
            print(f"\n{name} (0x{reg_info['address']:02X}):")
            print(f"  Description: {reg_info['description']}")
            if reg_info.get('readonly'):
                print("  Access: Read-only")
            if 'bits' in reg_info:
                print("  Bit definitions:")
                for bit, desc in reg_info['bits'].items():
                    print(f"    Bit {bit}: {desc}")

    def print_configuration(self):
        """Print current configuration with descriptions"""
        print("\n=== BGT60TR13C Configuration ===")
        for name, config in self.config_values.items():
            print(f"{name}: 0x{config['value']:02X} - {config['description']}")

    def get_register_info(self, register_name):
        """Get detailed information about a specific register"""
        if register_name in self.registers:
            return self.registers[register_name]
        else:
            # Try to find by address
            for name, info in self.registers.items():
                if info['address'] == register_name:
                    return {name: info}
            return None

    def begin(self):
        """Initialize the BGT60TR13C radar sensor"""
        try:
            # Setup GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.cs_pin, GPIO.OUT, initial=GPIO.HIGH)
            
            if self.rst_pin is not None:
                GPIO.setup(self.rst_pin, GPIO.OUT, initial=GPIO.HIGH)
                self.logger.info("Resetting BGT60TR13C...")
                GPIO.output(self.rst_pin, GPIO.LOW)
                time.sleep(0.01)
                GPIO.output(self.rst_pin, GPIO.HIGH)
                time.sleep(0.1)
            
            if self.irq_pin is not None:
                GPIO.setup(self.irq_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            # Setup SPI
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = 1000000
            self.spi.mode = 0
            self.spi.bits_per_word = 8

            # Configure registers with descriptions
            self.logger.info("Configuring BGT60TR13C registers...")
            
            # PLL Configuration
            self.logger.info(f"Setting PLL1_0: {self.config_values['PLL1_0']['description']}")
            self.write_register(self.REG_PLL1_0, self.PLL1_0_CONFIG)
            time.sleep(0.01)
            
            self.logger.info(f"Setting PLL1_1: {self.config_values['PLL1_1']['description']}")
            self.write_register(self.REG_PLL1_1, self.PLL1_1_CONFIG)
            time.sleep(0.01)
            
            self.logger.info(f"Setting PLL1_2: {self.config_values['PLL1_2']['description']}")
            self.write_register(self.REG_PLL1_2, self.PLL1_2_CONFIG)
            time.sleep(0.01)
            
            self.logger.info(f"Setting PLL1_3: {self.config_values['PLL1_3']['description']}")
            self.write_register(self.REG_PLL1_3, self.PLL1_3_CONFIG)
            time.sleep(0.01)
            
            # Power Amplifier Configuration
            self.logger.info(f"Setting PACR1: {self.config_values['PACR1']['description']}")
            self.write_register(self.REG_PACR1, self.PACR1_CONFIG)
            time.sleep(0.01)
            
            self.logger.info(f"Setting PACR2: {self.config_values['PACR2']['description']}")
            self.write_register(self.REG_PACR2, self.PACR2_CONFIG)
            time.sleep(0.01)
            
            # Sampling Frequency Control
            self.logger.info(f"Setting SFCTL: {self.config_values['SFCTL']['description']}")
            self.write_register(self.REG_SFCTL, self.SFCTL_CONFIG)
            time.sleep(0.01)

            # Verify configuration
            chip_id = self.read_register(self.REG_CHIP_ID)
            pll1_0_readback = self.read_register(self.REG_PLL1_0)
            
            self.logger.info(f"Chip ID: 0x{chip_id:02X}")
            self.logger.info(f"PLL1_0 readback: 0x{pll1_0_readback:02X}")

            if pll1_0_readback == self.PLL1_0_CONFIG:
                self.logger.info("BGT60TR13C initialized successfully")
                return True
            else:
                self.logger.error("Failed to verify configuration")
                return False

        except Exception as e:
            self.logger.error(f"Error initializing BGT60TR13C: {e}")
            return False

    def write_register(self, address, data):
        """Write to 8-bit register"""
        try:
            GPIO.output(self.cs_pin, GPIO.LOW)
            time.sleep(0.001)
            
            # 8-bit register write
            cmd = [address & 0x7F, data & 0xFF]
            self.spi.xfer2(cmd)
            
            time.sleep(0.001)
            GPIO.output(self.cs_pin, GPIO.HIGH)
            
        except Exception as e:
            GPIO.output(self.cs_pin, GPIO.HIGH)
            self.logger.error(f"Error writing register 0x{address:02X}: {e}")

    def read_register(self, address):
        """Read from 8-bit register"""
        try:
            GPIO.output(self.cs_pin, GPIO.LOW)
            time.sleep(0.001)
            
            # 8-bit register read
            cmd = [address | 0x80, 0x00]
            response = self.spi.xfer2(cmd)
            
            time.sleep(0.001)
            GPIO.output(self.cs_pin, GPIO.HIGH)
            
            return response[1]
            
        except Exception as e:
            GPIO.output(self.cs_pin, GPIO.HIGH)
            self.logger.error(f"Error reading register 0x{address:02X}: {e}")
            return 0

    def decode_status_register(self, stat0_value):
        """Decode STAT0 register bits"""
        status = {}
        status['fifo_ready'] = bool(stat0_value & 0x01)
        status['fifo_full'] = bool(stat0_value & 0x02)
        status['fifo_empty'] = bool(stat0_value & 0x04)
        status['pll_lock'] = bool(stat0_value & 0x08)
        return status

    def start_chirp(self):
        """Start a chirp sequence"""
        self.write_register(self.REG_MAIN, 0x01)

    def wait_for_fifo_ready(self, timeout=1.0):
        """Wait for FIFO to be ready"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            stat0 = self.read_register(self.REG_STAT0)
            status = self.decode_status_register(stat0)
            if status['fifo_ready']:
                return True
            time.sleep(0.001)
        return False

    def read_fifo_data(self, num_samples=None):
        """Read samples from FIFO"""
        if num_samples is None:
            num_samples = self.samples_per_chirp
            
        try:
            samples = []
            
            # Read samples one by one
            for _ in range(num_samples):
                sample = self.read_register(self.REG_FIFO_ACCESS)
                samples.append(sample)
            
            return samples
            
        except Exception as e:
            self.logger.error(f"Error reading FIFO data: {e}")
            return []

    def process_fft_analysis(self, samples):
        """Process samples using FFT analysis"""
        if len(samples) < self.samples_per_chirp:
            return None
            
        try:
            # Convert to numpy array and remove DC component
            arr = np.array(samples, dtype=np.float32)
            arr = arr - np.mean(arr)
            
            # Apply window function to reduce spectral leakage
            window = np.hanning(len(arr))
            arr = arr * window
            
            # Perform FFT
            fft = np.fft.fft(arr)
            mag = np.abs(fft)[:len(arr)//2]
            
            # Convert to dB
            mag_db = 20 * np.log10(np.maximum(mag, 1e-6))
            
            # Store spectrum data
            self.spectrum_data = mag_db
            
            # Find peaks
            peaks = self.find_peaks(mag_db, self.fft_threshold_db)
            
            return {
                'spectrum': mag_db,
                'peaks': peaks,
                'frequencies': np.fft.fftfreq(len(arr), 1.0)[:len(arr)//2]
            }
            
        except Exception as e:
            self.logger.error(f"Error in FFT analysis: {e}")
            return None

    def find_peaks(self, signal, threshold):
        """Find peaks in signal above threshold"""
        peaks = []
        for i in range(1, len(signal) - 1):
            if (signal[i] > threshold and 
                signal[i] > signal[i-1] and 
                signal[i] > signal[i+1]):
                peaks.append(i)
        return peaks

    def process_motion_detection(self, samples, fft_data=None):
        """Detect motion using both time domain and frequency domain"""
        if len(samples) < 32:
            return self.motion_detected
            
        # Time domain detection
        power = sum(abs(sample) for sample in samples) / len(samples)
        time_domain_motion = power > self.motion_threshold
        
        # Frequency domain detection
        freq_domain_motion = False
        if fft_data and fft_data['peaks']:
            # Motion typically shows up as peaks in low frequency bins
            low_freq_peaks = [p for p in fft_data['peaks'] if p < len(fft_data['spectrum']) // 4]
            freq_domain_motion = len(low_freq_peaks) > 0
        
        # Combine detections
        motion = time_domain_motion or freq_domain_motion
        
        # Update motion history
        self.motion_history.append(motion)
        if len(self.motion_history) > 10:
            self.motion_history.pop(0)
            
        # Require multiple detections to confirm motion
        self.motion_detected = sum(self.motion_history) > 5
        return self.motion_detected

    def process_breathing_rate(self, samples):
        """Estimate breathing rate from samples"""
        if len(samples) < 128:
            return self.breathing_rate
            
        try:
            # High-pass filter to remove DC and low frequency noise
            filtered = []
            window_size = 20
            for i in range(window_size, len(samples)):
                avg = sum(samples[i-window_size:i]) / window_size
                filtered.append(samples[i] - avg)
            
            if len(filtered) < 64:
                return self.breathing_rate
            
            # Find peaks in filtered signal
            peaks = []
            for i in range(1, len(filtered) - 1):
                if (filtered[i] > filtered[i-1] and 
                    filtered[i] > filtered[i+1] and 
                    abs(filtered[i]) > 10):
                    peaks.append(i)
            
            # Calculate breathing rate from peak intervals
            if len(peaks) > 2:
                intervals = [peaks[i+1] - peaks[i] for i in range(len(peaks)-1)]
                avg_interval = sum(intervals) / len(intervals)
                
                # Assume sample rate of 2000 Hz
                sample_rate = 2000
                breathing_rate = 60.0 / (avg_interval / sample_rate)
                
                # Filter unrealistic values
                if 5 <= breathing_rate <= 40:
                    self.breathing_rate = (self.breathing_filter_alpha * breathing_rate +
                                         (1 - self.breathing_filter_alpha) * self.breathing_rate)
            
            return self.breathing_rate
            
        except Exception as e:
            self.logger.error(f"Error in breathing rate calculation: {e}")
            return self.breathing_rate

    def process_distance(self, fft_data=None):
        """Estimate distance from FFT data"""
        if not fft_data or not fft_data['peaks']:
            return self.distance
            
        try:
            # Find the strongest peak (likely the primary target)
            spectrum = fft_data['spectrum']
            strongest_peak_idx = max(fft_data['peaks'], key=lambda x: spectrum[x])
            
            # Convert peak index to distance (simplified calculation)
            # This would need calibration based on actual radar parameters
            max_range = 5.0  # meters
            distance = (strongest_peak_idx / len(spectrum)) * max_range
            
            # Apply reasonable limits
            self.distance = max(0.1, min(5.0, distance))
            return self.distance
            
        except Exception as e:
            self.logger.error(f"Error in distance calculation: {e}")
            return self.distance

    def wait_for_interrupt(self, timeout=1.0):
        """Wait for interrupt signal"""
        if self.irq_pin is None:
            return True  # Skip interrupt check if pin not configured
            
        start_time = time.time()
        while time.time() - start_time < timeout:
            if GPIO.input(self.irq_pin) == GPIO.LOW:
                return True
            time.sleep(0.001)
        return False

    def get_sensor_data(self):
        """Get processed sensor data"""
        try:
            # Start chirp
            self.start_chirp()
            
            # Wait for FIFO ready
            if not self.wait_for_fifo_ready(timeout=2.0):
                self.logger.warning("Timeout waiting for FIFO ready")
                return {
                    'motion_detected': self.motion_detected,
                    'breathing_rate': self.breathing_rate,
                    'distance': self.distance,
                    'samples_count': 0,
                    'ready': False,
                    'spectrum': []
                }
            
            # Read samples
            samples = self.read_fifo_data(self.samples_per_chirp)
            
            if len(samples) > 0:
                # Process FFT analysis
                fft_data = self.process_fft_analysis(samples)
                
                # Process motion detection
                motion = self.process_motion_detection(samples, fft_data)
                
                # Process breathing rate
                breathing = self.process_breathing_rate(samples)
                
                # Process distance
                distance = self.process_distance(fft_data)
                
                return {
                    'motion_detected': motion,
                    'breathing_rate': breathing,
                    'distance': distance,
                    'samples_count': len(samples),
                    'ready': True,
                    'raw_samples': samples[-32:] if len(samples) >= 32 else samples,
                    'spectrum': fft_data['spectrum'].tolist() if fft_data else [],
                    'peaks': fft_data['peaks'] if fft_data else []
                }
            else:
                return {
                    'motion_detected': self.motion_detected,
                    'breathing_rate': self.breathing_rate,
                    'distance': self.distance,
                    'samples_count': 0,
                    'ready': False,
                    'spectrum': []
                }
                
        except Exception as e:
            self.logger.error(f"Error getting sensor data: {e}")
            return {
                'motion_detected': self.motion_detected,
                'breathing_rate': self.breathing_rate,
                'distance': self.distance,
                'samples_count': 0,
                'ready': False,
                'spectrum': []
            }

    def set_motion_threshold(self, threshold):
        """Set motion detection threshold"""
        self.motion_threshold = max(10, min(1000, threshold))
        self.logger.info(f"Motion threshold set to: {self.motion_threshold}")

    def set_fft_threshold(self, threshold_db):
        """Set FFT peak detection threshold"""
        self.fft_threshold_db = threshold_db
        self.logger.info(f"FFT threshold set to: {self.fft_threshold_db} dB")

    def get_status(self):
        """Get sensor status with detailed information"""
        stat0_raw = self.read_register(self.REG_STAT0)
        stat0_decoded = self.decode_status_register(stat0_raw)
        
        return {
            'chip_id': self.read_register(self.REG_CHIP_ID),
            'main_reg': self.read_register(self.REG_MAIN),
            'stat0_raw': stat0_raw,
            'stat0_decoded': stat0_decoded,
            'pll1_0': self.read_register(self.REG_PLL1_0),
        }

    def close(self):
        """Clean up resources"""
        try:
            if self.spi:
                self.spi.close()
            GPIO.cleanup()
            self.logger.info("BGT60TR13C cleanup completed")
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")


# Example usage
if __name__ == "__main__":
    radar = BGT60TR13C()
    
    # Print register map for debugging
    radar.print_register_map()
    
    # Print configuration
    radar.print_configuration()
    
    # Initialize sensor
    if radar.begin():
        print("Sensor initialized successfully!")
        
        # Get status with detailed info
        status = radar.get_status()
        print(f"\nSensor Status:")
        print(f"FIFO Ready: {status['stat0_decoded']['fifo_ready']}")
        print(f"PLL Lock: {status['stat0_decoded']['pll_lock']}")
    
    radar.close()