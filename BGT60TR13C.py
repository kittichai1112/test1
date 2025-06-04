import spidev
import RPi.GPIO as GPIO
import time
import numpy as np
import logging
import reg as reg

class BGT60TR13C:
    def __init__(self, spi_bus=0, spi_device=0, cs_pin=8, rst_pin=7, irq_pin=25):
        self.spi = spidev.SpiDev()
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.cs_pin = cs_pin
        self.rst_pin = rst_pin
        self.irq_pin = irq_pin

        # REGISTER MAP (corrected from datasheet)
        self.REG_MAIN        = 0x00
        self.REG_CHIP_ID     = 0x02 
        self.REG_PACR1       = 0x04
        self.REG_PACR2       = 0x05
        self.REG_SFCTL       = 0x06
        self.REG_PLL1_0      = 0x30
        self.REG_PLL1_1      = 0x31
        self.REG_PLL1_2      = 0x32
        self.REG_PLL1_3      = 0x33
        self.REG_STAT0       = 0x5D
        self.REG_FIFO_ACCESS = 0x60

        # Configuration values from datasheet
        self.PLL1_0_CONFIG = 0xC4
        self.PLL1_1_CONFIG = 0x09
        self.PLL1_2_CONFIG = 0x00
        self.PLL1_3_CONFIG = 0x80
        self.PACR1_CONFIG  = 0x01
        self.PACR2_CONFIG  = 0x07
        self.SFCTL_CONFIG  = 0x01

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

            # Configure registers
            self.logger.info("Configuring BGT60TR13C registers...")
            self.write_register(self.REG_PLL1_0, self.PLL1_0_CONFIG)
            time.sleep(0.01)
            self.write_register(self.REG_PLL1_1, self.PLL1_1_CONFIG)
            time.sleep(0.01)
            self.write_register(self.REG_PLL1_2, self.PLL1_2_CONFIG)
            time.sleep(0.01)
            self.write_register(self.REG_PLL1_3, self.PLL1_3_CONFIG)
            time.sleep(0.01)
            self.write_register(self.REG_PACR1, self.PACR1_CONFIG)
            time.sleep(0.01)
            self.write_register(self.REG_PACR2, self.PACR2_CONFIG)
            time.sleep(0.01)
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

    def start_chirp(self):
        """Start a chirp sequence"""
        self.write_register(self.REG_MAIN, 0x01)

    def wait_for_fifo_ready(self, timeout=1.0):
        """Wait for FIFO to be ready"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            stat0 = self.read_register(self.REG_STAT0)
            if stat0 & 0x01:  # FIFO ready bit
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
                    self.breathing_rate = (self._filter_alpha * breathing_rate +
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
        """Get sensor status"""
        return {
            'chip_id': self.read_register(self.REG_CHIP_ID),
            'main_reg': self.read_register(self.REG_MAIN),
            'stat0': self.read_register(self.REG_STAT0),
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