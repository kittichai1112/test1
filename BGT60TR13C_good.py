import spidev
import RPi.GPIO as GPIO
import time
import numpy as np
import logging

class BGT60TR13C:
    def __init__(self, spi_bus=0, spi_device=0, cs_pin=8, rst_pin=7, irq_pin=25,
                 miso_pin=9, mosi_pin=10, sclk_pin=11):
        
        self.spi = spidev.SpiDev()
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.cs_pin = cs_pin
        self.rst_pin = rst_pin
        self.irq_pin = irq_pin
        self.miso_pin = miso_pin #Master in Slave out
        self.mosi_pin = mosi_pin # Master Out Slave in
        self.sclk_pin = sclk_pin # Serial Clock

        # REGISTER MAP
        self.REG_MAIN        = 0x00
        self.REG_CHIP_ID     = 0x02 
        self.REG_PACR1       = 0x04
        self.REG_PACR2       = 0x05
        self.REG_SFCTL       = 0x06
        self.REG_PLL1_0      = 0x30  # FSU - Frequency Start Unit
        self.REG_PLL1_1      = 0x31  # RSU - Ramp Step Unit
        self.REG_PLL1_2      = 0x32  # RTU - Ramp Time Unit
        self.REG_PLL1_3      = 0x33  # Additional PLL config
        self.REG_STAT0       = 0x5D
        self.REG_FIFO_ACCESS = 0x60

        # Chirp calculation constants
        self.F_REF = 80e6      # Reference frequency: 80 MHz
        self.F_OSC = 80e6      # Oscillator frequency: 80 MHz
        self.C_LIGHT = 3e8     # Speed of light
        
        # Default chirp parameters (you can modify these)
        self.chirp_config = {
            'start_freq_ghz': 61.0,
            'bandwidth_mhz': 1000,
            'ramp_time_us': 100,
            'sample_rate_khz': 2000
        }
        
        # Calculate and set register values
        self.update_chirp_configuration()

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

    def calculate_FSU(self, start_freq_ghz):
        """Calculate Frequency Start Unit (FSU) value"""
        start_freq_hz = start_freq_ghz * 1e9
        f_base = 60e9  # Base frequency 60 GHz
        fsu = int((start_freq_hz - f_base) * (2**24) / self.F_REF)
        return fsu & 0xFFFFFF  # 24-bit mask
    
    def calculate_RSU(self, bandwidth_mhz, ramp_time_us):
        """Calculate Ramp Step Unit (RSU) value"""
        bandwidth_hz = bandwidth_mhz * 1e6
        ramp_time_s = ramp_time_us * 1e-6
        freq_step = bandwidth_hz / (ramp_time_s * self.F_OSC)
        rsu = int(freq_step * (2**24) / self.F_REF)
        return rsu & 0xFFFFFF  # 24-bit mask
    
    def calculate_RTU(self, ramp_time_us, adc_sample_rate_khz=2000):
        """Calculate Ramp Time Unit (RTU) value"""
        ramp_time_s = ramp_time_us * 1e-6
        adc_rate_hz = adc_sample_rate_khz * 1e3
        num_samples = int(ramp_time_s * adc_rate_hz)
        rtu = num_samples // 8 + 4  # Add setup time
        return rtu & 0xFFFFF  # 20-bit mask
    
    def calculate_range_resolution(self):
        """Calculate range resolution based on current bandwidth"""
        bandwidth_hz = self.chirp_config['bandwidth_mhz'] * 1e6
        return self.C_LIGHT / (2 * bandwidth_hz)
    
    def calculate_max_range(self):
        """Calculate maximum detectable range"""
        # Based on chirp time and speed of light
        ramp_time_s = self.chirp_config['ramp_time_us'] * 1e-6
        return (self.C_LIGHT * ramp_time_s) / 2
    
    def update_chirp_configuration(self):
        """Update register values based on current chirp configuration"""
        self.PLL1_0_CONFIG = self.calculate_FSU(self.chirp_config['start_freq_ghz'])
        self.PLL1_1_CONFIG = self.calculate_RSU(
            self.chirp_config['bandwidth_mhz'], 
            self.chirp_config['ramp_time_us']
        )
        self.PLL1_2_CONFIG = self.calculate_RTU(
            self.chirp_config['ramp_time_us'],
            self.chirp_config['sample_rate_khz']
        )
        
        # Keep other configurations as default
        self.PLL1_3_CONFIG = 0x80  # Default from datasheet
        self.PACR1_CONFIG  = 0x01
        self.PACR2_CONFIG  = 0x07
        self.SFCTL_CONFIG  = 0x01

        self.logger.info(f"Updated chirp configuration:")
        self.logger.info(f"  Start frequency: {self.chirp_config['start_freq_ghz']} GHz")
        self.logger.info(f"  Bandwidth: {self.chirp_config['bandwidth_mhz']} MHz")
        self.logger.info(f"  Ramp time: {self.chirp_config['ramp_time_us']} μs")
        self.logger.info(f"  Range resolution: {self.calculate_range_resolution():.3f} m")
        self.logger.info(f"  Max range: {self.calculate_max_range():.3f} m")
        self.logger.info(f"  PLL1_0 (FSU): 0x{self.PLL1_0_CONFIG:06X}")
        self.logger.info(f"  PLL1_1 (RSU): 0x{self.PLL1_1_CONFIG:06X}")
        self.logger.info(f"  PLL1_2 (RTU): 0x{self.PLL1_2_CONFIG:05X}")
    
    def configure_chirp_for_application(self, application_type="motion_detection"):
        """
        Configure chirp parameters for specific applications
        
        Args:
            application_type: "motion_detection", "breathing", "distance", or "high_resolution"
        """
        if application_type == "motion_detection":
            self.chirp_config = {
                'start_freq_ghz': 61.0,
                'bandwidth_mhz': 500,      # Lower bandwidth for motion
                'ramp_time_us': 80,        # Shorter ramp time
                'sample_rate_khz': 2000
            }
        elif application_type == "breathing":
            self.chirp_config = {
                'start_freq_ghz': 61.25,
                'bandwidth_mhz': 200,      # Very low bandwidth for breathing
                'ramp_time_us': 200,       # Longer integration time
                'sample_rate_khz': 1000
            }
        elif application_type == "distance":
            self.chirp_config = {
                'start_freq_ghz': 60.5,
                'bandwidth_mhz': 2000,     # High bandwidth for range
                'ramp_time_us': 150,
                'sample_rate_khz': 4000
            }
        elif application_type == "high_resolution":
            self.chirp_config = {
                'start_freq_ghz': 60.0,
                'bandwidth_mhz': 4000,     # Maximum bandwidth
                'ramp_time_us': 120,
                'sample_rate_khz': 8000
            }
        
        self.update_chirp_configuration()
    
    def set_custom_chirp(self, start_freq_ghz, bandwidth_mhz, ramp_time_us, sample_rate_khz=2000):
        """Set custom chirp parameters"""
        # Validate input ranges
        if not (59.0 <= start_freq_ghz <= 64.0):
            raise ValueError("Start frequency must be between 59-64 GHz")
        if not (50 <= bandwidth_mhz <= 4000):
            raise ValueError("Bandwidth must be between 50-4000 MHz")
        if not (10 <= ramp_time_us <= 1000):
            raise ValueError("Ramp time must be between 10-1000 μs")
        
        self.chirp_config = {
            'start_freq_ghz': start_freq_ghz,
            'bandwidth_mhz': bandwidth_mhz,
            'ramp_time_us': ramp_time_us,
            'sample_rate_khz': sample_rate_khz
        }
        
        self.update_chirp_configuration()

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

            # Configure registers with calculated values
            self.logger.info("Configuring BGT60TR13C registers...")
            self.write_register(self.REG_PLL1_0, (self.PLL1_0_CONFIG >> 16) & 0xFF)
            time.sleep(0.01)
            self.write_register(self.REG_PLL1_0 + 1, (self.PLL1_0_CONFIG >> 8) & 0xFF)
            time.sleep(0.01)
            self.write_register(self.REG_PLL1_0 + 2, self.PLL1_0_CONFIG & 0xFF)
            time.sleep(0.01)
            
            # Similar for RSU and RTU (assuming multi-byte registers)
            # You may need to adjust based on actual register layout
            
            self.write_register(self.REG_PACR1, self.PACR1_CONFIG)
            time.sleep(0.01)
            self.write_register(self.REG_PACR2, self.PACR2_CONFIG)
            time.sleep(0.01)
            self.write_register(self.REG_SFCTL, self.SFCTL_CONFIG)
            time.sleep(0.01)

            # Verify configuration
            chip_id = self.read_register(self.REG_CHIP_ID)
            self.logger.info(f"Chip ID: 0x{chip_id:02X}")

            if chip_id != 0:  # Any non-zero chip ID indicates communication
                self.logger.info("BGT60TR13C initialized successfully")
                return True
            else:
                self.logger.error("Failed to communicate with sensor")
                return False

        except Exception as e:
            self.logger.error(f"Error initializing BGT60TR13C: {e}")
            return False

    def write_register(self, address, data):
        """Write to 8-bit register"""
        try:
            GPIO.output(self.cs_pin, GPIO.LOW)
            time.sleep(0.001)
            
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
            if stat0 & 0x01:
                return True
            time.sleep(0.001)
        return False

    def read_fifo_data(self, num_samples=None):
        """Read samples from FIFO"""
        if num_samples is None:
            num_samples = self.samples_per_chirp
            
        try:
            samples = []
            for _ in range(num_samples):
                sample = self.read_register(self.REG_FIFO_ACCESS)
                samples.append(sample)
            return samples
        except Exception as e:
            self.logger.error(f"Error reading FIFO data: {e}")
            return []

    def process_fft_analysis(self, samples):
        """Perform FFT analysis on radar samples"""
        if len(samples) < 2:
            return None
            
        try:
            # Convert to numpy array and apply window function
            data = np.array(samples, dtype=np.float32)
            windowed_data = data * np.hanning(len(data))
            
            # Perform FFT
            fft_result = np.fft.fft(windowed_data)
            magnitude = np.abs(fft_result)
            
            # Convert to dB
            magnitude_db = 20 * np.log10(magnitude + 1e-10)
            
            # Calculate frequency bins
            sample_rate = self.chirp_config['sample_rate_khz'] * 1000
            freqs = np.fft.fftfreq(len(samples), 1/sample_rate)
            
            # Store spectrum data
            self.spectrum_data = {
                'frequencies': freqs[:len(freqs)//2],
                'magnitude_db': magnitude_db[:len(magnitude_db)//2],
                'magnitude': magnitude[:len(magnitude)//2]
            }
            
            return self.spectrum_data
            
        except Exception as e:
            self.logger.error(f"Error in FFT analysis: {e}")
            return None

    def detect_motion(self, samples):
        """Detect motion based on radar samples"""
        if len(samples) < 2:
            return False
            
        try:
            # Calculate RMS of current samples
            current_rms = np.sqrt(np.mean(np.array(samples)**2))
            
            if len(self.last_samples) > 0:
                # Calculate RMS of previous samples
                previous_rms = np.sqrt(np.mean(np.array(self.last_samples)**2))
                
                # Calculate difference
                rms_diff = abs(current_rms - previous_rms)
                
                # Motion detection threshold
                motion_detected = rms_diff > self.motion_threshold
                
                # Update motion history
                self.motion_history.append(motion_detected)
                if len(self.motion_history) > 10:
                    self.motion_history.pop(0)
                
                # Consider motion detected if recent history shows activity
                self.motion_detected = sum(self.motion_history) > 2
                
            self.last_samples = samples.copy()
            return self.motion_detected
            
        except Exception as e:
            self.logger.error(f"Error in motion detection: {e}")
            return False

    def estimate_distance(self, samples):
        """Estimate distance to the strongest target"""
        try:
            spectrum = self.process_fft_analysis(samples)
            if spectrum is None:
                return 0
                
            # Find peak frequency
            magnitude = spectrum['magnitude']
            peak_idx = np.argmax(magnitude)
            peak_freq = abs(spectrum['frequencies'][peak_idx])
            
            # Convert frequency to distance
            # Distance = (c * f * T) / (2 * B)
            # where T is ramp time, B is bandwidth
            ramp_time = self.chirp_config['ramp_time_us'] * 1e-6
            bandwidth = self.chirp_config['bandwidth_mhz'] * 1e6
            
            distance = (self.C_LIGHT * peak_freq * ramp_time) / (2 * bandwidth)
            
            self.distance = distance
            return distance
            
        except Exception as e:
            self.logger.error(f"Error in distance estimation: {e}")
            return 0

    def estimate_breathing_rate(self, samples):
        """Estimate breathing rate from radar samples"""
        try:
            if len(samples) < 32:
                return 0
                
            # Apply low-pass filtering for breathing detection
            data = np.array(samples, dtype=np.float32)
            
            # Simple moving average filter
            filtered_data = np.convolve(data, np.ones(5)/5, mode='same')
            
            # Find peaks in the filtered signal
            # This is a simplified breathing detection
            diff = np.diff(filtered_data)
            zero_crossings = np.where(np.diff(np.sign(diff)))[0]
            
            if len(zero_crossings) > 2:
                # Estimate breathing rate based on zero crossings
                # Assuming each breath cycle has 2 zero crossings
                sample_period = 1.0 / (self.chirp_config['sample_rate_khz'] * 1000)
                total_time = len(samples) * sample_period
                cycles = len(zero_crossings) / 2
                breathing_rate = (cycles / total_time) * 60  # Convert to breaths per minute
                
                # Apply simple filtering
                self.breathing_rate = (self.breathing_filter_alpha * breathing_rate + 
                                     (1 - self.breathing_filter_alpha) * self.breathing_rate)
            
            return self.breathing_rate
            
        except Exception as e:
            self.logger.error(f"Error in breathing rate estimation: {e}")
            return 0

    def get_sensor_data(self):
        """Get comprehensive sensor data"""
        try:
            # Start chirp and wait for data
            self.start_chirp()
            
            if self.wait_for_fifo_ready():
                samples = self.read_fifo_data()
                
                if samples:
                    # Process all measurements
                    motion = self.detect_motion(samples)
                    distance = self.estimate_distance(samples)
                    breathing = self.estimate_breathing_rate(samples)
                    spectrum = self.process_fft_analysis(samples)
                    
                    return {
                        'motion_detected': motion,
                        'distance_m': distance,
                        'breathing_rate_bpm': breathing,
                        'raw_samples': samples,
                        'spectrum': spectrum,
                        'timestamp': time.time()
                    }
            
            return None
            
        except Exception as e:
            self.logger.error(f"Error getting sensor data: {e}")
            return None

    def calibrate_sensor(self, num_samples=50):
        """Calibrate sensor by taking baseline measurements"""
        self.logger.info("Calibrating sensor...")
        
        baseline_samples = []
        for i in range(num_samples):
            self.start_chirp()
            if self.wait_for_fifo_ready():
                samples = self.read_fifo_data()
                if samples:
                    baseline_samples.extend(samples)
            time.sleep(0.01)
        
        if baseline_samples:
            baseline_rms = np.sqrt(np.mean(np.array(baseline_samples)**2))
            self.motion_threshold = baseline_rms * 2  # Set threshold as 2x baseline
            self.logger.info(f"Calibration complete. Motion threshold: {self.motion_threshold:.2f}")
        else:
            self.logger.warning("Calibration failed - no data received")

    def get_chirp_info(self):
        """Get current chirp configuration information"""
        return {
            'configuration': self.chirp_config,
            'range_resolution_m': self.calculate_range_resolution(),
            'max_range_m': self.calculate_max_range(),
            'register_values': {
                'FSU': f"0x{self.PLL1_0_CONFIG:06X}",
                'RSU': f"0x{self.PLL1_1_CONFIG:06X}",
                'RTU': f"0x{self.PLL1_2_CONFIG:05X}"
            }
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


# Example usage and demonstration
if __name__ == "__main__":
    # Create sensor instance
    sensor = BGT60TR13C()
    
    try:
        # Initialize sensor
        if sensor.begin():
            print("Sensor initialized successfully!")
            
            # Configure for motion detection
            sensor.configure_chirp_for_application("motion_detection")
            
            # Calibrate sensor
            sensor.calibrate_sensor()
            
            # Get configuration info
            info = sensor.get_chirp_info()
            print("\nChirp Configuration:")
            for key, value in info.items():
                print(f"  {key}: {value}")
            
            print("\nStarting continuous measurement...")
            print("Press Ctrl+C to stop")
            
            # Continuous measurement loop
            while True:
                data = sensor.get_sensor_data()
                
                if data:
                    print(f"\rMotion: {'YES' if data['motion_detected'] else 'NO'} | "
                          f"Distance: {data['distance_m']:.2f}m | "
                          f"Breathing: {data['breathing_rate_bpm']:.1f} BPM", end='')
                
                time.sleep(0.1)
                
        else:
            print("Failed to initialize sensor!")
            
    except KeyboardInterrupt:
        print("\nStopping measurement...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sensor.close()
        print("Sensor closed.")