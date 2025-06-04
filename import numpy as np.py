import numpy as np
import matplotlib.pyplot as plt
import spidev
import json
import time
from typing import Dict, List, Tuple, Optional
import struct
from dataclasses import dataclass
from scipy.fft import fft

@dataclass
class RadarConfig:
    """Configuration parameters for BGT60TR13C radar sensor"""
    tx_power: int = 31
    if_gain_db: int = 33
    sample_rate_hz: int = 1000000
    range_resolution_m: float = 0.15
    max_range_m: float = 9.59
    max_speed_m_s: float = 2.45
    speed_resolution_m_s: float = 0.19
    frame_period_ms: int = 1000
    num_s_per_frame: int = 16
    num_samples_per_chirp: int = 64
    tx_antennas: List[int] = None
    rx_antennas: List[int] = None

    def __post_init__(self):
        if self.tx_antennas is None:
            self.tx_antennas = [0]
        if self.rx_antennas is None:
            self.rx_antennas = [0, 1, 2]

class BGT60TR13C_SPI:
    """
    BGT60TR13C 60 GHz Radar Sensor Interface Class (SPI version)
    """

    def __init__(self, spi_bus: int = 0, spi_device: int = 0, spi_speed: int = 1000000):
        """
        Initialize BGT60TR13C radar sensor via SPI
        Args:
            spi_bus: SPI bus number
            spi_device: SPI device number (CS)
            spi_speed: SPI speed in Hz
        """
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.spi_speed = spi_speed
        self.spi = None
        self.config = RadarConfig()
        self.is_connected = False
        self.raw_data = None

    def connect(self) -> bool:
        """Establish SPI connection with radar sensor"""
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = self.spi_speed
            self.spi.mode = 0b00
            # Test connection (could be custom for your firmware)
            self.spi.xfer2([0xAA]) # Ping or version command (edit as needed)s
            self.is_connected = True
            print(f"Connected to BGT60TR13C via SPI bus {self.spi_bus}, device {self.spi_device}")
            return True
        except Exception as e:
            print(f"SPI connection error: {e}")
            return False

    def disconnect(self):
        """Disconnect from radar sensor"""
        if self.spi is not None:
            self.spi.close()
            self.is_connected = False
            print("Disconnected from radar sensor")

    def _spi_write_command(self, command: str):
        """Send ASCII command string via SPI (for example firmware)"""
        # Convert to bytes and send (this is protocol-specific)
        cmd_bytes = command.encode() + b'\n'
        for b in cmd_bytes:
            self.spi.xfer2([b])
        time.sleep(0.002)

    def _spi_readline(self, maxlen=128) -> str:
        """Read bytes until newline via SPI, decode to string (protocol-specific)"""
        buf = []
        for _ in range(maxlen):
            b = self.spi.xfer2([0x00])[0]
            if b == 0x0A:
                break
            buf.append(b)
        return bytes(buf).decode(errors='ignore').strip()

    def configure_sensor(self, config: Optional[RadarConfig] = None):
        """
        Configure radar sensor parameters via SPI ASCII commands
        """
        if config:
            self.config = config
        if not self.is_connected:
            raise RuntimeError("Sensor not connected")
        config_commands = [
            f"SET:TX_POWER:{self.config.tx_power}",
            f"SET:IF_GAIN:{self.config.if_gain_db}",
            f"SET:SAMPLE_RATE:{self.config.sample_rate_hz}",
            f"SET:FRAME_PERIOD:{self.config.frame_period_ms}",
            f"SET:CHIRPS_PER_FRAME:{self.config.num_chirps_per_frame}",
            f"SET:SAMPLES_PER_CHIRP:{self.config.num_samples_per_chirp}",
            f"SET:TX_ANTENNAS:{','.join(map(str, self.config.tx_antennas))}",
            f"SET:RX_ANTENNAS:{','.join(map(str, self.config.rx_antennas))}"
        ]
        for cmd in config_commands:
            self._spi_write_command(cmd)
            response = self._spi_readline()
            if response != 'OK':
                print(f"Warning: Command '{cmd}' returned: {response}")
        print("Sensor configured successfully")

    def start_measurement(self):
        """Start radar measurement"""
        if not self.is_connected:
            raise RuntimeError("Sensor not connected")
        self._spi_write_command('START')
        response = self._spi_readline()
        if response == 'OK':
            print("Measurement started")
        else:
            print(f"Failed to start measurement: {response}")

    def stop_measurement(self):
        """Stop radar measurement"""
        if not self.is_connected:
            raise RuntimeError("Sensor not connected")
        self._spi_write_command('STOP')
        response = self._spi_readline()
        if response == 'OK':
            print("Measurement stopped")
        else:
            print(f"Failed to stop measurement: {response}")

    def acquire_frame(self) -> np.ndarray:
        """
        Acquire one frame of radar data via SPI
        Returns:
            np.ndarray: Raw radar data [chirps, samples, rx_antennas]
        """
        if not self.is_connected:
            raise RuntimeError("Sensor not connected")
        self._spi_write_command('GET:FRAME')
        # Read 4 bytes for data size (little endian)
        size_bytes = [self.spi.xfer2([0x00])[0] for _ in range(4)]
        data_size = struct.unpack('<I', bytes(size_bytes))[0]
        # Read the frame data (data_size bytes)
        raw_bytes = []
        for _ in range(data_size):
            raw_bytes.append(self.spi.xfer2([0x00])[0])
        raw_bytes = bytes(raw_bytes)
        raw_data = np.frombuffer(raw_bytes, dtype=np.int16)
        num_rx = len(self.config.rx_antennas)
        data_shape = (
            self.config.num_chirps_per_frame,
            self.config.num_samples_per_chirp,
            num_rx, 2
        )
        reshaped_data = raw_data.reshape(data_shape)
        complex_data = reshaped_data[:, :, :, 0] + 1j * reshaped_data[:, :, :, 1]
        self.raw_data = complex_data
        return complex_data

    def process_range_fft(self, data: Optional[np.ndarray] = None) -> np.ndarray:
        """Perform Range FFT processing"""
        if data is None:
            if self.raw_data is None:
                raise RuntimeError("No data available for processing")
            data = self.raw_data
        window = np.hanning(data.shape[1])
        windowed_data = data * window[np.newaxis, :, np.newaxis]
        range_fft = fft(windowed_data, axis=1)
        return range_fft

    def process_doppler_fft(self, data: Optional[np.ndarray] = None) -> np.ndarray:
        """Perform Doppler FFT processing"""
        if data is None:
            data = self.process_range_fft()
        window = np.hanning(data.shape[0])
        windowed_data = data * window[:, np.newaxis, np.newaxis]
        doppler_fft = fft(windowed_data, axis=0)
        return doppler_fft

    def detect_targets(self, range_doppler_map: np.ndarray, threshold_db: float = -60.0) -> List[Dict]:
        """
        Detect targets from range-Doppler map
        Returns:
            List of detected targets with range, velocity, and power
        """
        power_db = 20 * np.log10(np.abs(range_doppler_map) + 1e-12)
        avg_power = np.mean(power_db, axis=2)
        peaks = np.where(avg_power > threshold_db)
        targets = []
        for chirp_idx, range_idx in zip(peaks[0], peaks[1]):
            range_m = (range_idx * self.config.range_resolution_m / 2)
            velocity_m_s = ((chirp_idx - self.config.num_chirps_per_frame // 2) * self.config.speed_resolution_m_s)
            power_db_val = avg_power[chirp_idx, range_idx]
            targets.append({
                'range_m': range_m,
                'velocity_m_s': velocity_m_s,
                'power_db': power_db_val,
                'chirp_idx': chirp_idx,
                'range_idx': range_idx
            })
        targets.sort(key=lambda x: x['power_db'], reverse=True)
        return targets

    def calculate_angle(self, data: np.ndarray, target_range_idx: int, target_chirp_idx: int) -> float:
        """
        Calculate angle of arrival using multiple RX antennas
        """
        if len(self.config.rx_antennas) < 2:
            return 0.0
        target_data = data[target_chirp_idx, target_range_idx, :]
        phase_diff = np.angle(target_data[1] / target_data[0])
        wavelength_m = 3e8 / 60e9
        antenna_spacing_m = wavelength_m / 2
        angle_rad = np.arcsin(phase_diff * wavelength_m / (2 * np.pi * antenna_spacing_m))
        angle_deg = np.degrees(angle_rad)
        return angle_deg

    def plot_range_profile(self, data: Optional[np.ndarray] = None):
        if data is None:
            data = self.process_range_fft()
        range_profile = np.mean(np.abs(data), axis=(0, 2))
        range_axis = np.arange(len(range_profile)) * self.config.range_resolution_m / 2
        plt.figure(figsize=(10, 6))
        plt.plot(range_axis, 20 * np.log10(range_profile + 1e-12))
        plt.xlabel('Range (m)')
        plt.ylabel('Power (dB)')
        plt.title('Range Profile')
        plt.grid(True)
        plt.show()

    def plot_range_doppler_map(self, data: Optional[np.ndarray] = None):
        if data is None:
            data = self.process_doppler_fft()
        rd_map = np.mean(np.abs(data), axis=2)
        rd_map_db = 20 * np.log10(rd_map + 1e-12)
        range_axis = np.arange(rd_map.shape[1]) * self.config.range_resolution_m / 2
        velocity_axis = (np.arange(rd_map.shape[0]) - rd_map.shape[0] // 2) * self.config.speed_resolution_m_s
        plt.figure(figsize=(12, 8))
        plt.imshow(rd_map_db, aspect='auto', origin='lower',
                  extent=[range_axis[0], range_axis[-1], velocity_axis[0], velocity_axis[-1]])
        plt.colorbar(label='Power (dB)')
        plt.xlabel('Range (m)')
        plt.ylabel('Velocity (m/s)')
        plt.title('Range-Doppler Map')
        plt.show()

    def save_data(self, filename: str, data: Optional[np.ndarray] = None):
        if data is None:
            data = self.raw_data
        if data is None:
            raise RuntimeError("No data to save")
        np.save(filename, data)
        config_dict = {
            'tx_power': self.config.tx_power,
            'if_gain_db': self.config.if_gain_db,
            'sample_rate_hz': self.config.sample_rate_hz,
            'range_resolution_m': self.config.range_resolution_m,
            'max_range_m': self.config.max_range_m,
            'max_speed_m_s': self.config.max_speed_m_s,
            'speed_resolution_m_s': self.config.speed_resolution_m_s,
            'frame_period_ms': self.config.frame_period_ms,
            'num_chirps_per_frame': self.config.num_chirps_per_frame,
            'num_samples_per_chirp': self.config.num_samples_per_chirp,
            'tx_antennas': self.config.tx_antennas,
            'rx_antennas': self.config.rx_antennas
        }
        with open(filename.replace('.npy', '_config.json'), 'w') as f:
            json.dump(config_dict, f, indent=2)
        print(f"Data saved to {filename}")

    def load_data(self, filename: str):
        self.raw_data = np.load(filename)
        config_file = filename.replace('.npy', '_config.json')
        try:
            with open(config_file, 'r') as f:
                config_dict = json.load(f)
            for key, value in config_dict.items():
                if hasattr(self.config, key):
                    setattr(self.config, key, value)
            print(f"Data loaded from {filename}")
        except FileNotFoundError:
            print(f"Configuration file {config_file} not found, using default config")

def main():
    print("BGT60TR13C Radar Sensor Demo (SPI)")
    print("=" * 40)
    radar = BGT60TR13C_SPI(spi_bus=0, spi_device=0)
    try:
        if not radar.connect():
            print("Failed to connect to radar sensor")
            return
        config = RadarConfig(
            tx_power=31,
            if_gain_db=33,
            num_chirps_per_frame=32,
            num_samples_per_chirp=128,
            frame_period_ms=100
        )
        radar.configure_sensor(config)
        radar.start_measurement()
        print("\nAcquiring radar data...")
        for frame_idx in range(10):
            print(f"Frame {frame_idx + 1}/10", end='\r')
            raw_data = radar.acquire_frame()
            range_fft = radar.process_range_fft(raw_data)
            doppler_fft = radar.process_doppler_fft(range_fft)
            targets = radar.detect_targets(doppler_fft, threshold_db=-50.0)
            if targets:
                print(f"\nFrame {frame_idx + 1}: Found {len(targets)} targets")
                for i, target in enumerate(targets[:3]):
                    angle = radar.calculate_angle(doppler_fft, target['range_idx'], target['chirp_idx'])
                    print(f"  Target {i+1}: Range={target['range_m']:.2f}m, "
                          f"Velocity={target['velocity_m_s']:.2f}m/s, "
                          f"Power={target['power_db']:.1f}dB, "
                          f"Angle={angle:.1f}°")
            time.sleep(0.1)
        print("\n\nGenerating plots...")
        radar.plot_range_profile()
        radar.plot_range_doppler_map()
        radar.save_data('radar_data.npy')
        radar.stop_measurement()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        radar.disconnect()

if __name__ == "__main__":
    main()