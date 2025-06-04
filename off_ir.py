import time
import logging
from datetime import datetime

class AirconController:
    def __init__(self, sensor):
        self.sensor = sensor
        
        # AC state
        self.ac_state = {
            'power': False,
            'temperature': 25,
            'mode': 'auto',
            'fan_speed': 'medium'
        }
        
        # Room state
        self.room_occupied = False
        self.sleep_mode = False
        self.last_motion_time = time.time()
        
        # Thresholds and settings
        self.empty_room_timeout = 600  # 10 minutes
        self.sleep_detection_timeout = 300  # 5 minutes
        self.sleep_breathing_threshold = 12  # BPM
        self.motion_confirmation_count = 3
        self.motion_buffer = []
        
        # Energy saving settings
        self.energy_save_temp_offset = 2  # degrees
        self.sleep_mode_temp_offset = 1   # degrees
        
        # Logging
        logging.basicConfig(level=logging.INFO, 
                          format='%(asctime)s - %(levelname)s - %(message)s')
        self.logger = logging.getLogger(__name__)
        
        # Statistics
        self.stats = {
            'total_runtime': 0,
            'motion_detections': 0,
            'sleep_periods': 0,
            'energy_save_periods': 0,
            'ac_state_changes': 0
        }
        
        self.start_time = time.time()

    def control_loop(self):
        """Main control loop for smart AC management"""
        self.logger.info("SYSTEM STARTUP: Smart AC Control System Initialized")
        self.logger.info("STATUS: Monitoring room occupancy and sleep patterns")
        
        loop_count = 0
        
        while True:
            try:
                # Get sensor data
                data = self.sensor.get_sensor_data()
                
                if data['ready']:
                    current_time = time.time()
                    
                    # Process motion detection with confirmation
                    self.process_motion_detection(data, current_time)
                    
                    # Process sleep detection
                    self.process_sleep_detection(data, current_time)
                    
                    # Update AC control based on room state
                    self.update_ac_control()
                    
                    # Log status every 10 loops (approximately every 10 seconds)
                    if loop_count % 10 == 0:
                        self.log_status(data)
                        
                    # Log detailed info every 60 loops (approximately every minute)
                    if loop_count % 60 == 0:
                        self.log_detailed_status(data)
                        
                else:
                    self.logger.warning("SENSOR WARNING: Sensor data not ready")
                
                # Update statistics
                self.update_statistics()
                
                time.sleep(1)
                loop_count += 1
                
            except KeyboardInterrupt:
                self.logger.info("SYSTEM SHUTDOWN: Smart AC Control System Terminated")
                self.log_final_statistics()
                break
                
            except Exception as e:
                self.logger.error(f"SYSTEM ERROR: Control loop exception - {e}")
                time.sleep(1)

    def process_motion_detection(self, data, current_time):
        """Process motion detection with confirmation buffer"""
        motion_detected = data['motion_detected']
        
        # Add to motion buffer for confirmation
        self.motion_buffer.append(motion_detected)
        if len(self.motion_buffer) > self.motion_confirmation_count:
            self.motion_buffer.pop(0)
        
        # Confirm motion if majority of recent readings show motion
        confirmed_motion = sum(self.motion_buffer) > len(self.motion_buffer) // 2
        
        if confirmed_motion:
            if not self.room_occupied:
                self.logger.info("OCCUPANCY DETECTED: Room status changed to occupied")
                self.stats['motion_detections'] += 1
                
            self.room_occupied = True
            self.last_motion_time = current_time
            self.sleep_mode = False
            
        # Check for room emptiness
        elif current_time - self.last_motion_time > self.empty_room_timeout:
            if self.room_occupied:
                self.logger.info("ENERGY SAVE MODE: Room vacant for 10+ minutes - Activating energy conservation")
                self.stats['energy_save_periods'] += 1
                
            self.room_occupied = False
            self.sleep_mode = False

    def process_sleep_detection(self, data, current_time):
        """Detect sleep patterns based on breathing rate and motion"""
        if not self.room_occupied:
            return
            
        breathing_rate = data['breathing_rate']
        time_since_motion = current_time - self.last_motion_time
        
        # Sleep detection criteria:
        # 1. Room is occupied
        # 2. No motion for 5+ minutes
        # 3. Breathing rate is slow and steady (< 12 BPM)
        # 4. Distance reading is stable (person lying down)
        
        sleep_conditions = [
            breathing_rate > 0,  # Valid breathing detected
            breathing_rate < self.sleep_breathing_threshold,  # Slow breathing
            time_since_motion > self.sleep_detection_timeout,  # No motion
            data['distance'] > 0.5  # Reasonable distance reading
        ]
        
        if all(sleep_conditions):
            if not self.sleep_mode:
                self.logger.info("SLEEP MODE ACTIVATED: Quiet operation mode enabled")
                self.stats['sleep_periods'] += 1
            self.sleep_mode = True
        elif time_since_motion < 60:  # Recent motion cancels sleep mode
            if self.sleep_mode:
                self.logger.info("NORMAL MODE RESTORED: Activity detected - Sleep mode deactivated")
            self.sleep_mode = False

    def update_ac_control(self):
        """Update AC settings based on room state"""
        old_state = self.ac_state.copy()
        
        if not self.room_occupied:
            # Energy saving mode - turn off AC
            self.ac_state['power'] = False
            
        elif self.sleep_mode:
            # Sleep mode - quiet and slightly warmer
            self.ac_state['power'] = True
            self.ac_state['temperature'] = 25 + self.sleep_mode_temp_offset
            self.ac_state['mode'] = 'quiet'
            self.ac_state['fan_speed'] = 'low'
            
        else:
            # Normal occupied mode
            self.ac_state['power'] = True
            self.ac_state['temperature'] = 25
            self.ac_state['mode'] = 'auto'
            self.ac_state['fan_speed'] = 'medium'
        
        # Check if state changed
        if old_state != self.ac_state:
            self.stats['ac_state_changes'] += 1
            self.log_ac_state_change(old_state, self.ac_state)

    def log_ac_state_change(self, old_state, new_state):
        """Log AC state changes"""
        changes = []
        for key in new_state:
            if old_state[key] != new_state[key]:
                changes.append(f"{key}: {old_state[key]} -> {new_state[key]}")
        
        if changes:
            self.logger.info(f"AC CONFIGURATION UPDATED: {', '.join(changes)}")

    def log_status(self, data):
        """Log current status"""
        if self.sleep_mode:
            room_status = "OCCUPIED (SLEEP MODE)"
        elif self.room_occupied:
            room_status = "OCCUPIED"
        else:
            room_status = "VACANT"
        
        ac_status = f"{'ACTIVE' if self.ac_state['power'] else 'STANDBY'}"
        if self.ac_state['power']:
            ac_status += f" | {self.ac_state['temperature']}°C | {self.ac_state['mode'].upper()}"
        
        motion_indicator = "YES" if data['motion_detected'] else "NO"
        
        self.logger.info(f"STATUS REPORT: Room: {room_status} | Motion: {motion_indicator} | "
                        f"Breathing Rate: {data['breathing_rate']:.1f} BPM | "
                        f"Distance: {data['distance']:.2f}m | AC: {ac_status}")

    def log_detailed_status(self, data):
        """Log detailed status with spectrum analysis"""
        self.logger.info("-" * 80)
        self.logger.info(f"DETAILED SYSTEM STATUS - {datetime.now().strftime('%H:%M:%S')}")
        self.logger.info(f"Room Occupancy Status: {'OCCUPIED' if self.room_occupied else 'VACANT'}")
        self.logger.info(f"Sleep Mode Status: {'ACTIVE' if self.sleep_mode else 'INACTIVE'}")
        self.logger.info(f"Time Since Last Motion: {(time.time() - self.last_motion_time):.0f} seconds")
        
        # Sensor data
        self.logger.info(f"Sensor Readings - Motion Detection: {data['motion_detected']}, "
                        f"Breathing Rate: {data['breathing_rate']:.1f} BPM, "
                        f"Distance Measurement: {data['distance']:.2f}m, "
                        f"Sample Count: {data['samples_count']}")
        
        # Spectrum analysis
        if data.get('peaks'):
            self.logger.info(f"FFT Analysis - Signal peaks detected at frequency bins: {data['peaks']}")
        else:
            self.logger.info("FFT Analysis - No significant frequency peaks detected")
        
        # AC status
        self.logger.info(f"AC System Status - Power: {'ON' if self.ac_state['power'] else 'OFF'}, "
                        f"Temperature Setting: {self.ac_state['temperature']}°C, "
                        f"Operation Mode: {self.ac_state['mode'].upper()}, "
                        f"Fan Speed: {self.ac_state['fan_speed'].upper()}")
        
        self.logger.info("-" * 80)

    def update_statistics(self):
        """Update runtime statistics"""
        self.stats['total_runtime'] = time.time() - self.start_time

    def log_final_statistics(self):
        """Log final statistics when shutting down"""
        runtime_hours = self.stats['total_runtime'] / 3600
        
        self.logger.info("=" * 60)
        self.logger.info("SYSTEM PERFORMANCE STATISTICS")
        self.logger.info(f"Total Runtime: {runtime_hours:.2f} hours")
        self.logger.info(f"Motion Detection Events: {self.stats['motion_detections']}")
        self.logger.info(f"Sleep Mode Activations: {self.stats['sleep_periods']}")
        self.logger.info(f"Energy Save Mode Activations: {self.stats['energy_save_periods']}")
        self.logger.info(f"AC Configuration Changes: {self.stats['ac_state_changes']}")
        
        # Calculate efficiency metrics
        if runtime_hours > 0:
            avg_state_changes_per_hour = self.stats['ac_state_changes'] / runtime_hours
            energy_save_percentage = (self.stats['energy_save_periods'] / runtime_hours * 100) if runtime_hours > 0 else 0
            
            self.logger.info(f"Average AC State Changes per Hour: {avg_state_changes_per_hour:.1f}")
            self.logger.info(f"Energy Conservation Efficiency: {energy_save_percentage:.1f}% of runtime")
        
        self.logger.info("=" * 60)

    def set_thresholds(self, empty_timeout=None, sleep_timeout=None, 
                      sleep_breathing_threshold=None):
        """Set detection thresholds"""
        if empty_timeout is not None:
            self.empty_room_timeout = empty_timeout
            self.logger.info(f"CONFIGURATION UPDATE: Empty room timeout set to {empty_timeout} seconds")
            
        if sleep_timeout is not None:
            self.sleep_detection_timeout = sleep_timeout
            self.logger.info(f"CONFIGURATION UPDATE: Sleep detection timeout set to {sleep_timeout} seconds")
            
        if sleep_breathing_threshold is not None:
            self.sleep_breathing_threshold = sleep_breathing_threshold
            self.logger.info(f"CONFIGURATION UPDATE: Sleep breathing threshold set to {sleep_breathing_threshold} BPM")

    def get_system_status(self):
        """Return current system status for external monitoring"""
        return {
            'room_occupied': self.room_occupied,
            'sleep_mode': self.sleep_mode,
            'ac_state': self.ac_state.copy(),
            'runtime_hours': (time.time() - self.start_time) / 3600,
            'statistics': self.stats.copy()
        }

    def force_ac_state(self, power=None, temperature=None, mode=None, fan_speed=None):
        """Manually override AC state (for testing or emergency control)"""
        old_state = self.ac_state.copy()
        
        if power is not None:
            self.ac_state['power'] = power
        if temperature is not None:
            self.ac_state['temperature'] = temperature
        if mode is not None:
            self.ac_state['mode'] = mode
        if fan_speed is not None:
            self.ac_state['fan_speed'] = fan_speed
            
        if old_state != self.ac_state:
            self.logger.info("MANUAL OVERRIDE: AC state manually adjusted")
            self.log_ac_state_change(old_state, self.ac_state)

    def reset_statistics(self):
        """Reset all statistics counters"""
        self.stats = {
            'total_runtime': 0,
            'motion_detections': 0,
            'sleep_periods': 0,
            'energy_save_periods': 0,
            'ac_state_changes': 0
        }
        self.start_time = time.time()
        self.logger.info("SYSTEM RESET: All statistics counters have been reset")