from BGT60TR13C import BGT60TR13C
from off_ir import AirconController
import time

if __name__ == "__main__":
    print("Starting Smart AC Control System...")
    
    sensor = BGT60TR13C(
        spi_bus=0, spi_device=0,
        cs_pin=8, rst_pin=7, irq_pin=25
    )
    
    try:
        if sensor.begin():
            print("Radar sensor initialized successfully!")
            
            sensor.set_motion_threshold(150)
            
            ac_controller = AirconController(sensor)
            
            print("System ready. Starting control loop...")
            print("Press Ctrl+C to stop")
            time.sleep(1)
            
            ac_controller.control_loop()
            
        else:
            print("Failed to initialize radar sensor")
            print("Please check wiring and connections")
            
    except KeyboardInterrupt:
        print("\nShutting down system...")
        
    except Exception as e:
        print(f"Unexpected error: {e}")
        
    finally:
        print("Cleaning up...")
        sensor.close()
        print("System stopped.")