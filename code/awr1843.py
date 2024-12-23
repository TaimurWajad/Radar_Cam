import serial
import time
import RPi.GPIO as GPIO

# Pin definitions
RADAR_PWR_EN = 16  # Enable power to radar

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(RADAR_PWR_EN, GPIO.OUT)

# Enable power to radar
GPIO.output(RADAR_PWR_EN, GPIO.HIGH)  # ENABLE POWER TO ULTRA SENSORS

# UART configuration
USER_UART_PORT = '/dev/ttyAMA4'  # UART port for AWR1843BOOST
USER_UART_BAUDRATE = 115200      # Default baud rate for User UART

def send_config(uart, config_file):
    """Send configuration commands to the radar via UART."""
    with open(config_file, 'r') as file:
        for line in file:
            if line.strip():
                uart.write(line.encode())
                print(f"Sent: {line.strip()}")
                time.sleep(0.2)  # Small delay between commands

def read_data(uart):
    """Read radar data from UART."""
    try:
        while True:
            if uart.in_waiting:
                data = uart.read(uart.in_waiting)
                print(f"Data Received: {data.hex()}")  # Print raw data in hex
    except KeyboardInterrupt:
        print("Data reading stopped.")

# Main workflow
if __name__ == "__main__":
    try:
        # Open User UART
        user_uart = serial.Serial(USER_UART_PORT, baudrate=USER_UART_BAUDRATE, timeout=1)
        print(f"Connected to {USER_UART_PORT}")

        # Send radar configuration
        print("Sending configuration...")
        send_config(user_uart, "radar_config.cfg")
        time.sleep(1)  # Wait for radar to initialize

        # Read radar data
        print("Reading radar data...")
        read_data(user_uart)

    finally:
        # Disable power to radar and cleanup GPIO
        GPIO.output(RADAR_PWR_EN, GPIO.LOW)  # DISABLE POWER TO ULTRA SENSORS
        GPIO.cleanup()
        print("Radar powered OFF and GPIO cleaned up.")

        # Close UART
        user_uart.close()
