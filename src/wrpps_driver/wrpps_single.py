import serial
import threading
import time
import re


class WrPPSSingleDriver:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=1):
        """
        Initialize the serial reader with the given parameters.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.thread = None
        self.running = False

    def open(self):
        """
        Open the serial port.
        """
        try:
            self.serial = serial.Serial(port=self.port,
                                        baudrate=self.baudrate,
                                        timeout=self.timeout)
            time.sleep(2)  # Wait for Arduino to reset
            print(f'Opened serial port: {self.port}')
            self.print_port_info()
        except serial.SerialException as e:
            print(f'Failed to open serial port {self.port}: {e}')
            raise

    def close(self):
        """
        Stop the thread and close the serial port.
        """
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join()
        if self.serial and self.serial.is_open:
            self.serial.close()
            print(f'Closed serial port: {self.port}')

    def print_port_info(self):
        """
        Print the serial port informations.
        """
        print(f'  port: {self.serial.name}')
        for k, v in self.serial.get_settings().items():
            print(f'  {k}: {v}')

    def start_reading(self):
        """
        Start a background thread to read serial data.
        """
        if not self.serial or not self.serial.is_open:
            self.open()
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        """
        Internal loop to read data from the serial port in a thread.
        """
        while self.running:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    data = self._parse_line(line)
                    data_time = time.time()
                    if data:
                        self._handle_data(data, data_time)
                    else:
                        print(f'Ignored line: {line}')
            except Exception as e:
                print(f'Error while reading: {e}')
                self.running = False

    def _parse_line(self, line):
        """
        Parse a line like 'intensity: 328, tof: 37' into a dictionary.
        """
        match = re.search(r'intensity:\s*(\d+),\s*tof:\s*(\d+)', line)
        if match:
            return {
                'intensity': int(match.group(1)),
                'tof': int(match.group(2))
            }
        return None

    def _handle_data(self, data, data_time):
        """
        Handle the parsed data (override this method if needed).
        """
        print(
            f'Received: time={data_time}, intensity={data["intensity"]}, tof={data["tof"]}')


if __name__ == '__main__':
    reader = WrPPSSingleDriver(port='/dev/ttyACM0', baudrate=115200)

    try:
        reader.start_reading()
        print('Press Ctrl+C to stop.')
        while True:
            time.sleep(0.1)  # Keep main thread alive
    except KeyboardInterrupt:
        print('\nStopping...')
    finally:
        reader.close()
