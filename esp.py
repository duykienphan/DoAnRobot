import time
import serial.tools.list_ports
import winsound

class COM_PORT:
    def __init__(self):
        self.processor_port = "None"
        self.processors = ["USB-SERIAL CH340", "Silicon Labs CP210x USB to UART Bridge", ]
        self.ports = list(serial.tools.list_ports.comports())
        self.count = 0
    
    def get_name_driver(self):
        if self.ports:
            for port in self.ports:
                driver = port.description.split('(')[0].strip()
                if driver == self.processor_port[1]:
                    return "Arduino"
                elif driver == self.processor_port[2]:
                    return "ESP32D"
                else:
                    return driver
        else:
            return "None"

    def get_name_port(self):
        if self.ports:
            for port in self.ports:
                return port.device
        else:
            return "None"

    def notification_sound(self):
        time.sleep(1)
        winsound.Beep(1000, 200)
        time.sleep(0.1)  

class ESP():
    def __init__(self):
        self.port = COM_PORT().get_name_port()
        self.driver = COM_PORT().get_name_driver()
        print(self.port + " " + self.driver)
        self.baudrate = 115200
        if self.port != "None":
            try:
                self.serialCom = serial.Serial(self.port, self.baudrate, timeout=1)
            except:
                print("Failed to initialize COM port.")
                self.serialCom = None

    def send_data(self, data):
        if self.serialCom is not None:
            try:
                self.serialCom.write(data.encode())
                time.sleep(1)
            except:
                print("Failed to send data!")

    def receive_data(self):
        if self.serialCom is not None:
            try:
                raw_data = self.serialCom.readline().strip()
                decoded_data = raw_data.decode("utf-8", errors="replace")
                return decoded_data
            except:
                print("Failed to receive data!")     

if __name__ == "__main__":
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        print(port)
