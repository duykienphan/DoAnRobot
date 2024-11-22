import time
import serial.tools.list_ports
import winsound
import random

class COM_PORT:
    def __init__(self):
        self.processors = ["USB-SERIAL CH340", "Silicon Labs CP210x USB to UART Bridge"]
        self.ports = list(serial.tools.list_ports.comports())
    
    def get_name_driver(self):
        if self.ports:
            for port in self.ports:
                driver = port.description.split('(')[0].strip()
                if driver == self.processors[0]:
                    return "Arduino"
                elif driver == self.processors[1]:
                    return "ESP32"
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
        self.comport = COM_PORT()
        self.port = self.comport.get_name_port()
        self.driver = self.comport.get_name_driver()
        print(self.port + " " + self.driver)
        self.baudrate = 115200
        if self.port != "None":
            try:
                self.serialCom = serial.Serial(self.port, self.baudrate, timeout=1)
                print("Initialize COM port.")
            except:
                print("Failed to initialize COM port.")
                self.serialCom = None

    def send_data(self, data):
        if self.serialCom is not None:
            try:
                self.serialCom.write(data.encode())
                print("Data send: ", data)
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
    esp = ESP()

    while True:
    #    print(esp.receive_data())
        
        pid = str(input("input: "))
        esp.send_data(pid)