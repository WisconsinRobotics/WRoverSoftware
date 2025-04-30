import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String

import numpy as np
import serial
import traceback 

class SensorsRawNode(Node):
    def __init__(self):
        super().__init__('sensors_raw')
        self.pub_fluoro = self.create_publisher(Int16MultiArray, '/sci_fluoro_raw', 1)
        self.pub_soil = self.create_publisher(Int16MultiArray, '/sci_soil_raw', 1)
        self.pub_arduino = self.create_publisher(String, '/sci_arduino_messages', 1)
        
        self.serial_data_port = self.declare_parameter('/serial_data_port', '/dev/ttyUSB0').value
        self.serial_baud_rate = self.declare_parameter('/serial_baud_rate', 115200).value

        # initiate the serial connection with the arduino
        try:
            self.ser = serial.Serial(port=self.serial_data_port,baudrate=self.serial_baud_rate,timeout=0.5)
        except:
            self.get_logger().warn("USB connection to Arduino unsuccessful :( maybe try turning it off and on again?")
            rclpy.shutdown()

        # initialize empty fluorometer message
        self.fluoro_vals = Int16MultiArray()
        self.fluoro_vals.layout.label[0] = "colors"
        self.fluoro_vals.layout.size[0] = 9
        self.fluoro_vals.layout.label[1] = "wavelength, reading"
        self.fluoro_vals.layout.size[1] = 2
        self.fluoro_vals.data = np.array([[415,445,480,515,555,590,630,680,0],
                                          [0,  0,  0,  0,  0,  0,  0,  0,  0]])

        # initialize empty soil sensor message
        self.soil_vals = Int16MultiArray()
        self.soil_vals.layout.label[0] = "measurements"
        self.soil_vals.layout.size[0] = 2
        self.soil_vals.data = [0,0]

        
    def operate(self):
        # Flush any backlog and read a line to make sure the port input is current. 
        self.ser.flushInput()
        self.ser.readline()
        
        while True:
            try: 
                # Here we read the serial port for a string that looks like "color:123,color:123", "temp:123,moisture:123", or "debug message"
                line = self.ser.readline().decode().strip() #blocking function, will wait until read entire line
                vals = line.split(",") # split by commas into individual data points or clauses
                
                if len(vals[0].split(":")) == 1: # evaluates to true if there are no colons in the substring
                    arduino_message = String()
                    arduino_message.data = line
                    self.pub_arduino.publish(arduino_message)
                    self.ser.write("\n".encode()) # send the arduino a newline character so it moves on (hopefully)
                elif len(vals) == 9: # evaluates to true if we're seeing a fluorometer reading
                    self.fluoro_vals.data[1,:] = [int(val.split(":")[1]) for val in vals]
                    self.pub_fluoro.publish(self.fluoro_vals)
                else: # evaluates to true if we're seeing a soil temp/moisture reading
                    self.soil_vals.data = [int(val.split(":")[1]) for val in vals]
                    self.pub_soil.publish(self.soil_vals)
            
            except KeyboardInterrupt:
                break
            except Exception: # the only way we ever get to here is if reading totally fails, which may never happen
                print('Bad line received on Arduino port - ignoring and continuing.')


def main(args=None):
    try: 
        rclpy.init(args=args)
        print('sensors_raw beginning')
        sensors_raw_instance = SensorsRawNode()
        sensors_raw_instance.operate()
    except KeyboardInterrupt:
        print('Keyboard Interrupt')
    except: 
        traceback.print_exc()
        
    sensors_raw_instance.destroy_node()
    rclpy.shutdown()
    
    print('sensors_raw exiting')


if __name__ == '__main__':
    main()

