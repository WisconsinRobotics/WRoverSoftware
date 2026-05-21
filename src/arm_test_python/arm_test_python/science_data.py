import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String

import numpy as np
import serial
import traceback 
import warnings

FLUORO_WAVELENGTHS = [515, 590]

class SensorsRawNode(Node):
    def __init__(self):
        super().__init__('sensors_raw')
        self.pub_fluoro = self.create_publisher(Int16MultiArray, '/sci_fluoro_raw', 1)
        self.pub_soil = self.create_publisher(Float64MultiArray, '/sci_soil_raw', 1)
        self.pub_ambient = self.create_publisher(Float64MultiArray, '/sci_ambient_raw', 1)
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
        # Index 0 = 515nm, Index 1 = 590nm
        self.fluoro_vals.data = [ 0 for _ in FLUORO_WAVELENGTHS ]

        # initialize empty soil sensor message
        self.soil_vals = Floatt64MultiArray()
        # Index 0 = Soil Temp, Index 1 = Soil Moisture
        self.soil_vals.data = [0., 0.]

        # intiialize empty ambient conditions message
        self.amb_vals = Float64MultiArray()
        # Index 0 = Ambient Temp, Index 1 = Ambient Humidity, Index 2 = Ambient Methane
        self.amb_vals.data = [0., 0., 0.]

        
    def operate(self):
        # Flush any backlog and read a line to make sure the port input is current. 
        self.ser.flushInput()
        self.ser.readline()
        
        while True:
            try: 
                # Here we read the serial port for a string that looks like "color:123,color:123", "temp:123,moisture:123", or "debug message"
                line = self.ser.readline().decode().strip() #blocking function, will wait until read entire line

                if line.startswith("@"):
                    if line.startswith("@data"):
                        typ, dat = line.split(" ")
                        typ, datc = typ.strip(), dat.strip()
                        match typ:
                            case "@data.fluoro":
                                self.fluoro_vals.data = [ int(i) for i in dat.split(",") ]
                                self.pub_fluoro.publish(self.fluoro_vals)
                            case "@data.soil":
                                self.soil_vals.data = [ float(i) for i in dat.split(",") ]
                                self.pub_soil.publish(self.soil_vals)
                            case "@data.ambient":
                                self.amb_vals.data = [ float(i) for i in dat.split(",") ]
                                self.pub_ambient.publish(self.amb_vals)
                            case _:
                                raise RuntimeError(f"Unexpected @data message ({typ}): \"{line}\"")
                    elif line.startswith("@warn"):
                        amsg = String()
                        text = line[len("@warn "):]
                        warnings.warn(f"Warning! (from Science System): {text}", RuntimeWarning)
                        self.end_arduino_message(f"Warning: {text}")
                    elif line.startswith("@fail"):
                        amsg = String()
                        text = line[len("@fail "):]
                        amsg.data = 
                        warnings.warn(f"Error! (from Science System): {text}", RuntimeWarning)
                        self.end_arduino_message(f"Error!! Error: {text}")
                else:
                    amsg = String()
                    amsg.data = 
                    self.end_arduino_message(f"Message: {line}")
            except KeyboardInterrupt:
                break
            except Exception as e: # the only way we ever get to here is if reading totally fails, which may never happen
                print('Bad line received on Arduino port - ignoring and continuing.')
                warnings.warn(f"Failed to parse Science System response (\"{line}\"): {e.__str__()}", RuntimeWarning)
                self.end_arduino_message("[[ failed to parse: \"{line}\" ]]")

    def end_arduino_message(self, amsg_text: str):
        try:
            amsg = String()
            amsg.data = amsg_text
            self.pub_arduino.publish(amsg)
            self.ser.write("\n".encode())
        except Exception as e:
            warnings.warn(f"Error ending arduino message! {e.__str__()}", RuntimeWarning)
            raise e

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

