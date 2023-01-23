#!/usr/bin/env python3'
# coding = utf-8
########################################################################################
##
## Maintainer: stefan.kull@gmail.com
## 
## Input: INA219 Current/Voltage-sensor (a.k.a "Battery State")
## Output: ROS2 node that publish a BatteryState.msg topic 
##
## Prerequisite:
## Software
## $ 
## $ sudo pip3 install adafruit-circuitpython-ina219
##
## Hardware: Power circuit via INA219 break-out-board (Power at VIn+, Drain/Source at VIn- )
## Host: Raspberry Pi 4(Ubuntu) via I2C
##
## Launch sequence:
## 1) $ ros2 run pet_ros2_currentsensor_ina219_pkg pet_current_sensor_ina219_node.py 
## 2) $ ros2 topic echo /xyz
##    $ ros2 topic echo /zyx
##

# Import the ROS2-stuff
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg  import BatteryState

# Import the Ubuntu/Linux-hardware stuff 
import time
import board
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219

# Import the common Ubuntu/Linux stuff 
import sys
from time import sleep
from math import modf

#Set Button pin(GPIO no.) and ROS2-topic-name
BATTERY_STATE_TOPIC = 'battery_state'


class BatteryStatePublisher(Node):
    '''
    ROS2 current & voltage sensor publisher node
    Create a BatteryStatePublisher class, which is a subclass of the Node class.
    The class publishes the battery state of an object at a specific time interval.
    '''
    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__("pet_current_sensor_node")

        i2c_bus = board.I2C()
        self.ina219 = INA219(i2c_bus)

        # optional : change configuration to use 32 samples averaging for both bus voltage and shunt voltage
        self.ina219.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.ina219.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S

        # optional : change voltage range to 16V
        self.ina219.bus_voltage_range = BusVoltageRange.RANGE_16V

        # display some of the advanced field (just to test)
        self.get_logger().info("INA219 Current/Voltage sensor. Config register:")
        self.get_logger().info(" - bus_voltage_range:    0x%1X" % self.ina219.bus_voltage_range)
        self.get_logger().info(" - gain:                 0x%1X" % self.ina219.gain)
        self.get_logger().info(" - bus_adc_resolution:   0x%1X" % self.ina219.bus_adc_resolution)
        self.get_logger().info(" - shunt_adc_resolution: 0x%1X" % self.ina219.shunt_adc_resolution)
        self.get_logger().info(" - mode:                 0x%1X" % self.ina219.mode)
        self.get_logger().info("")

        # Create Message  <https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/BatteryState.msg>
        current_time = modf(time.time())
        self.msg_battery = BatteryState()
        self.msg_battery.header.stamp.sec  = int(current_time[1])
        self.msg_battery.header.stamp.nanosec = int(current_time[0] * 1000000000) & 0xffffffff       
        self.msg_battery.header.frame_id = "18650 3S x1P main battery"  

        self.msg_battery.voltage     = float('NaN')     # Voltage in Volts (Mandatory)
        self.msg_battery.current     = float('NaN')     # Negative when discharging (A)  (If unmeasured NaN)        
        self.msg_battery.temperature = float('NaN')     # Temperature in Degrees Celsius (If unmeasured NaN)
        self.msg_battery.charge      = float('NaN')     # Current charge in Ah  (If unmeasured NaN)
        self.msg_battery.capacity    = float('NaN')     # Capacity in Ah (last full capacity)  (If unmeasured NaN)
        self.msg_battery.design_capacity = float('NaN') # Capacity in Ah (design capacity)  (If unmeasured NaN)
        self.msg_battery.percentage  = float('NaN')     # Charge percentage on 0 to 1 range  (If unmeasured NaN

        self.msg_battery.power_supply_status     = 0    # The charging status as reported. [uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0]
        self.msg_battery.power_supply_health     = 0    # The battery health metric. [uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0]
        self.msg_battery.power_supply_technology = 2    # The battery chemistry. [uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2]
        self.msg_battery.present = True                 # True if the battery is present
        
        self.msg_battery.location = 'Think "Inside the box"' # The location into which the battery is inserted. (slot number or plug)
        self.msg_battery.serial_number = '0000000'      # The best approximation of the battery serial number

        # Create publisher(s)  
        self.publisher_battery_state = self.create_publisher(BatteryState, '/battery_status', 10)
        
        # Setup time interval in seconds... for the callback
        timer_period = 5.0
        self.timer = self.create_timer(timer_period, self.get_battery_state_callback)

        # print("----------------------------------------")
        # print(self.msg_battery)
        # print("----------------------------------------")


        
    def get_battery_state_callback(self):
        """
        Callback function.
        This function gets called at the specific time interval.
        """
        # Update the message header
        current_time = modf(time.time())
        self.msg_battery.header.stamp.sec  = int(current_time[1])
        self.msg_battery.header.stamp.nanosec = int(current_time[0] * 1000000000) & 0xffffffff

        self.msg_battery.voltage = self.ina219.bus_voltage       # voltage on V- (load side)
        self.msg_battery.current = self.ina219.current /1000.0   # current in mA->A
        # print(self.msg_battery.voltage)
        
        # Publish BatteryState message 
        self.publisher_battery_state.publish(self.msg_battery) 

             
def main(args=None):
    rclpy.init(args=args)

     # Create the node
    battery_state_pub = BatteryStatePublisher()

    try:
        # Spin the node so the callback function is called.
        # Publish any pending messages to the topics.
        rclpy.spin(battery_state_pub) 

    except KeyboardInterrupt:
        print("**** * 💀 Ctrl-C detected...")
    
    finally:
        print("**** 🪦 battery_state_pub ending... ")
        print( str(sys.exc_info()[1]) )  # Need ´import sys´

        # Time to clean up stuff... - Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        battery_state_pub.destroy_node()
       
        # Time to clean up stuff... Shutdown the ROS client library for Python
        rclpy.shutdown()

if __name__ == "__main__":
    main()
