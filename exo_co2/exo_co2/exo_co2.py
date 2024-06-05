import sys
from atlas_i2c import atlas_i2c

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from exo_co2_messages.msg import GasConcentration   # type: ignore

class AtlasCo2Node(Node):
    """
    ROS2 publisher node for Atlas Scientific EXO-CO2
    
    The class publishes the CO2 concentration read by the sensor.
    In the future, this could be expanded to be a mode generic driver for
    Atlas Scientific instruments.
    """
    def __init__(self) -> None:
        super().__init__("exoco2")    # type: ignore
        self.poll_interval = 1.0 # seconds

        # WARNING: Don't attempt to refactor this initialization.
        # I know that it is a bit odd and looks inefficient, but 
        # this is a work-around for problem with the i2c sensor address not
        # being expected by the atlas_i2c library (and specifying an address 
        # during init is not supported by this upstream library)
        self.dev = atlas_i2c.AtlasI2C()
        self.dev.bus = 7
        self.dev.device_file.close()
        self.dev.open_file()
        self.dev.set_i2c_address(addr=105)

        # Initialize the messages and create publisher
        self.msg_co2 = GasConcentration()
        self.msg_co2.header.stamp = self.get_clock().now().to_msg()
        self.msg_co2.ppm = int(-1)
        self.publisher_exo = self.create_publisher(msg_type=GasConcentration, 
            topic='/exoco2/ppm', qos_profile=10)
        
        self.timer = self.create_timer(self.poll_interval, self.exo_callback)

    def exo_callback(self) -> None:
        """
        atlas_i2c timer callback

        Reads the sensor values and publishes the messages at 
        the node polling interval
        """
        self.msg_co2.header.stamp = self.get_clock().now().to_msg()
        result = self.dev.query(command="R", processing_delay=1000)
        self.msg_co2.ppm = int(result.data)

        self.publisher_exo.publish(msg=self.msg_co2)

def main(args=None) -> None:
    rclpy.init(args=args)
    exo_pub: AtlasCo2Node = AtlasCo2Node()

    try:
        rclpy.spin(exo_pub) 
    except KeyboardInterrupt:
        print("**** 💀 Ctrl-C detected... killing process ****")
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        print("**** 🪦 process dead ****")
        # Destroy node explicitly, dont wait for GC
        exo_pub.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()