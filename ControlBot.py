import rclpy
from rclpy.node import Node
from geometry_msg.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from enums.Lights import Lights

class ControlBot(Node):

    # Light parameters
    lightOn = 1;
    lightOff = 0;
    ledBar = String();

    # Speed parameters
    turtle = 0.5;
    rabbit = 1;

    # Robot parameters
    defaultMaxSpeed = 0.22;
    defaultAngle = 2.84;

    def __init__(self):
        super().__init__('move_controller_node')
        
        self.turbo = 0.5;
        self.enabled = False;
        self.oldMessage =[];
    
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10);
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10);
        self.lights = self.create_publisher(String, '/micro_ros_arduino_subscriber', 10);
    
        self.geometry = Twist();
        self.get_logger().info("Move Controller Node has been started");

    def joy_callback(self, message: Joy):
        """
        Callback function for the joystick subscriber

        :param message: Joy message
        :type message: Joy
        """

        self.setLight(message);
        self.setControlGeometry(message);
        self.publisher.publish(self.geometry);

    def setLight(self, message: Joy):
        """
        Sets the light based on the button pressed

        :param message: Joy message button pressed
        :type message: Joy
        """
                
        for i in range(0, len(message.buttons)):
            self.setTurbo(message.buttons[0]);

            if message.buttons[i] == self.lightOn:
                self.getLightReference(i);
    
    def setTurbo(self, message):
        """
        Sets the turbo robot speed based on the button pressed

        :param message: Joy message button pressed
        :type message: Joy
        """

        if message == self.lightOn:
            self.turbo = self.rabbit;
        else:
            self.turbo = self.turtle;

    def getLightReference(self, message: Joy):
        """
        Gets the light reference based on the button pressed

        :param message: Joy message button pressed
        :type message: Joy
        """

        match message: 
            case Lights.TURBO.value:
                self.setLightBar(Lights.TURBO.name.capitalize());
            case Lights.BOUNCE.value:
                self.setLightBar(Lights.BOUNCE.name.capitalize());
            case Lights.EVEN_ODD.value:
                self.setLightBar(Lights.EVEN_ODD.name.capitalize());
            case Lights.CENTER_OUT.value:
                self.setLightBar(Lights.CENTER_OUT.name.capitalize());
            case Lights.BLINK_LEFT.value:
                self.setLightBar(Lights.BLINK_LEFT.name.capitalize());
            case Lights.BLINK_RIGHT.value:
                self.setLightBar(Lights.BLINK_RIGHT.name.capitalize());
            case Lights.DISCO.value:
                self.setLightBar(Lights.DISCO.name.capitalize());

    def setLightBar(self, message: Joy):
        """
        Sets the light bar based on the button pressed

        :param message: Joy message button pressed
        :type message: Joy
        """

        self.ledBar.data = message;

        if self.ledBar.data.length() > 0:
            self.lights.publish(self.ledBar);


    def setControlGeometry(self, message: Joy, speed: float = defaultMaxSpeed, angle: float = defaultAngle):
        """
        Sets the control geometry based on the joystick axes

        :param message: Joy message button pressed
        :speed speed: Speed of the robot
        :angle angle: Angle of the robot
        :type message: Joy
        :type speed: float
        :type angle: float
        """

        self.geometry.linear.x = message.axes[1] * speed * self.turbo;
        self.geometry.angular.z = message.axes[0] * angle * self.turbo;
        
def main(arguments = None):
    rclpy.init(args = arguments);
    node = ControlBot();
    rclpy.spin(node);
    node.shutdown();

if __name__ == '__main__':
    main();
    



    
