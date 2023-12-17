import rclpy
from rclpy.node import Node
from geometry_msg.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from enums.Buttons import Buttons
from controllers.CameraController import CameraController

class ControlBot(Node):

    # Button parameters
    pressed = 1;
    unloosened = 0;

    # Light parameters
    ledBar = String();

    # Speed parameters
    turtle = 0.5;
    rabbit = 1;

    # Robot parameters
    defaultMaxSpeed = 0.22;
    defaultAngle = 2.84;

    # Camera infrared parameters
    depth_width = 848;
    depth_height = 480;
    depth_fps = 30.0;

    def __init__(self):
        super().__init__('move_controller_node')
        
        self.turbo = 0.5;
        self.enabled = False;
        self.oldMessage = [];
    
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10);
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10);
        self.lights = self.create_publisher(String, '/micro_ros_arduino_subscriber', 10);
        self.camera = CameraController();
    
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

        self.camera.getView(message);

    def setLight(self, message: Joy):
        """
        Sets the light based on the button pressed

        :param message: Joy message button pressed
        :type message: Joy
        """
                
        for i in range(0, len(message.buttons)):
            self.setTurbo(message.buttons[0]);

            if message.buttons[i] == self.pressed:
                self.getLightReference(i);
    
    def setTurbo(self, message):
        """
        Sets the turbo robot speed based on the button pressed

        :param message: Joy message button pressed
        :type message: Joy
        """

        if message == self.pressed:
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
            case Buttons.TURBO.value:
                self.setLightBar(Buttons.TURBO.name.capitalize());
            case Buttons.BOUNCE.value:
                self.setLightBar(Buttons.BOUNCE.name.capitalize());
            case Buttons.EVEN_ODD.value:
                self.setLightBar(Buttons.EVEN_ODD.name.capitalize());
            case Buttons.CENTER_OUT.value:
                self.setLightBar(Buttons.CENTER_OUT.name.capitalize());
            case Buttons.BLINK_LEFT.value:
                self.setLightBar(Buttons.BLINK_LEFT.name.capitalize());
            case Buttons.BLINK_RIGHT.value:
                self.setLightBar(Buttons.BLINK_RIGHT.name.capitalize());
            case Buttons.DISCO.value:
                self.setLightBar(Buttons.DISCO.name.capitalize());

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
    



    
