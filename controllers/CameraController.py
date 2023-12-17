from ControlBot import ControlBot
from CameraNode import CameraNode # source: https://github.com/antmicro/ros2-camera-node
from enums.Buttons import Buttons

class CameraController(ControlBot):

    def __init__(self):
        super().__init__();

        self.camera = self.create_publisher(CameraNode, '/camera', 10);

    def _callback(self, message: CameraNode):
        """
        Callback function for the camera subscriber

        :param message: CameraNode message
        :type message: CameraNode
        """

        self.setLight(message);
        self.setControlGeometry(message);
        self.publisher.publish(self.geometry);
    
    def getView(self, message: CameraNode):
        """
        Gets the camera view based on the button pressed

        :param message: CameraNode message button pressed
        :type message: CameraNode
        """

        if message.buttons[Buttons.CAMERA] == self.pressed:
            self.camera.publish(message);
    
    def recognition(self, message: CameraNode):
        """
        Camera recognize object in view

        :param message: CameraNode
        :type message: CameraNode
        """ 