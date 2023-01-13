import rclpy
from astra_camera_msgs.srv import GetString
from rclpy.node import Node
import json
import sys


class GetSupportedVideoModes(Node):

    def __init__(self):
        super().__init__("GetSupportedVideoModes")
        stream = sys.argv[1]
        self.cli = self.create_client(
            GetString, "/camera/get_" + stream + "_supported_video_modes")
        self.req = GetString.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    test_cli = GetSupportedVideoModes()
    response = test_cli.send_request()
    data = json.loads(response.data)
    for item in data:
        print("%s" % item)
    test_cli.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
