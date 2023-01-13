import rclpy
from astra_camera_msgs.srv import GetCameraParams
from rclpy.node import Node


class GetCameraParamsNode(Node):

    def __init__(self):
        super().__init__("GetCameraParams")
        self.cli = self.create_client(
            GetCameraParams, "/camera/get_camera_params")
        self.req = GetCameraParams.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    test_cli = GetCameraParamsNode()
    response : GetCameraParams.Response = test_cli.send_request()
    print(response)
    

if __name__ == '__main__':
    main()
