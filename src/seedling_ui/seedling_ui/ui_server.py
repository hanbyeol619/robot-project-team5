"""관제 UI 웹페이지를 띄우는 작은 웹서버.

  ros2 run seedling_ui ui_server            → 브라우저에서 http://localhost:8000
  ros2 run seedling_ui ui_server --ros-args -p port:=8080

페이지는 rosbridge(ws://localhost:9090)로 /ui/command, /ui/status 를 주고받아요.
"""
import functools
import os
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class UiServer(Node):
    def __init__(self):
        super().__init__('ui_server')
        port = int(self.declare_parameter('port', 8000).value)
        web = os.path.join(get_package_share_directory('seedling_ui'), 'web')
        handler = functools.partial(SimpleHTTPRequestHandler, directory=web)
        handler.log_message = lambda *a: None
        self.httpd = ThreadingHTTPServer(('0.0.0.0', port), handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f'관제 UI: http://localhost:{port}  (rosbridge ws://localhost:9090)')
        if self.declare_parameter('open_browser', True).value:
            import webbrowser
            threading.Timer(3.0, lambda: webbrowser.open(f'http://localhost:{port}')).start()


def main(args=None):
    rclpy.init(args=args)
    node = UiServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
