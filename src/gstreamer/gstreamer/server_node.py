import rclpy
from rclpy.node import Node
from rovr_interfaces.srv import SetClientIp, SetActiveCamera, SetEncoding
from .server_gstreamer import GstreamerServer


class ServerNode(Node):
    def __init__(self):
        super().__init__("server")
        # Change to dictionary to store multiple servers by port
        self.g_servers = {}  # key: port, value: GstreamerServer instance

        self.serv_set_client_ip = self.create_service(
            SetClientIp, "/set_client_ip", self.set_client_ip_callback
        )
        self.serv_set_active_camera = self.create_service(
            SetActiveCamera, "/set_active_camera", self.set_active_camera
        )
        self.serv_set_encoding = self.create_service(
            SetEncoding, "/set_encoding", self.set_encoding
        )

    def set_client_ip_callback(self, request: SetClientIp, response):
        print("received ip request")
        # Assuming request now includes a port field
        port = request.port if hasattr(request, "port") else 5000
        response.success = self.start_or_restart_server(port, ip_srv=request)
        return response

    def set_active_camera(self, request: SetActiveCamera, response):
        print("received camera request")
        port = request.port if hasattr(request, "port") else 5000
        response.success = self.start_or_restart_server(port, camera_srv=request)
        return response

    def set_encoding(self, request: SetEncoding, response):
        print("received encoding request")
        port = request.port if hasattr(request, "port") else 5000
        response.success = self.start_or_restart_server(port, encod_srv=request)
        return response

    def start_or_restart_server(self, port, ip_srv=None, camera_srv=None, encod_srv=None):
        # Get or create server config for this port
        if port not in self.g_servers:
            self.g_servers[port] = {
                "server": None,
                "ip_srv": None,
                "camera_srv": None,
                "encod_srv": None,
            }

        # Update the config with new values
        if ip_srv is not None:
            self.g_servers[port]["ip_srv"] = ip_srv
        if camera_srv is not None:
            self.g_servers[port]["camera_srv"] = camera_srv
        if encod_srv is not None:
            self.g_servers[port]["encod_srv"] = encod_srv

        config = self.g_servers[port]

        # Stop existing server on this port if it exists
        if config["server"] is not None:
            print(f"Stopping server on port {port}")
            config["server"].stop()

        # Validate all required configs are set
        if config["ip_srv"] is None:
            print(f"No client ip set for port {port}")
            return -1
        if config["encod_srv"] is None:
            print(f"No encoding set for port {port}")
            return -2
        if config["camera_srv"] is None:
            print(f"No camera set for port {port}")
            return -3

        # Create and start new server
        config["server"] = GstreamerServer(
            config["ip_srv"],
            config["camera_srv"],
            config["encod_srv"],
            port,  # Pass port to GstreamerServer
        )
        config["server"].run()
        print(f"Server started/restarted on port {port}")
        return 0

    def stop_all_servers(self):
        """Helper method to stop all running servers"""
        for port, config in self.g_servers.items():
            if config["server"] is not None:
                print(f"Stopping server on port {port}")
                config["server"].stop()


def main(args=None):
    rclpy.init(args=args)
    server = ServerNode()
    server.get_logger().info("Starting the Gstreamer server node!")
    try:
        rclpy.spin(server)
    finally:
        server.stop_all_servers()
        server.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()
