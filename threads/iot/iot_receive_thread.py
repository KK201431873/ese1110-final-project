from utils.pi_thread import PiThread
from utils.websocket_interface import WebSocketInterface

class IoTReceiveThread(PiThread):
    running: bool = False
    """Whether or not the robot (ControllerThread) is running. Can be toggled through IoT."""

    def _on_created_impl(self) -> None:
        # # Connect to WebSocket
        # WebSocketInterface._ensure_socket(self)
        self.update_robot_running()
    
    def _on_start_impl(self) -> None:
        self.print("Alive!")
    
    def _loop_impl(self) -> None:
        # Poll for incoming commands
        WebSocketInterface.poll(self)

        # Get next command if exists
        cmd = WebSocketInterface.get_command()
        if cmd:
            self.print(f"Received IoT command: {cmd}")
            
            if cmd == "START_ROBOT":
                self.running = True

            elif cmd == "STOP_ROBOT":
                self.running = False
        self.update_robot_running()

    def _on_shutdown_impl(self) -> None:
        WebSocketInterface.close()
    
    def update_robot_running(self) -> None:
        self["running"] = self.running
        WebSocketInterface.send_variable(self, "running", str(self.running))