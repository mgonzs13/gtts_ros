
import threading
from gtts import gTTS
from pygame import mixer

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from gtts_msgs.action import TTS


class GttsNode(Node):
    def __init__(self) -> None:
        super().__init__("gtts_node")

        self._goal_handle = None
        self._goal_lock = threading.Lock()

        self._action_server = ActionServer(
            self,
            TTS,
            "tts",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info("gtts_node started")

    def destroy(self) -> None:
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request) -> int:
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle) -> None:
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info("Aborting previous goal")
                mixer.music.stop()
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal) -> int:
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle) -> TTS.Result:
        self.get_logger().info("Executing goal")

        request: TTS.Goal = goal_handle.request

        gtts_obj = gTTS(
            text=request.text,
            lang=request.config.language
        )

        gtts_file = "/tmp/gtts_tmp_file.mp3"
        gtts_obj.save(gtts_file)

        mixer.init()
        mixer.music.load(gtts_file)
        mixer.music.set_volume(request.config.volume)
        mixer.music.play()

        while mixer.music.get_busy():

            if not goal_handle.is_active:
                return TTS.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                mixer.music.stop()
                return TTS.Result()

        goal_handle.succeed()
        return TTS.Result()


def main(args=None):
    rclpy.init(args=args)
    node = GttsNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
