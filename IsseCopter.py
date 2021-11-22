import signal
from threading import Thread
from time import sleep

from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer


class IsseCopter(AbstractVirtualCapability):

    def __init__(self, server):
        super().__init__(server)

    def execute_command(self, command: dict):
        command["type"] = "respond"
        self.send_message(command)

    def loop(self):
        sleep(5)
        self.send_message({"alive":True})


if __name__ == "__main__":
    # Needed for properly closing when process is being stopped with SIGTERM signal
    def handler(signum, frame):
        print("[Main] Received SIGTERM signal")
        listener.kill()
        quit(1)

    try:
        server = VirtualCapabilityServer()
        listener = IsseCopter(server)
        listener.start()
        signal.signal(signal.SIGTERM, handler)
        listener.join()
        # Needed for properly closing, when program is being stopped wit a Keyboard Interrupt
    except KeyboardInterrupt:
        print("[Main] Received KeyboardInterrupt")
        listener.kill()
