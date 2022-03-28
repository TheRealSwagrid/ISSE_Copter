import signal
import sys
from threading import Thread
from time import sleep

from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer, formatPrint

class IsseCopter(AbstractVirtualCapability):
    def __init__(self, server):
        super().__init__(server)
        self.ISSECopterPosition = [0., 0., 0.]

    def SetISSECopterPosition(self, params: dict) -> dict:
        #TODO Rossi Job.
        p = params["Position3D"]
        formatPrint(self, f"Flying To {p}")
        sleep(5)
        self.ISSECopterPosition = params["Position3D"]
        return self.GetISSECopterPosition(params)

    def GetISSECopterPosition(self, params: dict) -> dict:
        # TODO Rossi Job.
        return {"Position3D": self.ISSECopterPosition}

    def FlyToPosition(self, params: dict) -> dict:
        formatPrint(self, f"Flying to position {params}")
        return self.SetISSECopterPosition(params)

    def loop(self):
        pass


if __name__ == "__main__":
    # Needed for properly closing when process is being stopped with SIGTERM signal
    def handler(signum, frame):
        print("[Main] Received SIGTERM signal")
        listener.kill()
        quit(1)

    try:
        port = None
        if len(sys.argv[1:]) > 0:
            port = int(sys.argv[1])
        server = VirtualCapabilityServer(port)
        listener = IsseCopter(server)
        listener.start()
        signal.signal(signal.SIGTERM, handler)
        listener.join()
        # Needed for properly closing, when program is being stopped wit a Keyboard Interrupt
    except KeyboardInterrupt:
        print("[Main] Received KeyboardInterrupt")
        server.kill()
        listener.kill()
