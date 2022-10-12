#!/usr/bin/env python
import signal

from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer, formatPrint


class IsseCopter(AbstractVirtualCapability):
    def __init__(self, server):
        super().__init__(server)
        self.ISSECopterPosition = [0., 0., 0.]
        self.functionality = {"arm": None, "disarm": None, "SetISSECopterPosition": None, "GetISSECopterPosition": None, "GetArmingStatus" : None}

    def SetArmingStatus(self, params: dict):
        p = params["SimpleBooleanParameter"]
        if p and self.functionality["arm"] is not None:
            self.functionality["arm"]()
        elif not p and self.functionality["disarm"] is not None:
            self.functionality["disarm"]()
        return params

    def GetArmingStatus(self, params: dict):
        if self.functionality["GetArmingStatus"] is not None:
            return self.functionality["GetArmingStatus"]()
        return False

    def SetISSECopterPosition(self, params: dict) -> dict:
        p = params["Position3D"]
        if self.functionality["SetISSECopterPosition"] is not None:
            self.functionality["SetISSECopterPosition"](p)
        self.ISSECopterPosition = params["Position3D"]
        return self.GetISSECopterPosition(params)

    def GetISSECopterPosition(self, params: dict) -> dict:
        if self.functionality["GetISSECopterPosition"] is not None:
            pos = self.functionality["GetISSECopterPosition"]()
            self.ISSECopterPosition = pos
        return {"Position3D": self.ISSECopterPosition}

    def FlyToPosition(self, params: dict) -> dict:
        formatPrint(self, f"Flying to position {params}")
        return self.SetISSECopterPosition(params)

    def loop(self):
        pass


if __name__ == '__main__':
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
        server.kill()
        listener.kill()

