import json
import os
import socket
import sys
from abc import abstractmethod
from subprocess import Popen
from threading import Thread, Timer
from time import sleep, time


def formatPrint(c, string: str) -> None:
    """Prints the given string in a formatted way: [Classname] given string

    Parameters:
        c       (class)  : The class which should be written inside the braces
        string  (str)    : The String to be printed
    """
    sys.stderr.write(f"DOCKER\t\t[{type(c).__name__}] {string}\n")


class NoConnectionException(Exception):
    pass


class CommandNotFoundException(Exception):
    pass


class WrongNumberOfArgumentsException(Exception):
    pass


class VirtualCapabilityServer(Thread):
    '''Server meant to be run inside of a docker container as a Thread.

    '''

    def __init__(self, connectionPort: int = None):
        super().__init__()
        if connectionPort != None:
            self.connectionPort = connectionPort
        else:
            self.connectionPort = 9999
        self.running = False
        self.connected = False
        self.virtualDevice = AbstractVirtualCapability
        self.start()

    def run(self) -> None:
        #formatPrint(self, "started")
        self.running = True
        #formatPrint(self, f"Connecting on 0.0.0.0, {self.connectionPort}")
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.bind(("localhost", self.connectionPort))
            self.socket.listen(1)
            self.sock, self.adr = self.socket.accept()
            self.connected = True
            #formatPrint(self, f"connected on {str(self.sock)}")
            while self.running:
                self.loop()
        except Exception as e:
            formatPrint(self, f"Caught an Exception {e.args}")

    def loop(self) -> None:
        if self.running and self.connected:
            try:
                data = self.sock.recv(512)
                if data != b'':
                    try:
                        self.message_received(data.decode())
                    except Exception as e:
                        formatPrint(self, f"Some problem occured! {repr(e)}")
                        self.send_message(f"ERROR: {repr(e)}")
            except Exception as e:
                formatPrint(self, f"Some problem occured! {repr(e)}")
                raise e
        else:
            formatPrint(self.virtualDevice, "no longer connected or running!")

    def message_received(self, msg: str):
        #formatPrint(self.virtualDevice, f"Handling message: {msg}")
        if msg == "kill":
            # Kill the container
            self.kill()
            self.virtualDevice.kill()
        else:
            command = json.loads(msg)
            if command["type"] == "trigger":
                self.virtualDevice.command_list += [dict(command)]
            elif command["type"] == "response":
                if command["src"] in self.virtualDevice.sub_cap_server_callback.keys():
                    cap = command["capability"]
                    par = command["parameters"]
                    formatPrint(self.virtualDevice, f"Got Response from Sub-Capability {cap}:\t{par}")
                    self.virtualDevice.sub_cap_server_callback[command["src"]] = command["parameters"]

    def send_message(self, msg: str):
        if self.connected:
            #formatPrint(self.virtualDevice, f"Sending Msg {msg}")
            self.sock.send(msg.encode("UTF-8"))
        else:
            self.kill()

    def addVirtualCapability(self, vc):
        self.virtualDevice = vc

    def kill(self):
        formatPrint(self, "SHUTTING DOWN")
        try:
            self.running = False
            self.socket.close()
            if self.connected:
                self.socket.shutdown(socket.SHUT_RDWR)
                self.connected = False
        except OSError as e:
            formatPrint(self, f"Something went down while shutting down: {repr(e)}")


class RepeatedTimer(object):
    def __init__(self, interval, function, args, kwargs):
        self._timer     = None
        self.interval   = interval
        self.callback   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        #formatPrint(self, *self.args)
        self.callback(self.args)#, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False


class AbstractVirtualCapability(Thread):
    def __init__(self, server: VirtualCapabilityServer):
        Thread.__init__(self)
        self.uri = None
        self.dynamix = {}
        self.dev_name = None
        self.server = server
        self.server.addVirtualCapability(self)
        self.running = True
        self.timer_list = {}
        self.command_list = []
        self.started_commands_list = []
        # a list with sub_capabilities invoked by @invoke_sync and @invoke_async
        self.sub_cap_server_callback = {}

    def run(self) -> None:
        formatPrint(self, "starting...")
        while self.running:
            for command in self.command_list:
                if command not in self.started_commands_list:
                    t = Thread(target=self.__handle_command, args=(dict(command), ), daemon=True)
                    t.start()
                    self.started_commands_list.append(command)
                else:
                    self.command_list.remove(command)
                    self.started_commands_list.remove(command)
            self.loop()

    def __handle_command(self, command: dict):
        #formatPrint(self, f"Got Command {command}")
        cap = command["capability"]
        par = command["parameters"]

        formatPrint(self, f"Invoking Capability {cap} with params:\t {par}")

        if command["type"] == "trigger":
            ret = {"type": "response",
                       "capability":command["capability"]}
            if "src" in command.keys():
                ret["src"] = command["src"]
            if "streaming" in command.keys():
                ret["streaming"] = command["streaming"]
                if command["capability"] in self.timer_list.keys():
                    self.timer_list[command["capability"]].stop()
                # pop because of recursive function handling
                command.pop("streaming")
                if ret["streaming"] > 0:
                    self.timer_list[command["capability"]] = RepeatedTimer(1. / ret["streaming"], self.__handle_command, command, None)
                    formatPrint(self, f"Streaming now {command}")
                elif ret["streaming"] < 0:
                    try:
                        self.timer_list[command["capability"]].stop()
                    except:
                        pass
                else:
                    ret["parameters"] = self.__getattribute__(command["capability"])(command["parameters"])
                    #formatPrint(self, f"Executed Command {ret}")
                    self.send_message(ret)
            else:
                ret["parameters"] = self.__getattribute__(command["capability"])(command["parameters"])
                #formatPrint(self, f"Executed Command {ret}")
                self.send_message(ret)
        elif command["type"] == "response":
            pass
        else:
            raise CommandNotFoundException(f"Command {command} not found !!!")
        #This could be triggered outside by Timer
        if self.command_list.count(command) > 0:
            self.command_list.remove(command)

    @abstractmethod
    def loop(self):
        raise NotImplementedError

    def send_message(self, command: dict):
        cap = command["capability"]
        if command["type"] == "trigger":
            formatPrint(self, f"Triggering Sub-Capability: {cap}")
        elif command["type"] == "response":
            formatPrint(self, f"Capability successful: {cap}")
        self.server.send_message(json.dumps(command))

    def kill(self):
        self.running = False
        for timer in self.timer_list:
            self.timer_list[timer].stop()

    def invoke_sync(self, capability: str, params: dict) -> dict:
        """Invokes a subcap synchrony

        :param capability: the uri of the subcapability
        :param params: the parameter of the subcapability
        :return: the result of this query
        """
        execute_sub_cap_command = dict()
        src = f"{self.uri}-{capability}-{time()}"
        execute_sub_cap_command["type"] = "trigger"
        execute_sub_cap_command["src"] = src
        execute_sub_cap_command["capability"] = capability
        execute_sub_cap_command["parameters"] = params
        self.sub_cap_server_callback[src] = None
        self.send_message(execute_sub_cap_command)
        while self.sub_cap_server_callback[src] == None:
            sleep(1)
            #print(f"having some Trouble... {self.sub_cap_server_callback}")
        ret = self.sub_cap_server_callback[src]
        self.sub_cap_server_callback.pop(src)
        return ret

    def invoke_async(self, capability: str, params: dict, callback) -> Thread:
        """Invokes a subcap async

        :param capability: the uri of the subcapability
        :param params: the parameter of the subcapability
        :param callback: function to be called when subcap arrives, takes a dict as parameter
        :return: None
        """
        src = f"{self.uri}-{capability}-{time()}"
        def __wait_for_callback(callback):
            while self.sub_cap_server_callback[src] == None:
                sleep(1)
                #print(f"having some Trouble... 2 {self.sub_cap_server_callback}")
                pass
            callback(dict(self.sub_cap_server_callback[src]))
            self.sub_cap_server_callback.pop(src)
        src = f"{self.uri}-{capability}-{time()}"
        execute_sub_cap_command = dict()
        execute_sub_cap_command["type"] = "trigger"
        execute_sub_cap_command["src"] = src
        execute_sub_cap_command["capability"] = capability
        execute_sub_cap_command["parameters"] = params
        self.sub_cap_server_callback[src] = None
        self.send_message(execute_sub_cap_command)
        t = Thread(target=__wait_for_callback, args=(callback, ))
        t.start()
        return t