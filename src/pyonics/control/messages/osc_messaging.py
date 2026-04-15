import pythonosc
from pythonosc.dispatcher import Dispatcher
from pythonosc.udp_client import SimpleUDPClient
import pythonosc.osc_server
import argparse
import asyncio


"""
Network Controller
"""
class AsyncServer:
    """
    ip: source server ip as string
    port: client port as integer

    Server must be asynchronous to allow control loop to function intermittently.
    """
    def __init__(self, ip, port, signature_string, handler):
        self.dispatcher = pythonosc.dispatcher.Dispatcher()
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument("--ip", default=ip, help="The IP address to listen on")
        self.parser.add_argument("--port", type=int, default=port, help="The port to listen on")
        self.args = self.parser.parse_args()
        self.ip = ip
        self.port = port
        self.server = None
        self.transport = None
        self.protocol = None
        self.password = signature_string
        self.handler = handler
        self.log_osc = False

    async def make_endpoint(self):
        """Need to make this endpoint"""
        assert type(self.ip) == str
        self.server = pythonosc.osc_server.AsyncIOOSCUDPServer((self.ip, self.port),
                                                               self.dispatcher, asyncio.get_running_loop())
        self.transport, self.protocol = await self.server.create_serve_endpoint()
        print("Serving on {}".format(self.ip))
        return

    async def map(self, pattern, func, *args, **kwargs):
        """
        pattern: string var defining the OSC pattern to be recognized
        func: the function to map to
        args: any args for the function, this may need to be *args and **kwargs - needs more research
        """
        print("... performing mapping operation... ")
        def wrapper(address, *osc_args):
            if self.log_osc:
                print(f"[OSC RECEIVED] {address} {osc_args}")

            return func(address, *osc_args)
        self.dispatcher.map(pattern, wrapper)

    """
    Testing
    """

    async def enable_osc_logging(self, enabled: bool = True):
        self.log_osc = enabled

class AsyncTestClient:
    def __init__(self, ip="127.0.0.1", port=5005, address="/pressures"):
        self.ip = ip
        self.port = port
        self.address = address
        self.client = SimpleUDPClient(self.ip, self.port)

    async def send(self, pressures):
        """
        pressures: iterable of floats
        """
        print(f"[OSC SEND] {self.address} {pressures}")
        self.client.send_message(self.address, list(pressures))

    async def send_once(self, *pressures):
        """
        Convenience wrapper
        """
        await self.send(pressures)

    async def send_loop(self, generator_fn, dt=0.05, duration=2.0):
        """
        generator_fn: function t -> iterable of pressures
        dt: timestep (seconds)
        duration: total runtime
        """
        t = 0.0
        start = asyncio.get_running_loop().time()

        while True:
            now = asyncio.get_running_loop().time()
            t = now - start

            if t > duration:
                break

            pressures = generator_fn(t)
            await self.send(pressures)

            await asyncio.sleep(dt)
