from pythonosc.udp_client import SimpleUDPClient
import pyonics
import time
import asyncio

class TestServer:
    def __init__(self):
        self.received = []

    def handler(self, address, *args):
        print("RECEIVED:", address, args)
        self.received.append((address, args))



client = SimpleUDPClient("127.0.0.1", 9000)
client.send_message("/pressures", [0.1, 0.2, 0.3])
time.sleep(0.1)

async def osc_roundtrip_test():
    server = TestServer()

    osc = pyonics.AsyncServer("127.0.0.1", 9000, "test", server.handler)
    await osc.make_endpoint()

    osc.dispatcher.map("/pressures", osc.handler)

    client = SimpleUDPClient("127.0.0.1", 9000)
    client.send_message("/pressures", [1.0, 2.0, 3.0])

    await asyncio.sleep(0.2)

    assert len(server.received) > 0

asyncio.run(osc_roundtrip_test())