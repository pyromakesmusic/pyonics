from pythonosc.udp_client import SimpleUDPClient
import pyonics
from pyonics.control.messages import AsyncTestClient, AsyncServer
import time
import asyncio
import math
import pytest

class TestServer:
    def __init__(self):
        self.received = []

    def handler(self, address, *args):
        print("RECEIVED:", address, args)
        self.received.append((address, args))



client = SimpleUDPClient("127.0.0.1", 9000)
client.send_message("/pressures", [0.1, 0.2, 0.3])
time.sleep(0.1)

@pytest.mark.skip(reason="Async OSC tests not stable in CI yet")
async def osc_roundtrip_test():
    server = TestServer()

    osc = pyonics.AsyncServer("127.0.0.1", 9000, "test", server.handler)
    await osc.make_endpoint()

    osc.dispatcher.map("/pressures", osc.handler)

    client = SimpleUDPClient("127.0.0.1", 9000)
    client.send_message("/pressures", [1.0, 2.0, 3.0])

    await asyncio.sleep(0.2)

    assert len(server.received) > 0

@pytest.mark.skip(reason="Async OSC tests not stable in CI yet")
async def test_single():
    client = AsyncTestClient(port=9000)
    await client.send_once(1.0, 2.0, 3.0)

@pytest.mark.skip(reason="Async OSC tests not stable in CI yet")
async def test_constant():
    client = AsyncTestClient(port=5005)

    def constant(t):
        return [0.5, 0.5, 0.5]

    await client.send_loop(constant, dt=0.1, duration=2.0)

@pytest.mark.skip(reason="Async OSC tests not stable in CI yet")
async def test_sine():
    client = AsyncTestClient(port=5005)

    def sine_wave(t):
        return [
            0.5 + 0.5 * math.sin(2 * math.pi * 1.0 * t),
            0.5 + 0.5 * math.sin(2 * math.pi * 1.0 * t + math.pi/2),
        ]

    await client.send_loop(sine_wave, dt=0.02, duration=5.0)

asyncio.run(osc_roundtrip_test())
asyncio.run(test_single())