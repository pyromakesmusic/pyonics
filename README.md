# pyonics
A Python library for simulating and controlling pneumatic artificial muscles using Klamp't robotics library.

## Quick Start

Install:

```bash
pip install pyonics
```

Create a controller:
```python
import klampt
from pyonics import ExoController, MuscleEmulator

# Load configuration
config = {
    "has_robworld": True,
    "world_path": "data/world.xml",
    "core": "data/robot.rob",
    "attachments": "data/muscles.csv",
    "timestep": 0.01,
    "address": "127.0.0.1",
    "port": 5005
}

pcm = ExoController(config)
```

Create a dataframe of muscles that looks like the following:
```csv
name;link_a;link_b;transform_a;transform_b;label_a;label_b;turns;r_0;l_0;weave_length;max_pressure

right_inferior_latissimus;9;4;0,0,0;0,-1,0;superior;inferior;20;1;2;2;80000
```

```python
import pandas as pd
from klampt.sim.simulation import SimpleSimulator

sim = SimpleSimulator(pcm.world)

muscleinfo_df = pd.read_csv(config['attachments'], sep=";")

for _, row in muscleinfo_df.iterrows():
    row = muscleinfo_df.iloc[x] 
    muscle = MuscleEmulator(row, pcm, sim)
    sim.addEmulator(pcm.robot, muscle)
```

Start simulation:
```python
import asyncio

async def startup():
    await pcm.setup_osc_server()
    await pcm.server.enable_osc_logging()
    
    running = True
    while running:
        sim.simulate(0.01)
        await asyncio.sleep(0.01)
        
asyncio.run(startup())
```

## Features

- Pneumatic muscle emulation
- Pressure-driven actuation
- Klamp't simulator integration
- OSC network control

## Requirements

- Python 3.x
- Klamp't installed
- python-osc
- differint

### Configuration Format

`has_robworld`: Boolean. Currently must be true.

`world_path`: Filepath to the world, in XML format.

`core`: Filepath to the robot, in .rob format.

`attachments`: Filepath to the muscle attachment CSV.

`timestep`: Control rate for the robot.

`address`: IP address. Currently only tested locally.

`port`: Networking port to receive on.

### Muscle Attachment DataFrame Format
`name`: A name for the muscle.

`link_a`: The index of the first robot link to attach to.

`link_b`: The index of the second robot link to attach to.

`transform_a`: The local transform on link A to attach to.

`transform_b`: The local transform on link B to attach to.

`label_a`: For convenience. Functionality not yet implemented.

`label_b`: For convenience. Functionality not yet implemented.

`turns`: Number of turns in the weave.

`r_0`: Resting radius of the muscle.

`l_0`: Resting length of the muscle.

`weave_length`: Length of the weave surrounding the muscle.

`pressure`: Pressure of the muscle. Updated with muscle commands.

### Acknowledgments
Thanks to Dr. Paul Savala, my research advisor, for support in developing this library.
