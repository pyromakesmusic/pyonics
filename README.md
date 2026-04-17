# pyonics
A Python software library made to simplify the design, testing and production workflow of bionic products. Relies
heavily on the Klamp't library developed starting at Duke. First few versions are going to be designed around
modeling McKibben muscles, and biomechanics like those encountered in humanoid robots.

Thanks to Dr. Paul Savala, my research advisor, for support in developing this library.

## Quick Start

Install:

```bash
pip install pyonics
```

Create an ExoController:
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
}

pcm = ExoController(config)
```

Create a dataframe of muscles that looks like the following:
```csv
name;link_a;link_b;transform_a;transform_b;label_a;label_b;turns;r_0;l_0;weave_length;max_pressure

right_inferior_latissimus;9;4;0,0,0;0,-1,0;superior;inferior;20;1;2;2;80000
```

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

```python
import pandas as pd
from klampt.sim.simulation import SimpleSimulator

sim = SimpleSimulator(pcm.world)

muscleinfo_df = pd.read_csv(config['attachments'], sep=";")  # This dataframe contains info on every muscle attachment
muscleinfo_df.shape[0]  # This is the number of rows, so the loop should loop "row" many times

for x in range(muscleinfo_df.shape[0]):
    row = muscleinfo_df.iloc[x] # Locates the muscle information in the dataframe
    muscle = MuscleEmulator(row, pcm, sim)
    sim.addEmulator(pcm.robot, muscle)
```

Start simulation:
```
async def startup(self):
    await self.pcm.setup_osc_server()
    await self.pcm.server.enable_osc_logging()
    
    running = True
    while running:
        await sim.simulate(config["timestep"])
```