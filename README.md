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
```
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

pcm = pyonics.ExoController(config_data)
```