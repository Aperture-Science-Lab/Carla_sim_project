# Aperture Science's Digital "Don't-Hit-That-Wall" Driving Simulator

![Aperture Science Logo](https://img.shields.io/badge/Powered%20by-Science!-orange?style=for-the-badge)
![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
![CARLA](https://img.shields.io/badge/CARLA-Simulator-blue?style=for-the-badge)

## Video

https://github.com/user-attachments/assets/b7c6cbcb-a2d5-47d1-82a9-56a157fbc036

## Project Structure

```
├── module_7.py                    # Main execution script
├── behavioural_planner.py         # High-level FSM decision making
├── local_planner.py              # Local trajectory generation
├── collision_checker.py          # Path collision detection
├── velocity_planner.py           # Velocity profile generation
├── path_optimizer.py             # Spiral path optimization
├── controller2d.py               # Low-level PID controller
├── options.cfg                   # Configuration file
└── parked_vehicle_params.txt     # Static obstacle data
```

## Installation

```bash
# Install dependencies
pip install numpy matplotlib scipy pygame

# Start CARLA server (version 0.8.x)
./CarlaUE4.sh -quality-level=Low

# Run the planner
python module_7.py
```

## Configuration

Edit `options.cfg` to enable/disable features:

```ini
[live_plotting]
enabled = false  # Set to true for real-time visualization
```

## Key Features

- Finite State Machine for behavioral planning
- Cubic spiral path generation
- Collision detection with geometric approximation
- Velocity planning for stop signs and lead vehicles
- PID control for low-level actuation

## Troubleshooting

- **Connection errors**: Ensure CARLA server is running on correct port
- **Performance issues**: Disable live plotting in options.cfg
- **Import errors**: Verify all dependencies are installed

## License

Educational use - Self-Driving Cars Specialization, University of Toronto
