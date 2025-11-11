from .simulator import Simulator
from .gazebo.gazebo_simulator import GazeboSimulator

class SimulatorFactory:
    @staticmethod
    def create_simulator(simulator_type: str, config: dict = None) -> Simulator:
        simulators = {
            'gazebo': GazeboSimulator,
            # Add new simulators here
        }
        
        if simulator_type.lower() not in simulators:
            raise ValueError(f"Unknown simulator type: {simulator_type}")
            
        return simulators[simulator_type.lower()](config)