from abc import ABC, abstractmethod
from typing import Any, Optional

class Simulator(ABC):
    """Abstract base class for all simulators"""
    
    @abstractmethod
    def __init__(self, config: dict = None):
        pass
    
    @abstractmethod
    def launch(self) -> str:
        """Launch the simulator"""
        pass
    
    @abstractmethod
    def open_scene(self) -> bool:
        """Load a specific scene/world"""
        pass
    
    @abstractmethod
    def receive_sensor_data(self, topic: str) -> Any:
        """Receive sensor data from the simulator"""
        pass
    
    @abstractmethod
    def kill(self) -> bool:
        """Shutdown the simulator"""
        pass
    
    @abstractmethod
    def is_running(self) -> bool:
        """Check if simulator is running"""
        pass