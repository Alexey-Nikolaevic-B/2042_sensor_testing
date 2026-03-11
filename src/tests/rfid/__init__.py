# Import all RFID tests so their @register_test decorators fire on package load.
from .max_stable_read_distance_test import MaxStableReadDistanceTest
from .min_stable_read_distance_test import MinStableReadDistanceTest
from .mass_read_test                import MassReadTest
from .overlap_tags_test             import OverlapTagsTest
from .angle_dependence_test         import AngleDependenceTest
from .move_tags_test                import MoveTagsTest
from .antenna_rotation_test         import AntennaRotationTest

__all__ = [
    "MaxStableReadDistanceTest",
    "MinStableReadDistanceTest",
    "MassReadTest",
    "OverlapTagsTest",
    "AngleDependenceTest",
    "MoveTagsTest",
    "AntennaRotationTest",
]