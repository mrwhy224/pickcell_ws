"""Generate randomized, standalone chemical-bag URDF models on demand."""

from dataclasses import dataclass
import math
from pathlib import Path
import random
from typing import Optional

from ament_index_python.packages import get_package_share_directory
import xacro


@dataclass(frozen=True)
class GeneratedChemicalBag:
    """One internally consistent bag sample ready for a simulator spawner."""

    name: str
    urdf_xml: str
    mass: float
    size: tuple[float, float, float]
    center_of_mass: tuple[float, float, float]
    spawn_xyz: tuple[float, float, float]
    spawn_rpy: tuple[float, float, float]


class ChemicalBagGenerator:
    """Create a new physical and visual bag variation for every call."""

    NOMINAL_SIZE = (0.60, 0.40, 0.16)

    def __init__(self, seed: Optional[int] = None) -> None:
        # A supplied seed supports repeatable tests; normal use is non-deterministic.
        self._rng = random.Random(seed) if seed is not None else random.SystemRandom()
        self._xacro_path = (
            Path(get_package_share_directory("pickcell_description"))
            / "urdf"
            / "chemical_bag_standalone.urdf.xacro"
        )

    def generate(
        self,
        name: str = "chemical_bag",
        xy: tuple[float, float] = (0.0, 0.0),
    ) -> GeneratedChemicalBag:
        """Generate a randomized URDF and its recommended world spawn pose."""
        mass = self._rng.uniform(20.0, 24.0)
        length = self._rng.uniform(0.56, 0.64)
        width = self._rng.uniform(0.36, 0.44)
        height = self._rng.uniform(0.14, 0.18)

        com = (
            self._rng.uniform(-0.025, 0.025),
            self._rng.uniform(-0.018, 0.018),
            self._rng.uniform(-0.012, 0.012),
        )
        rpy = (
            self._rng.uniform(math.radians(-4.0), math.radians(4.0)),
            self._rng.uniform(math.radians(-4.0), math.radians(4.0)),
            self._rng.uniform(-math.pi, math.pi),
        )

        roll, pitch, _ = rpy
        support_height = 0.5 * (
            abs(math.sin(pitch)) * length
            + abs(math.sin(roll) * math.cos(pitch)) * width
            + abs(math.cos(roll) * math.cos(pitch)) * height
        )

        mappings = {
            "bag_name": name,
            "bag_mass": f"{mass:.9f}",
            "bag_scale_x": f"{length / self.NOMINAL_SIZE[0]:.9f}",
            "bag_scale_y": f"{width / self.NOMINAL_SIZE[1]:.9f}",
            "bag_scale_z": f"{height / self.NOMINAL_SIZE[2]:.9f}",
            "bag_com_x": f"{com[0]:.9f}",
            "bag_com_y": f"{com[1]:.9f}",
            "bag_com_z": f"{com[2]:.9f}",
        }
        urdf_xml = xacro.process_file(
            str(self._xacro_path), mappings=mappings
        ).toxml()

        return GeneratedChemicalBag(
            name=name,
            urdf_xml=urdf_xml,
            mass=mass,
            size=(length, width, height),
            center_of_mass=com,
            spawn_xyz=(xy[0], xy[1], support_height),
            spawn_rpy=rpy,
        )
