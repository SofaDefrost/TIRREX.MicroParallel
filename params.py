from dataclasses import dataclass, field

# Units are mm, kg, and s


@dataclass
class LegParams:

    width: float = 35
    height: float = 40
    radius: float = 1

    youngModulus: float = 5e3
    poissonRatio: float = 0.45
    density: float = 1.55e-6


@dataclass
class PlatformParams:

    youngModulus: float = 5e3
    poissonRatio: float = 0.45
    density: float = 1.55e-6

    radius: float = 1
    gapX: float = 7
    gapZ: float = 5


@dataclass
class RobotParams:

    platform: PlatformParams = PlatformParams()
    leg: LegParams = LegParams()
    nbLegs: int = 4
