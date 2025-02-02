# The following factor is used to convert degrees to the units used by the
# SimpleBGC 2.6 serial protocol.
degree_factor = 0.02197265625
degree_per_sec_factor = 0.1220740379

degree_hires_factor = 0.00034332275390625
degree_per_sec_hires_factor = 0.001


def from_degree(degree: float) -> int:
    return int(degree / degree_factor)


def from_degree_hires(degree: float) -> int:
    return int(degree / degree_hires_factor)


def to_degree(angle: float) -> float:
    return angle * degree_factor


def to_360_degree(angle: float) -> float:
    return to_degree(angle) % 360


def to_degree_hires(angle: int) -> float:
    return angle * degree_hires_factor


def from_degree_per_sec(degree_per_sec: float) -> int:
    return int(degree_per_sec / degree_per_sec_factor)


def from_degree_per_sec_hires(degree_per_sec: float) -> int:
    return int(degree_per_sec / degree_per_sec_hires_factor)


def to_degree_per_sec(angle_per_sec: float) -> float:
    return angle_per_sec * degree_per_sec_factor


def to_degree_per_sec_hires(angle_per_sec: int) -> float:
    return angle_per_sec * degree_per_sec_hires_factor
