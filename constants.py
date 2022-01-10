from enum import Enum

G = 9.81  # m/s/s


class StateVars(Enum):
    xIdx = 0
    yIdx = 1
    thetaIdx = 2
    vlIdx = 3
    vrIdx = 4
    alIdx = 5
    arIdx = 6


class ControlVars(Enum):
    jlIdx = 0
    jrIdx = 1
