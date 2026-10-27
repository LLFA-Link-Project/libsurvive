from dataclasses import dataclass, field

import numpy as np

@dataclass
class SurvivePose:
    Pos: np.array = field(default_factory=lambda: np.array((0, 0, 0)))
    Rot: np.array = field(default_factory=lambda: np.array((1, 0, 0, 0)))

@dataclass
class SurviveAxisAnglePose:
    Pos: np.array = field(default_factory=lambda: np.array((0., 0., 0)))
    AxisAngleRot: np.array = field(default_factory=lambda: np.array((0, 0, 0)))

@dataclass
class SurviveIMUBiasModel:
    AccScale: np.array = field(default_factory=lambda: np.array((0., 0., 0)))
    IMUCorrection: np.array = field(default_factory=lambda: np.array((1, 0., 0., 0)))
    AccBias: np.array = field(default_factory=lambda: np.array((0., 0., 0)))
    GyroBias: np.array = field(default_factory=lambda: np.array((0., 0., 0)))

@dataclass
class SurviveKalmanModel:
    Pose: SurvivePose
    Velocity: SurviveAxisAnglePose
    Acc: np.array = field(default_factory=lambda: np.array((0., 0., 0)))
    IMUBias: SurviveIMUBiasModel = field(default_factory=lambda: SurviveIMUBiasModel())

@dataclass
class SurviveIMUBiasErrorModel:
    AccScale: np.array = field(default_factory=lambda: np.array((0., 0., 0)))
    IMUCorrection: np.array = field(default_factory=lambda: np.array((0., 0., 0)))
    AccBias: np.array = field(default_factory=lambda: np.array((0., 0., 0)))
    GyroBias: np.array = field(default_factory=lambda: np.array((0., 0., 0)))

@dataclass
class SurviveKalmanErrorModel:
    Pose: SurviveAxisAnglePose
    Velocity: SurviveAxisAnglePose
    Acc: np.array = field(default_factory=lambda: np.array((0., 0., 0)))
    IMUBias: SurviveIMUBiasErrorModel = field(default_factory=lambda: SurviveIMUBiasErrorModel())

@dataclass
class SurviveLighthouseModel:
    Pose: SurvivePose

@dataclass
class SurviveLighthouseErrorModel:
    Pose: SurviveAxisAnglePose

@dataclass
class BaseStationCal:
    phase: float = 0
    tilt: float = 0
    curve: float = 0
    gibpha: float = 0
    gibmag: float = 0
    ogeephase: float = 0
    ogeemag: float = 0
    def __add__(self, other):
        return BaseStationCal(

        )

@dataclass
class SurviveJointKalmanModel:
    Object: SurviveKalmanModel
    Lighthouse: SurvivePose
    BSD0: BaseStationCal
    BSD1: BaseStationCal

@dataclass
class SurviveLighthouseKalmanModel:
    Lighthouse: SurvivePose
    BSD0: BaseStationCal
    BSD1: BaseStationCal

@dataclass
class SurviveLighthouseKalmanErrorModel:
    Lighthouse: SurviveAxisAnglePose
    BSD0: BaseStationCal
    BSD1: BaseStationCal

@dataclass
class SurviveJointKalmanErrorModel:
    Object: SurviveKalmanErrorModel
    Lighthouse: SurviveAxisAnglePose
    BSD0: BaseStationCal
    BSD1: BaseStationCal

Vec3d = np.array([0, 0, 0])
