from dataclasses import dataclass, field

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.docs_definitions import CarHarness, CarDocs, CarParts
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig
from openpilot.selfdrive.car import CarSpecs, dbc_dict, DbcDict, PlatformConfig, Platforms

Ecu = car.CarParams.Ecu

@dataclass
class FastbackCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control(ACC)"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.fca]))


@dataclass
class FastbackPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('fca_fastback_limited_edition_2024_generated', 'fca_fastback_limited_edition_2024_generated'))


@dataclass(frozen=True)
class FastbackCarSpecs(CarSpecs):
  minSteerSpeed = 10 * CV.KPH_TO_MS
  tireStiffnessFactor: float = .97  # not optimized yet

class CAR(Platforms):
  FASTBACK_LIMITED_EDITION_2024 = FastbackPlatformConfig(
    [FastbackCarDocs("Fastback Limited Edition 2024")],
    FastbackCarSpecs(mass=1253., wheelbase=2.695, steerRatio=16.89),
  )

class CarControllerParams:
  def __init__(self, CP):
    self.STEER_MAX = 1440
    self.STEER_DELTA_UP = 4
    self.STEER_DELTA_DOWN = 3
    self.STEER_ERROR_MAX = 200

    self.STEER_DRIVER_MULTIPLIER = 2  # weight driver torque heavily
    self.STEER_DRIVER_FACTOR = 1  # from dbc
    self.STEER_DRIVER_ALLOWANCE = 20

STEER_THRESHOLD = 10

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
  ],
  extra_ecus=[
  ],
)

DBC = CAR.create_dbc_map()
