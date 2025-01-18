#!/usr/bin/env python3
from cereal import car
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car import create_button_events, get_safety_config

ButtonType = car.CarState.ButtonEvent.Type

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "fiat"

    ret.radarUnavailable = True
    ret.steerActuatorDelay = 0
    ret.steerLimitTimer = 0.4

    # safety config
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.fiat)]

    # ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15, 0.30], [0.03, 0.05]] # chrysler
    # ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV    = [[0.00, 0.00], [0.00 , 0.00]]
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    # ret.lateralTuning.init('pid')
    # ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP  = [[9.  , 20. ], [9.   , 20.]]
    # ret.lateralTuning.pid.kpV , ret.lateralTuning.pid.kiV = [[0.8, 0.3], [0.00, 0.00]] # fastback
    # ret.lateralTuning.pid.kf = 0.00004
    # ret.lateralTuning.pid.kf = 0

    ret.centerToFront = ret.wheelbase * 0.44
    ret.enableBsm = False

    ret.experimentalLongitudinalAvailable = True
    ret.pcmCruise = not experimental_long
    ret.openpilotLongitudinalControl = experimental_long

    # Tuning for experimental long
    ret.longitudinalTuning.kiV = [2.0, 1.5]
    ret.stoppingDecelRate = 2.0  # reach brake quickly after enabling
    ret.vEgoStopping = 0.25
    ret.vEgoStarting = 0.25

    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_adas, self.cp_cam)

    self.CS.button_event = [
      *self.CS.button_events,
      *create_button_events(self.CS.distance_button, self.CS.prev_distance_button, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.CS.lkas_enabled, self.CS.prev_lkas_enabled, {1: ButtonType.altButton1}),
    ]

    self.CS.mads_enabled = self.get_sp_cruise_main_state(ret)

    self.CS.accEnabled = self.get_sp_v_cruise_non_pcm_state(ret, c.vCruise, self.CS.accEnabled,
                                                            enable_buttons=(ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.resumeCruise) if not self.CP.pcmCruiseSpeed else
                                                                           (ButtonType.accelCruise, ButtonType.decelCruise),
                                                            resume_button=(ButtonType.resumeCruise,) if not self.CP.pcmCruiseSpeed else
                                                                          (ButtonType.accelCruise, ButtonType.resumeCruise))

    if ret.cruiseState.available:
      if self.enable_mads:
        if not self.CS.prev_mads_enabled and self.CS.mads_enabled:
          self.CS.madsEnabled = True
        if any(b.type == ButtonType.altButton1 and b.pressed for b in self.CS.button_events):
          self.CS.madsEnabled = not self.CS.madsEnabled
          self.CS.lkas_disabled = not self.CS.lkas_disabled
        self.CS.madsEnabled = self.get_acc_mads(ret, self.CS.madsEnabled)
    else:
      self.CS.madsEnabled = False
    self.CS.madsEnabled = self.get_sp_started_mads(ret, self.CS.madsEnabled)

    if not self.CP.pcmCruise or (self.CP.pcmCruise and self.CP.minEnableSpeed > 0) or not self.CP.pcmCruiseSpeed:
      if any(b.type == ButtonType.cancel for b in self.CS.button_events):
        self.get_sp_cancel_cruise_state()
    if self.get_sp_pedal_disengage(ret):
      self.get_sp_cancel_cruise_state()
      ret.cruiseState.enabled = ret.cruiseState.enabled if not self.enable_mads else False if self.CP.pcmCruise else self.CS.accEnabled

    if self.CP.pcmCruise and self.CP.minEnableSpeed > 0 and self.CP.pcmCruiseSpeed:
      if ret.gasPressed and not ret.cruiseState.enabled:
        self.CS.accEnabled = False
      self.CS.accEnabled = ret.cruiseState.enabled or self.CS.accEnabled

    if self.CP.pcmCruise and self.CP.minEnableSpeed > 0 and self.CP.pcmCruiseSpeed:
      if ret.gasPressed and not ret.cruiseState.enabled:
        self.CS.accEnabled = False
      self.CS.accEnabled = ret.cruiseState.enabled or self.CS.accEnabled

    ret = self.get_sp_common_state(ret)

    ret.buttonEvents = [
      *self.CS.button_events,
      *self.button_events.create_mads_event(self.CS.madsEnabled, self.CS.out.madsEnabled)  # MADS BUTTON
    ]

    # events
    events = self.create_common_events(ret, c, extra_gears=[], pcm_enable=False)

    events, ret = self.create_sp_events(ret, events)

    # Low speed steer alert hysteresis logic
    self.low_speed_alert = self.CS.out.vEgo <= self.CP.minEnableSpeed
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.customStockLong = self.update_custom_stock_long()

    ret.events = events.to_msg()

    return ret
