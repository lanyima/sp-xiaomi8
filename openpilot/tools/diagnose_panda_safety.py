#!/usr/bin/env python3
"""Read-only Panda safety telemetry recorder.

Run offroad before a short reproduction drive.  It only subscribes to
``pandaStates`` and reads CarParams; it never opens Panda USB or writes CAN.
The log identifies which of the three independent Controls Mismatch guards
fired: safety configuration, RX validity, or controlsAllowed.
"""
import json
import time

import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car
from openpilot.common.params import Params

OUT = "/data/panda_safety_diagnose.log"


def expected_config():
  raw = Params().get("CarParams")
  if not raw:
    return {"carParams": "missing"}
  with car.CarParams.from_bytes(raw) as cp:
    return {
      # Newer CarParams schemas no longer expose carName.  Keep the recorder
      # usable across both package generations without changing vehicle state.
      "car": getattr(cp, "carName", getattr(cp, "carFingerprint", "unknown")),
      "alternativeExperience": int(cp.alternativeExperience),
      "safetyConfigs": [{"model": int(s.safetyModel.raw), "param": int(s.safetyParam)} for s in cp.safetyConfigs],
    }


def panda_snapshot(ps):
  return {
    "model": int(ps.safetyModel.raw),
    "param": int(ps.safetyParam),
    "alternativeExperience": int(ps.alternativeExperience),
    "controlsAllowed": bool(ps.controlsAllowed),
    "controlsAllowedLateral": bool(ps.controlsAllowedLateral),
    "controlsAllowedLongitudinal": bool(ps.controlsAllowedLongitudinal),
    "safetyRxInvalid": int(ps.safetyRxInvalid),
    "safetyRxChecksInvalid": bool(ps.safetyRxChecksInvalid),
    "safetyTxBlocked": int(ps.safetyTxBlocked),
    "heartbeatLost": bool(ps.heartbeatLost),
    "faultStatus": int(ps.faultStatus.raw),
    "faults": [int(f.raw) for f in ps.faults],
    "can": [{"busOff": bool(c.busOff), "rx": int(c.totalRxCnt), "tx": int(c.totalTxCnt),
             "rxLost": int(c.totalRxLostCnt), "txLost": int(c.totalTxLostCnt),
             "err": int(c.totalErrorCnt)} for c in (ps.canState0, ps.canState1, ps.canState2)],
  }


def main():
  sm = messaging.SubMaster(["pandaStates"])
  previous = None
  with open(OUT, "a", encoding="utf-8", buffering=1) as out:
    out.write(json.dumps({"ts": time.time(), "event": "start", "expected": expected_config()}, sort_keys=True) + "\n")
    while True:
      sm.update(1000)
      if not sm.updated["pandaStates"]:
        continue
      got = [panda_snapshot(ps) for ps in sm["pandaStates"]]
      if got != previous:
        out.write(json.dumps({"ts": time.time(), "expected": expected_config(), "pandas": got}, sort_keys=True) + "\n")
        previous = got


if __name__ == "__main__":
  main()
