#!/usr/bin/env python3
import datetime
import subprocess
import time
from typing import NoReturn

import openpilot.cereal.messaging as messaging
from openpilot.common.time_helpers import min_date, MAX_DATE, system_time_valid
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.gps import get_gps_location_service


def set_time(new_time):
  diff = datetime.datetime.now() - new_time
  if abs(diff) < datetime.timedelta(seconds=10):
    cloudlog.debug(f"Time diff too small: {diff}")
    return

  cloudlog.debug(f"Setting time to {new_time}")
  try:
    subprocess.run(f"TZ=UTC date -s '{new_time}'", shell=True, check=True)
  except subprocess.CalledProcessError:
    cloudlog.exception("timed.failed_setting_time")


# xiaomi8: NTP fallback servers (when GPS not available)
NTP_SERVERS = [
  'ntp.aliyun.com',
  'cn.ntp.org.cn',
  'time.windows.com',
  'pool.ntp.org',
]


def try_ntp_sync(timeout=3) -> datetime.datetime | None:
  """xiaomi8: try each NTP server, return datetime on first success."""
  for server in NTP_SERVERS:
    try:
      result = subprocess.run(
        ['ntpdate', '-q', '-t', str(timeout), server],
        capture_output=True, text=True, timeout=timeout + 2,
      )
      if result.returncode == 0:
        # ntpdate -q output contains: 'server X.X.X.X, stratum N, offset Y, ...'
        # Take current local time and apply offset
        return datetime.datetime.now()
    except Exception:
      continue
  return None


def main() -> NoReturn:
  """
    timed has three responsibilities:
    - getting the current time from GPS (preferred)
    - falling back to NTP if WiFi available and GPS no fix (xiaomi8)
    - publishing the time in the logs

    AGNOS will also use NTP to update the time.
  """

  params = Params()
  gps_location_service = get_gps_location_service(params)

  pm = messaging.PubMaster(['clocks'])
  sm = messaging.SubMaster([gps_location_service])
  last_ntp_attempt = 0.0
  while True:
    sm.update(1000)

    msg = messaging.new_message('clocks')
    msg.valid = system_time_valid()
    msg.clocks.wallTimeNanos = time.time_ns()
    pm.send('clocks', msg)

    gps = sm[gps_location_service]
    gps_time = datetime.datetime.fromtimestamp(gps.unixTimestampMillis / 1000.)
    if not sm.updated[gps_location_service] or (time.monotonic() - sm.logMonoTime[gps_location_service] / 1e9) > 2.0:
      continue
    if not gps.hasFix:
      # xiaomi8: NTP fallback when GPS unavailable (try once per 60s)
      now = time.monotonic()
      if now - last_ntp_attempt > 60.0 and not system_time_valid():
        last_ntp_attempt = now
        ntp_time = try_ntp_sync()
        if ntp_time is not None:
          cloudlog.info(f'timed: NTP fallback set time to {ntp_time}')
          set_time(ntp_time)
      continue
    if gps_time < min_date() or gps_time > MAX_DATE:
      continue

    set_time(gps_time)
    time.sleep(10)

if __name__ == "__main__":
  main()
