#!/usr/bin/env python3
"""
BCM4775 GPS Daemon for Xiaomi Mi 8 (dipper)
Manages lhd + glgps in Android chroot and publishes cereal gpsLocation messages.

Architecture:
  BCM4775 chip ─SPI─> BBD kernel driver ─> /dev/ttyBCM ─> lhd ─IPC─> glgps ─> NMEA pipe
  This daemon reads NMEA from the pipe and publishes cereal gpsLocation.
"""

import os
import sys
import math
import time
import signal
import subprocess
import datetime
import select

import cereal.messaging as messaging
from cereal import log

# Constants
CHROOT = "/data/android_root"
GPSPIPE = f"{CHROOT}/data/vendor/gps/gpspipe"
NSTANDBY = "/sys/devices/platform/soc/890000.spi/spi_master/spi32766/spi32766.0/nstandby"
LHD_CONF = "/vendor/etc/lhd.conf"
GPS_CONF = "/vendor/etc/gpsconfig.xml"
GPS_JOB = "Periodic"

CHR_ENV = "ANDROID_ROOT=/system ANDROID_DATA=/data LD_LIBRARY_PATH=/vendor/lib64:/system/lib64"


def write_file(path, value):
  try:
    with open(path, 'w') as f:
      f.write(str(value))
  except Exception as e:
    print(f"write_file({path}): {e}")


def is_process_running(name):
  try:
    result = subprocess.run(['pgrep', '-f', name], capture_output=True, timeout=3)
    return result.returncode == 0
  except Exception:
    return False


def sudo_run(args, **kwargs):
  """Run a command with sudo if not already root."""
  if os.getuid() != 0:
    args = ['sudo'] + list(args)
  return subprocess.run(args, **kwargs)


def sudo_popen(cmd, **kwargs):
  """Popen a command with sudo if not already root."""
  if os.getuid() != 0:
    cmd = f'sudo {cmd}'
  return subprocess.Popen(cmd, **kwargs)


def start_lhd():
  """Start the BCM4775 low-level host driver in chroot."""
  if is_process_running('vendor/bin/lhd'):
    print("lhd already running")
    return

  # Power on BCM4775
  sudo_run(['tee', NSTANDBY], input=b'1', capture_output=True, timeout=3)
  time.sleep(0.2)

  # Ensure BBD devices exist in chroot
  for dev in ['bbd_control', 'bbd_patch', 'bbd_sensor', 'ttyBCM']:
    chroot_dev = f"{CHROOT}/dev/{dev}"
    host_dev = f"/dev/{dev}"
    if not os.path.exists(chroot_dev) and os.path.exists(host_dev):
      sudo_run(['cp', '-a', host_dev, chroot_dev], capture_output=True, timeout=3)

  # Create log directory
  sudo_run(['mkdir', '-p', f"{CHROOT}/data/vendor/gps/log/lhd"], capture_output=True, timeout=3)

  # Mount sysfs if not already mounted (lhd needs nstandby path)
  sudo_run(['mountpoint', '-q', f'{CHROOT}/sys'], capture_output=True)
  sudo_run(['mount', '-t', 'sysfs', 'sysfs', f'{CHROOT}/sys'],
           capture_output=True, timeout=5)

  cmd = f'chroot {CHROOT} /system/bin/sh -c "{CHR_ENV} /vendor/bin/lhd {LHD_CONF}"'
  sudo_popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
  print("lhd started")
  time.sleep(3)  # Wait for firmware download


def start_glgps():
  """Start the Broadcom GPS location engine in chroot."""
  if is_process_running('vendor/bin/glgps'):
    print("glgps already running")
    return

  # Create required directories and pipes
  sudo_run(['mkdir', '-p', f"{CHROOT}/data/vendor/gps/log/gps"], capture_output=True, timeout=3)

  for pipe in ['gpspipe', 'glgpsctrl']:
    pipe_path = f"{CHROOT}/data/vendor/gps/{pipe}"
    if not os.path.exists(pipe_path):
      os.mkfifo(pipe_path)
      sudo_run(['chmod', '666', pipe_path], capture_output=True, timeout=3)

  cmd = f'chroot {CHROOT} /system/bin/sh -c "{CHR_ENV} /vendor/bin/glgps {GPS_CONF} {GPS_JOB}"'
  sudo_popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
  print("glgps started")
  time.sleep(2)  # Wait for glgps to connect to lhd


def parse_nmea_coord(raw, direction):
  """Parse NMEA coordinate (ddmm.mmmm or dddmm.mmmm) to decimal degrees."""
  if not raw or not direction:
    return None
  try:
    raw = float(raw)
  except ValueError:
    return None

  degrees = int(raw / 100)
  minutes = raw - degrees * 100
  result = degrees + minutes / 60.0

  if direction in ('S', 'W'):
    result = -result
  return result


def parse_nmea_time(time_str):
  """Parse NMEA time string (hhmmss.ss) to (hour, min, sec, microsec)."""
  if not time_str:
    return None
  try:
    h = int(time_str[0:2])
    m = int(time_str[2:4])
    s = int(time_str[4:6])
    us = int(float(time_str[6:]) * 1e6) if len(time_str) > 6 else 0
    return h, m, s, us
  except (ValueError, IndexError):
    return None


def parse_nmea_date(date_str):
  """Parse NMEA date string (ddmmyy) to (year, month, day)."""
  if not date_str:
    return None
  try:
    d = int(date_str[0:2])
    m = int(date_str[2:4])
    y = int(date_str[4:6]) + 2000
    return y, m, d
  except (ValueError, IndexError):
    return None


def verify_checksum(sentence):
  """Verify NMEA checksum."""
  if '*' not in sentence:
    return True  # No checksum to verify
  try:
    data, checksum = sentence.rsplit('*', 1)
    if data.startswith('$'):
      data = data[1:]
    calc = 0
    for c in data:
      calc ^= ord(c)
    return calc == int(checksum[:2], 16)
  except (ValueError, IndexError):
    return False


class NMEAState:
  """Accumulated state from NMEA sentences."""
  def __init__(self):
    self.lat = None
    self.lon = None
    self.alt = None
    self.speed = None          # m/s
    self.bearing = None        # degrees
    self.fix_quality = 0       # 0=no fix, 1=GPS, 2=DGPS
    self.num_sats = 0
    self.hdop = 99.9
    self.timestamp = None      # datetime object
    self.has_fix = False
    self.last_gga_time = 0
    self.last_rmc_time = 0


def process_gga(fields, state):
  """Process $GPGGA / $GNGGA sentence."""
  try:
    # $GPGGA,time,lat,N/S,lon,E/W,quality,numSats,HDOP,alt,M,geoid,M,age,refID
    if len(fields) < 10:
      return

    state.lat = parse_nmea_coord(fields[2], fields[3])
    state.lon = parse_nmea_coord(fields[4], fields[5])
    state.fix_quality = int(fields[6]) if fields[6] else 0
    state.num_sats = int(fields[7]) if fields[7] else 0
    state.hdop = float(fields[8]) if fields[8] else 99.9
    state.alt = float(fields[9]) if fields[9] else 0.0
    state.has_fix = state.fix_quality > 0
    state.last_gga_time = time.monotonic()
  except (ValueError, IndexError):
    pass


def process_rmc(fields, state):
  """Process $GPRMC / $GNRMC sentence."""
  try:
    # $GPRMC,time,status,lat,N/S,lon,E/W,speedKnots,bearing,date,magVar,magVarDir
    if len(fields) < 10:
      return

    status = fields[2]
    if status == 'V':
      state.has_fix = False

    lat = parse_nmea_coord(fields[3], fields[4])
    lon = parse_nmea_coord(fields[5], fields[6])
    if lat is not None:
      state.lat = lat
    if lon is not None:
      state.lon = lon

    # Speed: knots to m/s
    if fields[7]:
      state.speed = float(fields[7]) * 0.514444
    else:
      state.speed = 0.0

    # Bearing
    if fields[8]:
      state.bearing = float(fields[8])
    else:
      state.bearing = 0.0

    # Build timestamp from time + date
    time_parsed = parse_nmea_time(fields[1])
    date_parsed = parse_nmea_date(fields[9])
    if time_parsed and date_parsed:
      y, mo, d = date_parsed
      h, mi, s, us = time_parsed
      try:
        state.timestamp = datetime.datetime(y, mo, d, h, mi, s, us,
                                            tzinfo=datetime.timezone.utc)
      except ValueError:
        pass

    state.last_rmc_time = time.monotonic()
  except (ValueError, IndexError):
    pass


def should_publish(state):
  """Check if we have enough data to publish a location message."""
  now = time.monotonic()
  # Need both GGA and RMC data within the last 3 seconds
  if now - state.last_gga_time > 3 or now - state.last_rmc_time > 3:
    return False
  return True


def publish_location(pm, state):
  """Publish cereal gpsLocation message."""
  msg = messaging.new_message('gpsLocation', valid=state.has_fix)
  gps = msg.gpsLocation

  gps.latitude = state.lat if state.lat is not None else 0.0
  gps.longitude = state.lon if state.lon is not None else 0.0
  gps.altitude = state.alt if state.alt is not None else 0.0
  gps.speed = state.speed if state.speed is not None else 0.0
  gps.bearingDeg = state.bearing if state.bearing is not None else 0.0

  if state.timestamp:
    gps.unixTimestampMillis = int(state.timestamp.timestamp() * 1000)
  else:
    gps.unixTimestampMillis = int(time.time() * 1000)

  gps.source = log.GpsLocationData.SensorSource.android
  gps.hasFix = state.has_fix
  gps.satelliteCount = state.num_sats

  # Accuracy estimates from HDOP
  gps.horizontalAccuracy = state.hdop * 5.0 if state.hdop < 99 else 100.0
  gps.verticalAccuracy = state.hdop * 8.0 if state.hdop < 99 else 200.0
  gps.bearingAccuracyDeg = 180.0 if not state.has_fix else min(state.hdop * 10.0, 180.0)
  gps.speedAccuracy = state.hdop * 0.5 if state.hdop < 99 else 10.0

  # NED velocity (approximate from speed + bearing)
  if state.speed is not None and state.bearing is not None and state.speed > 0.1:
    bearing_rad = math.radians(state.bearing)
    vN = state.speed * math.cos(bearing_rad)
    vE = state.speed * math.sin(bearing_rad)
    gps.vNED = [vN, vE, 0.0]
  else:
    gps.vNED = [0.0, 0.0, 0.0]

  pm.send('gpsLocation', msg)


def main():
  print("bcmgpsd: BCM4775 GPS daemon starting")

  # Set up signal handler for clean shutdown
  running = [True]
  def signal_handler(sig, frame):
    running[0] = False
  signal.signal(signal.SIGTERM, signal_handler)
  signal.signal(signal.SIGINT, signal_handler)

  # Start lhd and glgps
  start_lhd()
  start_glgps()

  # Open NMEA pipe
  print(f"bcmgpsd: opening NMEA pipe at {GPSPIPE}")
  max_retries = 30
  pipe_fd = None
  for i in range(max_retries):
    if not running[0]:
      return
    try:
      pipe_fd = open(GPSPIPE, 'r')
      print("bcmgpsd: NMEA pipe opened")
      break
    except (FileNotFoundError, OSError) as e:
      print(f"bcmgpsd: waiting for pipe ({i+1}/{max_retries}): {e}")
      time.sleep(1)

  if pipe_fd is None:
    print("bcmgpsd: FATAL - could not open NMEA pipe")
    return

  # Set up cereal publisher
  pm = messaging.PubMaster(['gpsLocation'])

  state = NMEAState()
  last_publish = 0
  sentence_count = 0

  print("bcmgpsd: reading NMEA data...")

  try:
    while running[0]:
      # Use select for non-blocking read with timeout
      readable, _, _ = select.select([pipe_fd], [], [], 1.0)
      if not readable:
        # Check if processes are still alive
        if not is_process_running('vendor/bin/lhd'):
          print("bcmgpsd: lhd died, restarting...")
          start_lhd()
          start_glgps()
        elif not is_process_running('vendor/bin/glgps'):
          print("bcmgpsd: glgps died, restarting...")
          start_glgps()
        continue

      line = pipe_fd.readline()
      if not line:
        # Pipe closed, try to reopen
        print("bcmgpsd: pipe closed, reopening...")
        pipe_fd.close()
        time.sleep(1)
        try:
          pipe_fd = open(GPSPIPE, 'r')
        except OSError:
          time.sleep(2)
          continue
        continue

      line = line.strip()
      if not line or not line.startswith('$'):
        continue

      if not verify_checksum(line):
        continue

      sentence_count += 1
      fields = line.split(',')
      msg_type = fields[0]

      # Process relevant NMEA sentences
      if msg_type in ('$GPGGA', '$GNGGA'):
        process_gga(fields, state)
      elif msg_type in ('$GPRMC', '$GNRMC'):
        process_rmc(fields, state)

        # Publish at ~1 Hz after each RMC sentence (which has speed/bearing)
        now = time.monotonic()
        if now - last_publish >= 0.9 and should_publish(state):
          publish_location(pm, state)
          last_publish = now

          if sentence_count % 60 == 0:
            fix_str = "FIX" if state.has_fix else "NO FIX"
            print(f"bcmgpsd: {fix_str} sats={state.num_sats} "
                  f"lat={state.lat} lon={state.lon} "
                  f"spd={state.speed:.1f} hdop={state.hdop:.1f}")

  except Exception as e:
    print(f"bcmgpsd: error: {e}")
    import traceback
    traceback.print_exc()
  finally:
    pipe_fd.close()
    # Cleanup: kill glgps (lhd can stay for other uses)
    sudo_run(['killall', 'glgps'], capture_output=True, timeout=5)
    print("bcmgpsd: shutdown complete")


if __name__ == "__main__":
  main()
