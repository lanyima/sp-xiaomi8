"""
jetlink on this fork: the one module that holds every openpilot import
jetlink needs, and the functions the hooks call.

jetlink (the jetlink/ package, dropped beside openpilot/ so that
PYTHONPATH=/data/openpilot finds it) runs the large driving model on an
attached Jetson, Mac, iPhone or Android phone and never imports openpilot. It
defines what it needs as an interface (jetlink.openpilot.interface.Openpilot),
and Adapter below implements it. The hooks in manager, modeld, hardwared, the
model manager and the UI call the functions at the bottom, which answer as if
the link were off when jetlink is not present, or speaks another API.

Ported from zoompilot (MIT) to the xiaomi8 sp2026 tree. Differences from
upstream zoompilot:

  * Wi-Fi. zoompilot's adapter offers ('off', 'usb', 'ios'); this fork's
    primary use is a phone hotspot, so 'wifi' (jetlink.comma.wifi: the comma
    joins the hotspot and dials its default gateway on port 5599) is offered
    too. Nothing else changes: Wi-Fi builds no gadget and leaves USB alone.
  * chestnut_present() answers from the USB-GPU probe this fork actually has
    (openpilot.selfdrive.modeld.helpers.usbgpu_present) instead of zoompilot's
    modeld.helpers.chestnut_present, which this tree does not carry. Same
    vendor/product id (ADD1:0001), so the same "an accelerator is fitted over
    USB-C, keep the link off" answer.

manager imports this module to build the process list, and runs it as
jetlinkd, the resident gadget owner. So the top level is the standard library
only; everything else is imported where it is used.
"""
from __future__ import annotations

import functools
import os
import threading
from collections import namedtuple
from pathlib import Path

# the version of jetlink.openpilot's API this adapter is written to; any other
# is treated as jetlink being absent, with the reason as the offroad alert
API = 2

# the gadget owner, as manager names the process and selfdrived lists it
OWNER = 'jetlinkd'

# the Jetlink setting, stored as an index: jetlink.openpilot.MODES. 'wifi' is
# this fork's addition; the index is what goes in the param, so the order here
# is the order the UI and jetlink's own MODES must agree on.
MODES = ('off', 'usb', 'ios', 'wifi')

# the params jetlink reads and writes, all declared in params_keys.h. big_model
# and catalog are the model manager's big-model slot and catalog
# (models.helpers.ACTIVE_BUNDLE_KEYS['chestnut'] and ModelFetcher's cache): the
# owner cannot import the model manager, so they are written out here
_Keys = namedtuple('_Keys', 'link offroad progress spec pointers big_model catalog charge_phone')
KEYS = _Keys(link='JetlinkLink', offroad='IsOffroad', progress='AcceleratorProgress', spec='JetlinkSpec',
             pointers='JetlinkModelPointers', big_model='ModelManager_ActiveBundleChestnut',
             catalog='ModelManager_ModelsCache_Chestnut', charge_phone='JetlinkChargePhone')

# The accelerator ids this fork knows, written out so the owner imports nothing
# heavy. zoompilot's set is comma's chestnut board in its ROM plus the ASMedia
# bridges; this fork's USB-GPU probe only ever matches ADD1:0001, and the
# answer here only has to be conservative: a fitted accelerator keeps the link
# off.
CHESTNUT_IDS = frozenset({(0xADD1, 0x0001), (0x3801, 0x0001), (0x174C, 0x2464), (0x174C, 0x2463)})

# where the build puts the warp for each camera (tools/build_jetlink_warp.sh,
# which runs `python -m jetlink.openpilot.warp`) and modeld loads it from: in
# this fork's tree, under the adapter, never in the jetlink package (a file
# there leaves it dirty for the updater). Not Paths.comma_home(), which on
# AGNOS is a tmpfs overlay: the pickle would be gone every boot
WARP_DIR = Path(__file__).resolve().parent / 'models'

OWNER_LOG = Path('/data/log/jetlink-owner.log')

_AGNOS = os.path.isfile('/AGNOS')


def _params_dir() -> Path:
  """The params store's directory, by params.cc and hw.h's rule: PARAMS_ROOT,
  else /data/params on a device and ~/.comma<OPENPILOT_PREFIX>/params
  elsewhere, then /<OPENPILOT_PREFIX, or d>. Per call, so it follows a prefix
  as Params does; the environment only, so it never raises."""
  prefix = os.environ.get('OPENPILOT_PREFIX', '')
  root = os.environ.get('PARAMS_ROOT')
  if root is None:
    root = '/data/params' if _AGNOS else os.path.join(os.environ.get('HOME', ''), '.comma' + prefix, 'params')
  return Path(root) / os.environ.get('OPENPILOT_PREFIX', 'd')


def warp_path(cam_w: int, cam_h: int, model_w: int, model_h: int) -> Path:
  """The warp for one geometry: the build's target and what modeld opens."""
  return WARP_DIR / f'warp_{cam_w}x{cam_h}_{model_w}x{model_h}_tinygrad.pkl'


def owner_config():
  """What jetlinkd needs: data only, so the owner imports nothing heavy."""
  from jetlink.openpilot.interface import Keys, OwnerConfig

  from openpilot.common.basedir import BASEDIR
  return OwnerConfig(params_dir=_params_dir(), keys=Keys(**KEYS._asdict()), chestnut_ids=CHESTNUT_IDS,
                     adapter=__name__, cwd=Path(BASEDIR), env={'PYTHONPATH': BASEDIR}, log_file=OWNER_LOG)


def main() -> None:
  """jetlinkd: hold the USB gadget until manager stops this process. Only the
  cable modes need it; Wi-Fi dials the hotspot's gateway and holds nothing."""
  from jetlink.openpilot.owner import main as run_owner
  run_owner(owner_config())


def adapter() -> Adapter:
  """The adapter, for jetlink's entry points that run as their own process:
  the provisioning run and the warp build."""
  return Adapter()


class Joining:
  """The joined model as this fork's modelds call it.

  Stock modeld calls `model.run(bufs, transforms, inputs)` and jetlink's run
  takes `after_enqueue`; sunnypilot's modeld_v2 passes a fourth positional,
  `prepare_only`, a dropped-frame skip. A joining model has no such notion, so
  the wrapper drops it and every other attribute reaches the model underneath
  (in_control, frame_drop_ratio, lat_delay, vision_input_names, ...).

  This exists so the two modelds can keep their own call shapes; it is only
  ever wrapped around jetlink's model, never around ModelState.
  """

  def __init__(self, joined):
    object.__setattr__(self, '_joined', joined)

  def __getattr__(self, name):
    return getattr(object.__getattribute__(self, '_joined'), name)

  def __setattr__(self, name, value):
    setattr(object.__getattribute__(self, '_joined'), name, value)

  def run(self, bufs, transforms, inputs, prepare_only=False):
    return object.__getattribute__(self, '_joined').run(bufs, transforms, inputs)


class SmallModel:
  """This fork's own ModelState as jetlink's joining model calls it.

  jetlink always calls the small model's run() with a fourth positional,
  `after_enqueue` (the fork's modeld publishes work there while the large model
  answers), and this tree's ModelState takes three. The shim drops it and
  forwards every other read and write, so the joining model cannot tell: it
  compares the small model by identity, sets in_control and frame_drop_ratio on
  it, and reads lat_delay and vision_input_names off it.

  Only ever wrapped around ModelState, and only as the `small` jetlink holds.
  """

  def __init__(self, real):
    object.__setattr__(self, '_real', real)

  def __getattr__(self, name):
    return getattr(object.__getattribute__(self, '_real'), name)

  def __setattr__(self, name, value):
    setattr(object.__getattribute__(self, '_real'), name, value)

  def run(self, bufs, transforms, inputs, after_enqueue=None):
    return object.__getattribute__(self, '_real').run(bufs, transforms, inputs)


class Adapter:
  """jetlink.openpilot.interface.Openpilot over this fork."""

  def __init__(self):
    from jetlink.openpilot.interface import Keys

    from openpilot.common.basedir import BASEDIR
    from openpilot.common.swaglog import cloudlog
    self.keys = Keys(**KEYS._asdict())
    self.log = cloudlog
    self.basedir = Path(BASEDIR)
    # one Params per store: constructing one costs ~144 us on the comma against
    # ~110 us for the read, and the UI reads several five times a second. By
    # store, since a test or a bench runs under its own prefix
    self._stores: dict[Path, object] = {}
    # the camera geometry, resolved once (see camera())
    self._camera_size: tuple[int, int, int, int] | None = None

  # -- params ---------------------------------------------------------------

  def params_dir(self) -> Path:
    return _params_dir()

  def _params(self):
    where = _params_dir()
    store = self._stores.get(where)
    if store is None:
      from openpilot.common.params import Params
      store = self._stores[where] = Params()
    return store

  def get(self, key: str):
    # read from hardwared and the UI's threads, which UnknownKeyName (a
    # params library older than the key) must not take down
    try:
      return self._params().get(key)
    except Exception:
      return None

  def put(self, key: str, value, *, block: bool = False) -> None:
    self._params().put(key, value, block=block)

  def remove(self, key: str) -> None:
    self._params().remove(key)

  # -- the device -----------------------------------------------------------

  def chestnut_present(self) -> bool:
    # this fork's probe is the USB-GPU one; the id it matches is the same
    # ADD1:0001 chestnut, and "an accelerator over USB-C" is the answer that
    # matters: it keeps the link off beside one
    from openpilot.selfdrive.modeld.helpers import usbgpu_present
    return usbgpu_present()

  def camera(self) -> tuple[int, int, int, int]:
    """(cam_w, cam_h, model_w, model_h) for this device: the warp built is the
    one modeld asks for, so this must agree with the vipc stream modeld hands
    to attach().

    The static choice (SConscript's, _ar_ox_fisheye on tici) names comma3's
    1928x1208, but the xiaomi8 runs the IMX363 2x2-binned at 2016x1512. Rather
    than hard-coding either, the geometry whose warp the build actually made
    wins; both are listed so a device on either camera reports correctly.
    Cached: the panels ask at 5 Hz and this is a stat.
    """
    if self._camera_size is None:
      from openpilot.common.hardware import HARDWARE
      from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye
      from openpilot.common.transformations.model import MEDMODEL_INPUT_SIZE
      static = _os_fisheye if HARDWARE.get_device_type() == "mici" else _ar_ox_fisheye
      chosen = (static.width, static.height)
      for w, h in ((2016, 1512), chosen):
        if warp_path(w, h, *MEDMODEL_INPUT_SIZE).is_file():
          chosen = (w, h)
          break
      self._camera_size = (*chosen, *MEDMODEL_INPUT_SIZE)
    return self._camera_size

  def warp_path(self, cam_w: int, cam_h: int, model_w: int, model_h: int) -> Path:
    return warp_path(cam_w, cam_h, model_w, model_h)

  def model_root(self) -> Path:
    from openpilot.common.hardware.hw import Paths
    return Path(Paths.model_root())

  @property
  def catalog_selector(self) -> int:
    """jetlink's own selector, not this fork's REQUIRED_JSON_VERSION.

    `catalog_selector` is the version an entry's `minimum_selector_version` must
    equal to be usable. jetlink is the only consumer, and it uses the value both
    ways: models.catalog filters on it, and registry.catalog.merge_catalogs
    (whose own default is REQUIRED_SELECTOR_VERSION) retypes every entry to it.

    Porting zoompilot's adapter, which returns its fork's REQUIRED_JSON_VERSION,
    was wrong here: this tree is an older sunnypilot whose REQUIRED_JSON_VERSION
    is 16 while jetlink's is 19, so every catalog entry (all stamped 19) failed
    the filter. The catalog then read as empty, default_model() found nothing,
    and provisioning returned "no catalog yet" forever -- the link never joined.

    This fork's model manager does not use the value (it predates the big-model
    slot entirely), so agreeing with jetlink is the only thing that matters.
    """
    from jetlink.registry.catalog import REQUIRED_SELECTOR_VERSION
    return REQUIRED_SELECTOR_VERSION

  # -- modeld ---------------------------------------------------------------

  def model_face(self):
    """comma's large model's face: stock modeld's Parser, constants and action
    function, and modeld_v2's ModelConstants, which modeld_tinygrad reads off
    the model."""
    from jetlink.openpilot.interface import ModelFace

    from openpilot.selfdrive.modeld.constants import ModelConstants
    from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS, LONG_SMOOTH_SECONDS, get_action_from_model
    from openpilot.selfdrive.modeld.parse_model_outputs import Parser
    from openpilot.sunnypilot.modeld_v2.constants import ModelConstants as V2ModelConstants
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
    return ModelFace(parser=Parser, frame_size=lambda w, h: get_nv12_info(w, h)[3], desire_len=ModelConstants.DESIRE_LEN,
                     constants=V2ModelConstants, lat_smooth_seconds=LAT_SMOOTH_SECONDS,
                     long_smooth_seconds=LONG_SMOOTH_SECONDS, get_action_from_model=get_action_from_model)

  def event(self, name: str, **fields) -> None:
    self.log.event(name, **fields)

  # -- the build ------------------------------------------------------------

  def make_warp(self, cam_w: int, cam_h: int, model_w: int, model_h: int):
    # compile_modeld first: it patches tinygrad's firmware fetch as it loads
    from openpilot.selfdrive.modeld.compile_modeld import NV12Frame, make_warp
    from openpilot.selfdrive.modeld.constants import ModelConstants
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
    nv12 = NV12Frame(cam_w, cam_h, *get_nv12_info(cam_w, cam_h))
    # this fork's make_warp takes frame_skip for its signature only; the graph
    # it builds has none of the frame-queue behaviour, so the value is unused
    frame_skip = ModelConstants.MODEL_RUN_FREQ // ModelConstants.MODEL_CONTEXT_FREQ
    return make_warp(nv12, model_w, model_h, frame_skip), nv12.size


# -- what the hooks call ------------------------------------------------------

class _Absent:
  """jetlink's answers when it cannot run here: not present (why is None), or a
  package this build cannot use (why says so, as the offroad alert, to someone
  who turned the link on)."""

  def __init__(self, why: str | None):
    self.why = why

  def enabled(self) -> bool:
    return False

  def status(self):
    return None

  def reason(self) -> str | None:
    if self.why is None:
      return None
    # the setting as jetlink reads it, a file: hardwared asks twice a second
    try:
      on = 0 < int((_params_dir() / KEYS.link).read_bytes()) < len(MODES)
    except (OSError, ValueError):
      on = False
    return self.why if on else None

  def prepare(self) -> bool:
    return False

  def attach(self, small, cam_w: int, cam_h: int):
    return None

  def request_shutdown(self, reason: str = '') -> bool:
    return False

  def shutdown_pending(self) -> bool:
    return False

  def should_extend_catalog(self) -> bool:
    return False

  def extend_catalog(self, catalog: dict) -> dict:
    return catalog

  def model_state(self, ref: str) -> str | None:
    return None


_bound = None
_binding = threading.Lock()


def _api():
  """jetlink for this process, bound to the adapter on first use. Kept, the
  null answers included: Python does not cache a failed import, and searching
  the path again on every UI and hardwared call costs more than the call. One
  per process: prepare() and attach() have to reach the same one."""
  global _bound
  if _bound is None:
    with _binding:
      if _bound is None:
        _bound = _bind()
  return _bound


def _bind():
  try:
    import jetlink
    if getattr(jetlink, '__file__', None) is None:
      return _Absent(None)  # a directory Python takes for a namespace package
    import jetlink.openpilot as jl
  except ModuleNotFoundError as e:
    if e.name == 'jetlink':
      return _Absent(None)  # not dropped in: the link does not exist on this device
    if (e.name or '').startswith('jetlink.'):
      return _unusable("jetlink package too old for this build", e)
    return _unusable(f"jetlink failed to load: {e}", e)
  except Exception as e:
    return _unusable(f"jetlink failed to load: {type(e).__name__}: {e}", e)
  api = getattr(jl, 'API', None)
  if api != API:
    return _unusable(f"jetlink package API {api}, this build expects {API}")
  try:
    return jl.bind(Adapter())
  except Exception as e:
    return _unusable(f"jetlink failed to start: {type(e).__name__}: {e}", e)


def _unusable(why: str, error: Exception | None = None) -> _Absent:
  _log_failure(why, error)
  return _Absent(why)


# manager, hardwared, the model manager and the UI call in here on every
# device, link on or off, and modeld on every drive: whatever jetlink does
# wrong turns the link off and is logged, and never takes one of them down.
# jetlink's own readers never raise; this is the net under that promise.
# Hook -> the failure last logged for it, cleared by a call that works
_failed_hooks: dict[str, str] = {}


def _log_failure(what: str, error: Exception | None) -> None:
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.error("jetlink: %s", what, exc_info=error)
  except Exception:
    pass


def _guarded(default):
  def wrap(hook):
    @functools.wraps(hook)
    def call(*args, **kwargs):
      try:
        result = hook(*args, **kwargs)
      except Exception as e:
        # once per distinct error, as jetlink's readers log: the UI would log
        # a failing status five times a second
        error = f"{type(e).__name__}: {e}"
        if _failed_hooks.get(hook.__name__) != error:
          _failed_hooks[hook.__name__] = error
          _log_failure(f"{hook.__name__}() failed", e)
        return default(*args, **kwargs) if callable(default) else default
      _failed_hooks.pop(hook.__name__, None)
      return result
    return call
  return wrap


@_guarded(False)
def should_run(started: bool, params, CP) -> bool:
  """manager's rule for jetlinkd: run whenever the link is on and no
  accelerator is fitted.

  Not "the cable modes only". Over Wi-Fi the owner holds no gadget (the owner
  releases the link; jetlink.comma.owner), so it looks idle, but it is also the
  only thing that starts the offroad provisioning run (jetlink.openpilot.owner
  spawns `jetlink.openpilot.provision`). That run dials the hotspot itself and
  fetches the picked large model; with no owner there is nobody to spawn it, and
  the link stays on "no large model has been picked yet" forever.
  """
  return _api().enabled()


@_guarded(None)
def status():
  """One snapshot for the UI and the panels (jetlink.openpilot.Status), or
  None when there is no jetlink here."""
  return _api().status()


@_guarded(None)
def reason() -> str | None:
  """Why the link the user turned on cannot run: hardwared's offroad alert.
  Files only, so hardwared can ask twice a second."""
  return _api().reason()


@_guarded(False)
def prepare() -> bool:
  """modeld, before config_realtime_process: will the link join this modeld?
  The GPU's setup has to happen now, or its threads inherit the frame loop's
  realtime priority and core."""
  return _api().prepare()


@_guarded(True)
def in_control(sm) -> bool:
  """modeld, before every frame, onto the model: is openpilot or MADS in
  control? jetlink's large model swaps in only while it is not. A service late
  or invalid counts as in control."""
  # carControlSP only exists on sunnypilot; a stock subscription would not have
  # it, so it is consulted only when present
  names = [n for n in ('carState', 'carControl', 'carControlSP') if n in sm.data]
  if not (sm.all_alive(names) and sm.all_valid(names)):
    return True
  if sm['carControl'].enabled:
    return True
  if 'carControlSP' in sm.data:
    return bool(sm['carControlSP'].mads.enabled)
  return False


@_guarded(None)
def attach(small, cam_w: int, cam_h: int):
  """modeld, once the camera is up and `small` is built: the model to run,
  `small` driving until the link has joined; None unless prepare() said yes.
  Both sides are shimmed for this fork's call shapes (see Joining, SmallModel);
  modeld only reaches here when the link is joining, so the wraps are made once
  and cost the frame loop nothing."""
  joined = _api().attach(SmallModel(small), cam_w, cam_h)
  return None if joined is None else Joining(joined)


@_guarded(False)
def request_shutdown(reason: str = '') -> bool:
  """hardwared, once, when the comma is about to power off for good: ask for
  the far end to go down with it. Returns at once: True when the request now
  waits for jetlinkd, which shutdown_pending() follows."""
  return _api().request_shutdown(reason)


@_guarded(False)
def shutdown_pending() -> bool:
  """hardwared, every loop after request_shutdown(), until it puts DoShutdown:
  has jetlinkd still to take the request? A stat."""
  return _api().shutdown_pending()


@_guarded(None)
def model_state(ref: str) -> str | None:
  """The big-model list, when it opens: 'ready' when the host has built the
  model, 'downloaded' when its file is on the comma, else None."""
  return _api().model_state(ref)


@_guarded(False)
def should_extend_catalog() -> bool:
  """Should the big-model catalog carry the models newer catalogs list?
  Hardware, not the link setting: the model manager drops a pick its catalog
  does not list."""
  return _api().should_extend_catalog()


@_guarded(lambda catalog: catalog)
def extend_catalog(catalog: dict) -> dict:
  """The big-model catalog with those models folded in."""
  return _api().extend_catalog(catalog)


if __name__ == '__main__':
  # manager runs this module as jetlinkd; the process list only starts it for
  # the cable modes (should_run), and it holds the gadget until manager stops it
  main()
