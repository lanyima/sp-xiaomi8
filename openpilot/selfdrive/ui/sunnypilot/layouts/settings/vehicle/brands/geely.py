"""
xiaomi8 port: Geely (吉利) brand settings.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp


RADAR_FALLBACK_DESCRIPTION = tr_noop(
  "Use a stable, in-lane Geely ARS410 radar target when the vision lead is weak. "
  "Only applies to Geely. Keep disabled unless the radar installation and lane matching have been verified."
)


class GeelySettings(BrandSettings):
  def __init__(self):
    super().__init__()
    self.radar_only_fallback = toggle_item_sp(
      lambda: tr("Radar Lead Fallback (Beta)"),
      description=lambda: tr(RADAR_FALLBACK_DESCRIPTION),
      param="GeelyRadarOnlyFallback",
    )
    self.items = [self.radar_only_fallback]
