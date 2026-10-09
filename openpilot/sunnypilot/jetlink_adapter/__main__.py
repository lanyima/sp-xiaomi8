"""manager runs this package as jetlinkd: `python -m openpilot.sunnypilot.jetlink_adapter`.

A package is executed through its `__main__.py`, not its `__init__.py`, so the
entry point has to live here for the process_config line to start anything.
"""
from openpilot.sunnypilot.jetlink_adapter import main

main()
