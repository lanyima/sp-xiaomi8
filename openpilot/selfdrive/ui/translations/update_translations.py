#!/usr/bin/env python3
from itertools import chain
import os
from openpilot.common.basedir import BASEDIR
from openpilot.system.ui.lib.multilang import SYSTEM_UI_DIR, UI_DIR, TRANSLATIONS_DIR, multilang
from openpilot.selfdrive.ui.translations.potools import extract_strings, generate_pot, merge_po, init_po

LANGUAGES_FILE = os.path.join(str(TRANSLATIONS_DIR), "languages.json")
POT_FILE = os.path.join(str(TRANSLATIONS_DIR), "app.pot")


def update_translations():
  files = []
  # xiaomi8: 加入 sunnypilot 专属 UI 目录, 否则这些 tr() 字符串不进 .po → 中文界面显示英文
  for root, _, filenames in chain(os.walk(SYSTEM_UI_DIR),
                                  os.walk(os.path.join(str(UI_DIR), "widgets")),
                                  os.walk(os.path.join(str(UI_DIR), "layouts")),
                                  os.walk(os.path.join(str(UI_DIR), "onroad")),
                                  os.walk(os.path.join(str(UI_DIR), "sunnypilot")),
                                  os.walk(os.path.join(str(SYSTEM_UI_DIR), "sunnypilot"))):
    for filename in filenames:
      if filename.endswith(".py"):
        files.append(os.path.relpath(os.path.join(root, filename), BASEDIR))

  # Extract translatable strings and generate .pot template
  entries = extract_strings(files, BASEDIR)
  generate_pot(entries, POT_FILE)

  # Generate/update translation files for each language
  for name in multilang.languages.values():
    po_file = os.path.join(str(TRANSLATIONS_DIR), f"app_{name}.po")
    if os.path.exists(po_file):
      merge_po(po_file, POT_FILE)
    else:
      init_po(POT_FILE, po_file, name)


if __name__ == "__main__":
  update_translations()
