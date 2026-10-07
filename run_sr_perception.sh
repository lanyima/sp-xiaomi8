#!/bin/sh
export PYTHONPATH=/data/openpilot
PY_BIN=""
for p in /usr/local/venv/bin/python /data/openpilot/.venv/bin/python /usr/local/pyenv/versions/3.11.4/bin/python3 /usr/local/pyenv/shims/python3 $(which python3 2>/dev/null) /data/data/com.termux/files/usr/bin/python3; do
    if [ -x "$p" ] && "$p" -c "import capnp; import cereal" 2>/dev/null; then
        PY_BIN="$p"
        break
    fi
done
if [ -z "$PY_BIN" ]; then
    if [ -x /usr/local/venv/bin/python ]; then PY_BIN="/usr/local/venv/bin/python"
    elif [ -x /data/data/com.termux/files/usr/bin/python3 ]; then PY_BIN="/data/data/com.termux/files/usr/bin/python3"
    else PY_BIN="python3"
    fi
fi
exec "$PY_BIN" -u /data/openpilot/c2_sr_perception_service.py > /tmp/sr.log 2>&1