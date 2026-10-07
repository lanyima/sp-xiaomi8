# Xiaomi 8 branch installer

`branch_installer` is a statically linked aarch64 ELF that clones a custom
openpilot branch onto the device. It is what the AGNOS setup UI calls
"custom software": the setup downloads this URL, checks the first four bytes
are ELF, then executes it.

## Why it must be hosted as a raw file

`system/ui/tici_setup.py` does:

```python
with urllib.request.urlopen(req, timeout=30) as response:
    ...
is_elf = f.read(4) == b'\x7fELF'
if not is_elf:
    self.download_failed(url, "No custom software found at this URL.")
```

So the URL must return the **binary**, not an HTML page. Hence
`raw.githubusercontent.com` (serves `application/octet-stream`,
byte-for-byte) rather than the GitHub web page.

## Usage

Enter this URL in the setup UI (Enter URL → for Custom Software):

```
https://raw.githubusercontent.com/lanyima/sp-xiaomi8/main/branch_installer?repo=https://github.com/lanyima/sp-xiaomi8.git
```

Optional query parameters (parsed from the URL by the installer itself):

| param   | meaning                          | default |
|---------|----------------------------------|---------|
| `repo`  | git URL to clone                 | commaai/openpilot |
| `branch`| branch/tag to check out          | master  |
| `depth` | shallow clone depth (`--depth=N`)| full    |
| `clean` | `1` = delete existing tree first | `0` (backup instead) |

Example with a branch and shallow clone:

```
.../branch_installer?repo=https://github.com/lanyima/sp-xiaomi8.git&branch=main&depth=1
```

## Notes

- Never passes `--shallow-submodules`: some upstream msgq/opendbc commits are
  absent from a shallow history, which makes git report success while the
  checkout silently fails.
- Backs up `/data/openpilot` to `/data/openpilot.bak` (or deletes it when
  `clean=1`) and rolls back on any failure.
- Logs to `/data/installer.log`.
- Keeps an existing `/data/continue.sh` (it holds hardware init); falls back
  to `/data/.continue.sh.full`, then to a minimal embedded one.
