# Visor's Spotter pack

A pack for [Spotter](https://github.com/mikegrundvig/frc-spotter)'s agent on a coprocessor, so the
robot program can see what Visor the board has and put it right as it starts. It ships in each
Visor release (`spotter/visor/` in the archive), and the robot program pushes it to its boards
with its other packs.

| | What |
|---|---|
| `visor.status.version` | The Visor in `/opt/visor`, by its version (`2026.11.4`); empty when there's none |
| `visor.status.config` | The SHA-256 of its configuration, `/opt/visor/visor.toml`; empty when there's none |
| `visor.status.running-version` | The version of the Visor running, as it said when it started (in `/run/visor/running`, believed only from the unit's main process); empty while none runs. Unlike `version` while a program put in place hasn't been started (a restart that failed) |
| `visor.status.running-config` | The SHA-256 of the configuration it runs, as `config` gives it; empty while none runs |
| `visor.status.cameras` | The cameras plugged in, comma-separated, each once by the name the installed program gives it (`visor cameras --names`, so Visor alone has the rule): its `/dev/v4l/by-id/` name when it has a USB serial of its own; with no serial, always its `/dev/v4l/by-path/` name, its USB port's (label the ports and cables). Empty when there's no program installed that can say |
| `visor.log` | Visor's journal (`visor.service`) |
| `visor.install` | Takes a release archive, `visor-<version>-linux-arm64.tar.gz`, as its input: refuses one holding anything but files and folders (a link, a device) before anything in it runs, unpacks it beside the running program, has the new program say its version, then moves it into place and restarts Visor. One that won't run on the board leaves it as it was. A restart that fails after the move fails the run, and `running-version` then shows what runs |
| `visor.configure` | Takes a configuration as its input: Visor checks it (`visor check`), and only then is it moved into place and Visor restarted. One Visor refuses leaves the board as it was, and the response says why |

Both actions run only while the robot is disabled: the manager refuses them otherwise.

## What the board's image gives it

The agent runs as its own account, which may do almost nothing by default. What this pack needs of
the board ships in each release, under `os/`, at the paths a package's own files take, and the
board's image installs that folder at its root (copied as it is, then `systemd-sysusers visor.conf`,
`systemd-tmpfiles --create visor.conf`, and `systemctl enable visor.service`):

| In `os/` | From | What it gives |
|---|---|---|
| `usr/lib/sysusers.d/visor.conf` | `deploy/visor.sysusers` | The `visor` account, in the `video` group, which Visor's unit runs as |
| `usr/lib/tmpfiles.d/visor.conf` | `deploy/visor.tmpfiles` | `/opt/visor`, the agent's account's (`frc-spotter`) to write, readable by anyone |
| `usr/lib/systemd/system/visor.service` | `deploy/visor.service` | Visor's unit, which runs `/opt/visor/visor` as `visor` once the robot has configured it |
| `usr/lib/systemd/system/frc-spotter.service.d/visor.conf` | `deploy/frc-spotter-visor.conf` | The agent may write `/opt/visor` (`ReadWritePaths=`) |
| `usr/share/polkit-1/rules.d/50-frc-spotter-visor.rules` | `deploy/50-frc-spotter-visor.rules` | The agent may restart `visor.service`, and nothing more: it sorts before Spotter's own `99-frc-spotter.rules` |

What stays the image's own is the board's: keeping `/opt/visor` across boots on a read-only root,
and which cores Visor runs on (a drop-in for `visor.service` with `CPUAffinity=`). A drop-in in the
board's `/etc` names its file differently from the release's, or it would replace that one.

Two things need nothing granted:

- **What's running:** Visor's unit makes `/run/visor` (`RuntimeDirectory=`), readable by anyone,
  and its main process ID is anyone's to read (`systemctl show`). The unit comes with the image
  (the release's `os/`); an install changes only the program, so a board whose unit has no
  `RuntimeDirectory=` never reports a running Visor until it's given a newer image.
- **The cameras' names:** `/dev/v4l/by-id/`, `/dev/v4l/by-path/` and the cameras' serials and
  indexes under `/sys/class/video4linux/` are readable by anyone, so listing them needs nothing:
  `visor cameras --names` opens no camera.

`VISOR_DIR`, `VISOR_RUN`, `VISOR_MAIN_PID`, `VISOR_BY_ID`, `VISOR_BY_PATH`, `VISOR_SYSFS` and
`VISOR_RESTART`, in the agent's environment, change where it looks
and how it restarts Visor, for an image laid out otherwise (and the tests). `journal` is Spotter's
catalog's, under its MIT License, keeping journalctl's exit status: a journal it can't read, or a
cursor it refuses, fails the page rather than returning an empty one. It's a copy because a pack
runs only its own files.
