# Visor's Spotter pack

A pack for [Spotter](https://github.com/mikegrundvig/frc-spotter)'s agent on a coprocessor, so the
robot program can see what Visor the board has and put it right as it starts. It ships in each
Visor release (`spotter/visor/` in the archive), and the robot program pushes it to its boards
with its other packs.

| | What |
|---|---|
| `visor.status.version` | The Visor in `/opt/visor`, by its version (`2026.11.4`); empty when there's none |
| `visor.status.config` | The SHA-256 of its configuration, `/opt/visor/visor.toml`; empty when there's none |
| `visor.status.cameras` | The cameras plugged in, by their `/dev/v4l/by-id/` names, comma-separated |
| `visor.log` | Visor's journal (`visor.service`) |
| `visor.install` | Takes a release archive, `visor-<version>-linux-arm64.tar.gz`, as its input: unpacks it beside the running program, has the new program say its version, then moves it into place and restarts Visor. One that won't run on the board leaves it as it was |
| `visor.configure` | Takes a configuration as its input: Visor checks it (`visor check`), and only then is it moved into place and Visor restarted. One Visor refuses leaves the board as it was, and the response says why |

Both actions run only while the robot is disabled: the manager refuses them otherwise.

## What the board's image gives it

The agent runs as its own account, which may do almost nothing by default. The image grants this
pack what it needs, and nothing more:

- **Visor's systemd unit,** enabled: Visor's `deploy/visor.service`, which runs `/opt/visor/visor`
  as the `visor` user and keeps trying until the first install puts it there.
- **`/opt/visor`, writable by the agent:** owned by its account (`frc-spotter`), and kept across
  boots on a read-only root. A drop-in for its unit,
  `/etc/systemd/system/frc-spotter.service.d/visor.conf`:

  ```ini
  [Service]
  ReadWritePaths=/opt/visor
  ```

- **Restarting Visor, and only that:** a polkit rule, sorting before Spotter's own
  `99-frc-spotter.rules`, as `/etc/polkit-1/rules.d/50-frc-spotter-visor.rules`:

  ```js
  polkit.addRule(function (action, subject) {
    if (action.id == "org.freedesktop.systemd1.manage-units" &&
        action.lookup("unit") == "visor.service" &&
        action.lookup("verb") == "restart" &&
        subject.user == "frc-spotter") {
      return polkit.Result.YES;
    }
  });
  ```

- **The cameras' names:** `/dev/v4l/by-id/` is readable by anyone, so listing them needs nothing.

`VISOR_DIR`, `VISOR_DEVICES` and `VISOR_RESTART`, in the agent's environment, change where it looks
and how it restarts Visor, for an image laid out otherwise (and the tests). `journal` is Spotter's
catalog's, under its MIT License.
