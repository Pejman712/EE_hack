# Publishing this repo as the `go2-foxglove` WendyOS template

This repo is **two things at once**:

1. **A deployable WendyOS app** — the repo root has real default values
   (`192.168.123.161`, `192.168.123.18`, `8765`, `appId: go2-foxglove`) and a
   complete [`wendy.json`](wendy.json) declaring all seven services, so you can
   clone it and `wendy run` directly.
2. **The source for the `go2-foxglove` template** in the WendyOS templates
   registry, scaffolded with `wendy init --template go2-foxglove`.

A registry template's source files contain Go `text/template` placeholders
(`{{.GO2_IP}}` etc.), so a placeholdered tree is **not** directly runnable. To
keep the repo runnable *and* produce the template, the placeholdered form is
**generated** rather than committed:

```bash
scripts/make-template.sh
```

writes the registry payload to `dist/templates/go2-foxglove/` (gitignored).

## Layout produced

```
dist/templates/go2-foxglove/
  meta.json          # interactive wizard (phases → questions); see below
  template.json      # python language variant: variables + defaults
  python/            # the app source with {{.VAR}} placeholders applied
    wendy.json  bridge/  camera/  ros2/  recorder/  sit_stand/  slam/  nav2/  ...
```

To publish, copy `dist/templates/go2-foxglove/` into the templates registry repo
(`wendy init --template <name> --branch <branch>` fetches
`<owner>/<repo>/<ref>/<name>/…`).

## Variables

Declared in [`meta.json`](meta.json) (the wizard) and [`template.json`](template.json)
(the python variant). Every question `id` is available as `{{.ID}}` in any
rendered file.

| Variable          | Default           | Where it lands                                              |
|-------------------|-------------------|------------------------------------------------------------|
| `APP_ID`          | `go2viz`          | `wendy.json` `appId`                                        |
| `GO2_IP`          | `192.168.123.161` | camera WebRTC + ROS 2 NIC selection (Dockerfiles, entrypoints) |
| `GO2_DDS_ADDRESS` | `192.168.123.18`  | `bridge/`, `ros2/`, `sit_stand/` `cyclonedds.xml`          |
| `FOXGLOVE_PORT`   | `8765`            | `bridge/Dockerfile` + `bridge/app.py`                      |

Rendering the payload with the default answers reproduces the deployable repo
byte-for-byte (verified by `scripts/make-template.sh` + a render/diff against the
repo sources).

> **Best-effort caveat.** The exact `meta.json` / `template.json` schema
> (`https://wendy.sh/schemas/template.schema.json`) and the registry repo are not
> publicly reachable from this environment; these manifests were authored from
> the field names and authoring docs embedded in the `wendy` CLI binary
> (`phases`/`questions`/`options`, question `type`s `input`/`radio`/`checkbox`,
> Go `text/template` substitution, `UPPER_SNAKE_CASE` convention). Validate them
> against the live registry before relying on the wizard, and adjust field names
> if the schema differs.

## Service topology (why the `wendy.json` is shaped this way)

- `bridge`, `camera` — non-ROS, `network: host`.
- `ros2`, `sit_stand`, `nav2` — `frameworks.ros2` (`domainId: 0`,
  `rmw_cyclonedds_cpp`, `humble`). Their entrypoints `export ROS_LOCALHOST_ONLY=0`
  to undo WendyOS's loopback-only injection so DDS reaches the robot on the LAN.
- `recorder`, `slam` — **plain `network: host`, not `frameworks.ros2`**. They
  self-bind DDS in their entrypoints and do *not* undo the loopback injection, so
  declaring them as `frameworks.ros2` would let WendyOS force
  `ROS_LOCALHOST_ONLY=1` and break their path to the robot. `recorder` also has a
  `persist` entitlement for rosbags at `/data`.
