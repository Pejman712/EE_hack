# Publishing this repo as the `go2-foxglove` WendyOS template

This repo follows the conventions of
[`wendylabsinc/templates`](https://github.com/wendylabsinc/templates), which
already ships a 2-service `python/go2-foxglove` (bridge + camera). This repo is
the **full-stack expansion** of it — adding `ros2`, `recorder`, `sit_stand`,
`slam` and `nav2` — and is **two things at once**:

1. **A deployable WendyOS app** — the repo root uses real default values
   (`192.168.123.161`, `192.168.123.18`, `8765`, `appId: go2-foxglove`) and a
   complete [`wendy.json`](wendy.json), so you can clone it and `wendy run`.
2. **The source for the registry's `go2-foxglove` template** — scaffolded with
   `wendy init --template go2-foxglove --language python`.

A registry template's files contain Go `text/template` placeholders
(`{{.GO2_IP}}` etc.), so the template-source form is **not** directly runnable.
To keep the repo runnable *and* produce the template, the placeholdered form is
**generated** rather than committed:

```bash
scripts/make-template.sh        # writes dist/templates/python/go2-foxglove/ (gitignored)
```

## How `wendylabsinc/templates` is structured

```
templates/
  meta.json                       # registry INDEX: { templates: [...], languages: [...] }
  python/
    go2-foxglove/
      template.json               # this template's variables (per language)
      wendy.json                  # appId: "{{.APP_ID}}"
      README.md  foxglove-layout.json
      bridge/ camera/ ros2/ ...    # source files with {{.VAR}} placeholders
  swift/ rust/ node/ cpp/ ...
```

There is **one** top-level `meta.json` (a flat index of every template), and each
template is a `<language>/<name>/` directory whose only manifest is a
**`template.json`**. (There is no per-template `meta.json`.)

This repo mirrors that:

| This repo                  | Registry equivalent                                |
|----------------------------|----------------------------------------------------|
| `template.json`            | `python/go2-foxglove/template.json`                |
| `wendy.json` (+ services)  | `python/go2-foxglove/wendy.json` (+ services)      |
| `meta-entry.json`          | the `templates[]` entry to add/refresh in top `meta.json` |
| `scripts/make-template.sh` | tooling (not shipped in the registry)              |

## To publish / update the registry

```bash
scripts/make-template.sh
# 1. copy the payload into the registry's python/ dir:
cp -r dist/templates/python/go2-foxglove <path-to>/templates/python/
# 2. add or refresh the go2-foxglove object in the registry's top-level meta.json
#    using meta-entry.json from this repo.
```

## Variables (`template.json`)

| Variable          | Type    | Default           | Rendered into                                  |
|-------------------|---------|-------------------|------------------------------------------------|
| `APP_ID`          | string  | (required)        | `wendy.json` `appId`                           |
| `FOXGLOVE_PORT`   | integer | `8765`            | `bridge/Dockerfile` (`ENV` + `EXPOSE`)         |
| `GO2_IP`          | string  | `192.168.123.161` | `camera/recorder/slam/nav2` `Dockerfile` `ENV` |
| `GO2_DDS_ADDRESS` | string  | `192.168.123.18`  | `bridge/ros2/sit_stand` `cyclonedds.xml`       |
| `RECORDER_PORT`   | integer | `7000`            | `recorder/Dockerfile` (`ENV` + `EXPOSE`)       |
| `NAV2_PORT`       | integer | `7100`            | `nav2/Dockerfile` (`ENV` + `EXPOSE`)           |

Following the registry's convention, only the Dockerfile `ENV`/`EXPOSE`
declarations and `cyclonedds.xml` carry placeholders; code and entrypoint
fallback defaults stay literal (the `ENV` value overrides them at runtime).
Rendering the payload with the default answers reproduces the deployable repo
(verified by `scripts/make-template.sh` + a render/validate of `wendy.json`).

## Service topology (why `wendy.json` is shaped this way)

- `bridge`, `camera` — non-ROS, `network: host`. `camera` `dependsOn: ["bridge"]`
  (it forwards JPEG frames to the bridge's localhost ingest).
- `ros2`, `sit_stand`, `nav2` — `frameworks.ros2` (`domainId: 0`,
  `rmw_cyclonedds_cpp`, `humble`). Their entrypoints `export ROS_LOCALHOST_ONLY=0`
  to undo WendyOS's loopback-only injection so DDS reaches the robot on the LAN.
- `recorder`, `slam` — **plain `network: host`, not `frameworks.ros2`**. They
  self-bind DDS in their entrypoints and do *not* undo the loopback injection, so
  declaring them as `frameworks.ros2` would let WendyOS force
  `ROS_LOCALHOST_ONLY=1` and break their path to the robot. `recorder` also has a
  `persist` entitlement for rosbags at `/data`.

> Note: no existing registry template uses `frameworks.ros2` (e.g. `go2-rosbag`'s
> recorder is plain `network: host`); these three services are the first to use
> it. That is correct for them — verify on a live device when publishing.
