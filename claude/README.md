# Issue notes

One file per issue that was investigated: what happened, the cause, how it was measured, what was fixed, and what is still
open. `CLAUDE.md` keeps a one-line pointer to each. Add new ones here rather than growing `CLAUDE.md`.

| File | Issue |
|---|---|
| [rosbridge-camera-starvation.md](rosbridge-camera-starvation.md) | Turning over `/cmd_vel` did nothing: fast commands starved by the camera stream in a shared rosbridge (fixed 2026-11-23) |
