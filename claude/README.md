# Notes

Topic notes (rules, references) and one file per investigated issue (what happened, the cause, how it was measured, what was
fixed, and what is still open). `CLAUDE.md` keeps a one-line pointer to each. Add new ones here rather than growing `CLAUDE.md`.

| File | Topic |
|---|---|
| [rosbridge-camera-starvation.md](rosbridge-camera-starvation.md) | Turning over `/cmd_vel` did nothing: fast commands starved by the camera stream in a shared rosbridge (fixed 2026-11-23); also joystick-rate floods causing "phantom" driving (2026-11-25) |
| [urc-rules.md](urc-rules.md) | URC 2027 rules: where the PDF is, summary of all missions, the astronaut sub-mission (Follow/Stay/Fetch/Come/Give, the four command methods and their points), what UrcAutonomySite models, notes for a playable astronaut |
