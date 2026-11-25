# MuJoCo sensor handoff smoke test

Ran on the NYUAD server with MuJoCo 3.17.0, Python 3.10.22, and EGL. Three seeds covered red apple to green tray and blue block to yellow tray. All 6 episodes produced an oracle RGB-D localization and manipulation handoff. The mean surface-point distance to the simulated object center was 3.73 cm and the maximum was 4.24 cm. Recomputed all six reported positions from their archived RGB-D files and verified the recorded initial state and replay array dimensions. Observed both robot models in the browser replay.

Raw reports and recordings are in sim_workspace/runs/archive/suite-smoke. The grasp and placement stage is pending.
