The `dum_e_control/core` solver, command-state model and JSON calibration were
copied from `Code/arm_interface` in ArtemisFTW/Dum-E-Robotic-Arm-v0 at commit
67cadab (the full commit is available in the local Git history). Only the
`model.py` import was made package-relative. This preserves the Windows
controller's calibration and numerical checks without importing its UI.

The upstream repository does not supply a license file; ROS package metadata
therefore records the license as `Unspecified`.

The ROS GUI's `gui_app.py` and `gui_pose_loop.py` are adapted copies of the same
commit's `app.py` and `pose_loop.py`. Layout, controls and loop behavior are
preserved. Imports use ROS-backed model/transport adapters, status text identifies
ROS/dry-run operation, Linux scrolling is supported, and the port selector can
configure the serial node. The original Windows files are unchanged.
