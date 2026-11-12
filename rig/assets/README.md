# Drop the models here

Everything in this folder except this file and the `.gitkeep` files is ignored by Git.
Copy Haichen's files in under these names (the second column is the original name):

| Put it at | Original file | Required |
| --- | --- | --- |
| `human/pose.pt` | `yolo11s-pose.pt` (a fresh Ultralytics download if the supplied copy is incomplete) | yes |
| `human/cap.pt` | cap detector `best.pt` | yes |
| `human/camera_matrix.npy` | human camera's `camera_matrix.npy` | yes |
| `human/distortion_coefficients.npy` | human camera's `distortion_coefficients.npy` | yes |
| `robot/robot.pt` | `robot_test.pt` (OBB robot model) | yes |
| `robot/camera_matrix.npy` | robot camera's `media/camera_matrix.npy` | yes |
| `robot/distortion_coefficients.npy` | robot camera's `media/distortion_coefficients.npy` | yes |
| `robot/toy.pt` | `best (4).pt` (toy OBB model) | no, the toy is skipped without it |

The two cameras have different lenses/positions, so each needs **its own** calibration pair.
Check the folder at any time with `bash scripts/run_rig.sh --check`.
