# OAK-D Lite camera

Install the camera dependencies and open the RGB preview from the repository
root:

```powershell
python -m pip install -r src/camera/requirements.txt
python src/camera/oak_preview.py
```

Press `Q` or `Esc` to close the preview. The script only reads camera frames;
it does not command the robot.

## Record movement training data

Use a new numeric user ID and choose which arm is visible:

```powershell
python src/camera/record_movement.py --user-id 6 --side right
```

Press `Space` to start and `Space` again to save one sequence. Record at least
four sequences per new user; 15–20 is recommended. The files are saved directly
in `src/ml/data/training`, where `train_gru.py` reads them. These landmark files
are biometric movement data, so review them before publishing the repository.
