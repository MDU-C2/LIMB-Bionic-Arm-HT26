# Tests

Run the software checks from the project environment:

```powershell
python scripts/check_project.py
```

The runner uses separate processes to avoid conflicts between camera and training libraries.
The checks cover recording, EMG filtering and grip labels, sensor mapping,
model loading, simulation, the app and setup. Hardware tests use mock data.

Use the checks in `scripts/` for connected devices. Physical grip accuracy and
sensor placement still need a recording with the electrodes mounted on an arm.

Check code style:

```powershell
python -m pip install -r requirements-dev.txt
python -m ruff check src scripts tests
python -m ruff format --check src scripts tests
```
