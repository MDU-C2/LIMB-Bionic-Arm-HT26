# Tests

Run each test module in a separate process from the project environment:

```powershell
python -m compileall -q src
Get-ChildItem tests/test_*.py | ForEach-Object {
    python -m unittest discover -s tests -p $_.Name
    if ($LASTEXITCODE -ne 0) { throw "Failed: $($_.Name)" }
}
```

Separate processes avoid conflicts between the camera and training libraries.
The checks cover recording, EMG filtering and grip labels, sensor mapping,
model loading, simulation, the app and setup. Hardware tests use mock data.

Use the checks in `scripts/` for connected devices. Physical grip accuracy and
sensor placement still need a recording with the electrodes mounted on an arm.
