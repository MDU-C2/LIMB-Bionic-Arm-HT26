# Tests

Run the software checks from the project environment:

```powershell
python scripts/check_project.py
```

The runner uses separate processes to avoid conflicts between camera and training libraries.
The checks cover recording, EMG filtering and grip labels, sensor mapping,
model loading, simulation, the app and setup. Device tests use generated inputs
and connection doubles; they do not validate connected hardware.
EMG checks include participant separation, causal feature parity, model export,
metadata validation and interrupted runs. Fusion checks use synthetic motion.

The default `runtime` suite includes LDA learning checks but excludes neural
training jobs. Use `--suite training` after installing `requirements-training.txt`,
or `--suite all` to include both. Missing training prerequisites are reported
separately. Package/resource checks use `python scripts/check_package.py`.

`test_gui_workflows.py` runs native desktop forms through actual offline
processing, plotting, export, replay, LDA training, model selection, errors and
cancellation. It uses temporary paths with spaces and non-ASCII characters.
Native actions skip explicitly when Tk cannot open a display; run that module
on a desktop to include them. Export and form validation checks also run headlessly.

Use the checks in `scripts/` for connected devices. Physical grip accuracy and
sensor placement still need a recording with the electrodes mounted on an arm.

Check code style:

```powershell
python -m pip install -r requirements-dev.txt
python -m ruff check src scripts tests
python -m ruff format --check src scripts tests
```
