# Majd
"""Keep installed resources separate from writable session outputs."""

from pathlib import Path

PACKAGE_DIRECTORY = Path(__file__).resolve().parent
_checkout = PACKAGE_DIRECTORY.parent.parent
CHECKOUT_DIRECTORY = (
    _checkout
    if (_checkout / "pyproject.toml").is_file()
    and (_checkout / "src" / "aurora").resolve() == PACKAGE_DIRECTORY
    else None
)
REPOSITORY_ROOT = CHECKOUT_DIRECTORY or Path.cwd()
RESOURCE_DIRECTORY = CHECKOUT_DIRECTORY or PACKAGE_DIRECTORY / "resources"
MODEL_DIRECTORY = RESOURCE_DIRECTORY / "artifacts" / "models"
RECORDINGS_DIRECTORY = REPOSITORY_ROOT / "outputs" / "recordings"
