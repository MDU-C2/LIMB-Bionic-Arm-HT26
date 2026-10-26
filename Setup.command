#!/bin/bash
# Majd
set -euo pipefail

repository="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
state_directory="$repository/.aurora"
micromamba="$state_directory/bin/micromamba-2.3.3"
prefix="$state_directory/env"
export MAMBA_ROOT_PREFIX="$state_directory/mamba"
export PYTHONUTF8=1
export PYTHONIOENCODING=utf-8

failed() {
    echo "Setup failed. Check the error above, internet connection, and disk space."
    if [[ -t 0 ]]; then read -r -p 'Press Enter to close...' || true; fi
}
trap failed ERR

launch=true
start=false
for argument in "$@"; do
    case "$argument" in
        --no-launch) launch=false ;;
        --start) start=true ;;
        *) echo "Unknown option: $argument"; exit 1 ;;
    esac
done

case "$(uname -s):$(uname -m)" in
    Darwin:x86_64) platform=osx-64 ;;
    Darwin:arm64) platform=osx-arm64 ;;
    Linux:x86_64) platform=linux-64 ;;
    *) echo 'Supported: Intel/Apple Silicon macOS and Intel/AMD Linux desktops.'; exit 1 ;;
esac

cd -- "$repository"
if $start; then
    trap - ERR
    if [[ ! -x "$prefix/bin/python" ]]; then
        echo 'Run Setup.command once before using Start.command.'
        exit 1
    fi
    exec "$prefix/bin/python" scripts/setup_project.py --launch
fi

mkdir -p -- "$state_directory"
if [[ ! -x "$micromamba" ]]; then
    for utility in curl tar bzip2; do
        command -v "$utility" >/dev/null || { echo "Required utility missing: $utility"; exit 1; }
    done
    echo 'Downloading the local Python environment manager...'
    curl --fail --location --retry 3 "https://micro.mamba.pm/api/micromamba/$platform/2.3.3" \
        --output "$state_directory/micromamba.tar.bz2"
    tar -xjf "$state_directory/micromamba.tar.bz2" -C "$state_directory" bin/micromamba
    mv -- "$state_directory/bin/micromamba" "$micromamba"
    rm -f -- "$state_directory/micromamba.tar.bz2"
fi

echo 'Installing Python, GUI, simulation, camera, and sensor dependencies...'
operation=create
if [[ -f "$prefix/conda-meta/history" ]]; then operation=install; fi
"$micromamba" --no-rc "$operation" -y -p "$prefix" -f environment.yml
arguments=(--install)
if $launch; then arguments+=(--launch); fi
"$micromamba" --no-rc run -p "$prefix" python scripts/setup_project.py "${arguments[@]}"
