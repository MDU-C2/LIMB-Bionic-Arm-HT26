#!/bin/bash
# Christian och Majd
set -euo pipefail
repository="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$repository/Setup.command" --start "$@"
