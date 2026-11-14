#!/bin/bash
# Build the ranging firmware and push it to one board, from WSL.
#   ./push.sh
# `wendy` shows a picker: choose the board and press Enter. Run it once per
# board. (wendy does not accept /dev/ttyACM0 or a board name in --device for USB
# boards; the picker is how it selects them.)
# Copies the project into WSL's own disk first: ESP-IDF builds far faster there
# than on /mnt/c.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
work="$HOME/pingpong-range"
mkdir -p "$work"
rsync -a --delete --exclude build --exclude sdkconfig --exclude sdkconfig.old \
      --exclude managed_components --exclude .cache "$here"/ "$work"/
cd "$work"
wendy run --detach
echo "Pushed. Watch it with: wendy device logs   (pick the board)"
