#!/bin/sh
# Make the shared key that boards and box authenticate each other with.
# Writes firmware/main/secret.h and box/secret.key; both are git-ignored.
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)
key=$(od -An -N32 -tx1 /dev/urandom | tr -d ' \n')
printf '%s\n' "$key" > "$root/box/secret.key"
chmod 600 "$root/box/secret.key"
printf '#define TAPLOC_KEY { %s }\n' "$(printf '%s' "$key" | sed 's/../0x&, /g')" > "$root/firmware/main/secret.h"
echo "Key written. For 'wendy run' on the box: export TAPLOC_KEY=\$(cat box/secret.key)"
