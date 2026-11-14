#!/bin/sh
# Shared key nodes, dongle and Anchor authenticate each other with (scheme from
# esp-pingpong tools/genkey.sh). Writes main/secret.h and the Anchor's
# secret.key; both are git-ignored.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
anchor=${1:-"$here/../anchor"}
key=$(od -An -N32 -tx1 /dev/urandom | tr -d ' \n')
printf '%s\n' "$key" > "$anchor/secret.key"
chmod 600 "$anchor/secret.key"
printf '#define SOUNDSYNC_KEY { %s }\n' "$(printf '%s' "$key" | sed 's/../0x&, /g')" > "$here/main/secret.h"
echo "key written to main/secret.h and $anchor/secret.key"
