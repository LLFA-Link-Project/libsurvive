#!/bin/sh
# Run in the foreground; Ctrl-C stops the HTTP server.
set -eu

case "${1-}" in
    -h|--help)
        cat <<'HELP'
Usage: tools/lighthouse-viewer.sh [PORT] [viewer options]

Starts the Lighthouse viewer in the foreground, accessible from the LAN.
Default port: 8765. Stop with Ctrl-C.

Examples:
  ./tools/lighthouse-viewer.sh 8080
  ./tools/lighthouse-viewer.sh 8080 --dumps /path/to/LH-Dump
  ./tools/lighthouse-viewer.sh 8080 --host 127.0.0.1

For all viewer options: python3 tools/lighthouse_viewer.py --help
HELP
        exit 0
        ;;
    ''|--*) viewer_port=8765 ;;
    *) viewer_port=$1; shift ;;
esac

case "$viewer_port" in
    ''|*[!0-9]*)
        printf '%s\n' 'Port must be an integer from 1 to 65535.' >&2
        exit 2
        ;;
esac
if ! [ "$viewer_port" -ge 1 ] 2>/dev/null || ! [ "$viewer_port" -le 65535 ] 2>/dev/null; then
    printf '%s\n' 'Port must be an integer from 1 to 65535.' >&2
    exit 2
fi

viewer_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec python3 "$viewer_dir/lighthouse_viewer.py" --host 0.0.0.0 --port "$viewer_port" "$@"
