#!/usr/bin/env bash
unset DEBUGINFOD_URLS
exec /usr/bin/gdb "$@"
