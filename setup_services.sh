#!/usr/bin/env bash

 if [ "$EUID" -ne 0 ]; then
  echo "This script must be executed with sudo" >&2
  exit 1
fi

#TODO: install the services to make it work :p


# install -m 644 "$REMINDERS_DIR/mcu-monitor@.service" /etc/systemd/system/