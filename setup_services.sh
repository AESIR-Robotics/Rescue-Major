#!/usr/bin/env bash

 if [ "$EUID" -ne 0 ]; then
  echo "This script must be executed with sudo" >&2
  exit 1
fi

#TODO: install the services to make it work :p


install -m 644 "services/robot@.service" /etc/systemd/system/
install -m 644 "services/robotSystem.target" /etc/systemd/system/