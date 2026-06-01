#!/bin/bash

usermod -aG netdev $(whoami)

cat << CAT_START > /etc/polkit-1/localauthority/50-local.d/org.freedesktop.NetworkManager.pkla

[nm-applet]
Identity=unix-group:netdev
Action=org.freedesktop.NetworkManager.*
ResultAny=yes
ResultInactive=no
ResultActive=yes

CAT_START


systemctl restart NetworkManager