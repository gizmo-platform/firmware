#!/bin/sh

echo "Use this to add a secret without risking a git check-in"
echo "secrets.h must NOT exist to prioritize config in config.h"

read -p "Wifi SSID: " SSID
read -p "Wifi PSK: " PSK

cat > secrets.h <<EOF
#define BRI_PRIVATE_WIFI_SSID "$SSID"
#define BRI_PRIVATE_WIFI_PSK "$PSK"
EOF
