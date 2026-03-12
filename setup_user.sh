#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SAMPLE_CONFIG="$SCRIPT_DIR/smart_home_pytree/config/house_info.yaml"
BASE_DIR="$HOME/shr_user"
ENV_FILE="$SCRIPT_DIR/.env"

if [[ ! -f "$SAMPLE_CONFIG" ]]; then
  echo "Sample config not found: $SAMPLE_CONFIG" >&2
  exit 1
fi

read -r -p "Enter SHR user name: " USER_NAME
USER_NAME="${USER_NAME// /}"

if [[ -z "$USER_NAME" ]]; then
  echo "User name cannot be empty." >&2
  exit 1
fi

USER_DIR="$BASE_DIR/$USER_NAME"
CONFIG_DIR="$USER_DIR/configs"
AUDIO_DIR="$USER_DIR/audios"
VIDEO_DIR="$USER_DIR/videos"
IMAGE_DIR="$USER_DIR/images"
LOG_DIR="$USER_DIR/logs"
DB_DIR="$USER_DIR/database"
MAP_DIR="$USER_DIR/map"
HOUSE_CONFIG="$CONFIG_DIR/house_config.yaml"

mkdir -p \
  "$CONFIG_DIR" \
  "$AUDIO_DIR" \
  "$VIDEO_DIR" \
  "$IMAGE_DIR" \
  "$LOG_DIR" \
  "$DB_DIR" \
  "$MAP_DIR"

if [[ ! -f "$HOUSE_CONFIG" ]]; then
  cp "$SAMPLE_CONFIG" "$HOUSE_CONFIG"
  echo "Created config: $HOUSE_CONFIG"
else
  echo "Config already exists, leaving unchanged: $HOUSE_CONFIG"
fi

echo
echo "User data root ready: $USER_DIR"
echo "Subfolders:"
echo "  - configs/"
echo "  - audios/"
echo "  - videos/"
echo "  - images/"
echo "  - logs/"
echo "  - database/"
echo "  - map/"
echo
echo "Add these required lines to ~/.bashrc:"
echo "==============================="
echo 
echo "export SHR_USER_DIR=\"$USER_DIR\""
echo "source $ENV_FILE"
echo
echo "==============================="
echo "Then reload your shell:"
echo "source ~/.bashrc"
