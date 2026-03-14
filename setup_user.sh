#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SAMPLE_CONFIG="$SCRIPT_DIR/smart_home_pytree/config/house_config_example.yaml"
BASE_DIR="$HOME/shr_user"
ENV_FILE="$SCRIPT_DIR/.env"
BASHRC_FILE="$HOME/.bashrc"

if [[ ! -f "$SAMPLE_CONFIG" ]]; then
  echo "Sample config not found: $SAMPLE_CONFIG" >&2
  exit 1
fi

append_line_if_missing() {
  local file_path="$1"
  local line="$2"

  touch "$file_path"
  if grep -Fxq "$line" "$file_path"; then
    echo "Already present in $(basename "$file_path"): $line"
  else
    printf '\n%s\n' "$line" >> "$file_path"
    echo "Added to $(basename "$file_path"): $line"
  fi
}

ensure_shr_user_dir_line() {
  local file_path="$1"
  local desired_line="$2"
  local existing_line

  touch "$file_path"
  existing_line="$(grep -E '^export SHR_USER_DIR=' "$file_path" | head -n 1 || true)"

  if [[ -z "$existing_line" ]]; then
    printf '\n%s\n' "$desired_line" >> "$file_path"
    echo "Added to $(basename "$file_path"): $desired_line"
    return
  fi

  if [[ "$existing_line" == "$desired_line" ]]; then
    echo "Already present in $(basename "$file_path"): $desired_line"
  else
    sed -i "0,/^export SHR_USER_DIR=.*/s|^export SHR_USER_DIR=.*|$desired_line|" "$file_path"
    echo "Updated in $(basename "$file_path"): $desired_line"
  fi

  awk '
    BEGIN { seen = 0 }
    /^export SHR_USER_DIR=/ {
      seen++
      if (seen > 1) next
    }
    { print }
  ' "$file_path" > "${file_path}.tmp" && mv "${file_path}.tmp" "$file_path"
}

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
USER_ALREADY_EXISTS=0

if [[ -d "$USER_DIR" ]]; then
  USER_ALREADY_EXISTS=1
fi

mkdir -p \
  "$CONFIG_DIR" \
  "$AUDIO_DIR" \
  "$VIDEO_DIR" \
  "$IMAGE_DIR" \
  "$LOG_DIR" \
  "$DB_DIR" \
  "$MAP_DIR"

if [[ "$USER_ALREADY_EXISTS" -eq 1 ]]; then
  echo "User already exists: $USER_DIR"
  echo "Choose an option:"
  echo "  1. Reset house_config with example config file"
  echo "  2. Do Nothing"

  while true; do
    read -r -p "Enter choice [1-2]: " EXISTING_USER_CHOICE
    case "$EXISTING_USER_CHOICE" in
      1)
        cp "$SAMPLE_CONFIG" "$HOUSE_CONFIG"
        echo "Reset config: $HOUSE_CONFIG"
        break
        ;;
      2)
        echo "Leaving existing user data unchanged."
        break
        ;;
      *)
        echo "Invalid choice. Enter 1 or 2."
        ;;
    esac
  done
elif [[ ! -f "$HOUSE_CONFIG" ]]; then
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

EXPORT_LINE="export SHR_USER_DIR=\"$USER_DIR\""
SOURCE_LINE="source $ENV_FILE"

ensure_shr_user_dir_line "$BASHRC_FILE" "$EXPORT_LINE"
append_line_if_missing "$BASHRC_FILE" "$SOURCE_LINE"

echo
echo "Bash setup updated in: $BASHRC_FILE"
echo "To load the new environment now, run:"
echo "source ~/.bashrc"
