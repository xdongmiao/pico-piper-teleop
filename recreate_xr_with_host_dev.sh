#!/usr/bin/env bash
set -euo pipefail

old_name="xr"
backup_name="xr_before_host_dev_$(date +%Y%m%d_%H%M%S)"
image="xr_preserved_host_dev:$(date +%Y%m%d_%H%M%S)"
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if ! docker ps -a --format '{{.Names}}' | grep -qx "$old_name"; then
  echo "Container '$old_name' does not exist."
  exit 1
fi

echo "Committing current $old_name state -> $image ..."
docker commit "$old_name" "$image"

echo "Stopping $old_name ..."
docker stop "$old_name"

echo "Renaming $old_name -> $backup_name ..."
docker rename "$old_name" "$backup_name"

echo "Creating new $old_name with host /dev mounted ..."
docker run -dit \
  --name "$old_name" \
  --privileged \
  --network host \
  --hostname pico-piper \
  -w /projects \
  -v /dev:/dev \
  -v "$project_dir":/projects \
  "$image" \
  bash

echo "New container created."
echo "Backup container: $backup_name"
echo "Preserved image: $image"
echo
echo "Verify:"
echo "  docker exec xr bash -lc 'ls -l /dev/v4l/by-path/*video-index0 /dev/v4l/by-id/*video-index0'"
echo
echo "Rollback if needed:"
echo "  docker stop xr && docker rm xr && docker rename $backup_name xr && docker start xr"
