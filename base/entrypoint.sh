#!/bin/bash
set -e

USERNAME=ros

# Function to assign a specific GID to a group
# Usage: assign_gid <group_name> <target_gid>
grp_idx=9999
assign_gid() {
    local group_name="$1"
    local target_gid="$2"
    local current_owner

    # If no target GID is provided, do nothing
    if [ -z "$target_gid" ]; then return; fi

    # 1. Is this GID already taken by ANOTHER group?
    current_owner=$(getent group "$target_gid" | cut -d: -f1)
    if [ -n "$current_owner" ] && [ "$current_owner" != "$group_name" ]; then
        groupmod -g "$grp_idx" "$current_owner"
        grp_idx=$((grp_idx + 1))
    fi

    # 2. Does our group exist? If not, create it.
    if ! getent group "$group_name" > /dev/null; then
        groupadd -g "$target_gid" "$group_name"
    else
        # It exists, change its GID
        groupmod -g "$target_gid" "$group_name"
    fi

    # 3. Add the user to the group
    usermod -aG "$group_name" "$USERNAME"
}


# 1. User Management (Primary UID/GID)
HOST_UID=${HOST_UID:-1000}
HOST_GID=${HOST_GID:-1000}

# Update UID if it does not match
if [ "$(id -u "$USERNAME")" != "$HOST_UID" ]; then
    # Move potential conflicting user
    existing_user=$(getent passwd "$HOST_UID" | cut -d: -f1)
    if [ -n "$existing_user" ]; then usermod -u 9999 "$existing_user"; fi

    # Change ID
    usermod -u "$HOST_UID" "$USERNAME"
fi

# Update GID if it does not match
if [ "$(id -g "$USERNAME")" != "$HOST_GID" ]; then
    # Move potential conflicting group
    existing_grp=$(getent group "$HOST_GID" | cut -d: -f1)
    if [ -n "$existing_grp" ] && [ "$existing_grp" != "$USERNAME" ]; then
        groupmod -g 9997 "$existing_grp"
    fi
    groupmod -g "$HOST_GID" "$USERNAME"
    usermod -g "$HOST_GID" "$USERNAME"
fi

# Fix home directory permissions
chown -R "$USERNAME:$USERNAME" "/home/$USERNAME"

# 2. Hardware Groups Management (Docker, Input, Render)
# Use variables passed by docker-compose
assign_gid "docker" "$HOST_DOCKER_GID"
assign_gid "input"  "$HOST_INPUT_GID"
assign_gid "render" "$HOST_RENDER_GID"
assign_gid "video"  "$HOST_VIDEO_GID" # Useful for cameras

# 3. Environment Setup & Run
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Run additional setup scripts if they exist
if [ -d "/.entrypoint-setup" ]; then
  for f in /.entrypoint-setup/*.sh; do
    if [ -f "$f" ]; then
      gosu "$USERNAME" bash "$f"
    fi
  done
fi

# --- END OF HOOKS ---

exec gosu "$USERNAME" "$@"
