#!/usr/bin/sh

# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

#
# This script is designed to unload all the FBOSS BSP kernel modules
# from running system.
#

# "kmods.json" contains the list of kmods included in the BSP package.
KVER=$(uname -r)
KMOD_LIST="/usr/local/fboss_bsp/${KVER}/kmods.json"

kmod_is_loaded() {
    kmod="$1"

    if lsmod | grep "$kmod" > /dev/null 2>&1; then
        return 0
    fi

    return 1
}

kmod_remove_all() {
    if [ ! -e "$KMOD_LIST" ]; then
	echo "Unable to locate $KMOD_LIST. Exiting.."
	exit 0
    fi

    # Make sure "sharedKmods" are removed after "bspKmods", because some
    # bspKmods depend on sharedKmods.
    for kmodType in "bspKmods" "sharedKmods"; do
        jq -r ".${kmodType}[]" "$KMOD_LIST" | while read kmod; do
            if kmod_is_loaded "$kmod"; then
                echo "rmmod $kmod.."
                rmmod "$kmod"
            fi
        done
    done
}

warn_and_confirm() {
    if [ "$1" = "-f" ] || [ "$1" = "--force" ]; then
        return 0
    fi

    echo "Warning: all the BSP kernel modules will be unloaded!"
    read -r -t 10 -p "Do you wish to continue? [y/N]: " user_input
    if [ "$user_input" != "y" ] && [ "$user_input" != "Y" ] ; then
        echo ""
	echo "Task cancelled. Exiting now."
        exit 0
    fi
}

#
# Main entry starts here.
#
warn_and_confirm "$1"

kmod_remove_all
