#!/bin/bash
# Build the FBOSS BSP within the Distro Image framework

set -e

SPEC_FILE="rpmbuild/fboss_bsp_kmods.spec"

# Install build dependencies for the spec
dnf builddep -y --spec "$SPEC_FILE"

# Use gcc-toolset-12 for the build
sudo update-alternatives --set gcc /opt/rh/gcc-toolset-12/root/usr/bin/gcc
source /opt/rh/gcc-toolset-12/enable

echo "Setting up RPM build environment in $FBOSS_BSP_DIR/rpmbuild..."

# Get the absolute path to the fboss-bsp directory and switch to it
FBOSS_BSP_DIR="$(dirname "$(readlink -f "$0")")"
cd "$FBOSS_BSP_DIR"

# Create RPM build directory structure
mkdir -p rpmbuild/{BUILD,RPMS,SOURCES,SPECS,SRPMS}
rm -rf rpmbuild/RPMS/*

# Copy spec file to SPECS directory
cp "$SPEC_FILE" rpmbuild/SPECS/

# Create source tarball from kmods
mkdir -p rpmbuild/SOURCES/fboss-bsp/kmods
NAME="fboss_bsp_kmods"
VERSION=$(grep '^Version:' "$SPEC_FILE" | awk '{print $2}')

tar -czvf "rpmbuild/SOURCES/${NAME}-${VERSION}.tar.gz" \
  --transform "s,^,fboss_bsp_kmods-${VERSION}/," \
  kmods/

# Build the RPM
rpmbuild -ba rpmbuild/SPECS/"$(basename "$SPEC_FILE")" --define "_topdir $FBOSS_BSP_DIR/rpmbuild"

RPM_PATH=$(find rpmbuild/RPMS -name "*.rpm" | head -1)

if [ -n "$RPM_PATH" ]; then
  echo "RPM built successfully: $RPM_PATH"
else
  echo "Error: RPM build failed or RPM not found"
  exit 1
fi

# Package the built RPMs into /output tarball for the Distro image
cd rpmbuild/RPMS/*
tar -cf /output/bsp-fboss.tar --strip-components=3 *.rpm
