#!/bin/bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

echo "start compile & replace so ..."

# Build all required libraries before replacing installed copies.
make -B -C contrlSDK libMarvinSDK.so
make -B -C kinematicsSDK libKine.so

chmod 755 contrlSDK/libMarvinSDK.so kinematicsSDK/libKine.so
for destination in SDK_PYTHON "DEMO_C++"; do
    if [ -d "$destination" ]; then
        cp -v contrlSDK/libMarvinSDK.so "$destination/"
        chmod 755 "$destination/libMarvinSDK.so"
        cp -v kinematicsSDK/libKine.so "$destination/"
        chmod 755 "$destination/libKine.so"
    fi
done

echo "finished!"
