#!/bin/bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

echo "start compile & replace so ..."

# Build all required libraries before replacing installed copies.
make -B -C contrlSDK100343 libMarvinSDK.so
make -B -C kinematicsSDK libKine.so
(
    cd interferenceCheck/Interf
    g++ *.cpp ../FxCfg/CfgBase.cpp -Wall -O2 -fPIC -shared -o libInterfCheck.so
)

chmod 755 contrlSDK100343/libMarvinSDK.so kinematicsSDK/libKine.so interferenceCheck/Interf/libInterfCheck.so
for destination in SDK_PYTHON "DEMO_C++"; do
    if [ -d "$destination" ]; then
        cp -v contrlSDK100343/libMarvinSDK.so "$destination/"
        chmod 755 "$destination/libMarvinSDK.so"
        cp -v kinematicsSDK/libKine.so "$destination/"
        chmod 755 "$destination/libKine.so"
        cp -v interferenceCheck/Interf/libInterfCheck.so "$destination/"
        chmod 755 "$destination/libInterfCheck.so"
    fi
done

echo "finished!"
