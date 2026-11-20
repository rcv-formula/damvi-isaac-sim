#!/usr/bin/env bash
set -euo pipefail
ISAAC_INSTALL="${ISAAC_INSTALL:-/home/shin/isaac_sim}"
USD_EXT=("$ISAAC_INSTALL"/extscache/omni.usd.libs-*)
PHYSX_EXT=("$ISAAC_INSTALL"/extscache/omni.usd.schema.physx-*)
export PYTHONPATH="${USD_EXT[0]}:${PHYSX_EXT[0]}${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$ISAAC_INSTALL/kit/lib:${USD_EXT[0]}/bin:${PHYSX_EXT[0]}/bin${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PXR_PLUGINPATH_NAME="${PHYSX_EXT[0]}/plugins/PhysxSchema/resources:${PHYSX_EXT[0]}/plugins/PhysxSchemaAddition/resources:${PHYSX_EXT[0]}/plugins/OmniUsdPhysicsDeformableSchema/resources${PXR_PLUGINPATH_NAME:+:$PXR_PLUGINPATH_NAME}"
exec "$ISAAC_INSTALL/kit/python/bin/python3" "$@"
