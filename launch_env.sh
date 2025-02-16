#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1
export SKIP_FW_QUERY="True"
export FINGERPRINT="FASTBACK_LIMITED_EDITION_2024"

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="11.6"
fi

export STAGING_ROOT="/data/safe_staging"
