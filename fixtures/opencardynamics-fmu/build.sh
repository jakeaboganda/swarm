#!/usr/bin/env bash
# Reproducibly build opencardynamics.fmu from source.
#
# Fetches Open-Car-Dynamics at a pinned commit + Eigen 3.4.0 into ./_work,
# builds the trimmed FMI 3.0 CS wrapper, runs the proof harness, and assembles
# the .fmu. Idempotent: re-run to rebuild. Linux/x86_64 only for now (the .fmu
# ships a single binaries/x86_64-linux/opencardynamics.so, like the VanDerPol
# reference fixture).
#
# Requirements: git, cmake >= 3.18, a C++20 compiler, zip.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="${HERE}/_work"
OCD_PIN="94f8fb187fb0ed22bba1d809bd74f66d1ff75af4"
EIGEN_TAG="3.4.0"

mkdir -p "${WORK}"

# --- Open-Car-Dynamics (pinned, recursive submodules) ---
OCD="${WORK}/Open-Car-Dynamics"
if [ ! -d "${OCD}/.git" ]; then
  git clone https://github.com/TUMFTM/Open-Car-Dynamics.git "${OCD}"
fi
git -C "${OCD}" fetch --quiet origin
git -C "${OCD}" checkout --quiet "${OCD_PIN}"
git -C "${OCD}" submodule update --init --recursive --quiet

# --- Eigen (header-only, installed to a local prefix) ---
EIGEN_PREFIX="${WORK}/eigen-prefix"
if [ ! -f "${EIGEN_PREFIX}/share/eigen3/cmake/Eigen3Config.cmake" ]; then
  EIGEN_SRC="${WORK}/eigen-src"
  [ -d "${EIGEN_SRC}" ] || git clone --depth 1 --branch "${EIGEN_TAG}" \
    https://gitlab.com/libeigen/eigen.git "${EIGEN_SRC}"
  cmake -S "${EIGEN_SRC}" -B "${WORK}/eigen-cfg" -DCMAKE_INSTALL_PREFIX="${EIGEN_PREFIX}" >/dev/null
  cmake --install "${WORK}/eigen-cfg" >/dev/null
fi

# --- Build the wrapper + harness ---
cmake -S "${HERE}" -B "${WORK}/build" \
  -DOCD_ROOT="${OCD}" \
  -DEIGEN_INCLUDE="${EIGEN_PREFIX}/include"
cmake --build "${WORK}/build" -j "$(nproc)"

# --- Prove it drives before packaging ---
"${WORK}/build/harness" "${WORK}/build/opencardynamics.so"

# --- Assemble the .fmu (zip: modelDescription.xml + binaries/x86_64-linux/*.so) ---
STAGE="${WORK}/fmu"
rm -rf "${STAGE}"
mkdir -p "${STAGE}/binaries/x86_64-linux"
cp "${HERE}/modelDescription.xml" "${STAGE}/"
cp "${WORK}/build/opencardynamics.so" "${STAGE}/binaries/x86_64-linux/opencardynamics.so"
( cd "${STAGE}" && zip -r -q "${HERE}/opencardynamics.fmu" modelDescription.xml binaries )

echo "built ${HERE}/opencardynamics.fmu"
