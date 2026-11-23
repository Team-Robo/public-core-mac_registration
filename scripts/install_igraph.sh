#!/usr/bin/env bash
# Build and install the igraph C library 0.10.17 into /usr/local.
#
# Why from source:
#   * Ubuntu 22.04's apt package is igraph 0.9.6, but the vendored MAC code uses the 0.10 API
#     (igraph_vector_int_list_t, igraph_maximal_cliques with int lists).
#   * igraph 1.0 (2025) changed the C API again, so we pin the last 0.10.x release.
#
# Usage:   sudo ./install_igraph.sh            (JOBS=4 by default; we never use all cores)
# Result:  /usr/local/lib/libigraph.so*, /usr/local/include/igraph/, CMake config igraph::igraph
set -euo pipefail

VERSION=0.10.17
SHA256=21016ecf309a235f2bd9e208606d5a24c14a1f701dd7b2497e2e5f6bf2c5f848
PREFIX=${PREFIX:-/usr/local}
JOBS=${JOBS:-4}
WORK=${WORK:-${HOME}/.cache/igraph-${VERSION}-build}

if pkg-config --exists igraph 2>/dev/null && [ "$(pkg-config --modversion igraph)" = "${VERSION}" ]; then
  echo "igraph ${VERSION} is already installed in $(pkg-config --variable=prefix igraph)"
  exit 0
fi

mkdir -p "${WORK}"
cd "${WORK}"
TARBALL="igraph-${VERSION}.tar.gz"
if [ ! -f "${TARBALL}" ]; then
  curl -fL --retry 3 -o "${TARBALL}" \
    "https://github.com/igraph/igraph/releases/download/${VERSION}/${TARBALL}"
fi
echo "${SHA256}  ${TARBALL}" | sha256sum -c -

rm -rf "igraph-${VERSION}" build
tar xzf "${TARBALL}"

# Release tarballs ship the generated parser sources, so bison/flex are not needed.
# Every optional dependency uses igraph's own vendored copy: no system BLAS/LAPACK/ARPACK/GMP
# version mismatches. GLPK and GraphML are not used by MAC, so they are switched off.
cmake -S "igraph-${VERSION}" -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTING=OFF \
  -DIGRAPH_USE_INTERNAL_BLAS=ON \
  -DIGRAPH_USE_INTERNAL_LAPACK=ON \
  -DIGRAPH_USE_INTERNAL_ARPACK=ON \
  -DIGRAPH_USE_INTERNAL_GMP=ON \
  -DIGRAPH_USE_INTERNAL_PLFIT=ON \
  -DIGRAPH_GLPK_SUPPORT=OFF \
  -DIGRAPH_GRAPHML_SUPPORT=OFF \
  -DIGRAPH_OPENMP_SUPPORT=OFF \
  -DIGRAPH_WARNINGS_AS_ERRORS=OFF

cmake --build build -j "${JOBS}"
cmake --install build
ldconfig

echo "Installed: igraph $(pkg-config --modversion igraph) in ${PREFIX}"
# The build tree is a few hundred MB; the disk is small, so remove it.
cd /
rm -rf "${WORK}"
