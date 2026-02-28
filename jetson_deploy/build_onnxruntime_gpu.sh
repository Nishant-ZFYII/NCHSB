#!/usr/bin/env bash
# =============================================================================
# build_onnxruntime_gpu.sh
# Build onnxruntime-gpu from source for Jetson Orin Nano (JetPack 6.2)
#
# Target config:
#   L4T:        R36.4.7
#   JetPack:    6.2
#   CUDA:       12.6
#   TensorRT:   10.3.0.30
#   Python:     3.10
#   GPU arch:   87 (Ampere / Orin)
#
# Estimated build time: 2–4 hours on 6-core Orin Nano 8 GB
# =============================================================================

set -e

# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------
ORT_VERSION="1.20.1"
ORT_BRANCH="rel-1.20.1"
ORT_REPO="https://github.com/microsoft/onnxruntime.git"
BUILD_DIR="${HOME}/build/onnxruntime"
CUDA_HOME="/usr/local/cuda"
CUDNN_HOME="/usr"                          # libcudnn lives in /usr/lib/aarch64-linux-gnu
TRT_HOME="/usr"                            # libnvinfer lives in /usr/lib/aarch64-linux-gnu
CUDA_ARCH="87"                             # Jetson Orin = Ampere sm_87
N_JOBS=$(nproc)

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
log()  { echo -e "${GREEN}[ORT-BUILD]${NC} $*"; }
warn() { echo -e "${YELLOW}[ORT-WARN]${NC}  $*"; }
err()  { echo -e "${RED}[ORT-ERR]${NC}   $*"; exit 1; }

# ---------------------------------------------------------
# Step 0: Prerequisites (requires sudo — run once)
# ---------------------------------------------------------
install_deps() {
    log "Installing apt dependencies (needs sudo)..."
    sudo apt-get update -qq
    sudo apt-get install -y \
        libcudnn9-cuda-12 libcudnn9-dev-cuda-12 \
        build-essential git curl \
        libssl-dev zlib1g-dev patchelf \
        libnccl-dev 2>/dev/null || true   # nccl optional

    log "Ensuring pip cmake / ninja are up to date..."
    export PATH="$HOME/.local/bin:$PATH"
    pip3 install --upgrade cmake ninja setuptools wheel pip
}

# ---------------------------------------------------------
# Step 1: Clone onnxruntime
# ---------------------------------------------------------
clone_repo() {
    mkdir -p "$BUILD_DIR"
    if [ -d "${BUILD_DIR}/onnxruntime/.git" ]; then
        warn "Repo already cloned at ${BUILD_DIR}/onnxruntime — skipping clone."
    else
        log "Cloning onnxruntime ${ORT_VERSION} ..."
        git clone --recursive "$ORT_REPO" \
            --branch "$ORT_BRANCH" \
            --depth 1 \
            "${BUILD_DIR}/onnxruntime"
    fi
}

# ---------------------------------------------------------
# Step 2: Build
# ---------------------------------------------------------
build_ort() {
    cd "${BUILD_DIR}/onnxruntime"

    export PATH="$HOME/.local/bin:$PATH"
    export CUDACXX="${CUDA_HOME}/bin/nvcc"

    log "Starting build with ${N_JOBS} parallel jobs..."
    log "This takes 2–4 hours on Orin Nano. Logs → build_gpu.log"
    log "Monitor progress: tail -f ${BUILD_DIR}/onnxruntime/build_gpu.log"

    python3 tools/ci_build/build.py \
        --build_dir "${BUILD_DIR}/onnxruntime/build/Linux" \
        --config Release \
        --build_shared_lib \
        --update \
        --build \
        --use_cuda \
        --cuda_home "${CUDA_HOME}" \
        --cudnn_home "${CUDNN_HOME}" \
        --use_tensorrt \
        --tensorrt_home "${TRT_HOME}" \
        --allow_running_as_root \
        --build_wheel \
        --skip_tests \
        --parallel "${N_JOBS}" \
        --cmake_extra_defines \
            CMAKE_CUDA_ARCHITECTURES="${CUDA_ARCH}" \
            onnxruntime_USE_CUDA_NHWC_OPS=ON \
            onnxruntime_ENABLE_PYTHON=ON \
        2>&1 | tee build_gpu.log

    log "Build finished!"
}

# ---------------------------------------------------------
# Step 3: Install wheel
# ---------------------------------------------------------
install_wheel() {
    WHEEL=$(find "${BUILD_DIR}/onnxruntime/build/Linux/Release/dist" \
            -name "onnxruntime_gpu-*.whl" | head -1)
    if [ -z "$WHEEL" ]; then
        err "Could not find built wheel in build/Linux/Release/dist/. Check build_gpu.log"
    fi
    log "Installing wheel: $(basename "$WHEEL")"
    # Uninstall CPU version first to avoid conflict
    pip3 uninstall -y onnxruntime 2>/dev/null || true
    pip3 install "$WHEEL"
    log "Wheel installed successfully."
}

# ---------------------------------------------------------
# Step 4: Verify
# ---------------------------------------------------------
verify() {
    log "Verifying installation..."
    python3 -c "
import onnxruntime as ort
print('OnnxRuntime version:', ort.__version__)
print('Available providers:', ort.get_available_providers())

expected = ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']
for ep in expected:
    status = '✓' if ep in ort.get_available_providers() else '✗ MISSING'
    print(f'  {status}  {ep}')
"
}

# ---------------------------------------------------------
# Main
# ---------------------------------------------------------
usage() {
    echo "Usage: $0 [all|deps|build|install|verify]"
    echo "  all     - run everything (default)"
    echo "  deps    - install apt/pip dependencies (needs sudo)"
    echo "  build   - clone and compile (no sudo needed after deps)"
    echo "  install - install the built wheel"
    echo "  verify  - test the installed wheel"
}

CMD="${1:-all}"
case "$CMD" in
    all)     install_deps; clone_repo; build_ort; install_wheel; verify ;;
    deps)    install_deps ;;
    build)   clone_repo; build_ort ;;
    install) install_wheel ;;
    verify)  verify ;;
    *)       usage; exit 1 ;;
esac
