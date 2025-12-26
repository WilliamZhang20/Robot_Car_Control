#!/bin/bash

# Build script with g2o caching optimization
# This script builds the Docker image with proper layer caching for g2o

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Default values
IMAGE_NAME="limo-sim"
TAG="latest"
DOCKERFILE="docker/sim.Dockerfile"
CACHE_FROM=""
PUSH_CACHE=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -t|--tag)
            TAG="$2"
            shift 2
            ;;
        -n|--name)
            IMAGE_NAME="$2"
            shift 2
            ;;
        --cache-from)
            CACHE_FROM="$2"
            shift 2
            ;;
        --push-cache)
            PUSH_CACHE="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  -t, --tag TAG        Image tag (default: latest)"
            echo "  -n, --name NAME      Image name (default: limo-sim)"
            echo "  --cache-from REPO    Pull cache from registry (e.g., myregistry/limo-sim-cache)"
            echo "  --push-cache REPO    Push cache to registry (e.g., myregistry/limo-sim-cache)"
            echo "  -h, --help           Show this help"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

cd "$PROJECT_ROOT"

echo "Building $IMAGE_NAME:$TAG with optimized g2o caching..."

# Build command with cache options
BUILD_CMD="docker build -f $DOCKERFILE -t $IMAGE_NAME:$TAG"

# Add cache-from if specified
if [[ -n "$CACHE_FROM" ]]; then
    echo "Using cache from: $CACHE_FROM"
    BUILD_CMD="$BUILD_CMD --cache-from $CACHE_FROM:g2o-base --cache-from $CACHE_FROM:builder --cache-from $CACHE_FROM:latest"
fi

# Add build context
BUILD_CMD="$BUILD_CMD ."

echo "Running: $BUILD_CMD"
eval $BUILD_CMD

# Push cache if specified
if [[ -n "$PUSH_CACHE" ]]; then
    echo "Pushing cache layers to: $PUSH_CACHE"
    
    # Tag and push intermediate stages for caching
    docker tag $(docker images -q --filter "label=stage=g2o-base" | head -1) "$PUSH_CACHE:g2o-base" 2>/dev/null || true
    docker tag $(docker images -q --filter "label=stage=builder" | head -1) "$PUSH_CACHE:builder" 2>/dev/null || true
    docker tag "$IMAGE_NAME:$TAG" "$PUSH_CACHE:latest"
    
    docker push "$PUSH_CACHE:g2o-base" || echo "Warning: Could not push g2o-base cache"
    docker push "$PUSH_CACHE:builder" || echo "Warning: Could not push builder cache"
    docker push "$PUSH_CACHE:latest"
fi

echo "Build completed: $IMAGE_NAME:$TAG"
echo ""
echo "To run the container:"
echo "  docker run -it --rm $IMAGE_NAME:$TAG"
echo ""
echo "For development with X11 forwarding:"
echo "  docker run -it --rm -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix $IMAGE_NAME:$TAG"