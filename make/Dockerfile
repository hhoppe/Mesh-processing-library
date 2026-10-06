# A Linux environment with the tools and libraries to build the programs and to run the unit tests and demos.
# Usage:
#   docker build -f make/Dockerfile -t mesh-processing .  # Build all programs and run the unit tests.
#   docker run -it --rm mesh-processing  # Start a shell, with the programs in the PATH.
#   docker run --init --rm mesh-processing xvfb-run -a make -C demos create check  # Create and check the demos.
# (The --init is necessary for xvfb-run, which otherwise hangs as the first process in the container.)
# The programs are compiled with -march=native, so an image suits the machine that builds it.
FROM ubuntu:26.04

# Xvfb and the Mesa drivers provide an X display with software OpenGL for the viewers, ffmpeg video I/O, and wget
# (with the certificates for https) the reading of files given as URLs.
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      make clang libgl-dev libx11-dev libjpeg-dev libpng-dev zlib1g-dev \
      ffmpeg xvfb xauth libgl1-mesa-dri xfonts-base wget ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /mesh_processing
COPY . .
# Build all programs, then run the unit tests; any failing test makes the image build fail.
RUN make -j"$(nproc)" progs test
ENV PATH=/mesh_processing/bin/unix:/mesh_processing/bin:$PATH
CMD ["bash"]
