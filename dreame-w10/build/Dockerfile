# Reproducible aarch64 cross-build of the bridge artifacts for the robot.
#
# Pinned to debian:bullseye (glibc 2.31) ON PURPOSE, exactly like the camera tap:
# a no_std cdylib built here references only libc symbols versioned <= what we
# call, so it loads on the robot's glibc 2.23. A newer base (glibc >= 2.34) would
# emit @GLIBC_2.34 symbols and fail on the robot.
FROM debian:bullseye

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl ca-certificates make gcc libc6-dev \
        gcc-aarch64-linux-gnu libc6-dev-arm64-cross && \
    rm -rf /var/lib/apt/lists/*

ENV RUSTUP_HOME=/opt/rustup \
    CARGO_HOME=/opt/cargo \
    PATH=/opt/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
        sh -s -- -y --default-toolchain stable --profile minimal && \
    rustup target add aarch64-unknown-linux-gnu aarch64-unknown-linux-musl

# avatap (LD_PRELOAD cdylib) must be glibc-linked to interpose ava's libc; the
# bullseye cross gcc keeps it loadable on the robot's glibc 2.23.
ENV CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc
# avatap-relay is a normal process: build it fully static against musl so it has
# NO glibc dependency at all (a glibc build binds bullseye's >2.23 symbols and
# would fail on the robot). rust-lld + self-contained musl needs no C toolchain.
ENV CARGO_TARGET_AARCH64_UNKNOWN_LINUX_MUSL_LINKER=rust-lld \
    CARGO_TARGET_AARCH64_UNKNOWN_LINUX_MUSL_RUSTFLAGS="-C linker-flavor=ld.lld -C link-self-contained=yes"

WORKDIR /src
COPY . /src

RUN cargo build --release --target aarch64-unknown-linux-gnu  -p avatap && \
    cargo build --release --target aarch64-unknown-linux-musl -p avatap-relay && \
    mkdir -p /out && \
    cp target/aarch64-unknown-linux-gnu/release/libavatap.so   /out/avatap.so && \
    cp target/aarch64-unknown-linux-musl/release/avatap-relay  /out/avatap-relay && \
    aarch64-linux-gnu-strip /out/avatap.so /out/avatap-relay
