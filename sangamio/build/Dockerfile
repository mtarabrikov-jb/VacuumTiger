# Reproducible aarch64 cross-build of the SangamIO daemon for the Dreame W10.
#
# The robot is aarch64 with an old glibc (2.23). We build a FULLY STATIC musl
# binary (like the dreame-w10 w10-mcud / avatap-relay) so there is NO glibc
# dependency at all and it loads regardless of the robot's glibc. rust-lld +
# self-contained musl needs no C cross toolchain. A host C compiler (cc) is
# still required because build scripts and proc-macro crates (proc-macro2, libc,
# quote, ...) compile for the BUILD host. protoc is required by prost-build at
# compile time (build.rs compiles proto/sangamio.proto).
#
# Do not run `docker build` on this file directly: use build-aarch64.sh, which
# assembles the minimal context this Dockerfile expects at /src:
#   /src/sangamio/            (the daemon crate; edition 2024)
#   /src/dreame-w10/proto/    (path dep: dreame-w10-proto, no_std pure parser)
FROM debian:bullseye

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl ca-certificates protobuf-compiler && \
    rm -rf /var/lib/apt/lists/*

ENV RUSTUP_HOME=/opt/rustup \
    CARGO_HOME=/opt/cargo \
    PATH=/opt/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
        sh -s -- -y --default-toolchain stable --profile minimal && \
    rustup target add aarch64-unknown-linux-musl

# Host C toolchain for build scripts / proc-macros (they target the build host).
RUN apt-get update && \
    apt-get install -y --no-install-recommends gcc libc6-dev && \
    rm -rf /var/lib/apt/lists/*

# Fully static musl: rust-lld links the self-contained musl libc, no glibc.
ENV CARGO_TARGET_AARCH64_UNKNOWN_LINUX_MUSL_LINKER=rust-lld \
    CARGO_TARGET_AARCH64_UNKNOWN_LINUX_MUSL_RUSTFLAGS="-C linker-flavor=ld.lld -C link-self-contained=yes"

WORKDIR /src/sangamio
COPY . /src

RUN cargo build --release --target aarch64-unknown-linux-musl && \
    mkdir -p /out && \
    cp target/aarch64-unknown-linux-musl/release/sangamio /out/sangamio && \
    ( aarch64-linux-gnu-strip /out/sangamio 2>/dev/null || true )
