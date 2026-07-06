fn main() {
    // dlsym() lives in libdl.so.2 on the robot's glibc 2.23 (it only moved into
    // libc in glibc 2.34). Link it explicitly so the .so resolves regardless of
    // whether the host process already pulled in libdl.
    println!("cargo:rustc-link-lib=dylib=dl");
}
