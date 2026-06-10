// Build script for car firmware
// Copies memory.x to the output directory where the linker can find it

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
  // Put memory.x in the linker search path
  let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
  File::create(out.join("memory.x"))
    .unwrap()
    .write_all(include_bytes!("memory.x"))
    .unwrap();
  println!("cargo:rustc-link-search={}", out.display());

  println!("cargo:rerun-if-changed=memory.x");
}
