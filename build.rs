//! Compiles NAPA, and makes the version ID every frame carries from the package's version.

fn main() {
    // NAPA picks SSE2 on x86-64 and NEON on 64-bit Arm by itself, from the compiler's target. It's
    // always built optimised, so tests and debug builds detect at full speed too.
    cc::Build::new()
        .file("vendor/napa/napa_core.c")
        .include("vendor/napa")
        .std("c99")
        .opt_level(3)
        .flag("-fno-wrapv")
        .flag("-fvisibility=hidden")
        .warnings(false)
        .compile("napa");
    println!("cargo:rustc-link-lib=m");
    println!("cargo:rerun-if-changed=vendor/napa");

    // 2026.11.2 is 20261102: year, month and day, so a newer version always has a larger ID.
    let part = |name: &str| -> u32 {
        std::env::var(name)
            .expect("cargo sets the package version")
            .parse()
            .expect("the version is three numbers")
    };
    let (year, month, day) = (
        part("CARGO_PKG_VERSION_MAJOR"),
        part("CARGO_PKG_VERSION_MINOR"),
        part("CARGO_PKG_VERSION_PATCH"),
    );
    assert!(
        (1..=12).contains(&month) && (1..=31).contains(&day),
        "the version is a date, year.month.day: {year}.{month}.{day}"
    );
    println!(
        "cargo:rustc-env=VISOR_VERSION_ID={}",
        year * 10_000 + month * 100 + day
    );
}
