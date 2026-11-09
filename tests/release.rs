//! tools/release.sh publishes the client from the working tree beside the binary the tag built, so
//! it refuses unless that tree is the tag's: HEAD at `v<version>` and nothing changed.
//!
//! Each case runs the script in a small repository of its own, with a `gh` that only says it was
//! reached, so nothing is downloaded or published.

use std::os::unix::fs::PermissionsExt;
use std::path::{Path, PathBuf};
use std::process::Command;

const VERSION: &str = "2026.1.2";

/// The state of the repository the script runs in.
#[derive(Debug, Clone, Copy)]
enum Tree {
    Untagged,
    TaggedOtherVersion,
    TaggedButChanged,
    Tagged,
}

fn git(dir: &Path, args: &[&str]) {
    let status = Command::new("git")
        .args([
            "-c",
            "user.name=Visor test",
            "-c",
            "user.email=test@example.invalid",
        ])
        .args(["-c", "commit.gpgsign=false", "-c", "tag.gpgsign=false"])
        .args(args)
        .current_dir(dir)
        .output()
        .unwrap();
    assert!(status.status.success(), "git {args:?}: {status:?}");
}

/// A repository with the script and a Cargo.toml of `VERSION`, in the given state, and a directory
/// holding the stand-in `gh`.
fn repository(tree: Tree) -> (PathBuf, PathBuf) {
    let dir = std::env::temp_dir().join(format!("visor-release-{tree:?}-{}", std::process::id()));
    let repo = dir.join("repo");
    let bin = dir.join("bin");
    std::fs::create_dir_all(repo.join("tools")).unwrap();
    std::fs::create_dir_all(&bin).unwrap();
    let script = Path::new(env!("CARGO_MANIFEST_DIR")).join("tools/release.sh");
    std::fs::copy(script, repo.join("tools/release.sh")).unwrap();
    std::fs::write(
        repo.join("Cargo.toml"),
        format!("[package]\nversion = \"{VERSION}\"\n"),
    )
    .unwrap();
    let gh = bin.join("gh");
    std::fs::write(&gh, "#!/bin/sh\necho 'gh reached' >&2\nexit 3\n").unwrap();
    std::fs::set_permissions(&gh, std::fs::Permissions::from_mode(0o755)).unwrap();

    git(&repo, &["init", "-q"]);
    git(&repo, &["add", "."]);
    git(&repo, &["commit", "-q", "-m", "release"]);
    match tree {
        Tree::Untagged => {}
        Tree::TaggedOtherVersion => git(&repo, &["tag", "-a", "v2026.1.1", "-m", "older"]),
        Tree::TaggedButChanged => {
            git(&repo, &["tag", "-a", &format!("v{VERSION}"), "-m", VERSION]);
            std::fs::write(repo.join("client.java"), "changed after the tag").unwrap();
        }
        Tree::Tagged => git(&repo, &["tag", "-a", &format!("v{VERSION}"), "-m", VERSION]),
    }
    (repo, bin)
}

#[test]
fn the_client_is_published_only_from_the_release_s_tag() {
    // (the repository's state, part of what the script says on stderr)
    let cases = [
        (Tree::Untagged, "HEAD isn't the tag v2026.1.2"),
        (Tree::TaggedOtherVersion, "HEAD isn't the tag v2026.1.2"),
        (Tree::TaggedButChanged, "the working tree has changes"),
        // Past the checks, it goes on to download the release.
        (Tree::Tagged, "gh reached"),
    ];
    for (tree, said) in cases {
        let (repo, bin) = repository(tree);
        let path = format!("{}:{}", bin.display(), std::env::var("PATH").unwrap());
        let output = Command::new("bash")
            .arg(repo.join("tools/release.sh"))
            .arg(&repo)
            .env("PATH", path)
            .env("XDG_CACHE_HOME", repo.parent().unwrap().join("cache"))
            .output()
            .unwrap();
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(!output.status.success(), "{tree:?}: {stderr}");
        assert!(stderr.contains(said), "{tree:?}: {stderr}");
        std::fs::remove_dir_all(repo.parent().unwrap()).unwrap();
    }
}
