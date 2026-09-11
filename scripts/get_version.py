"""
PlatformIO pre-build script: derives FIRMWARE_VERSION from git so it never
has to be kept in sync with git tags by hand. Runs identically for local
dev builds and CI (.github/workflows/release.yml) - both just build
against whatever the current git state actually is.

`git describe --tags --always --dirty` gives:
  - "v1.2.3"                 exactly on a tagged commit
  - "v1.2.3-4-gabc1234"      4 commits past the v1.2.3 tag
  - "v1.2.3-4-gabc1234-dirty" same, with uncommitted local changes
  - "abc1234"                no tags exist yet at all (falls back to the
                              short commit hash)
"""
Import("env")

import subprocess


def get_version():
    try:
        result = subprocess.run(
            ["git", "describe", "--tags", "--always", "--dirty"],
            capture_output=True, text=True, check=True,
            cwd=env["PROJECT_DIR"],
        )
        version = result.stdout.strip()
        return version if version else "unknown"
    except Exception:
        # No git available (e.g. building from a source tarball, not a
        # clone) - not fatal, update_check.cpp just always sees itself as
        # "older" than whatever's on GitHub and offers to update, which is
        # a safe direction to be wrong in.
        return "unknown"


version = get_version()
print(f"[get_version.py] FIRMWARE_VERSION = {version}")
env.Append(CPPDEFINES=[("FIRMWARE_VERSION", env.StringifyMacro(version))])
