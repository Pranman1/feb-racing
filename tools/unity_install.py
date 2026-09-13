#!/usr/bin/env python3
"""Install the Unity editor and build-target modules on Linux without root or Unity Hub.

    tools/unity_install.py ~/Unity/dl ~/Unity/Hub/Editor/2022.3.52f1

The download folder holds Unity-<ver>.tar.xz plus the target modules. Linux
modules are tar.xz; the Mac and Windows target modules are only shipped as
macOS .pkg (xar) files, which are unpacked here with a minimal xar reader.
"""
import io
import gzip
import pathlib
import struct
import subprocess
import sys
import tarfile
import xml.etree.ElementTree as ET
import zlib

MODULES = {  # archive name -> destination inside the editor folder
    "linux-il2cpp.tar.xz": "",
    "mac-mono.pkg": "Editor/Data/PlaybackEngines/MacStandaloneSupport",
    "windows-mono.pkg": "Editor/Data/PlaybackEngines/WindowsStandaloneSupport",
}


def xar_payload(path):
    """Return the gzip'd cpio 'Payload' of a flat macOS .pkg (xar archive)."""
    data = path.read_bytes()
    magic, header_size, _version, toc_len, toc_uncompressed = struct.unpack(">4sHHQQ", data[:24])
    if magic != b"xar!":
        raise SystemExit(f"{path} is not a xar archive")
    toc = ET.fromstring(zlib.decompress(data[header_size:header_size + toc_len]))
    heap = header_size + toc_len
    for f in toc.iter("file"):
        if f.findtext("name") == "Payload":
            d = f.find("data")
            off, length = int(d.findtext("offset")), int(d.findtext("length"))
            return data[heap + off: heap + off + length]
    raise SystemExit(f"no Payload in {path}")


def extract_cpio(blob, dest):
    """Unpack a gzip'd odc/newc cpio stream with the system cpio (present on every distro)."""
    dest.mkdir(parents=True, exist_ok=True)
    subprocess.run(["cpio", "-idm", "--quiet"], input=gzip.decompress(blob), cwd=dest, check=True)


def main(dl, editor):
    dl, editor = pathlib.Path(dl), pathlib.Path(editor)
    editor.mkdir(parents=True, exist_ok=True)
    tarball = next(dl.glob("Unity-*.tar.xz"))
    print(f"extracting {tarball.name} -> {editor}")
    subprocess.run(["tar", "xJf", str(tarball), "-C", str(editor)], check=True)
    for name, sub in MODULES.items():
        src = dl / name
        if not src.exists():
            print(f"skip {name} (not downloaded)")
            continue
        dest = editor / sub
        print(f"extracting {name} -> {dest}")
        if name.endswith(".tar.xz"):
            with tarfile.open(src) as t:
                t.extractall(dest)
        else:
            extract_cpio(xar_payload(src), dest)
    print("done:", editor / "Editor" / "Unity")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
