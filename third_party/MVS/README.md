# Hikrobot MVS SDK snapshot

This directory contains the unmodified MVS SDK snapshot that was already used
by the repository's existing `hik-driver/hikSDK` package.

Contents:

- `include/`: MVS C/C++ API headers.
- `lib/amd64/`: Linux x86_64 shared libraries.
- `lib/arm64/`: Linux aarch64 shared libraries.
- `SHA256SUMS`: integrity manifest for all vendor files.

The top-level MIT license does not apply to these vendor files. They remain
subject to Hikrobot's licensing and redistribution terms. The project does not
modify these headers or binaries. At runtime, the ROS node logs the SDK version
reported by `MV_CC_GetSDKVersion()`.

To verify the snapshot on Linux:

```bash
cd third_party/MVS
sha256sum --check SHA256SUMS
```
