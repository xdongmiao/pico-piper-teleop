# External runtime libraries

The original XR service launch prepends `libstdcxx_jammy_ppa/usr/lib/x86_64-linux-gnu` under this directory to `LD_LIBRARY_PATH`. The corresponding binary files were machine/runtime dependencies, not project source, and are excluded from this repository.

Install the XRoboToolkit SDK/service and compatible native dependencies from their official providers. If a compatible libstdc++ override is required, provision it at the existing path through the OS/package provider and retain the provider's source and license information. On systems whose normal libstdc++ already satisfies the SDK, the missing optional directory simply contributes no library.

No SDK installer or Docker image is downloaded or started automatically by the repository checks.
