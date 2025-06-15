# Cppcheck

[Cppcheck](https://cppcheck.sourceforge.io/) is used to perform static analysis on code.

If Cppcheck is not installed globally, follow these steps to setup Cppcheck in a local repository:

1. Initialize and update submodules

   `git submodule update --init`

2. Navigate to `cppcheck` source directory

   `cd tools/cppcheck/cppcheck`

3. Configure CMake build

   `cmake -G Ninja -B build`

4. Build `cppcheck`

   `cmake --build build`

Cppcheck can now be run from the repository root with `cmake --build build -t cppcheck` assuming CMake has been configured with build directory `build`.

To enable the pre-commit hook that automatically runs Cppcheck before each commit, run `git config core.hooksPath tools/hooks`. Note: This pre-commit hooks may also run other checks.