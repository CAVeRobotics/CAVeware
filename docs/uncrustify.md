# Uncrustify

[Uncrustify](https://github.com/uncrustify/uncrustify) is used to automatically format code to ensure the project has a consistent style.

To setup Uncrustify in a local repository, follow these steps:

1. Initialize and update submodules
   `git submodule update --init`

2. Navigate to `uncrustify` directory
   `cd tools/uncrustify/uncrustify`

3. Configure CMake build
   `cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -B build`

4. Build `uncrustify`
   `cmake --build build --config Release`

Uncrustify can now be run from the repository root with `cmake --build build -t uncrustify` assuming CMake has been configured with build directory `build`.

To enable the pre-commit hook that automatically performs an Uncrustify check before each commit, run `git config core.hooksPath tools/hooks`. Note: This pre-commit hooks may also run other checks.