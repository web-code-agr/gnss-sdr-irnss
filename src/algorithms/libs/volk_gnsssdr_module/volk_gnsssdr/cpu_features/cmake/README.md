# CMake build instructions

<!-- prettier-ignore-start -->
[comment]: # (
SPDX-License-Identifier: Apache-2.0
)

[comment]: # (
SPDX-FileCopyrightText: 2017 Google LLC
)
<!-- prettier-ignore-end -->

## Recommended usage : Incorporating cpu_features into a CMake project

For API / ABI compatibility reasons, it is recommended to build and use
cpu_features in a subdirectory of your project or as an embedded dependency.

This is similar to the recommended usage of the googletest framework (
https://github.com/google/googletest/blob/master/googletest/README.md )

Build and use step-by-step

1- Download cpu_features and copy it in a sub-directory in your project. or add
cpu_features as a git-submodule in your project

2- You can then use the cmake command `add_subdirectory()` to include
cpu_features directly and use the `cpu_features` target in your project.

3- Add the `cpu_features` target to the `target_link_libraries()` section of
your executable or of your library.

## Enabling tests

CMake default options for cpu_features is Release built type with tests
disabled. To enable testing set cmake `BUILD_TESTING` variable to `ON`,
[.travis.yml](https://github.com/google/cpu_features/blob/master/.travis.yml)
and
[appveyor.yml](https://github.com/google/cpu_features/blob/master/appveyor.yml)
have up to date examples.
