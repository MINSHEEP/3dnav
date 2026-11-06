#!/usr/bin/env python3
"""验证安装树重定位后的 CMake 导出、shell 钩子和离线消费，不触碰真机。"""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


CONSUMER_CMAKE = """cmake_minimum_required(VERSION 3.16)
project(sdk_install_consumer LANGUAGES CXX)
find_package(astrall_driver REQUIRED)
get_target_property(sdk_path astrall_sdk::sdk IMPORTED_LOCATION)
if(NOT sdk_path STREQUAL EXPECTED_VENDOR_LIB)
  message(FATAL_ERROR "SDK imported target did not relocate: ${sdk_path}")
endif()
add_executable(consumer main.cpp)
target_compile_features(consumer PRIVATE cxx_std_17)
target_link_libraries(consumer PRIVATE astrall_driver::astrall_sdk_client)
"""

CONSUMER_CPP = """#include "astrall_driver/fake_sdk_client.hpp"
#include "astrall_driver/sdk_client.hpp"

int main() {
  astrall_driver::SdkClient native;
  if (native.Heartbeat().Value() != ASTRALL_RES_NOT_INIT) {
    return 1;
  }
  astrall_driver::FakeSdkClient fake;
  if (!fake.Init({}).Succeeded()) {
    return 2;
  }
  AstrallSystemStatus status;
  auto result = fake.GetSystemStatus(&status);
  fake.Deinit();
  return result.Succeeded() ? 0 : 3;
}
"""


def run(command, environment):
    """失败时保留完整构建/加载错误；不打印调用环境。"""
    result = subprocess.run(
        command, env=environment, text=True, capture_output=True, timeout=45
    )
    if result.returncode:
        raise RuntimeError(
            f"Command failed ({result.returncode}): {command}\n"
            f"{result.stdout}\n{result.stderr}"
        )
    return result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vendor-prefix", type=Path, required=True)
    parser.add_argument("--driver-prefix", type=Path, required=True)
    parser.add_argument("--interfaces-prefix", type=Path, required=True)
    args = parser.parse_args()
    originals = (args.vendor_prefix.resolve(), args.driver_prefix.resolve(),
                 args.interfaces_prefix.resolve())
    environment = os.environ.copy()
    for variable in ("CMAKE_PREFIX_PATH", "AMENT_PREFIX_PATH", "LD_LIBRARY_PATH"):
        entries = environment.get(variable, "").split(os.pathsep)
        environment[variable] = os.pathsep.join(
            entry for entry in entries if entry and not any(
                Path(entry).resolve().is_relative_to(prefix)
                for prefix in originals
            )
        )
    with tempfile.TemporaryDirectory(prefix="astrall-sdk-install-") as directory:
        root = Path(directory)
        vendor = root / "vendor"
        driver = root / "driver"
        interfaces = root / "interfaces"
        shutil.copytree(originals[0], vendor, symlinks=False)
        shutil.copytree(originals[1], driver, symlinks=False)
        shutil.copytree(originals[2], interfaces, symlinks=False)
        source = root / "consumer"
        source.mkdir()
        (source / "CMakeLists.txt").write_text(CONSUMER_CMAKE)
        (source / "main.cpp").write_text(CONSUMER_CPP)
        environment["CMAKE_PREFIX_PATH"] = os.pathsep.join(
            (str(driver), str(vendor), str(interfaces),
             environment["CMAKE_PREFIX_PATH"])
        )
        build = root / "build"
        run(["cmake", "-S", str(source), "-B", str(build),
             f"-DEXPECTED_VENDOR_LIB={vendor}/lib/libASTRALL_SDK.so.1"],
            environment)
        run(["cmake", "--build", str(build)], environment)
        setup = (
            'set -e; source "$1/share/astrall_sdk_vendor/local_setup.bash"; '
            'source "$2/share/astrall_driver/local_setup.bash"; '
        )
        shell = ["bash", "--noprofile", "--norc", "-c"]
        arguments = ["sdk-install-test", str(vendor), str(driver),
                     str(build / "consumer")]
        run(shell + [setup + 'exec "$3"'] + arguments, environment)
        libraries = run(shell + [setup + 'exec ldd "$3"'] + arguments,
                        environment)
        for library in (vendor / "lib/libASTRALL_SDK.so.1",
                        driver / "lib/libastrall_sdk_client.so"):
            if str(library) not in libraries:
                raise RuntimeError(f"Not loaded from relocated tree: {library}")
        print("PASS: relocated imported targets, shell hooks and offline runtime")


if __name__ == "__main__":
    main()
