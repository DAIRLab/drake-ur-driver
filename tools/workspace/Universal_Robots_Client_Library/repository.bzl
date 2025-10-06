# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def liburclient_repository(
        name,
        mirrors = None):
    archive_override(
        name = name,
        repository = "UniversalRobots/Universal_Robots_Client_Library",
        tag = "2.3.0",
        # sha256 = "f0616d01ef09aa1d5948d385a6cfe18c514bc6e0921e4fac97ada7262b8722e4",  # noqa
        build_file = "//tools/workspace/Universal_Robots_Client_Library:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )