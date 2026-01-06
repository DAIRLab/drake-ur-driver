# -*- mode: python -*-
# vi: set ft=python :

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _liburclient_repository_impl(module_ctx):
    http_archive(
        name = "liburclient",
        sha256 = "9635c813f548658440b3ca506a10bbbfd0dd7a6e2beaa1cf9d3053ac6f7b218c",  # noqa
        build_file = "//tools/workspace/Universal_Robots_Client_Library:package.BUILD.bazel",  # noqa
        strip_prefix = "Universal_Robots_Client_Library-2.3.0",
        urls = ["https://github.com/UniversalRobots/Universal_Robots_Client_Library/archive/refs/tags/2.3.0.zip"]
    )

liburclient_repository = module_extension(
    implementation = _liburclient_repository_impl,
)