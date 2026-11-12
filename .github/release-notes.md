<!--
Fill in the sections below before publishing, one line per change:
  * Fixed the muxer hanging when the relay closes the session [name] ([#12](https://github.com/openmoq/FFmpeg/pull/12))
    + Details on indented lines starting with a plus
Delete the empty sections and this comment.
-->
{{CHANGES}}

Downloads
---------

* **Windows x86_64:** `ffmpeg-moq-windows-x86_64-{{REV}}.zip`, for Windows 10 or later.
* **macOS Apple silicon:** `ffmpeg-moq-macos-arm64-{{REV}}.tar.xz`, for macOS 12 or later.
* **macOS Intel:** `ffmpeg-moq-macos-x86_64-{{REV}}.tar.xz`, for macOS 12 or later.
  + The macOS binaries are not signed. If macOS blocks them after a browser download, run `xattr -d com.apple.quarantine ffmpeg ffprobe`.
* **Linux x86_64 and aarch64:** `ffmpeg-moq-linux-x86_64-{{REV}}.tar.xz` and `ffmpeg-moq-linux-aarch64-{{REV}}.tar.xz`, for any distribution with glibc 2.35 or later (Ubuntu 22.04, Debian 12 and newer).
* **Debug symbols:** the `.debug.xz`, `.dSYM.zip` and `.debug.zip` files next to each package, for reading crash reports. They are not needed to run ffmpeg.

Check that the muxer is there with `ffmpeg -hide_banner -h muxer=moq`.

Built With
----------

* FFmpeg [{{FFMPEG_REV}}](https://github.com/openmoq/FFmpeg/commit/{{FFMPEG_REV}}) with `--enable-gpl --enable-libx264`, so the binaries are GPLv2 or later (see `COPYING.GPLv2` in each package)
* libmoq [openmoq/moq5@{{MOQ5_SHORT}}](https://github.com/openmoq/moq5/commit/{{MOQ5_SHA}}), with picoquic {{PICOQUIC_SHORT}} and picotls {{PICOTLS_SHORT}}
* x264 {{X264_SHORT}} and OpenSSL {{OPENSSL_VERSION}}, linked statically
* `BUILDINFO.txt` in each package lists the exact revisions and configure options

### Checksums
{{CHECKSUMS}}
