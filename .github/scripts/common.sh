# Sourced by every build script: platform, work tree and helpers.
#
# The steps, each its own script, in order:
#   build-openssl  build-libmoq  build-x264  build-zlib (Windows)
#   build-ffmpeg-moq  check-ffmpeg-moq  package-ffmpeg-moq
#
# MOQ_WORK_DIR (default: <repo>/.moq-build) holds deps/ (one cacheable
# directory per dependency, each with the buildinfo lines that end up in
# BUILDINFO.txt), logs/, ffmpeg/ (the build) and out/ (the packages).

script_dir=$(cd "$(dirname "$0")" && pwd)
project_root=$(cd "${script_dir}/../.." && pwd)
script_name=${0##*/}

log() { printf '[%s] %s\n' "${script_name}" "$*" >&2; }
die() { printf '[%s] ERROR: %s\n' "${script_name}" "$*" >&2; exit 1; }

resolve_revision() {
    local repo=$1 ref=$2 revision
    if printf '%s' "${ref}" | grep -qE '^[0-9a-f]{40}$'; then
        printf '%s\n' "${ref}"
        return 0
    fi
    revision=$(git ls-remote "${repo}" "${ref}" | head -1 | cut -f1)
    [ -n "${revision}" ] || die "could not resolve '${ref}' in ${repo}"
    printf '%s\n' "${revision}"
}

arch=$(uname -m)
exe=''
case $(uname -s) in
Linux)
    platform=linux
    platform_id="linux-${arch}"
    ;;
Darwin)
    platform=macos
    export MACOSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-12.0}"
    platform_id="macos-${arch}-${MACOSX_DEPLOYMENT_TARGET}"
    ;;
MINGW* | MSYS*)
    platform=windows
    exe='.exe'
    platform_id="windows-${arch}"
    ;;
*)
    die "unsupported OS: $(uname -s)"
    ;;
esac

work_dir="${MOQ_WORK_DIR:-${project_root}/.moq-build}"
deps_dir="${work_dir}/deps"
log_dir="${work_dir}/logs"
build_dir="${work_dir}/ffmpeg"
out_dir="${work_dir}/out"
openssl_prefix="${deps_dir}/openssl"
libmoq_prefix="${deps_dir}/libmoq"
picoquic_deps="${deps_dir}/picoquic"
picotls_build="${picoquic_deps}/picotls/build"
x264_prefix="${deps_dir}/x264"
# Windows has no system zlib and gets OpenSSL and zlib from vcpkg, static
# like everything else; vcpkg installs into <root>/<triplet>.
vcpkg_triplet=x64-windows-static
zlib_prefix="${deps_dir}/zlib/${vcpkg_triplet}"
[ "${platform}" != windows ] || openssl_prefix="${deps_dir}/openssl/${vcpkg_triplet}"

jobs="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)}"

# Runs a noisy build step with its output in a log, shown only on failure.
quiet() {
    local name=$1
    shift
    mkdir -p "${log_dir}"
    if ! "$@" >>"${log_dir}/${name}.log" 2>&1; then
        tail -n 60 "${log_dir}/${name}.log" >&2
        die "${name} failed, full log in ${log_dir}/${name}.log"
    fi
}

up_to_date() {
    [ -r "$1" ] && [ "$(cat "$1")" = "$2" ]
}

# D:/a/b, which both MSVC tools and MSYS programs take.
native() {
    cygpath -m "$1"
}

# Stops unless an earlier step installed $2 (its buildinfo is written last).
require_dep() {
    [ -r "$2/buildinfo" ] || die "no $1 in $2: run $3 first"
}

# Windows: checks for cl and puts MSVC's link.exe ahead of MSYS2's coreutils
# link.
require_msvc() {
    [ "${platform}" = windows ] || return 0
    command -v cl >/dev/null 2>&1 \
        || die 'cl not found: run this inside an x64 MSVC developer environment'
    PATH="$(dirname "$(command -v cl)"):${PATH}"
}

# vcpkg_install <log name> <install root> <port>... with the static triplet.
vcpkg_install() {
    local name=$1 root=$2
    shift 2
    [ -n "${VCPKG_INSTALLATION_ROOT:-}" ] || die 'VCPKG_INSTALLATION_ROOT is not set'
    quiet "${name}" "$(cygpath -u "${VCPKG_INSTALLATION_ROOT}")/vcpkg.exe" install \
        "${@/%/:${vcpkg_triplet}}" "--x-install-root=$(native "${root}")"
}

# The Version: of a .pc file.
pc_version() {
    sed -n 's/^Version:[[:space:]]*//p' "$1" | tr -d '\r'
}

runtime_deps() {
    case ${platform} in
    macos) otool -L "$1" | tail -n +2 | awk '{print $1}' ;;
    windows) objdump -p "$1" | awk '/DLL Name:/ {print tolower($3)}' ;;
    *) ldd "$1" | awk '{print $1}' ;;
    esac
}
