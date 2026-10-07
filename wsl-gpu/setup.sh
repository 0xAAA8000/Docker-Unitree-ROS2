#!/bin/bash
# WSL2 上で GPU (D3D12) 描画を有効にする。entrypoint.sh から source される。
#
# ホストの /usr/lib/wsl を /usr/lib/wsl-host に読み取り専用でマウントしておくと、
# コンテナ内の /usr/lib/wsl をシンボリックリンクで再構成する。
# その際、コンテナの glibc では読み込めないドライバの .so (Qualcomm Adreno 等) は
# バージョン要求を外したコピーに差し替え、不足シンボルを互換ライブラリで補う。

WSL_HOST_DIR=/usr/lib/wsl-host
WSL_DIR=/usr/lib/wsl
COMPAT_LIB=/usr/local/lib/libwsl_glibc_compat.so

# $1: 元の .so, $2: 出力先, $3...: 不足しているバージョン名
wsl_gpu_patch_so() {
    local src="$1" dst="$2"
    shift 2
    local syms args=()
    syms=$(objdump -T "${src}" | awk -v vers="$*" '
        BEGIN { n = split(vers, v, " "); for (i = 1; i <= n; i++) want["(" v[i] ")"] = 1 }
        NF >= 2 && ($(NF - 1) in want) { print $NF }') || return 1
    for s in ${syms}; do args+=(--clear-symbol-version "${s}"); done

    rm -f "${dst}" &&
        cp "${src}" "${dst}" &&
        chmod u+w "${dst}" &&
        patchelf "${args[@]}" --add-needed "${COMPAT_LIB}" "${dst}" &&
        python3 /opt/wsl-gpu/weaken_verneed.py "${dst}" "$@"
}

if [ -d "${WSL_HOST_DIR}/lib" ] && [ -e /dev/dxg ]; then
    rm -rf "${WSL_DIR}"
    mkdir -p "${WSL_DIR}/drivers"
    ln -s "${WSL_HOST_DIR}/lib" "${WSL_DIR}/lib"

    for drv in "${WSL_HOST_DIR}"/drivers/*; do
        name="$(basename "${drv}")"
        patched=false
        for so in "${drv}"/*.so; do
            [ -f "${so}" ] || continue
            missing=$(ldd "${so}" 2>/dev/null | sed -n "s/.*version \`\([^']*\)' not found.*/\1/p" | sort -u)
            [ -n "${missing}" ] || continue

            if [ "${patched}" = false ]; then
                mkdir -p "${WSL_DIR}/drivers/${name}"
                ln -s "${drv}"/* "${WSL_DIR}/drivers/${name}/"
                patched=true
            fi
            if wsl_gpu_patch_so "${so}" "${WSL_DIR}/drivers/${name}/$(basename "${so}")" ${missing}; then
                echo "[ info ] WSL GPU: patched ${name}/$(basename "${so}") ($(echo ${missing}))"
            else
                echo "[ warn ] WSL GPU: failed to patch ${name}/$(basename "${so}")"
            fi
        done
        [ "${patched}" = true ] || ln -s "${drv}" "${WSL_DIR}/drivers/${name}"
    done

    export LD_LIBRARY_PATH="${WSL_DIR}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    export GALLIUM_DRIVER=d3d12
    echo "[ info ] WSL GPU (D3D12) Enabled"
fi
