#!/bin/bash
# 外部リポジトリ (git サブモジュール) を取得し, patches/ にあるこちらの変更を当てる.
# 何度実行してもよい (取得済みなら取り込み元のコミットとパッチの状態を確かめるだけ).
#
#   ./scripts/fetch-external.sh          # すべて (common, l1, l2)
#   ./scripts/fetch-external.sh l1       # common と l1 だけ (setup-raspi.sh が呼ぶ)
#
# 取り込み元のコミットは, このリポジトリが記録しているサブモジュールのコミット (git ls-files -s) を使う.
# 部分クローン (--filter=blob:none) と sparse-checkout で, 使わないファイル (下の EXCLUDE) は取らない.
#   * Point-LIO の image/ (README 用の GIF, 約 250 MB) と Log/
#   * Unilidar SDK の ROS1 版 (unitree_lidar_ros) と docs/. ROS1 版は colcon が拾うとビルドに失敗する
# 普通に git submodule update で取った場合に備えて, ROS1 版には COLCON_IGNORE を置き,
# Docker には .dockerignore で入れない.
set -euo pipefail

REPO=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$REPO"
TARGET=${1:-all}
case "$TARGET" in
    all|l1|l2|sim) ;;
    *) echo "usage: $0 [all|l1|l2|sim]" >&2; exit 1 ;;
esac

# 取らないもの (sparse-checkout の除外. パスはサブモジュールの中の位置)
declare -A EXCLUDE=(
    [src/common/point_lio_ros2]="/image/ /Log/ /.gitmodules"
    [src/l1/unilidar_sdk]="/unitree_lidar_ros/ /docs/"
    [src/l2/unilidar_sdk2]="/unitree_lidar_ros/ /docs/ /README_CN.md /VERSION.md"
    [src/l2/ndt_omp_ros2]="/.vscode/"
)
# ROS1 版 (colcon が拾わないよう COLCON_IGNORE を置く)
ROS1_DIRS="unitree_lidar_ros"

wanted() {
    case "$1" in
        src/common/*) return 0 ;;
        src/l1/*) [[ "$TARGET" == all || "$TARGET" == l1 ]] ;;
        src/l2/*) [[ "$TARGET" == all || "$TARGET" == l2 ]] ;;
        *) [[ "$TARGET" == all ]] ;;
    esac
}

apply_patch() {
    # サブモジュールを「取り込み元 + 今の版のパッチ」の状態にする.
    # 今の版か過去の版 (git の履歴) のどれかが当たったままなら, その版を外して取り込み元に戻してから今の版を当てる.
    # どの版を外しても取り込み元に戻らない (手で変えたファイルがある) ときは, 何も変えずにエラーにする
    local path=$1 patch=$2 rel=${2#"$REPO"/}
    local dir cand commit found='' n=0
    dir=$(mktemp -d)
    cp "$patch" "$dir/0.patch"
    for commit in $(git -C "$REPO" log --format=%H -- "$rel"); do
        n=$((n + 1))
        git -C "$REPO" show "$commit:$rel" > "$dir/$n.patch" 2>/dev/null || rm -f "$dir/$n.patch"
    done
    for cand in $(ls "$dir" | sort -n); do
        cand="$dir/$cand"
        git -C "$path" apply --whitespace=nowarn --reverse --check "$cand" 2>/dev/null || continue
        git -C "$path" apply --whitespace=nowarn --reverse "$cand"
        if [[ -z "$(git -C "$path" status --porcelain --untracked-files=no)" ]]; then
            found=$cand
            break
        fi
        git -C "$path" apply --whitespace=nowarn "$cand"     # 違う版だったので元に戻す
    done
    if [[ -n "$found" ]] && cmp -s "$found" "$patch"; then
        git -C "$path" apply --whitespace=nowarn "$patch"
        echo "  パッチは当たっている: $rel"
    elif git -C "$path" apply --whitespace=nowarn --check "$patch" 2>/dev/null; then
        git -C "$path" apply --whitespace=nowarn "$patch"
        if [[ -n "$found" ]]; then
            echo "  前の版のパッチを外して, 今の版を当てた: $rel"
        else
            echo "  パッチを当てた: $rel"
        fi
    else
        [[ -n "$found" ]] && git -C "$path" apply --whitespace=nowarn "$found"   # 外した版を戻す
        rm -rf "$dir"
        echo "エラー: $rel を $path に当てられない (手で変えたファイルとぶつかっている?)" >&2
        return 1
    fi
    rm -rf "$dir"
}

git config -f .gitmodules --get-regexp '^submodule\..*\.path$' | while read -r _ path; do
    wanted "$path" || continue
    url=$(git config -f .gitmodules --get "submodule.$path.url")
    sha=$(git ls-files -s -- "$path" | awk '$1 == "160000" {print $2}')
    if [[ -z "$sha" ]]; then
        echo "エラー: $path のコミットが記録されていない" >&2
        exit 1
    fi
    echo "== $path ($url @ ${sha:0:7})"

    if [[ ! -e "$path/.git" ]]; then
        if [[ -n "$(ls -A "$path" 2>/dev/null)" ]]; then
            # 以前このリポジトリに取り込んでいたころの残り (git pull で消えなかった地図 PCD/scans.pcd,
            # __pycache__ など). 消さずに退避する
            backup="$REPO/.external-backup/$path-$(date +%Y%m%d_%H%M%S)"
            mkdir -p "$(dirname "$backup")"
            mv "$path" "$backup"
            mkdir -p "$path"
            echo "  残っていたファイルを ${backup#"$REPO"/} に退避した"
        fi
        git clone -q --filter=blob:none --no-checkout "$url" "$path"
        git -C "$path" sparse-checkout init --no-cone
        patterns=("/*")
        for e in ${EXCLUDE[$path]:-}; do patterns+=("!$e"); done
        git -C "$path" sparse-checkout set --no-cone "${patterns[@]}"
        git -C "$path" -c advice.detachedHead=false checkout -q "$sha"
        echo "  取得した"
    else
        head=$(git -C "$path" rev-parse HEAD)
        if [[ "$head" != "$sha" ]]; then
            if [[ -n "$(git -C "$path" status --porcelain --untracked-files=no)" ]]; then
                echo "エラー: $path は ${head:0:7} で, 変更がある. 記録されている ${sha:0:7} に切り替えられない" >&2
                echo "  変更を残すならパッチにして patches/ へ. 捨てるなら git -C $path checkout -f ." >&2
                exit 1
            fi
            git -C "$path" fetch -q --filter=blob:none origin "$sha" || git -C "$path" fetch -q origin
            git -C "$path" -c advice.detachedHead=false checkout -q "$sha"
            echo "  ${head:0:7} -> ${sha:0:7} に切り替えた"
        fi
    fi

    name=$(basename "$path")
    if [[ -f "patches/$name.patch" ]]; then
        apply_patch "$path" "$REPO/patches/$name.patch"
    fi
    for d in $ROS1_DIRS; do
        while IFS= read -r -d '' dir; do
            touch "$dir/COLCON_IGNORE"
        done < <(find "$path" -type d -name "$d" -not -path '*/.git/*' -print0)
    done
done
