#!/usr/bin/env bash
#
# update-from-upstream.sh — 把本 fork 合并到最新上游 sunnypilot
#
# 背景 (为什么需要这个脚本)
# ------------------------
# 车机 setup 界面"输入网址"只是重新克隆 GitHub 上的本仓库
# (lanyima/sp-xiaomi8), 拿到的是你上次 push 的版本 —— 不会带入上游更新。
# 想跟上游, 必须在这台开发机上做一次 merge + 解冲突 + push, 然后车机再装一次。
#
# 用法
# ----
#   tools/update-from-upstream.sh check     只看上游有没有新提交 (只读)
#   tools/update-from-upstream.sh preview   预演: 列出会冲突的文件 (只读)
#   tools/update-from-upstream.sh merge     执行合并; 干净就直接提交, 有冲突则停下
#   tools/update-from-upstream.sh verify    自检仓库是否适合上车机 (只读)
#   tools/update-from-upstream.sh finish    解完冲突后跑这个 (归一化+自检+提交)
#   tools/update-from-upstream.sh push      推送到 origin/main
#   tools/update-from-upstream.sh all       merge + finish + push 一条龙
#
# 环境变量可覆盖: UPSTREAM_REMOTE UPSTREAM_BRANCH TARGET_REMOTE TARGET_BRANCH
#
set -euo pipefail

UPSTREAM_REMOTE="${UPSTREAM_REMOTE:-upstream}"
UPSTREAM_BRANCH="${UPSTREAM_BRANCH:-master}"
TARGET_REMOTE="${TARGET_REMOTE:-origin}"
TARGET_BRANCH="${TARGET_BRANCH:-main}"

if [ -t 1 ]; then
  C_RED=$'\033[31m'; C_GRN=$'\033[32m'; C_YEL=$'\033[33m'; C_CYN=$'\033[36m'; C_RST=$'\033[0m'
else
  C_RED=''; C_GRN=''; C_YEL=''; C_CYN=''; C_RST=''
fi
log()  { printf '%s==>%s %s\n' "$C_CYN" "$C_RST" "$*"; }
ok()   { printf '%s  ✓%s %s\n' "$C_GRN" "$C_RST" "$*"; }
warn() { printf '%s  !%s %s\n' "$C_YEL" "$C_RST" "$*"; }
die()  { printf '%s  ✗%s %s\n' "$C_RED" "$C_RST" "$*" >&2; exit 1; }

REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null)" || die "不在 git 仓库里"
cd "$REPO_ROOT"

UP_REF="$UPSTREAM_REMOTE/$UPSTREAM_BRANCH"

require_clean() {
  # 已跟踪文件的改动是硬性阻止项 —— 合并会覆盖它们。
  local tracked
  tracked="$(git status --porcelain --untracked-files=no)"
  if [ -n "$tracked" ]; then
    warn "已跟踪文件有未提交改动:"
    printf '%s\n' "$tracked" | sed 's/^/      /'
    die "先 commit 或 stash 再跑 (避免合并时丢改动)"
  fi
  # 未跟踪文件只提醒 —— 若与上游新增文件撞路径, git 自己会报错。
  local untracked
  untracked="$(git ls-files --others --exclude-standard)"
  if [ -n "$untracked" ]; then
    warn "有未跟踪文件 (不影响合并, 仅提示):"
    printf '%s\n' "$untracked" | head -10 | sed 's/^/      /'
  fi
  ok "已跟踪文件干净"
}

require_remote() {
  git remote get-url "$UPSTREAM_REMOTE" >/dev/null 2>&1 || die "没有 remote '$UPSTREAM_REMOTE' — 先加:
      git remote add $UPSTREAM_REMOTE https://github.com/sunnypilot/sunnypilot.git"
}

fetch_upstream() {
  require_remote
  log "拉取 $UP_REF"
  git fetch --no-tags "$UPSTREAM_REMOTE" "$UPSTREAM_BRANCH" >/dev/null 2>&1 \
    || die "fetch 失败 — 检查网络"
  ok "$UP_REF = $(git rev-parse --short=10 "$UP_REF")  $(git log -1 --format=%ci "$UP_REF")"
}

summary() {
  local cur up base
  cur="$(git rev-parse HEAD)"
  up="$(git rev-parse "$UP_REF")"
  base="$(git merge-base HEAD "$up" 2>/dev/null || echo "$cur")"
  printf '    HEAD           %s  %s\n' "${cur:0:10}" "$(git log -1 --format=%s HEAD | cut -c1-44)"
  printf '    上游           %s  %s\n' "${up:0:10}" "$(git log -1 --format=%s "$up" | cut -c1-44)"
  printf '    merge-base     %s\n' "${base:0:10}"
  printf '    本分支独有     %s 个提交\n' "$(git rev-list --count "$base"..HEAD)"
  printf '    上游领先       %s 个提交\n' "$(git rev-list --count HEAD.."$up")"
}

behind_count() { git rev-list --count "HEAD..$UP_REF"; }

# ---------------------------------------------------------------- check
cmd_check() {
  fetch_upstream
  echo
  summary
  echo
  local n; n="$(behind_count)"
  if [ "$n" -eq 0 ]; then
    ok "已是最新, 无需操作"
  else
    warn "上游有 $n 个新提交 — 跑: tools/update-from-upstream.sh merge"
  fi
}

# -------------------------------------------------------------- preview
cmd_preview() {
  require_clean
  fetch_upstream
  echo
  summary
  echo
  log "试合并 (只读; 结束后回滚)"
  local rc=0
  git merge --no-commit --no-ff "$UP_REF" >/tmp/.upd_merge.log 2>&1 || rc=$?
  local conflicts; conflicts="$(git diff --name-only --diff-filter=U || true)"
  if [ -n "$conflicts" ]; then
    warn "会冲突 $(printf '%s\n' "$conflicts" | grep -c .) 个文件:"
    printf '%s\n' "$conflicts" | sed 's/^/      /'
    rc=1
  else
    ok "干净合并, 无冲突"
  fi
  git merge --abort 2>/dev/null || git reset --hard HEAD >/dev/null 2>&1 || true
  ok "已回滚到合并前"
  return 0
}

# ---------------------------------------------------------------- merge
cmd_merge() {
  require_clean
  fetch_upstream
  echo
  summary
  echo
  local n; n="$(behind_count)"
  if [ "$n" -eq 0 ]; then ok "已是最新, 无需合并"; return 0; fi

  local bak="refs/backup/pre-merge-$(date +%Y%m%d_%H%M%S)"
  git update-ref "$bak" HEAD
  ok "备份当前 HEAD → $bak  (回滚: git reset --hard $bak)"

  log "合并 $UP_REF ($n 个新提交)"
  git merge --no-commit --no-ff "$UP_REF" >/tmp/.upd_merge.log 2>&1 || true

  local conflicts; conflicts="$(git diff --name-only --diff-filter=U || true)"
  if [ -z "$conflicts" ]; then
    ok "无冲突"
    cmd_finish
    return 0
  fi

  warn "有 $(printf '%s\n' "$conflicts" | grep -c .) 个文件冲突:"
  printf '%s\n' "$conflicts" | sed 's/^/      /'
  echo
  cat <<'GUIDE'
  ── 解决步骤 ──────────────────────────────────────────────
    1. 编辑上面每个文件, 处理 <<<<<<< / ======= / >>>>>>> 标记
       (每个文件里搜 <<<<<<< 就能定位)
    2. git add <解决好的文件>
    3. tools/update-from-upstream.sh finish

  ── 常见冲突的处理原则 (dipper 移植的固定套路) ────────────
    SConstruct / */SConscript    保留上游结构, 把 xiaomi8 的 target 追加进去
    launch_env.sh                保留本机 AGNOS 版本号 (19.5) + NO_DM/NO_WIDE
    locationd/paramsd.py         上游把 liveParameters 改名成 vehicleParameters
    controls/controlsd.py        保留 xiaomi8 的 x=1.0 / sr=CP.steerRatio 安全值
    manager/process_config.py    保留 xiaomi8 的 enabled=False 省电策略
    camerad/cameras/hw.h         保留 dipper 的 ROAD/WIDE_ROAD/DRIVER 三配置
    tools/op.sh                  保留 Ubuntu 版本白名单
    opendbc_repo (子模块指针)     git update-index --cacheinfo 160000,<保留的sha>,opendbc_repo
                                  (取我们这边的 sha: git ls-files -u opendbc_repo)
    .gitattributes / .lfsconfig   直接删掉上游那侧的 (finish 会自动归一化)
    modeld/SConscript            保留 arch=='comma_arm64' → DEV=QCOM
GUIDE
  return 1
}

# --------------------------------------------------------------- finish
post_merge_normalize() {
  # 上游的 .gitattributes 带 filter=lfs, 合并进来会让二进制又变成 LFS 指针。
  # 本仓库刻意把 LFS 内容内联 (车机上 branch_installer 只做 git clone,
  # 不会跑 git lfs pull), 这里把状态拉回来。
  local changed=0
  if [ -f .lfsconfig ]; then
    warn "移除 .lfsconfig (它指向 sunnypilot 的 HuggingFace LFS)"
    git rm -q --cached .lfsconfig 2>/dev/null || true
    rm -f .lfsconfig
    changed=1
  fi
  if grep -q 'filter=lfs' .gitattributes 2>/dev/null; then
    warn ".gitattributes 又出现 filter=lfs — 剥离"
    grep -v 'filter=lfs' .gitattributes > .gitattributes.norm
    mv .gitattributes.norm .gitattributes
    git add .gitattributes
    changed=1
  fi
  if [ "$changed" -eq 1 ]; then
    log "把被转成 LFS 指针的文件恢复成真实字节"
    git add --renormalize -A -- . \
      ':(exclude)openpilot/selfdrive/modeld/models/big_driving_tinygrad.pkl' 2>/dev/null || true
  fi
  ok "LFS 归一化完成"
}

verify_impl() {
  local failed=0

  log "Python 语法"
  python3 - "$PWD" <<'PY' || failed=1
import pathlib, py_compile, sys
root = pathlib.Path(sys.argv[1])
bad = []
n = 0
for p in root.rglob('*.py'):
    s = str(p)
    if '/.git/' in s or '__pycache__' in s:
        continue
    n += 1
    try:
        py_compile.compile(s, doraise=True, cfile='/tmp/.updchk.pyc')
    except Exception as e:
        bad.append((s.replace(str(root) + '/', ''), str(e)[:110]))
print(f"    {n} 个文件, {len(bad)} 个失败")
for f, e in bad[:20]:
    print(f"      ✗ {f}: {e}")
sys.exit(1 if bad else 0)
PY

  log "Shell 语法"
  local shbad=0
  while IFS= read -r f; do
    bash -n "$f" 2>/dev/null || { echo "      ✗ $f"; shbad=$((shbad+1)); }
  done < <(git ls-files '*.sh')
  if [ "$shbad" -eq 0 ]; then ok "全部 .sh 通过"; else warn "$shbad 个 .sh 失败"; failed=1; fi

  log "残留 LFS 指针"
  # 唯一例外: big_driving_tinygrad.pkl 有 740 MB, 超过 GitHub 100 MB 单文件上限,
  # 只能留指针。车机端不生成这个文件, 缺它不影响运行。
  local allow='openpilot/selfdrive/modeld/models/big_driving_tinygrad.pkl'
  local ptr=0
  while IFS= read -r f; do
    if [ -f "$f" ] && head -c 40 "$f" 2>/dev/null | grep -q 'git-lfs.github.com/spec'; then
      if [ "$f" = "$allow" ]; then
        echo "      · $f (已知例外: 740MB 超 GitHub 上限, 车机不需要)"
      else
        echo "      ⚠ $f"; ptr=$((ptr+1))
      fi
    fi
  done < <(git ls-files)
  if [ "$ptr" -eq 0 ]; then ok "无意外 LFS 指针 (全部真实内容)"; else warn "$ptr 个文件仍是指针"; failed=1; fi

  log "branch_installer"
  if [ -f branch_installer ]; then
    if [ "$(head -c4 branch_installer | od -An -tx1 | tr -d ' \n')" = "7f454c46" ]; then
      ok "是合法 ELF ($(stat -c%s branch_installer) 字节, md5 $(md5sum branch_installer | cut -c1-12))"
    else
      warn "不是 ELF! setup 会报 'No custom software found'"; failed=1
    fi
  else
    warn "branch_installer 不存在 — 车机将无法用网址安装"; failed=1
  fi

  grep -q 'filter=lfs' .gitattributes 2>/dev/null && { warn ".gitattributes 仍有 filter=lfs"; failed=1; }
  [ -f .lfsconfig ] && { warn ".lfsconfig 仍存在"; failed=1; }

  return $failed
}

cmd_verify() {
  log "自检 (只读, 不改任何文件)"
  echo
  if verify_impl; then
    echo
    ok "全部通过 — 仓库状态适合上车机"
  else
    echo
    die "自检未通过 (见上面 ⚠ 项)"
  fi
}

cmd_finish() {
  local conflicts; conflicts="$(git diff --name-only --diff-filter=U || true)"
  if [ -n "$conflicts" ]; then
    warn "还有未解决的冲突:"
    printf '%s\n' "$conflicts" | sed 's/^/      /'
    die "先把它们解完并 git add"
  fi

  if ! git rev-parse -q --verify MERGE_HEAD >/dev/null 2>&1; then
    # 不在 merge 中 — 可能是干净合并, 或已提交
    if git diff --cached --quiet 2>/dev/null && git diff --quiet 2>/dev/null; then
      ok "没有待提交内容"
    fi
  fi

  post_merge_normalize

  if ! verify_impl; then
    die "自检未通过 — 修好再跑 finish"
  fi

  if git rev-parse -q --verify MERGE_HEAD >/dev/null 2>&1 || ! git diff --cached --quiet 2>/dev/null; then
    log "生成合并提交"
    git commit --no-edit
    ok "已提交 $(git rev-parse --short=10 HEAD)"
  else
    ok "无需提交"
  fi
}

# ----------------------------------------------------------------- push
cmd_push() {
  local conflicts; conflicts="$(git diff --name-only --diff-filter=U || true)"
  [ -n "$conflicts" ] && die "还有未解决的冲突"

  log "推送 HEAD → $TARGET_REMOTE/$TARGET_BRANCH"
  git push "$TARGET_REMOTE" "HEAD:$TARGET_BRANCH"
  ok "已推送 $(git rev-parse --short=10 HEAD)"
  echo
  cat <<'NEXT'
  下一步: 车机上重新输入安装器网址

      https://raw.githubusercontent.com/lanyima/sp-xiaomi8/main/branch_installer

  车机会备份 /data/openpilot 后重新克隆, 约 1-2 分钟 +
  首次启动编译 (20-40 分钟).
NEXT
}

# ------------------------------------------------------------------ all
cmd_all() { cmd_merge && cmd_push; }

usage() {
  # 打印文件顶部的注释块 (第 2 行起, 遇到第一行非注释就停)
  awk 'NR > 1 { if (/^#/) { sub(/^# ?/, ""); print } else exit }' "$0"
  exit 1
}

case "${1:-}" in
  check)   cmd_check ;;
  preview) cmd_preview ;;
  merge)   cmd_merge ;;
  verify)  cmd_verify ;;
  finish)  cmd_finish ;;
  push)    cmd_push ;;
  all)     cmd_all ;;
  ""|-h|--help|help) usage ;;
  *) die "未知子命令: $1  (用 --help 看用法)" ;;
esac
