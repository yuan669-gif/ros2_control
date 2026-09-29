# Humble Git workflow

Use the normal .git in D:/2027-1/FineMote/ros2_control.
The old external metadata workflow and its baseline hashes were incorrect. Do not use them.

- Branch: humble (tracks upstream/humble).
- origin: https://github.com/yuan669-gif/ros2_control.git (user fork).
- upstream: https://github.com/ros-controls/ros2_control.git (official source).
- baseline-humble: 469f3055da3b0f097d0616c8b214072434529053.
- v1-hierarchical-prototype: c9e6452a (experimental; not integrated into the manager).
- Research contract: a2ff98a2.

```powershell
cd D:\2027-1\FineMote\ros2_control
git status --short --branch
git log --oneline --decorate -5
git diff baseline-humble..HEAD
```

When publishing is requested, explicitly target origin, not official upstream.
Do not force-push shared history as a routine update.

## New to this work? Read these first

| 想做什么 | 打开 |
|---|---|
| 完全不了解背景，想先懂概念 | `doc/ONBOARDING.md`（入口：三句话版 + 阅读路径 + 术语表 + "不能声称"清单） |
| 想从零读源码 | `doc/CODEBASE_TOUR.md`（一个周期的 trace、四层阅读顺序、最小可跑例子、陷阱清单） |
| 想跑测试 / 复现数字 | `doc/TESTING_GUIDE.md`（环境、命令、套件目录、已知 flaky 的判定方法、测量脚本口径） |
| 想知道开发经过与被推翻的结论 | `doc/DEVELOPMENT_HISTORY.md`（阶段时间线、决策记录、撤回清单） |
| 想知道我们和 FineMote 论文的异同 | `doc/PAPER_ALIGNMENT_2026-09-28.md`（逐条对照 + 判定：一致 / 更窄 / 缺失 / 更强） |
| 想要权威的"当前实现状态 / 还没做什么" | `doc/IMPLEMENTATION_GUIDE.md` §12 + `doc/HANDOFF_MANUAL.md` |

Read doc/HANDOFF_2026-09-19.md for the earliest research status; the newest status is
`doc/IMPLEMENTATION_GUIDE.md` and `doc/HANDOFF_MANUAL.md`.
