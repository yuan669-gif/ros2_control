# Humble Git workflow

Use the normal .git in D:/2027-1/FineMote/ros2_control.
The old external metadata workflow and its baseline hashes were incorrect. Do not use them.

- Branch: humble (tracks upstream/humble).
- origin: https://github.com/yuan669-gif/ros2_control.git (user fork).
- upstream: https://github.com/ros-controls/ros2_control.git (official source).
- baseline-humble: 469f3055da3b0f097d0616c8b214072434529053 = **上游 Humble 的提交**，它是
  `origin/humble`（fork 的 humble 分支）的祖先，所以 `git fetch origin humble` 之后本地就能解析它：
  `git diff --stat 469f3055 HEAD` 是"上游 Humble → 本项目"的完整对比（当前 149 文件、+36454/−98）。
  注意两点：① 本分支自己的历史是**独立**的（根提交 `b1bf616` 已包含上游源码树 + 第一版内核），
  所以 `git log --oneline` 列出的 31 个提交**就是**本项目的工作；② `origin/humble` 比上游多 4 个
  早期提交（v1 原型 `c9e6452`、研究文档 `a2ff98a`/`c74c1cd`、cycle_tree 实验 `ea3992e`），
  它们**不在** `humble-work` 里（那条原型已由 `hierarchical_control` 重写取代）。
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
| **想上手用起来**（装进 ROS、开启分层树、选库模式/管理器模式） | `doc/USER_GUIDE.md` |
| 完全不了解背景，想先懂概念 | `doc/ONBOARDING.md`（入口：三句话版 + 阅读路径 + 术语表 + "不能声称"清单） |
| 想从零读源码 | `doc/CODEBASE_TOUR.md`（一个周期的 trace、四层阅读顺序、最小可跑例子、陷阱清单） |
| 想跑测试 / 复现数字 | `doc/TESTING_GUIDE.md`（环境、命令、套件目录、已知 flaky 的判定方法、测量脚本口径） |
| 想知道开发经过与被推翻的结论 | `doc/DEVELOPMENT_HISTORY.md`（阶段时间线、决策记录、撤回清单） |
| **接下来做什么**（待办清单，含"两趟独立分支"的方案） | `doc/PENDING_WORK.md` |
| **要给别人汇报 / 一页看懂全貌** | `doc/FINAL_REPORT_2026-09-28.md`（最终报告：问题、方法、证据表、不能声称、下一步） |
| 想知道我们和 FineMote 论文的异同 | `doc/PAPER_ALIGNMENT_2026-09-28.md`（逐条对照 + 判定：一致 / 更窄 / 缺失 / 更强） |
| 想要权威的"当前实现状态 / 还没做什么" | `doc/IMPLEMENTATION_GUIDE.md` §12 + `doc/HANDOFF_MANUAL.md` |

Read doc/HANDOFF_2026-09-19.md for the earliest research status; the newest status is
`doc/IMPLEMENTATION_GUIDE.md` and `doc/HANDOFF_MANUAL.md`.
