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

Read doc/HANDOFF_2026-09-19.md for research status and next steps.
