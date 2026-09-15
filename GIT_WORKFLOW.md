# Git workflow for the Humble prototype

This directory is based on the ROS 2 Humble `ros2_control` branch.

Because the downloaded repository's `.git` directory is read-only in the current workspace, the
writable Git metadata is stored outside the source tree:

```text
D:/2027-1/FineMote/.git-ros2control-humble
```

Use these commands from `D:/2027-1/FineMote`:

```powershell
$meta = 'D:/2027-1/FineMote/.git-ros2control-humble'
$work = 'D:/2027-1/FineMote/ros2_control_humble'
git --git-dir=$meta --work-tree=$work status
git --git-dir=$meta --work-tree=$work log --oneline --decorate --all
git --git-dir=$meta --work-tree=$work diff baseline-humble..master
```

Branches/commits:

- `baseline-humble` (`17f8df7`): unmodified upstream ROS 2 Humble source tree.
- `master` (`9a8f041`): first hierarchical-controller prototype, followed by cleanup of temporary
  metadata.

For future changes, commit to `master` and compare against `baseline-humble` before each milestone.
