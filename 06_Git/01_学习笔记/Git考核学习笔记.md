# Git 学习笔记

姓名：王世伟　创建日期：2026-10-04

本笔记根据本人考核操作、截图与追问整理，由 Codex 辅助编写. 重点记录如何使用命令行、VS Code 和 Git Graph

## 1. 任务与使用环境

本次使用自己的 PWM_Breath 和 UART_DMA Keil 工程完成版本管理练习.

| 项目 | 本次使用 |
| --- | --- |
| 命令行 | Git Bash，截图终端标识为 UCRT64 |
| 图形界面 | VS Code，中文界面 |
| 辅助扩展 | Git Graph，发布者 mhutchie，英文菜单 |
| 远程仓库 | https://github.com/Wang-666-0/git-assessment |
| 初始本地仓库 | `D:\Project\27电软正式批考核\06_Git` |
| 本地副本 A | `D:\Project\git-assessment-A` |
| 本地副本 B | `D:\Project\git-assessment-B` |
| 主分支 | `main` |

考核内容包括创建仓库、暂存、提交、推送、克隆、忽略规则、reset、revert、分支合并及双仓库协作. 除创建仓库和解决冲突外，对应操作还需要用 VS Code 演示.

以下命令按 Git Bash 写法记录. 示例用于复习流程，已经完成的操作不需要重新执行.

## 2. 创建仓库、暂存和提交

先将自己的 Keil 工程复制到练习目录，再进入目录执行：

```bash
git init
git status
```

`git init` 将当前目录初始化为仓库，通常会生成隐藏目录 `.git`. 它是普通本地仓库的标志，保存版本管理信息，不要随意删除. 本次初始化后的分支是 `main`.

`git status` 查看当前状态. `Untracked files` 表示文件还没有被 Git 跟踪，`No commits yet` 表示尚未提交任何版本.

先编写下一节的 `.gitignore`，然后暂存并提交：

```bash
git add .
git status
git commit -m "首次提交PWM和串口工程"
git log --oneline
```

- `git add .`：暂存当前目录及其子目录中的改动，遵守忽略规则.
- `git commit -m`：将暂存区内容保存为一次提交，引号中填写提交说明.
- `git log --oneline`：每笔提交显示一行，方便查看版本.

使用时可记为：修改文件 → 保存 → 暂存 → 提交 → 推送.

## 3. .gitignore 的使用与失效处理

在仓库根目录新建 `.gitignore`，本次用于忽略 Keil 编译产物和本机记录：

```gitignore
# Keil 编译输出目录
Objects/
Listings/

# 编译产物
*.o
*.d
*.crf
*.axf
*.hex
*.map
*.lnp
*.lst
*.dep
*.build_log.htm

# 本机界面与调试记录
*.uvguix.*
*.uvoptx
*.scvd
```

每条规则单独占一行. `.gitignore` 自己需要提交，工程源码、`.ioc` 和 `.uvprojx` 等工程文件需要保留.

### 验证规则生效

```bash
echo "test" > ignore-test.axf
git status --short
git check-ignore -v ignore-test.axf
```

`echo` 输出文字，`>` 将文字写入文件并覆盖原内容. `git check-ignore -v` 显示匹配的忽略文件、行号和规则. 测试文件存在，但不会出现在普通的待提交列表中.

### 文件已经提交过，为什么添加忽略规则还不生效?

`.gitignore` 主要作用于未跟踪文件. 已经被跟踪的文件，需要先停止跟踪.

为了演示，先创建并提交 `ignore-demo.txt`，再把这个文件名加入 `.gitignore`，之后执行：

```bash
git rm --cached ignore-demo.txt
git add .gitignore
git commit -m "test: 停止跟踪忽略演示文件"
git check-ignore -v ignore-demo.txt
git status --short
```

`git rm --cached` 从 Git 跟踪记录中移除文件，但保留本地文件. 提交中出现 `delete mode`，表示新版本不再包含这个文件，并不表示本地文件被删除. 历史提交中的文件仍然保留.

### 这次遇到的换行问题

曾将规则追加成：

```text
*.scvdignore-demo.txt
```

这是一条错误的合并规则，无法匹配 `ignore-demo.txt`. 应改为：

```gitignore
*.scvd
ignore-demo.txt
```

修正后重新暂存、提交 `.gitignore`. `git check-ignore -v ignore-demo.txt` 已显示匹配规则.

忽略规则修正与验证图：________

## 4. 添加远程、推送和克隆

在 GitHub 创建空仓库后，将地址关联到本地：

```bash
git remote add origin https://github.com/Wang-666-0/git-assessment.git
git remote -v
git push -u origin main
```

- `origin`：远程地址的本地简称，并不是 GitHub 仓库名称.
- `remote -v`：检查远程地址.
- `push`：将本地提交上传到远程.
- `-u`：建立本地 `main` 与远程 `main` 的跟踪关系，后续可直接执行 `git push`.

首次推送显示 `main -> main` 和跟踪关系设置成功，GitHub 页面能看到工程文件，说明上传成功.

### 克隆成另一份本地仓库

```bash
cd /d/Project
git clone https://github.com/Wang-666-0/git-assessment.git git-assessment-A
cd git-assessment-A
git log --oneline
```

`clone` 下载代码和提交历史，在 `D:\Project` 下创建 `git-assessment-A`，并自动配置 `origin`. 它不在原来的 `06_Git` 文件夹里面.

### cd 和 checkout 有什么区别?

`cd` 切换终端当前文件夹. `git checkout` 是 Git 操作，可以切换分支或版本，也可以恢复文件. 二者作用不同.

```bash
cd /d/Project/git-assessment-A
explorer .
```

第二条打开当前目录的资源管理器，方便确认自己操作的是哪一份仓库.

## 5. reset 回退、恢复与更新远程

### 回退一次代码

确认没有待保存的修改，再准备演示提交：

```bash
git status --short
echo "// Git reset demo" >> UART_DMA/Core/Src/main.c
git add UART_DMA/Core/Src/main.c
git commit -m "test: 添加reset演示注释"
git branch reset-backup
git reset --hard HEAD~1
```

`>>` 在文件末尾追加内容. `reset-backup` 记录回退前位置. `HEAD` 是当前提交，`HEAD~1` 是它的上一笔提交. `reset --hard` 同时恢复分支位置、暂存区和工作区，会丢弃未提交的已跟踪文件改动.

### 取消本地 reset

```bash
git reset --hard reset-backup
```

作用：回到备份分支记住的版本. 取消 reset **本质**是再次把分支移动到回退前的位置.

### reset 后更新远程

先推送恢复后的演示提交，再回退本地并更新远程：

```bash
git push origin main
git reset --hard HEAD~1
git push --force-with-lease origin main
```

普通推送不能直接让远程历史倒退. `--force-with-lease` 允许更新历史，但会检查远程是否仍处于本地记录的位置，避免覆盖未知的远程更新. 本次在自己的练习仓库演示.

终端出现 `forced update`，表示远程回退完成. 回退后不要点击 VS Code 的“同步更改”，否则它可能先拉取远程，把旧提交重新带回来.

reset 及远程回退结果图：________

### 三种 reset 模式怎么选?

| 模式 | 分支位置 | 暂存区 | 文件内容 |
| --- | --- | --- | --- |
| Soft | 回退 | 保留 | 保留 |
| Mixed | 回退 | 恢复到目标提交 | 保留 |
| Hard | 回退 | 恢复到目标提交 | 恢复到目标提交 |

本次界面操作中，分支回退后曾仍显示 `Uncommitted Changes` 和文件 `M`，说明还有未提交改动，不能只看分支位置就认定文件恢复完成. 选择 Hard 后，未提交改动消失.

## 6. revert 撤销与冲突处理

### 撤销最新提交

```bash
echo "// Git revert demo" >> UART_DMA/Core/Src/main.c
git add UART_DMA/Core/Src/main.c
git commit -m "test: 添加revert演示注释"
git revert --no-edit HEAD
git log --oneline -3
git push origin main
```

`revert` 撤销指定提交的改动，并新增一笔反向提交. `--no-edit` 使用默认说明，不打开编辑器. 原来的演示提交仍在，因此可以普通推送.

| 对比 | reset | revert |
| --- | --- | --- |
| 做法 | 将分支移到指定版本 | 新增提交撤销某笔改动 |
| 原提交历史 | 后面的提交不再属于该分支当前历史 | 保留 |
| 已推送版本的回退 | 通常涉及强制推送 | 通常普通推送即可 |

### 人为制造 revert 冲突

先追加并提交 `// Git conflict demo: version 1`，再把同一行改成 `version 2` 并提交. 然后执行：

```bash
git revert --no-edit HEAD~1
```

撤销上一笔“添加注释”的提交需要删除这行，但最新提交修改了它，Git 因此暂停，显示 `CONFLICT` 和 `main|REVERTING`.

### 手动解决并继续

```bash
notepad UART_DMA/Core/Src/main.c
```

文件中会出现类似标记：

```c
<<<<<<< HEAD
// Git conflict demo: version 2
=======
>>>>>>> ...
```

`HEAD` 一侧是当前内容，另一侧是撤销希望得到的内容. 本次选择删除演示注释，删除整个对应冲突区域的标记和注释，保留其他原有代码，保存后执行：

```bash
git add UART_DMA/Core/Src/main.c
git -c core.editor=true revert --continue
git status
git push origin main
```

`add` 标记冲突已解决. `revert --continue` 完成暂停的撤销. `-c core.editor=true` 仅本次临时指定一个直接返回成功的编辑器命令，保留默认提交说明，不弹出编辑窗口. 它不会自动处理冲突.

如果要放弃这次撤销并恢复操作前状态，执行 `git revert --abort`. 关闭错误弹窗不等于取消撤销.

## 7. 创建分支、切换和合并

### 创建与切换

```bash
git branch feature-demo
git switch feature-demo
git branch
git switch main
```

`branch feature-demo` 只创建分支，`switch` 才切换. `git branch` 列出本地分支，星号表示当前分支.

创建并立即切换可以合成：

```bash
git switch -c 新分支名
```

### switch 和 checkout 的区别

切换已有分支时，`git switch 分支名` 与 `git checkout 分支名` 都可以. 创建并切换分别用 `switch -c` 和 `checkout -b`.

`switch` 专门用于分支切换，`checkout` 还可以操作历史版本和恢复文件. 日常切分支优先使用 `switch`.

### 制造合并冲突

在 main 准备初始文件：

```bash
echo "// Git merge demo: base" > merge-demo.c
git add merge-demo.c
git commit -m "test: 添加合并演示文件"
git switch -c merge-demo
```

在新分支将同一行改为 feature 并提交：

```bash
echo "// Git merge demo: feature" > merge-demo.c
git add merge-demo.c
git commit -m "test: 分支修改演示注释"
```

回 main 修改同一行并合并：

```bash
git switch main
echo "// Git merge demo: main" > merge-demo.c
git add merge-demo.c
git commit -m "test: 主分支修改演示注释"
git merge merge-demo
```

`merge` 将指定分支合并到当前分支. 两边改了同一行，出现 `CONFLICT`，状态变为 `main|MERGING`.

打开 `merge-demo.c`，删除冲突标记，将最终内容确定为：

```c
// Git merge demo: main and feature
```

再执行：

```bash
git add merge-demo.c
git commit -m "test: 合并merge-demo并解决冲突"
git log --oneline --graph -5
git push origin main
```

解决冲突就是决定最终内容，可以保留一侧、双方或重新组织内容. `--graph` 用线条显示提交和分支关系.

## 8. 两份本地仓库协作

### 从相同版本开始

先让 A 推送完成，再克隆 B：

```bash
cd /d/Project
git clone https://github.com/Wang-666-0/git-assessment.git git-assessment-B
```

### A 先提交并推送

```bash
cd /d/Project/git-assessment-A
echo "// Git collaboration demo: A" >> merge-demo.c
git add merge-demo.c
git commit -m "test: A添加协作演示注释"
git push origin main
```

### B 在旧版本上提交并推送

```bash
cd /d/Project/git-assessment-B
echo "// Git collaboration demo: B" >> merge-demo.c
git add merge-demo.c
git commit -m "test: B添加协作演示注释"
git push origin main
```

出现 `[rejected]`、`fetch first` 或 `non-fast-forward`，表示远程有 B 尚未整合的提交. B 的本地提交没有丢失. 此时是历史分歧，还不一定出现文件内容冲突，不应通过强制推送覆盖 A 的工作.

### 拉取、解决冲突、重新推送

```bash
git pull --no-rebase origin main
```

`pull` 获取并整合远程提交，`--no-rebase` 明确使用合并方式. 本次双方修改相同位置，合并产生内容冲突.

编辑 B 的 `merge-demo.c`，删除冲突标记，整理为：

```c
// Git merge demo: main and feature
// Git collaboration demo: A
// Git collaboration demo: B
```

```bash
git add merge-demo.c
git commit -m "test: 合并A和B的修改并解决冲突"
git push origin main
```

最后让 A 获取合并结果：

```bash
cd /d/Project/git-assessment-A
git pull --ff-only origin main
git status
cat merge-demo.c
```

`--ff-only` 只允许直接向前更新. 本次 A 没有额外新提交，显示 Fast-forward，成功同步. `cat` 显示文件内容.

双仓库推送拒绝与最终同步图：________

## 9. VS Code 界面操作速查

使用“文件 → 打开文件夹”打开目标仓库. 左侧“源代码管理”快捷键为 `Ctrl + Shift + G`，命令面板为 `Ctrl + Shift + P`.

| 操作 | 本次使用方法 |
| --- | --- |
| 暂存 | 保存文件，在更改列表点击文件旁的 `+` |
| 提交 | 输入说明，点击“提交” |
| 推送 | 源代码管理 `…` 中选择“推送”，或命令面板 Git: Push |
| 拉取 | `…` 中选择“拉取”，确认正确的远程 |
| 添加远程 | Git: Add Remote，填写地址与本地简称 |
| 创建分支 | 点击左下角分支名称，选择“创建新分支” |
| 切换分支 | 点击左下角分支名称，选择已有分支 |
| 合并分支 | Git: Merge Branch，选择要合入当前分支的分支 |
| 克隆 | Git: Clone，输入地址或从 GitHub 选择仓库 |
| 查看历史 | 内置源代码管理图表，或 Git Graph |

菜单可能随版本与汉化方式不同，用命令面板搜索对应 Git 操作即可.

### 推送与同步有什么区别?

推送上传本地提交. “同步更改”会先拉取再推送. `1↑` 表示有待推送提交，`1↓` 表示有已知的远程提交待整合，计数以当前获取到的远程信息为准.

演示推送被拒绝时，要选择“推送”，不能提前点同步，否则可能自动整合远程内容，无法展示拒绝过程.

### VS Code 的忽略规则验证

在 `.gitignore` 末尾另起一行加入 `vscode-ignore-test.txt`，创建并保存同名文件. 更改列表只出现 `.gitignore`，测试文件不出现，说明规则生效. 暂存、提交、推送 `.gitignore`.

### 添加远程的界面演示

保留已有 `origin`，使用 Git: Add Remote 为相同地址新增简称 `vscode-demo`. 添加远程只保存地址，不会新建 GitHub 仓库，也不会自动上传. 后续协作练习选择 `origin`.

### 为什么选择第一个 GitHub 仓库后“没反应”?

本次在 GitHub 选择入口选中 git-assessment 后，没有要求选择保存目录. 关闭当前文件夹再操作，出现“是否要打开存储库”，打开后又回到已有 A 仓库. 据此判断，该次入口复用了已有本地仓库，没有创建新副本.

这不代表 Git 禁止克隆已经打开的仓库. 相同远程可以有多份本地副本，A、B 就是例子. 遇到类似现象应确认打开的文件夹位置和 Git 输出日志，不要仅凭界面没有变化认定克隆成功或失败.

### Git Graph 的 reset 和 revert

内置图表没有找到 reset 操作，因此安装 Git Graph 完成图形界面演示. 本次原版菜单为英文，操作名称对照如下：

| 英文 | 作用 |
| --- | --- |
| Create Branch | 创建分支 |
| Checkout | 切换分支或版本 |
| Reset current branch to this Commit | 将当前分支重置到这笔提交 |
| Revert | 新增提交，撤销这笔改动 |
| Merge into current branch | 合并到当前分支 |
| Push Branch | 推送分支 |

本次 reset 流程：创建演示提交 → 创建备份分支 → 切回 main → 右键上一笔提交执行 reset → 重置到备份提交恢复 → 推送演示提交 → 再回退 → 强制推送使远程回退.

选择 Hard 前确认没有需要保留的未提交修改. 本次备份位置是 `664d1b1c`，回退目标是 `a4cb3891`，这些提交号只用于对应本次记录.

曾误点 Revert 导致冲突，通过 VS Code 终端执行 `git revert --abort` 取消后重新操作. Reset 与 Revert 是不同功能，不能混用.

图形界面执行 revert 时，另建一笔演示提交，右键最新提交选择 Revert. 成功后原提交仍在，顶部新增 Revert 提交，再普通推送.

VS Code reset、备份恢复与 revert 图：________

### VS Code 双仓库协作

先在 B 拉取，让双方从相同版本开始. A 创建 `vscode-collab-A.txt`，提交并推送. B 不先拉取，独立创建 `vscode-collab-B.txt`，提交后选择推送，出现拒绝提示.

B 再拉取，合并两边的提交后推送. 最后 A 拉取. 两边新增的是不同文件，正常情况下不需要手动解决内容冲突. 最终两份仓库均包含两个文件，B 图表显示合并提交，待同步计数消失.

本次最终截图中的两个文件为空，不影响展示 Git 的提交与整合流程，未据此记录文件内已有文字.

VS Code 双仓库协作完成图：________

## 10. 常用指令汇总

| 指令 | 用途 |
| --- | --- |
| `cd 路径` | 切换终端目录 |
| `echo "文字" > 文件` | 创建文件或覆盖内容 |
| `echo "文字" >> 文件` | 在末尾追加内容 |
| `cat 文件` | 查看文件内容 |
| `notepad 文件` | 用记事本编辑 |
| `explorer .` | 打开当前文件夹 |
| `git init` | 创建本地仓库 |
| `git status` | 查看状态 |
| `git status --short` | 简洁显示改动 |
| `git add 文件` | 暂存指定文件 |
| `git add .` | 暂存当前目录范围的改动 |
| `git add -A` | 暂存整个仓库的新增、修改和删除 |
| `git commit -m "说明"` | 保存暂存内容 |
| `git log --oneline -5` | 查看最近五笔提交 |
| `git log --oneline --graph --all` | 查看各分支的提交关系 |
| `git diff` | 查看未暂存改动 |
| `git diff --cached` | 查看已暂存改动 |
| `git remote add 名称 地址` | 添加远程 |
| `git remote -v` | 查看远程地址 |
| `git push -u origin main` | 首次推送并建立跟踪关系 |
| `git push origin main` | 推送 main |
| `git fetch origin` | 获取远程信息，不直接合并工作区 |
| `git pull --no-rebase origin main` | 获取并用合并方式整合远程 |
| `git pull --ff-only origin main` | 只允许直接向前更新 |
| `git clone 地址 目录名` | 克隆到新目录 |
| `git check-ignore -v 文件` | 检查匹配的忽略规则 |
| `git rm --cached 文件` | 停止跟踪，保留本地文件 |
| `git branch` | 查看本地分支 |
| `git branch 名称` | 创建分支 |
| `git switch 名称` | 切换分支 |
| `git switch -c 名称` | 创建并切换分支 |
| `git checkout 名称` | 另一种切换分支写法 |
| `git merge 分支名` | 合并到当前分支 |
| `git merge --abort` | 取消正在进行的冲突合并 |
| `git reset --hard 目标` | 重置分支、暂存区和文件 |
| `git reflog` | 查询本地分支位置变化，辅助找回回退前提交 |
| `git push --force-with-lease origin main` | 在远程位置检查通过后更新历史 |
| `git revert --no-edit 提交` | 用新提交撤销改动 |
| `git revert --continue` | 解决冲突后继续撤销 |
| `git revert --abort` | 取消当前撤销 |

## 11. 本次容易遇到的问题

| 提示或现象 | 原因与处理 |
| --- | --- |
| `pathspec ... did not match any files` | 文件名或路径写错，本次将 demo 写成 dome |
| `unknown option 'cashed'` | 参数拼错，正确写法为 `--cached` |
| `no path specified` | `check-ignore` 后没有提供文件路径 |
| `?? 文件` | 文件未被跟踪，若预期忽略则检查规则与换行 |
| `LF will be replaced by CRLF` | 换行转换提醒，本次不影响暂存与提交 |
| `nothing added to commit` | 没有暂存内容，先检查 `git add` 是否成功 |
| `fetch first` / `non-fast-forward` | 远程有未整合提交，先拉取并处理 |
| `CONFLICT` | 文件内容需要人工确定，处理后暂存并完成操作 |
| `main|MERGING` | 合并尚未结束 |
| `main|REVERTING` | 撤销尚未结束 |
| reset 后文件仍标记 M | 分支移动了，但工作区仍有修改，检查重置模式 |
| 关闭错误弹窗后仍冲突 | 弹窗只显示信息，要继续完成操作或执行 abort |

最后检查本地状态与远程位置，确认更改列表为空、没有待推送提交. 考核演示时重点说明自己处在哪份仓库、当前分支是什么、操作前后版本如何变化.
