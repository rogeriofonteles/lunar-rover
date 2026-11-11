# Python editing in Neovim

The host Neovim configuration uses Pyright with Neovim's built-in LSP completion.
Open Python files from this checkout normally; `pyrightconfig.json` supplies the
library paths. After changing the Python setup, run `:PythonReload` to apply it
without closing Neovim or its Zellij panel. This reloads Python shortcuts and
completion in open Python buffers; it is not a full Neovim restart.

Press Space and then the letter in Normal mode (not simultaneously).

| Mode | Shortcut | Action |
| --- | --- | --- |
| Normal | `Space k` | Documentation and types under the cursor |
| Normal | `Space d` | Go to definition |
| Normal | `Space b` / `Space f` | Jump back / forward |
| Normal | `Space s` | Function signature / parameters |
| Normal | `Space r` | Find references |
| Normal | `Space n` | Rename symbol |
| Normal | `Space e` | Focus explorer (workbench) |
| Normal | `Space ve` | Show diagnostic |
| Insert | `Ctrl+Space` | Request completion |
| Insert | `Ctrl+F` / `Ctrl+D` | Select next / previous completion |
| Insert | `Tab` (or `Ctrl+Y`) | Accept completion; Tab selects the first if none is selected |
| Insert | `Ctrl+E` | Dismiss completion |
| Insert | `Ctrl+L` | Function signature / parameters |

Completion also opens automatically as you type. These keys are available in
the host's current Zellij Normal mode; return to that mode before editing.
Ctrl+F/D retain their normal Insert-mode behavior when no completion menu is
open. Space is only a shortcut prefix in Normal mode, so typing spaces in
Insert mode is unaffected. Tab indents normally when the completion menu is closed.

## Libraries from Docker

The ignored `.python-libs/` cache contains Python sources and type stubs copied
from the Drake container and ROS Jazzy image. It is for editor analysis only;
continue running the simulations in Docker. No container needs to stay running
for completion after the cache has been created.

After installing or upgrading container libraries, refresh the cache:

```sh
python3 scripts/sync_python_editor_libs.py
```

The defaults are the running container `lunar-rover-jazzy-drake` and local image
`lunar-rover-jazzy-ros:v0.0.3`. Override them using `--drake-container` and
`--ros-image`, or refresh just one with `--only drake` / `--only ros`.
The script requires host Python 3.12+ and Docker access.

Drake is analyzed as Python 3.14 and ROS Jazzy as Python 3.12. Strict type checking
is disabled because this setup focuses on completion and navigation; unresolved
imports are still reported. Compiled Drake functions generally jump to their
`.pyi` API declarations, while Python functions can jump to copied source.
Documentation availability depends on what the installed library exposes in
its sources and stubs.

The legacy Curiosity scripts use ROS 1 `rospy` and `gazebo_msgs`, which are absent
from the Jazzy image. Those imports still show warnings and need a separate
ROS 1 editor environment for full completion; a ROS 2 cache cannot provide their
ROS 1 APIs.

## Local editor configuration

The configuration lives in `~/.config/nvim/lua/python_lsp.lua`, loaded by
`~/.config/nvim/init.vim`. Pyright is installed under
`~/.local/share/nvim/python-lsp/`. The original `init.vim` is backed up as
`~/.config/nvim/init.vim.before-python-lsp`.

The separate `nvim-workbench` profile also loads this shared Python configuration
from its `init.lua`, so the Zellij workbench uses the same setup. Python buffers
use native completion there; Blink remains available for other file types.

Check the language server with `:checkhealth vim.lsp` inside a Python buffer.
The setup uses Neovim 0.11+ APIs and was verified with this host's Neovim 0.13
development build.
