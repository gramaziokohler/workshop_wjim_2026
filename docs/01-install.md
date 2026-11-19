<p class="step">STEP 01 / 07</p>

# Install

| You need | |
|---|---|
| [Rhino 8](https://www.rhino3d.com/download/) | with Grasshopper |
| [VS Code](https://code.visualstudio.com/download) | the `code` command on your PATH ([macOS](https://code.visualstudio.com/docs/setup/mac#_launch-vs-code-from-the-command-line)) |
| [git](https://git-scm.com/downloads) | or [GitHub Desktop](https://desktop.github.com/) |

## 1 · Clone the repo

```bash
git clone https://github.com/gramaziokohler/workshop_wjim_2026.git
cd workshop_wjim_2026
```

No git? **[GitHub Desktop](https://desktop.github.com/) → File → Clone repository** → paste the URL.

!!! tip "Pick a path without spaces or special characters"
    e.g. `~/workshop_wjim_2026` or `C:\Users\you\workshop_wjim_2026`.
    Moved the folder later? Run `00_install.ghx` again.

## 2 · Rhino: open `00_install.ghx`

Open the file from the repo root in Grasshopper and wait.
It builds one Python environment, **`wjim_ncyu2026`**, and prints what it installed.

<div class="shot-todo">00_install.ghx on the canvas, its panel listing compas_timber 3.0.2, compas_fab 2.1.0 …</div>

!!! tip "Takes a few minutes, once"
    Every other workshop file only *uses* the environment (`# venv: wjim_ncyu2026`).
    Restart Rhino when it's done.

## 3 · VS Code: install the joint editor

=== "macOS"

    ```bash
    code --install-extension joint-editor-0.1.0.vsix --force
    ```

=== "Windows (PowerShell)"

    ```powershell
    code.cmd --install-extension joint-editor-0.1.0.vsix --force
    ```

Or skip the terminal: in the Extensions view, **⋯ → Install from VSIX…** and pick the file.

Then <kbd>Cmd</kbd>/<kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd> → **Developer: Reload Window**.

## 4 · VS Code: open the repo root

**File → Open Folder…** → the repo itself, not `joinery/`.
Open `joinery/corner_node.py` once.

The extension downloads its own Python and `compas_timber`. When the status bar says
**compas_timber 3.0.2**, you're set.

![VS Code status bar: compas_timber 3.0.2 · release](assets/shots/statusbar.png){ width="320" }

??? failure "Something's off?"
    | Symptom | Fix |
    |---|---|
    | Rhino reinstalls over and over / venv broken | close Rhino, delete `~/.rhinocode/py39-rh8/site-envs/wjim_ncyu2026-*`, open `00_install.ghx` again |
    | `No module named 'wjim_lib'` | open `00_install.ghx` again, restart Rhino |
    | Reinstalling the `.vsix` does nothing | `--force`, then **Reload Window** |
    | Windows: `code --install-extension …` just opens a new VS Code window | use `code.cmd` instead of `code`, or **⋯ → Install from VSIX…** in the Extensions view |
    | Joint file full of unresolved imports | accept the editor's offer to use its environment |
