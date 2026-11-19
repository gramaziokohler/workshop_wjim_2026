<p class="step">STEP 01 / 07</p>

# Install

| You need | Version | |
|---|---|---|
| [Rhino 8](https://www.rhino3d.com/download/) | 8, latest service release | with Grasshopper |
| [VS Code](https://code.visualstudio.com/download) | 1.85 or newer, latest recommended | the `code` command on your PATH ([macOS](https://code.visualstudio.com/docs/setup/mac#_launch-vs-code-from-the-command-line)) |
| [git](https://git-scm.com/downloads) | any | VS Code uses it to clone |

!!! warning "Update Rhino and VS Code before anything else"
    Rhino: **Help → Check for Updates**. VS Code: **Code → Check for Updates…** (macOS) or
    **Help → Check for Updates…** (Windows). Older versions are where most setup problems come from.

## 1 · Clone the repo, from VS Code

<kbd>Cmd</kbd>/<kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd> → **Git: Clone** → paste

```text
https://github.com/gramaziokohler/workshop_wjim_2026.git
```

→ pick a folder → **Open** when VS Code asks. The repo is now the open folder, which is what step 4 needs.

??? note "From a terminal instead"
    ```bash
    git clone https://github.com/gramaziokohler/workshop_wjim_2026.git
    ```

!!! tip "Pick a path without spaces or special characters"
    e.g. `~/workshop_wjim_2026` or `C:\Users\you\workshop_wjim_2026`.
    Moved the folder later? Run `00_install.ghx` again.

## 2 · Rhino: open `00_install.ghx`

Open the file from the repo root in Grasshopper and wait.
It builds one Python environment, **`wjim_ncyu2026`**, and prints what it installed.

![00_install.ghx: the Setup component and its panel listing the venv wjim_ncyu2026, compas_timber 3.0.2, compas_fab 2.1.0 …](assets/shots/install_gh.png)

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

Cloned from VS Code in step 1? It's already open. Otherwise **File → Open Folder…** → the repo
itself, not `joinery/`. Open `joinery/corner_node.py` once.

The extension downloads its own Python and `compas_timber`. Then it asks to use that environment
for the editor too: click **Select It**, or the imports in your joint files stay underlined in red.

![Joint Editor asks: select its environment for this workspace? Select It](assets/shots/select.png){ width="480" }

When the status bar says **compas_timber 3.0.2**, you're set.

![VS Code status bar: compas_timber 3.0.2 · release](assets/shots/statusbar.png){ width="320" }

??? failure "Something's off?"
    | Symptom | Fix |
    |---|---|
    | Rhino reinstalls over and over / venv broken | close Rhino, delete `~/.rhinocode/py39-rh8/site-envs/wjim_ncyu2026-*`, open `00_install.ghx` again |
    | `No module named 'wjim_lib'` | open `00_install.ghx` again, restart Rhino |
    | Reinstalling the `.vsix` does nothing | `--force`, then **Reload Window** |
    | Windows: `code --install-extension …` just opens a new VS Code window | use `code.cmd` instead of `code`, or **⋯ → Install from VSIX…** in the Extensions view |
    | Joint file full of unresolved imports | click **Select It** when the editor offers its environment; missed it? check the bell at the bottom right |
