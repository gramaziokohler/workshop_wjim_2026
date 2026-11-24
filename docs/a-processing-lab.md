<p class="step">APPENDIX</p>

# Processing lab

A small web page on your own computer: pick a processing, see its toolpath on a beam and the IRB2600 following it.
No Rhino needed. It uses the same toolpaths as group **8** in [Toolpaths](07-toolpaths.md).

## 1 · Start it

You need [Install](01-install.md) steps 1, 3 and 4. In VS Code, with the repo root open: **Terminal → New Terminal**, then paste

=== "Windows (PowerShell)"

    ```powershell
    & "$env:APPDATA\Code\User\globalStorage\joint-editor.joint-editor\uv\0.10.2\uv.exe" run processing_lab/server.py
    ```

=== "macOS"

    ```bash
    "$HOME/Library/Application Support/Code/User/globalStorage/joint-editor.joint-editor/uv/0.10.2/uv" run processing_lab/server.py
    ```

=== "uv already installed"

    ```bash
    uv run processing_lab/server.py
    ```

The long path is the [uv](https://docs.astral.sh/uv/) the joint editor downloaded for itself.

!!! tip "Takes a few minutes, once"
    The first start downloads about 500 MB. After that it takes seconds.
    The environment stays out of the repo, so VS Code keeps using the joint editor's.

## 2 · Open it

When the terminal says `Processing lab on http://localhost:8000`, open [localhost:8000](http://localhost:8000).

| | |
|---|---|
| **Beam** | length, width, height (mm) |
| **Processing**, **Variant** | what is cut. Grey: no variants yet |
| **Setup** | which side faces up, if there is more than one |
| **▶** | run the robot along the toolpath |

Some processings take a minute or more: wait while it says *computing…*

## 3 · Stop it

<kbd>Ctrl</kbd>+<kbd>C</kbd> in the terminal. After a **Pull**, or an edit to `joinery/toolpath_lib.py`,
stop it, start it again and reload the page.

??? failure "Something's off?"
    | Symptom | Fix |
    |---|---|
    | Windows: `uv.exe` is *not recognized* · macOS: *no such file or directory* | check that the joint editor is set up ([Install](01-install.md) step 4). Still there? [Install uv](https://docs.astral.sh/uv/getting-started/installation/), restart VS Code, use the third tab |
    | `processing_lab/server.py` not found | the terminal isn't in the repo root: **File → Open Folder…** → the repo, then a new terminal |
    | *computing…* never ends, and the terminal is back at its prompt | the lab crashed, which happens now and then: start it again and reload the page |
    | *Address already in use* | it's already running in another terminal, use that one or stop it there |
    | The page opens but stays empty | it loads three.js from the internet: check the connection, reload |
