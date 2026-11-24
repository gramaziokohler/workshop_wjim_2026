![With Joints in Mind](docs/assets/GKR_With-Joints-in-Mind_Oxblood-T1_DIN_banner.svg)

# With Joints in Mind

23–27.11.2026 · NYCU

Design a timber structure. Pick the place where three beams meet.
Write the joint for it in code. Mill it with a robot.

[Slides](https://docs.google.com/presentation/d/14CyE4a5VKYwaq2KSOc8zTqV_KtUEoWpNZnO6R036oT8/edit?usp=sharing)

- `00_install.ghx`: open once in Rhino 8 to build the `wjim_ncyu2026` environment.
- `joint-editor-0.1.0.vsix`: the joint editor, for VS Code (`code --install-extension joint-editor-0.1.0.vsix --force`).
  Built from the `joint_editor` repo: `npm ci && npm run package` in its `extension/`, then copy the `.vsix` here.
- `reset_joint_editor.ps1`: Windows, VS Code closed. Removes the joint editor and its environments, reinstalls the `.vsix` (`powershell -ExecutionPolicy Bypass -File reset_joint_editor.ps1`).
- [`joinery/`](joinery/README.md): the Grasshopper definition, toolpaths and the milling cell.
- [`wjim_lib/`](wjim_lib/README.md): the workshop's extension of `compas_timber`.
- `docs/`: the tutorial site (`uv run zensical serve`).
- `processing_lab/`: try a processing's toolpaths on a beam (`uv run processing_lab/server.py`, [how to start it](docs/a-processing-lab.md)).
  It has its own environment, outside the repo, from the top of `server.py`.
- `exercises/`: the basic exercises, plain COMPAS and COMPAS Timber.

`uv sync` builds one `.venv` for the site and `joinery/check_joint.py` (Rhino has its own, from `00_install.ghx`).
