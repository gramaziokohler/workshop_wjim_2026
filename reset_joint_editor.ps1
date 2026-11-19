# Removes the joint editor, its Python environments and this workspace's VS Code state,
# then installs the .vsix next to this script. Close VS Code first.
# Run: powershell -ExecutionPolicy Bypass -File reset_joint_editor.ps1
$vsix = Join-Path $PSScriptRoot "joint-editor-0.1.0.vsix"
$U = "$env:APPDATA\Code\User"

Get-Process python* -ErrorAction SilentlyContinue |
  Where-Object { $_.Path -like "*joint-editor.joint-editor*" } | Stop-Process -Force
code.cmd --uninstall-extension joint-editor.joint-editor
# the extension's environments and uv
Remove-Item -Recurse -Force "$U\globalStorage\joint-editor.joint-editor" -ErrorAction SilentlyContinue
# this workspace's state: interpreter selection and "Don't Ask Again"
Get-ChildItem "$U\workspaceStorage\*\workspace.json" |
  Where-Object { (Get-Content $_) -match "workshop_ncyu_2026" } |
  ForEach-Object { Remove-Item -Recurse -Force $_.Directory }

if (Test-Path "$U\globalStorage\joint-editor.joint-editor") {
  Write-Error "Could not remove the joint editor's environments - is VS Code still running?"
  exit 1
}
code.cmd --install-extension $vsix --force
