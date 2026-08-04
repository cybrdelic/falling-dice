$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$Joined = ($Root + '\archive-parts\part*.b64')
$Text = (Get-ChildItem $Joined | Sort-Object Name | ForEach-Object { Get-Content $_ -Raw }) -join ''
$Bytes = [Convert]::FromBase64String($Text)
$Out = Join-Path $Root 'falling-dice-maintained-source.tar.xz'
[IO.File]::WriteAllBytes($Out, $Bytes)
$Hash = (Get-FileHash $Out -Algorithm SHA256).Hash.ToLower()
if ($Hash -ne 'e9a94160631ef596b0708b16be6c24f9a7978fea6fd09c4b9ebafefad71c2ad9') { throw "SHA-256 mismatch: $Hash" }
Write-Host "Reconstructed: $Out"
