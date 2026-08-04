# Complete source archive

The full maintained dice + nonlinear metamaterial source payload is stored as ordered Base64 parts under `archive-parts/` because the connected GitHub write API does not support a local directory upload in one request.

Linux/macOS:

```bash
./source/reconstruct.sh
mkdir source-tree
cd source-tree
tar -xJf ../source/falling-dice-maintained-source.tar.xz
```

Windows PowerShell:

```powershell
./source/reconstruct.ps1
mkdir source-tree
cd source-tree
tar -xf ../source/falling-dice-maintained-source.tar.xz
```

Expected SHA-256:

```text
e9a94160631ef596b0708b16be6c24f9a7978fea6fd09c4b9ebafefad71c2ad9
```

The maintained headers and project documentation are also committed directly under `projects/`; the archive contains the complete source tree and all preserved source revisions.
