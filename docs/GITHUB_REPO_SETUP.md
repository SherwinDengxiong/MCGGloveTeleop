# GitHub Repository Setup

This folder is prepared for GitHub with:

- `README.md` for the repository landing page
- `environment.yml` for the `gloveTeleop` conda environment
- `.gitignore` for Python, conda, editor, and local tool files
- `scripts/` for setup and verification helpers
- `docs/` for setup and usage documentation

The copied release folder intentionally excludes local state such as `.git`,
`.codex`, `.agents`, and `__pycache__`.

## Initialize Git

From this folder:

```bash
git init -b main
git status
```

## First Commit

```bash
git add README.md environment.yml .gitignore docs scripts glove_l6_bridge.py glove_l20_bridge.py glove_o6_bridge.py
git commit -m "Prepare MCG glove teleop bridge"
```

## Push to GitHub

Create an empty repository on GitHub, then connect this folder:

```bash
git remote add origin git@github.com:YOUR_USER/MCGGloveTeleop.git
git push -u origin main
```

If you use the GitHub CLI:

```bash
gh repo create YOUR_USER/MCGGloveTeleop --private --source=. --remote=origin --push
```

Use `--public` instead of `--private` if the repository should be public.
