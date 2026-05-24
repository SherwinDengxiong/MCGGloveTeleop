# Conda Setup

This project uses a conda environment named `gloveTeleop`.

## Automated Setup

From the repository root:

```bash
bash scripts/setup_glove_teleop.sh
```

The script creates `gloveTeleop` if it does not exist. If the environment
already exists, it updates it from `environment.yml` with `--prune`.

After setup:

```bash
conda activate gloveTeleop
python scripts/check_environment.py
```

If `conda` is not in your PATH, use the absolute path from your Miniconda or
Anaconda install, for example:

```bash
~/miniconda3/bin/conda run --no-capture-output -n gloveTeleop python scripts/check_environment.py
```

## Manual Setup

Create the environment:

```bash
conda env create -f environment.yml
conda activate gloveTeleop
```

Update an existing environment:

```bash
conda env update -n gloveTeleop -f environment.yml --prune
conda activate gloveTeleop
```

## Verify

```bash
python scripts/check_environment.py
```

## Notes

- `environment.yml` intentionally does not include a machine-specific
  `prefix:` line.
- The RealHand SDK is installed from a pinned GitHub commit through pip.
- The Linux CAN interfaces, such as `can0` and `can1`, still need to be brought
  up on the robot or workstation before running non-dry-run teleop.
