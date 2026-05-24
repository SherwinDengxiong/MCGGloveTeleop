# MCGGloveTeleop

G7s glove teleoperation bridge for RealHand L6, O6, and L20 hands over CAN.

This repository is prepared as a clean GitHub-ready copy of the local
`glovetesting` scripts. It uses the `gloveTeleop` conda environment defined in
`environment.yml`.

## Repository Contents

```text
glove_l6_bridge.py   # RealHand L6 entrypoint; also supports mixed models
glove_o6_bridge.py   # RealHand O6 entrypoint
glove_l20_bridge.py  # RealHand L20 entrypoint
environment.yml      # conda environment named gloveTeleop
scripts/             # environment setup and verification helpers
docs/                # setup, GitHub, and usage notes
```

## Quick Start

From the repository root:

```bash
bash scripts/setup_glove_teleop.sh
conda activate gloveTeleop
python scripts/check_environment.py
```

If `conda` is not in your PATH, the setup script also checks common local
install paths such as `~/miniconda3/bin/conda`.

## CAN Interface Setup

Bring up the Linux SocketCAN interface once per boot before running teleop.
RealHand CAN examples use 1 Mbps by default.

Single hand on `can0`:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
ip -details link show can0
```

Dual hands on `can0` and `can1`:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can1 type can bitrate 1000000
sudo ip link set up can0
sudo ip link set up can1
ip -details link show can0
ip -details link show can1
```

If an interface is already up and Linux refuses to change its bitrate, bring it
down first, then run the matching setup commands again:

```bash
sudo ip link set can0 down
sudo ip link set can1 down
```

## Run Examples

Single L20 hand:

```bash
conda run --no-capture-output -n gloveTeleop python glove_l20_bridge.py \
  --glove-ip 172.16.3.12 \
  --glove-hand right \
  --hand-side right \
  --can-interface can0 \
  --open-on-exit
```

Dual L6 hands from one G7s UDP stream:

```bash
conda run --no-capture-output -n gloveTeleop python glove_l6_bridge.py \
  --glove-ip 192.168.0.237 \
  --both-hands \
  --hand-model l6 \
  --left-l6-side left \
  --right-l6-side right \
  --left-can-interface can0 \
  --right-can-interface can1 \
  --left-thumb-abd-source neutral \
  --right-thumb-abd-source side \
  --can-type socketcan \
  --open-on-exit
```

Swap `can0` and `can1` if the left and right hands are connected to the opposite
CAN adapters.

## Documentation

- [Conda setup](docs/CONDA_SETUP.md)
- [Usage examples and tuning](docs/USAGE.md)
- [GitHub repository setup](docs/GITHUB_REPO_SETUP.md)

## GitHub Upload Checklist

```bash
git init -b main
git add README.md environment.yml .gitignore docs scripts glove_l6_bridge.py glove_l20_bridge.py glove_o6_bridge.py
git commit -m "Prepare MCG glove teleop bridge"
git remote add origin git@github.com:YOUR_USER/MCGGloveTeleop.git
git push -u origin main
```
