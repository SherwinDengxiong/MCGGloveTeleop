# Usage

This bridge receives 9MOTCAP G7s UDP frames and sends normalized angle targets
to a RealHand L6, L20, or O6 hand through the official `realhand` SDK.

Run these commands from the repository root. Replace `172.16.3.12` with the G7s
glove IP if yours is different.

## Entrypoints

```text
glove_l6_bridge.py   # RealHand L6
glove_o6_bridge.py   # RealHand O6
glove_l20_bridge.py  # RealHand L20
```

Stop the bridge with `Ctrl+C`. The examples below include `--open-on-exit`,
which sends the hand open pose before closing the SDK connection.

## Control a Single Hand

Right glove data to a right L20 hand on `can0`:

```bash
conda run --no-capture-output -n gloveTeleop python glove_l20_bridge.py \
  --glove-ip 172.16.3.12 \
  --glove-hand right \
  --hand-side right \
  --can-interface can0 \
  --open-on-exit
```

Left glove data to a left L20 hand on `can0`:

```bash
conda run --no-capture-output -n gloveTeleop python glove_l20_bridge.py \
  --glove-ip 172.16.3.12 \
  --glove-hand left \
  --hand-side left \
  --can-interface can0 \
  --open-on-exit
```

For L6 or O6, use the same single-hand flags and change only the script name:

```bash
conda run --no-capture-output -n gloveTeleop python glove_l6_bridge.py \
  --glove-ip 172.16.3.12 \
  --glove-hand right \
  --hand-side right \
  --can-interface can0 \
  --open-on-exit
```

```bash
conda run --no-capture-output -n gloveTeleop python glove_o6_bridge.py \
  --glove-ip 172.16.3.12 \
  --glove-hand right \
  --hand-side right \
  --can-interface can0 \
  --open-on-exit
```

## Control Both Hands

Dual L20 control with the left hand on `can0` and the right hand on `can1`:

```bash
conda run --no-capture-output -n gloveTeleop python glove_l20_bridge.py \
  --glove-ip 172.16.3.12 \
  --both-hands \
  --left-can-interface can0 \
  --right-can-interface can1 \
  --open-on-exit
```

Dual L6 control:

```bash
conda run --no-capture-output -n gloveTeleop python glove_l6_bridge.py \
  --glove-ip 172.16.3.12 \
  --both-hands \
  --left-can-interface can0 \
  --right-can-interface can1 \
  --open-on-exit
```

Dual O6 control:

```bash
conda run --no-capture-output -n gloveTeleop python glove_o6_bridge.py \
  --glove-ip 172.16.3.12 \
  --both-hands \
  --left-can-interface can0 \
  --right-can-interface can1 \
  --open-on-exit
```

In `--both-hands` mode the bridge reads `leftHand` and `rightHand` from the same
G7s UDP stream. By default, left uses `can0`, right uses `can1`, left side is
`left`, and right side is `right`.

## Mixed Hand Models

If the left and right RealHands are different models, run the shared
`glove_l6_bridge.py` entrypoint and override each side:

```bash
conda run --no-capture-output -n gloveTeleop python glove_l6_bridge.py \
  --glove-ip 172.16.3.12 \
  --both-hands \
  --left-hand-model l20 \
  --right-hand-model l6 \
  --left-can-interface can0 \
  --right-can-interface can1 \
  --open-on-exit
```

## Joint Orders

The L6 and O6 output order is:

```text
thumb_flex, thumb_abd, index, middle, ring, pinky
```

The L20 output order is:

```text
thumb_abd, thumb_yaw, thumb_root1, thumb_tip,
index_abd, index_root1, index_tip,
middle_abd, middle_root1, middle_tip,
ring_abd, ring_root1, ring_tip,
pinky_abd, pinky_root1, pinky_tip
```

Default G7s to L20 mapping:

```text
thumb_abd    -> neutral by default
roll[0]      -> thumb_yaw
pitch[0]     -> thumb_root1
end_pitch[0] -> thumb_tip
side[1..4]   -> index/middle/ring/pinky *_abd
pitch[1..4]  -> index/middle/ring/pinky *_root1
max(two_pitch[i], end_pitch[i]) -> index/middle/ring/pinky *_tip
```

## Useful Tuning Flags

```text
--g7s-thumb-mode pitch-side
--thumb-abd-source neutral
--left-thumb-abd-source neutral
--right-thumb-abd-source side
--l20-thumb-tip-source end
--show-thumb
--torque 50
--ranges '{"thumb_flex":[0,60],"thumb_abd":[0,90],"index":[0,90],"middle":[0,90],"ring":[0,90],"pinky":[0,90]}'
--invert thumb_flex,index,middle,ring,pinky
--rate-hz 50
--smoothing-alpha 0.35
--deadband 0.8
--open-on-start
--open-on-exit
```

By default, flexion joints are inverted because G7s flexion angles usually grow
as the human finger bends, while the RealHand SDK uses larger normalized values
for opening and smaller values for closing. Abduction and yaw joints are not
inverted by default.

By default, the bridge sets every joint torque to `50` when each RealHand is
opened. This is half of the SDK's normalized `0..100` torque range and applies
to L6, O6, and L20. Use `--torque 30` or another value if you want a softer or
stronger limit.

The default thumb mode is `pitch-side`: `pitch[0]` controls thumb flex/root. In
the current G7s stream, `side[0]` is constant while the left thumb moves, so
`thumb_abd` defaults to neutral for L6, O6, and L20. To test whether a source can
drive thumb side motion, run with `--show-thumb` while moving only the thumb
sideways and watch the printed `pitch`, `side`, and `roll` arrays. Then select
the changing source:

```bash
--thumb-abd-source side
--thumb-abd-source roll
--thumb-abd-source pitch
```

When using `--both-hands`, `--left-thumb-abd-source` and
`--right-thumb-abd-source` can override the global source for each glove hand.
This is useful when one glove exposes a changing thumb side channel and the
other does not.

If none of those raw fields changes during thumb side motion, the bridge has no
changing lateral signal to send and the glove stream or calibration needs to be
checked.
