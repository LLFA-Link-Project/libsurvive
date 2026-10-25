#!/usr/bin/env python3
"""Plot every recorded optical/fused position sample in a paired replay window.

Requires NumPy and Matplotlib. Inputs must share calibration, reference point
and receive-clock origin. No alignment, interpolation or smoothing is applied.
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from lighthouse_optical_metrics import read


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('before', type=Path)
    p.add_argument('after', type=Path)
    p.add_argument('--serial', required=True)
    p.add_argument('--lo', type=float, required=True)
    p.add_argument('--hi', type=float, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if not args.lo < args.hi:
        p.error('Window start must precede end')
    inputs = [read(path)[0][args.serial] for path in (args.before, args.after)]
    fig, axes = plt.subplots(2, 3, figsize=(13, 6.5), sharex=True, layout='constrained')
    for row, kind in enumerate(('optical', 'fused')):
        selected = [data[kind][(data[kind][:, 0] >= args.lo) & (data[kind][:, 0] <= args.hi)] for data in inputs]
        if any(not len(samples) for samples in selected):
            p.error(f'Both inputs need {kind} poses in this window')
        origin = selected[0][0, 2:5]
        for col, axis in enumerate('XYZ'):
            ax = axes[row, col]
            for samples, label, color in zip(selected, ('Before', 'Adaptive optical fit'), ('#d18135', '#168b8c')):
                ax.plot(samples[:, 0], (samples[:, col+2]-origin[col])*1000,
                        lw=.7, alpha=.9, label=f'{label} ({len(samples)} samples)', color=color)
            ax.set_title(f'{kind.capitalize()} / world {axis}')
            ax.set_ylabel('Position from common first sample (mm)')
            ax.grid(alpha=.2)
            ax.set_xlim(args.lo, args.hi)
            if not row and not col:
                ax.legend(fontsize=8)
            if row:
                ax.set_xlabel('Input recording receive time (s)')
    fig.suptitle(f'{args.serial}: {args.lo:g}–{args.hi:g} s\n'
                 'All actual samples; same libsurvive calibration; no output smoothing. Physical motion is included.')
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()
