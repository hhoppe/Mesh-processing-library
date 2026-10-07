#!/usr/bin/env python3
"""Run the shell examples of progs/README.md to check that they still work.

Each ```shell block is run (in order, since later blocks read files written by earlier ones) in a
scratch directory containing links to the repository's demos/ and bin/ directories, with the built
programs in the PATH.  Each viewer (G3dOGL, G3dVec, VideoViewer) is made to run without a window and
to quit after a moment, or with --screenshots, to save an image once its input is read; the images
are then assembled into a single sheet (in reading order of the examples) for visual inspection.

Usage: run_readme_examples.py [--dir DIR] [--only N,M,...] [--keep] [--screenshots FILE]
The programs must be built, and the demo results created (make -C demos create).
"""

import argparse
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile
import time

VIEWERS = ('G3dOGL', 'G3dVec', 'VideoViewer')
HIDDEN = " -hidden -hwdelay 1 -hwkey '\\2\\c'"
TILE = (640, 480)  # Size of each screenshot in the sheet.
SUPERSAMPLE = 2  # The screenshots are rendered at this multiple of the tile size, then downsampled.
COLUMNS = 6  # Number of screenshots per row of the sheet.
# Blocks containing these strings are not run, for the given reasons.
SKIP = {
    '-wait_on_visualizer': 'interactive',
    'StitchPM -rootname': 'needs the terrain tiles created by demos/create_terrain_hierarchy',
}


def blocks(page: pathlib.Path) -> list[tuple[int, str]]:
  """Return the (line number, text) of each ```shell block of the page."""
  result, lines, in_block = [], [], False
  for number, line in enumerate(page.read_text(encoding='utf-8').split('\n'), 1):
    if line.startswith('```shell'):
      in_block, start, lines = True, number, []
    elif line.startswith('```') and in_block:
      in_block = False
      result.append((start, '\n'.join(lines) + '\n'))
    elif in_block:
      lines.append(line)
  return result


def hide_viewers(text: str, screenshots: list[pathlib.Path] | None = None) -> str:
  """Append window-less, self-terminating arguments to each viewer command of the text.

  With a list (its first element naming the block), each G3dOGL or G3dVec instead saves a screenshot
  (G3dOGL after reading all its input), whose path is appended to the list.
  """
  text = text.replace('\\\n', ' ')
  segments, segment, quote = [], '', ''
  for char in text:
    if quote:
      quote = '' if char == quote else quote
    elif char in '"\'':
      quote = char
    elif char in '|;\n':
      segments.append(segment)
      segment = ''
      segments.append(char)
      continue
    segment += char
  segments.append(segment)
  for i, segment in enumerate(segments):
    words = segment.strip().lstrip('(').split()
    command = next((word for word in words if '=' not in word), '')
    if command in VIEWERS:
      hidden = ' -hidden' if '-video' in words else HIDDEN
      # (VideoViewer takes no screenshot: a hidden window cannot be resized, as its "=" key does.)
      if screenshots is not None and '-video' not in words and command != 'VideoViewer':
        path = screenshots[0].parent / f'{screenshots[0].stem}_{len(screenshots)}.png'
        screenshots.append(path)
        capture = f' -imagename {path} -picture' if command == 'G3dOGL' else f' -offscreen {path}'
        geometry = f' -geom {TILE[0] * SUPERSAMPLE}x{TILE[1] * SUPERSAMPLE}'
        hidden = ' -hidden' + geometry + capture
        segment = segment.replace(
            ' -async', ''
        )  # So that the picture is taken after reading the input.
      segments[i] = (segment.rstrip()[:-1] + hidden + ')' if segment.rstrip().endswith(')') else
                     segment.rstrip() + hidden) + ' '  # fmt: skip
  return ''.join(segments)


def assemble(
    paths: list[pathlib.Path], output: pathlib.Path, env: dict[str, str], scratch: pathlib.Path
):
  """Assemble the screenshots into a sheet of COLUMNS columns, each downsampled to the TILE size."""
  w, h = TILE

  def run(args: list[str], out: pathlib.Path) -> None:
    with open(out, 'wb') as f:
      subprocess.run(args, env=env, stdout=f, stderr=subprocess.DEVNULL, check=True)

  tiles = []
  for path in paths:
    tile = path.with_suffix('.tile.png')
    run(
        ['Filterimage', str(path), '-filter', 'lanczos6', '-scaleunif', str(1 / SUPERSAMPLE)]
        + ['-color', '255', '255', '255', '255', '-boundaryrule', 'border']
        + ['-croptodims', str(w), str(h), '-to', 'png'],
        tile,
    )
    tiles.append(tile)
  rows = []
  for r in range(0, len(tiles), COLUMNS):
    chunk = tiles[r : r + COLUMNS]
    row = scratch / f'row{r // COLUMNS}.png'
    pad = -(COLUMNS - len(chunk)) * w  # A negative crop extends the row to the full width.
    run(
        ['Filterimage', '-assemble', str(len(chunk)), '1', *map(str, chunk)]
        + ['-color', '255', '255', '255', '255', '-boundaryrule', 'border']
        + ['-cropsides', '0', str(pad), '0', '0', '-to', 'png'],
        row,
    )
    rows.append(row)
  run(
      ['Filterimage', '-assemble', '1', str(len(rows)), *map(str, rows), '-to', output.suffix[1:]],
      output,
  )
  print(f'Assembled {len(paths)} screenshots into {output} ({", ".join(p.stem for p in paths)})')


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument('--dir', help='scratch directory (default: a temporary directory)')
  parser.add_argument('--only', help='comma-separated block numbers to run')
  parser.add_argument('--keep', action='store_true', help='keep the scratch directory')
  parser.add_argument('--screenshots', help='assemble the viewer screenshots into this image file')
  args = parser.parse_args()
  root = pathlib.Path(__file__).resolve().parent.parent.parent
  page = root / 'progs' / 'README.md'
  scratch = (
      pathlib.Path(args.dir)
      if args.dir
      else pathlib.Path(tempfile.mkdtemp(prefix='readme_examples_'))
  )
  scratch.mkdir(parents=True, exist_ok=True)
  for name in ('demos', 'bin'):
    if not (scratch / name).exists():
      os.symlink(root / name, scratch / name, target_is_directory=True)
  env = dict(os.environ)
  dirs = [
      root / 'bin' / config for config in ('unix', 'cygwin', 'clang', 'mingw', 'win', 'msbuild')
  ]
  env['PATH'] = os.pathsep.join(
      [str(root / 'bin')] + [str(d) for d in dirs if d.is_dir()] + [env['PATH']]
  )
  only = {int(n) for n in args.only.split(',')} if args.only else None
  print(f'Running the examples of {page.relative_to(root)} in {scratch}', flush=True)
  num_failed = 0
  all_screenshots: list[pathlib.Path] = []
  for i, (line, text) in enumerate(blocks(page), 1):
    label = f'block {i} (line {line}, "{text.split()[0]}")'
    if only and i not in only:
      continue
    if reason := next((reason for key, reason in SKIP.items() if key in text), None):
      print(f'{label}: skipped ({reason})', flush=True)
      continue
    screenshots = [scratch / f'block{i:02}'] if args.screenshots else None
    script = hide_viewers(text, screenshots)
    start = time.time()
    log = scratch / f'block{i:02}.log'
    with open(log, 'w', encoding='utf-8') as f:
      f.write(script + '\n----\n')
      f.flush()
      try:
        status = subprocess.run(
            ['bash', '-e', '-o', 'pipefail', '-c', script],
            cwd=scratch,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=f,
            stderr=subprocess.STDOUT,
            timeout=1200,
        ).returncode
      except subprocess.TimeoutExpired:
        status = 'timeout'
    elapsed = time.time() - start
    # A viewer that quits while its input is still streaming kills the writer with SIGPIPE
    # (status 141), which the demo scripts also accept as success.
    if status in (0, 141):
      print(f'{label}: ok ({elapsed:.1f} s)', flush=True)
      if screenshots:
        all_screenshots += [path for path in screenshots[1:] if path.exists()]
    else:
      num_failed += 1
      print(
          f'{page}:{line}: error: {label} failed (status {status}, {elapsed:.1f} s); see {log}',
          flush=True,
      )
  if args.screenshots and all_screenshots:
    assemble(all_screenshots, pathlib.Path(args.screenshots), env, scratch)
  if not args.keep and not args.dir:
    shutil.rmtree(scratch)
  print(f'{num_failed} failed')
  return 1 if num_failed else 0


if __name__ == '__main__':
  sys.exit(main())
