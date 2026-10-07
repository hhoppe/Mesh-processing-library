#!/usr/bin/env python3
"""Run the shell examples of progs/README.md to check that they still work.

Each ```shell block is run (in order, since later blocks read files written by earlier ones) in a
scratch directory containing links to the repository's demos/ and bin/ directories, with the built
programs in the PATH.  Each viewer (G3dOGL, G3dVec, VideoViewer) is made to run without a window and
to quit after a moment.

Usage: .github/scripts/run_readme_examples.py [--dir DIR] [--only N,M,...] [--keep]
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


def hide_viewers(text: str) -> str:
  """Append window-less, self-terminating arguments to each viewer command of the text."""
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
      segments[i] = (segment.rstrip()[:-1] + hidden + ')' if segment.rstrip().endswith(')') else
                     segment.rstrip() + hidden) + ' '  # fmt: skip
  return ''.join(segments)


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument('--dir', help='scratch directory (default: a temporary directory)')
  parser.add_argument('--only', help='comma-separated block numbers to run')
  parser.add_argument('--keep', action='store_true', help='keep the scratch directory')
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
  for i, (line, text) in enumerate(blocks(page), 1):
    label = f'block {i} (line {line}, "{text.split()[0]}")'
    if only and i not in only:
      continue
    if reason := next((reason for key, reason in SKIP.items() if key in text), None):
      print(f'{label}: skipped ({reason})', flush=True)
      continue
    script = hide_viewers(text)
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
    else:
      num_failed += 1
      print(
          f'{page}:{line}: error: {label} failed (status {status}, {elapsed:.1f} s); see {log}',
          flush=True,
      )
  if not args.keep and not args.dir:
    shutil.rmtree(scratch)
  print(f'{num_failed} failed')
  return 1 if num_failed else 0


if __name__ == '__main__':
  sys.exit(main())
