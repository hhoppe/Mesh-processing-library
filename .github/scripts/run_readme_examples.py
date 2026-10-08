#!/usr/bin/env python3
"""Run the shell examples of progs/README.md to check that they still work.

Each ```shell block is run (in order, since later blocks read files written by earlier ones) in a
scratch directory containing links to the repository's demos/ and bin/ directories, with the built
programs in the PATH.  Each viewer (G3dOGL, G3dVec, VideoViewer) is made to run without a window and
to quit after a moment, or with --screenshots, to save an image once its input is read; the images
are then assembled into a single sheet (in reading order of the examples) for visual inspection, and
their statistics are checked against the reference values of the platform (Linux, using Mesa's
llvmpipe software renderer, or macOS) in readme_examples_reference_PLATFORM.txt (using
bin/check_reference_values).  Each image is named after a hash of the text of its block, so editing
an example requires updating the reference values (with --update), but inserting one does not.

Usage: run_readme_examples.py [--dir DIR] [--only N,M,...] [--keep] [--screenshots FILE] [--update]
The programs must be built, and the demo results created (make -C demos create).
"""

import argparse
import hashlib
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
TAIL_LINES = 40  # Number of final log lines shown for a failed block.
# The reference values differ across renderers, so they are kept for the two platforms run in CI.
PLATFORM = {'linux': 'linux', 'darwin': 'macos'}.get(sys.platform)
REFERENCE = pathlib.Path(__file__).parent / f'readme_examples_reference_{PLATFORM}.txt'
CHECK = pathlib.Path(__file__).resolve().parents[2] / 'bin' / 'check_reference_values'
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
      if command == 'VideoViewer':
        # Resizing the hidden window with the "=" key crashes the X server of XQuartz (macOS).
        segment = re.sub(r'-key \S+', lambda m: m.group().replace('=', ''), segment)
      # (VideoViewer takes no screenshot: a hidden window cannot be resized, as its "=" key does.)
      if screenshots is not None and '-video' not in words and command != 'VideoViewer':
        path = screenshots[0].parent / f'{screenshots[0].stem}_{len(screenshots)}.png'
        screenshots.append(path)
        capture = f' -imagename {path} -picture' if command == 'G3dOGL' else f' -offscreen {path}'
        geometry = f' -geom {TILE[0] * SUPERSAMPLE}x{TILE[1] * SUPERSAMPLE}'
        hidden = ' -hidden -noinfo 1' + geometry + capture
        segment = segment.replace(' -async', '')  # So the picture is taken after reading the input.
        # Omit the key 'J' (automatic flight), whose motion until the picture depends on the timing.
        segment = re.sub(r'\s-key\s+(\S+)', lambda m: unflown(m.group(1)), segment)
      # Insert the arguments before any trailing comment and closing parenthesis.
      comment = re.search(r'\s#.*$', segment)
      command_part = segment[: comment.start()] if comment else segment
      rest = segment[comment.start() :] if comment else ''
      if command_part.rstrip().endswith(')'):
        command_part = command_part.rstrip()[:-1] + hidden + ')'
      else:
        command_part = command_part.rstrip() + hidden
      segments[i] = command_part + ' ' + rest.strip() + ' '
  return ''.join(segments)


def unflown(keys: str) -> str:
  """Return the argument ' -key KEYS' without the key 'J', or '' if no other key remains."""
  keys = keys.replace('J', '')
  return f' -key {keys}' if keys.strip('\'"') else ''


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


def update_reference(shots: dict[str, str], directory: pathlib.Path, env: dict[str, str]) -> int:
  """Rewrite REFERENCE to list exactly the given screenshots (mapped to a description), then fill
  in their values.

  The header (up to the first empty line) is kept, as are the comments preceding each entry and its
  tol= field for each screenshot that is still produced.  A new screenshot is preceded by a comment
  with its description.
  """
  lines = REFERENCE.read_text(encoding='utf-8').splitlines() if REFERENCE.exists() else []
  header = lines[: lines.index('') + 1] if '' in lines else ['']
  kept: dict[str, list[str]] = {}
  comments: list[str] = []
  for line in lines[len(header) :]:
    if line.startswith('#') or not line.strip():
      comments.append(line)
      continue
    name, *fields = line.split()
    kept[name] = comments + [' '.join([name] + [f for f in fields if f.startswith('tol=')])]
    comments = []
  output = list(header)
  for name, description in shots.items():
    output += kept.get(name) or [f'# {description}', name]
  REFERENCE.write_text('\n'.join(output) + '\n', encoding='utf-8', newline='\n')
  args = ['bash', CHECK.as_posix(), '--update', REFERENCE.as_posix(), directory.as_posix()]
  return subprocess.run(args, env=env).returncode


def check_reference(shots: dict[str, str], directory: pathlib.Path, env: dict[str, str]) -> int:
  """Check the screenshots against REFERENCE, and return the number of failures."""
  num_failed = 0
  listed = {
      line.split()[0]
      for line in REFERENCE.read_text(encoding='utf-8').splitlines()
      if line.strip() and not line.startswith('#')
  }
  for name, description in shots.items():
    if name not in listed:
      num_failed += 1
      print(f'*** {name} ({description}) is not in {REFERENCE.name}; rerun with --update.')
  args = ['bash', CHECK.as_posix(), REFERENCE.as_posix(), directory.as_posix()]
  sys.stdout.flush()
  if subprocess.run(args, env=env).returncode:
    num_failed += 1
  return num_failed


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument('--dir', help='scratch directory (default: a temporary directory)')
  parser.add_argument('--only', help='comma-separated block numbers to run')
  parser.add_argument('--keep', action='store_true', help='keep the scratch directory')
  parser.add_argument('--screenshots', help='assemble the viewer screenshots into this image file')
  parser.add_argument(
      '--update', action='store_true', help='update the reference values of the screenshots'
  )
  args = parser.parse_args()
  if args.update and args.only:
    parser.error('--update requires running all the blocks')
  if args.update and not PLATFORM:
    parser.error(f'there are no reference values for platform {sys.platform}')
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
  if PLATFORM == 'linux':
    env['GALLIUM_DRIVER'] = 'llvmpipe'  # The renderer of the reference values (rather than a GPU).
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
  shots: dict[str, str] = {}  # The description of each screenshot, by file name.
  shots_dir = scratch / 'screenshots'
  take_screenshots = bool(args.screenshots or args.update)
  if take_screenshots:
    shots_dir.mkdir(exist_ok=True)
  for i, (line, text) in enumerate(blocks(page), 1):
    label = f'block {i} (line {line}, "{text.split()[0]}")'
    if only and i not in only:
      continue
    if reason := next((reason for key, reason in SKIP.items() if key in text), None):
      print(f'{label}: skipped ({reason})', flush=True)
      continue
    digest = hashlib.sha1(text.encode()).hexdigest()[:8]
    screenshots = [shots_dir / digest] if take_screenshots else None
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
        first_line = text.split('\n')[0].rstrip(' \\')
        for k, path in enumerate(screenshots[1:], 1):
          if path.exists():
            shots[path.name] = f'{first_line} (screenshot {k})'
    else:
      num_failed += 1
      print(
          f'{page}:{line}: error: {label} failed (status {status}, {elapsed:.1f} s); see {log}',
          flush=True,
      )
      # Show the end of the log, where the error or assertion message lies.
      tail = log.read_text(encoding='utf-8', errors='replace').splitlines()[-TAIL_LINES:]
      print('\n'.join(f'    {t}' for t in tail), flush=True)
  if args.screenshots and all_screenshots:
    assemble(all_screenshots, pathlib.Path(args.screenshots), env, scratch)
  if args.update and num_failed:
    print('The reference values are not updated, because some blocks failed.')
  elif args.update:
    num_failed += update_reference(shots, shots_dir, env) != 0
  elif take_screenshots and not args.only and PLATFORM:
    num_failed += check_reference(shots, shots_dir, env)
  elif take_screenshots and not args.only:
    print(f'The screenshots are not checked: there are no reference values for {sys.platform}.')
  if not args.keep and not args.dir:
    shutil.rmtree(scratch)
  print(f'{num_failed} failed')
  return 1 if num_failed else 0


if __name__ == '__main__':
  sys.exit(main())
