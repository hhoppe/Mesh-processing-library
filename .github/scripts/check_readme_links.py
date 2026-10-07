#!/usr/bin/env python3
"""Check the relative links of the README.md pages.

For each README.md in the repository, every relative link or image source must name an existing file
or directory, and every "#anchor" must match an explicit id="..." or a heading of the target page
(for a directory, its README.md).
Absolute URLs are not checked.  Run from any directory: .github/scripts/check_readme_links.py
"""

import os
import pathlib
import re
import sys


def anchors(path: pathlib.Path) -> set[str]:
  """Return the anchors of a Markdown page: explicit ids and GitHub's slugs of the headings."""
  text = path.read_text(encoding='utf-8')
  result = set(re.findall(r'\bid="([^"]+)"', text))
  in_fence = False
  for line in text.split('\n'):
    if line.lstrip().startswith('```'):
      in_fence = not in_fence
    elif not in_fence and (match := re.match(r'#{1,6} +(.*)', line)):
      title = re.sub(r'<[^>]*>', '', match[1]).strip().lower()
      result.add(re.sub(r'[^\w\- ]', '', title).replace(' ', '-'))
  return result


def main() -> int:
  root = pathlib.Path(__file__).resolve().parent.parent.parent
  pages = sorted(path for path in root.rglob('README.md') if 'results' not in path.parts)
  num_links = num_errors = 0
  for page in pages:
    for number, line in enumerate(page.read_text(encoding='utf-8').split('\n'), 1):
      targets = (
          re.findall(r'\]\(([^)\s]+)\)', line)
          + re.findall(r'(?:href|src|srcset)="([^"]+)"', line)
          + re.findall(r'^\[[^\]]+\]:\s*(\S+)', line)
      )
      for target in targets:
        if re.match(r'(https?:|mailto:)', target):
          continue
        num_links += 1
        file_part, _, anchor = target.partition('#')
        target_path = pathlib.Path(os.path.normpath(page.parent / file_part)) if file_part else page
        if target_path.is_dir() and anchor:
          target_path = target_path / 'README.md'
        if not target_path.exists():
          error = 'missing target'
        elif anchor and anchor not in anchors(target_path):
          error = 'missing anchor'
        else:
          error = None
        if error:
          print(f'{page.relative_to(root)}:{number}: error: {error} {target}')
          num_errors += 1
  print(f'{len(pages)} pages, {num_links} relative links, {num_errors} errors')
  return 1 if num_errors else 0


if __name__ == '__main__':
  sys.exit(main())
