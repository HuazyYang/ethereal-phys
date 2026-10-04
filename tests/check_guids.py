#!/usr/bin/env python3
"""GUID check for ethereal-phys: every interface ID and class ID is a fresh GUID.

donut/nvrhi/doc/com-interface-abi.md, section 3.1: "Generate a fresh GUID; never copy one." A design sketch
writes IDs as placeholders (ffffffff-...), and a copied declaration carries its ID with it. Both compile, and
both make QueryInterface answer for the wrong type.

Scans the NVRHI_IID / NVRHI_CCLSID / NVRHI_SCLSID / NVRHI_CLASS_CLSID declarations of the given directories and
fails on:
  - an ID that is not 8-4-4-4-12 hex digits,
  - an ID that starts with ffffffff (the placeholder sentinel) or is all zeros,
  - an ID used by two declarations,
  - an ID that also appears in a --reference directory (nvrhi, Donut, the samples: copied from there).

usage: check_guids.py <dir> [<dir> ...] [--reference <dir> [<dir> ...]]
"""
import os
import re
import sys

DECLARATION = re.compile(r'\bNVRHI_(?:IID|CCLSID|SCLSID|CLASS_CLSID)\s*\(\s*([\w:]+)\s*,\s*"([^"]*)"\s*\)')
WELL_FORMED = re.compile(r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$')
EXTENSIONS = ('.h', '.hpp', '.cpp', '.inl')


def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    return re.sub(r'//[^\n]*', '', text)


def scan(root):
    """[(file, name, guid)] of the declarations under root."""
    found = []
    for dirpath, _, files in os.walk(root):
        for name in sorted(files):
            if not name.endswith(EXTENSIONS):
                continue
            path = os.path.join(dirpath, name)
            with open(path, encoding='utf-8', errors='replace') as fh:
                code = strip_comments(fh.read())
            for m in DECLARATION.finditer(code):
                found.append((path, m.group(1), m.group(2).lower()))
    return found


def main(argv):
    args = argv[1:]
    if '--reference' in args:
        split = args.index('--reference')
        roots, references = args[:split], args[split + 1:]
    else:
        roots, references = args, []
    if not roots:
        print(__doc__)
        return 2

    declarations = []
    for root in roots:
        if not os.path.isdir(root):
            print(f'check_guids: {root} is not a directory')
            return 2
        declarations += scan(root)
    if not declarations:
        print('check_guids: no IID or CLSID declaration found')
        return 1

    reference = {}
    for root in references:
        for path, name, guid in scan(root):
            reference.setdefault(guid, f'{name} ({path})')

    errors = []
    seen = {}
    for path, name, guid in declarations:
        where = f'{path}: {name} "{guid}"'
        if not WELL_FORMED.match(guid):
            errors.append(f'{where}: not a GUID (xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx)')
            continue
        if guid.startswith('ffffffff'):
            errors.append(f'{where}: a placeholder (starts with ffffffff); generate a fresh GUID')
        if set(guid) <= set('0-'):
            errors.append(f'{where}: all zeros')
        if guid in seen:
            errors.append(f'{where}: the same ID as {seen[guid]}')
        else:
            seen[guid] = f'{name} ({path})'
        if guid in reference:
            errors.append(f'{where}: copied from {reference[guid]}')

    for e in errors:
        print(e)
    if errors:
        print(f'check_guids: {len(errors)} violation(s)')
        return 1
    print(f'check_guids: OK ({len(declarations)} IDs, {len(reference)} reference IDs compared)')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
