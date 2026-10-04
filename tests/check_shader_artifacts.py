#!/usr/bin/env python3
"""Checks that the kernels of ethereal_phys were really compiled.

ethereal_compile_shaders() drops the shader formats of the graphics APIs Donut was configured without and,
when none is left, creates an empty target without an error; the engine would then find no kernel at run
time. This test fails such a build: for every row of the manifest and every shader format given, the blob
must exist, must not be empty and must hold every permutation of the row.

A row without -D axes is a raw shader binary. A row with axes is an NVSP container (ShaderToolBlob.h):
"NVSP", then per permutation two little-endian uint32 (key size, data size), the key ("NAME=VALUE" pairs
sorted by name and joined by spaces) and the data.

usage: check_shader_artifacts.py <manifest.cfg> <output dir> <format>...     (format: dxil, spirv, dxbc)
"""
import itertools
import os
import re
import struct
import sys

MAGIC = {
    'dxil': (b'DXBC', 'a DXBC/DXIL container'),
    'dxbc': (b'DXBC', 'a DXBC/DXIL container'),
    'spirv': (struct.pack('<I', 0x07230203), 'a SPIR-V module'),
}


def read_manifest(path):
    rows = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            if re.match(r'\s*(#|$)', line):
                continue
            tokens = line.split()
            source = tokens[0]
            entry = 'main'
            axes = {}
            i = 1
            while i < len(tokens):
                if tokens[i] == '-E':
                    entry = tokens[i + 1]
                    i += 2
                elif tokens[i] == '-D' or tokens[i].startswith('-D'):
                    define = tokens[i + 1] if tokens[i] == '-D' else tokens[i][2:]
                    i += 2 if tokens[i] == '-D' else 1
                    name, _, value = define.partition('=')
                    m = re.fullmatch(r'\{(.*)\}', value)
                    axes[name] = m.group(1).split(',') if m else [value or '1']
                else:
                    i += 1
            stem = os.path.splitext(source)[0]
            name = stem if entry == 'main' else '%s_%s' % (stem, entry)
            keys = []
            if axes:
                names = sorted(axes)
                for combination in itertools.product(*(axes[n] for n in names)):
                    keys.append(' '.join('%s=%s' % (n, v) for n, v in zip(names, combination)))
            rows.append((name + '.bin', keys))
    return rows


def permutations(data):
    """The permutations of an NVSP container: {key: data}."""
    found = {}
    cursor = 4
    while len(data) - cursor > 8:
        key_size, data_size = struct.unpack_from('<II', data, cursor)
        cursor += 8
        if data_size == 0 or cursor + key_size + data_size > len(data):
            break
        key = data[cursor:cursor + key_size].decode('ascii', 'replace')
        found[key] = data[cursor + key_size:cursor + key_size + data_size]
        cursor += key_size + data_size
    return found


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    manifest, output = argv[1], argv[2]
    formats = argv[3:]
    if not formats:
        print('check_shader_artifacts: no shader format is enabled in this build: the kernels were not compiled')
        return 1

    rows = read_manifest(manifest)
    if not rows:
        print('check_shader_artifacts: %s lists no kernels' % manifest)
        return 1

    problems = []
    shaders = 0
    for fmt in formats:
        if fmt not in MAGIC:
            print('check_shader_artifacts: unknown format %s' % fmt)
            return 2
        magic, what = MAGIC[fmt]
        for blob, keys in rows:
            path = os.path.join(output, fmt, blob)
            if not os.path.isfile(path):
                problems.append('%s: missing' % path)
                continue
            with open(path, 'rb') as f:
                data = f.read()
            if not data:
                problems.append('%s: empty' % path)
                continue
            if not keys:
                if not data.startswith(magic):
                    problems.append('%s: not %s' % (path, what))
                shaders += 1
                continue
            if not data.startswith(b'NVSP'):
                problems.append('%s: not a permutation container, but the row has -D axes' % path)
                continue
            found = permutations(data)
            for key in keys:
                if key not in found:
                    problems.append('%s: permutation "%s" is missing (it holds: %s)'
                                    % (path, key, ', '.join('"%s"' % k for k in found) or 'nothing'))
                elif not found[key].startswith(magic):
                    problems.append('%s: permutation "%s" is not %s' % (path, key, what))
                else:
                    shaders += 1

    for problem in problems:
        print(problem)
    if problems:
        print('check_shader_artifacts: %d problem(s)' % len(problems))
        return 1
    print('check_shader_artifacts: OK (%d rows, formats %s, %d shaders)' % (len(rows), ' '.join(formats), shaders))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
