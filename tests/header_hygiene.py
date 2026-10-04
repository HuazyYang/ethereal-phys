#!/usr/bin/env python3
"""Header hygiene for the public headers of ethereal_phys (donut/nvrhi/doc/com-interface-abi.md, sections 2
and 7). The counterpart of donut/nvrhi/tests/header_hygiene.cmake, which is written for nvrhi's directory
layout and macro names.

It fails on:
  - an #include of <string> <vector> <optional> <array> <deque> <set> <map> <unordered_map> <list>
    <filesystem> <mutex> <memory> <functional>;
  - a std::string / vector / optional / function / shared_ptr / unique_ptr / weak_ptr / array / map token;
  - an #include of a Donut header other than <donut/core/math/math.h>, or of <nvrhi/core/foundation.h> or
    <nvrhi/core/threading.h> (implementation-side headers);
  - ETHEREAL_PHYS_API outside common/export.h (only ETHEREAL_PHYS_C_API functions are exported);
  - an ETHEREAL_PHYS_C_API declaration that is not noexcept, or whose name does not start with etherealPhys;
  - an include guard that is not ETHEREAL_PHYS_<PATH>_H, or an #endif that does not repeat it in a comment.

usage: header_hygiene.py <ethereal/phys include dir>
"""
import os
import re
import sys

FORBIDDEN_INCLUDES = ['string', 'vector', 'optional', 'array', 'deque', 'set', 'map', 'unordered_map', 'list',
                      'filesystem', 'mutex', 'memory', 'functional']
FORBIDDEN_TOKENS = ['string', 'vector', 'optional', 'function', 'shared_ptr', 'unique_ptr', 'weak_ptr', 'array',
                    'map', 'unordered_map']
FORBIDDEN_NVRHI = ['nvrhi/core/foundation.h', 'nvrhi/core/threading.h']
ALLOWED_DONUT = ['donut/core/math/math.h']


def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    return re.sub(r'//[^\n]*', '', text)


def check(root):
    violations = []
    count = 0
    for dirpath, _, files in os.walk(root):
        for name in sorted(files):
            if not name.endswith('.h'):
                continue
            count += 1
            rel = os.path.relpath(os.path.join(dirpath, name), root).replace(os.sep, '/')
            with open(os.path.join(dirpath, name), encoding='utf-8') as fh:
                raw = fh.read().replace('\r\n', '\n')
            code = strip_comments(raw)

            for include in re.findall(r'#\s*include\s*[<"]([^>"]+)[>"]', code):
                if include in FORBIDDEN_INCLUDES:
                    violations.append(f'{rel}: includes <{include}>')
                if include in FORBIDDEN_NVRHI:
                    violations.append(f'{rel}: includes <{include}> (an implementation-side header)')
                if include.startswith('donut/') and include not in ALLOWED_DONUT:
                    violations.append(f'{rel}: includes <{include}> (only donut::math may cross the boundary)')

            for token in FORBIDDEN_TOKENS:
                if re.search(r'\bstd\s*::\s*%s\b' % token, code):
                    violations.append(f'{rel}: uses std::{token}')

            if rel != 'common/export.h':
                if re.search(r'\bETHEREAL_PHYS_API\b', code):
                    violations.append(f'{rel}: uses ETHEREAL_PHYS_API (only ETHEREAL_PHYS_C_API functions are exported)')
                for decl in re.findall(r'\bETHEREAL_PHYS_C_API\b[^;{}]*', code):
                    decl = ' '.join(decl.split())
                    if not re.search(r'\bnoexcept\b', decl):
                        violations.append(f'{rel}: C export without noexcept: {decl}')
                    m = re.search(r'(\w+)\s*\(', decl)
                    if not m or not m.group(1).startswith('etherealPhys'):
                        violations.append(f'{rel}: C export not prefixed etherealPhys: {decl}')

            guard = 'ETHEREAL_PHYS_' + re.sub(r'[^A-Za-z0-9]', '_', rel[:-2]).upper() + '_H'
            lines = [line for line in raw.split('\n') if line.strip()]
            if len(lines) < 3 or lines[0] != f'#ifndef {guard}' or lines[1] != f'#define {guard}':
                violations.append(f'{rel}: the include guard is not {guard}')
            elif lines[-1] != f'#endif /* {guard} */':
                violations.append(f'{rel}: the last line is not "#endif /* {guard} */"')
    return count, violations


def main(argv):
    if len(argv) != 2:
        print(__doc__)
        return 2
    count, violations = check(argv[1])
    if count == 0:
        print(f'header_hygiene: no headers under {argv[1]}')
        return 1
    for v in violations:
        print(v)
    if violations:
        print(f'header_hygiene: {len(violations)} violation(s)')
        return 1
    print(f'header_hygiene: OK ({count} headers checked)')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
