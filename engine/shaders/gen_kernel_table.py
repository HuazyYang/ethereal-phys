#!/usr/bin/env python3
"""Generates engine/src/kernel_table.inl: the kernels of ethereal_phys and what each one binds.

    gen_kernel_table.py            rewrite engine/src/kernel_table.inl
    gen_kernel_table.py --check    exit 1 if the file on disk, or the KernelId enum of the public header,
                                   differs from what the manifest and the sources say (the ctest
                                   ethereal_phys_kernel_table)
    gen_kernel_table.py --enum     print the enumerators of ephys::KernelId
    gen_kernel_table.py --audit    print, per register, which view types the kernels declare there

The inputs are the manifest (ethereal_phys.cfg, one row per kernel source and entry point) and the resource
declarations of the kernel sources. A kernel's layout is what its source declares after preprocessing with
the defines of its permutation:

    cbuffer X : register(bN)                          constant buffer
    StructuredBuffer<T> x : register(tN)              structured SRV      RWStructuredBuffer -> UAV at uN
    ByteAddressBuffer x : register(tN)                raw SRV             RWByteAddressBuffer -> UAV at uN
    Buffer<uint> / Buffer<uint4> x : register(tN)     typed SRV, R32_UINT / RGBA32_UINT (RWBuffer -> UAV)
    Texture3D<float> x : register(tN)                 texture SRV
    SamplerState x : register(sN)                     sampler

For a structured buffer the table also holds the element stride, computed from the element type (scalars,
vectors, matrices and the structs the source defines; structured buffer elements are packed without
padding). The library refuses a buffer with another stride: D3D11 does, too, and on the other backends the
kernel would read garbage.

For a constant buffer the table holds its size, computed with the cbuffer packing rules of HLSL (16-byte
registers; a vector does not straddle a register; a struct and every array element start a register). The
library binds a buffer of at least that size and refuses a block that is larger.

One register holds different view types in different kernels (run --audit), which is why there is one
binding layout per distinct declaration list and not one layout for all kernels.

The preprocessor here handles what the kernel sources use: #include "file", object-like #define / #undef,
#if / #ifdef / #ifndef / #elif / #else / #endif with integer expressions and defined(). Function-like
macros are kept as text and never expanded; a resource must not be declared through one.
"""

import os
import re
import sys

SHADER_DIR = os.path.dirname(os.path.abspath(__file__))
MANIFEST = os.path.join(SHADER_DIR, 'ethereal_phys.cfg')
OUTPUT = os.path.normpath(os.path.join(SHADER_DIR, '..', 'src', 'kernel_table.inl'))
PUBLIC_HEADER = os.path.normpath(os.path.join(SHADER_DIR, '..', 'include', 'ethereal', 'phys', 'kernels.h'))

WAVE_AXIS = 'EPHYS_WAVE'
XPBD_AXIS = 'NVFLEX_XPBD'

MAX_SRV_SLOTS = 32
MAX_UAV_SLOTS = 8
MAX_CB_SLOTS = 2
MAX_CB_BYTES = 512  # ephys::c_KernelMaxConstantBufferSize


class Error(Exception):
    pass


# ---- the manifest ---------------------------------------------------------------------------------------

def read_manifest():
    rows = []
    with open(MANIFEST, encoding='utf-8') as f:
        for number, line in enumerate(f, 1):
            if re.match(r'\s*(#|$)', line):
                continue
            tokens = line.split()
            source, tokens = tokens[0], tokens[1:]
            entry, axes = None, {}
            i = 0
            while i < len(tokens):
                token = tokens[i]
                if token == '-T':
                    if tokens[i + 1] != 'cs':
                        raise Error('%s:%d: only compute kernels are expected' % (MANIFEST, number))
                    i += 2
                elif token == '-E':
                    entry = tokens[i + 1]
                    i += 2
                elif token == '-D' or token.startswith('-D'):
                    define = tokens[i + 1] if token == '-D' else token[2:]
                    i += 2 if token == '-D' else 1
                    m = re.fullmatch(r'(\w+)=\{([\d,]+)\}', define)
                    if not m:
                        raise Error('%s:%d: a -D must be an axis NAME={a,b}: %s' % (MANIFEST, number, define))
                    axes[m.group(1)] = [int(v) for v in m.group(2).split(',')]
                else:
                    raise Error('%s:%d: unexpected option %s' % (MANIFEST, number, token))
            if not entry:
                raise Error('%s:%d: -E is missing' % (MANIFEST, number))
            for name, values in axes.items():
                if name not in (WAVE_AXIS, XPBD_AXIS) or values != [0, 1]:
                    raise Error('%s:%d: unknown permutation axis %s=%s' % (MANIFEST, number, name, values))
            rows.append({'source': source, 'entry': entry, 'axes': axes})
    return rows


# ---- a small preprocessor --------------------------------------------------------------------------------

def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', lambda m: '\n' * m.group(0).count('\n'), text, flags=re.S)
    return re.sub(r'//[^\n]*', '', text)


class Preprocessor:
    def __init__(self, defines):
        self.macros = {name: str(value) for name, value in defines.items()}
        self.function_macros = set()
        self.lines = []

    def expand(self, text, depth=0):
        if depth > 32:
            raise Error('macro expansion too deep: ' + text)

        def replace(m):
            name = m.group(0)
            if name in self.macros and name not in self.function_macros:
                return self.expand(self.macros[name], depth + 1)
            return name
        return re.sub(r'\b[A-Za-z_]\w*\b', replace, text)

    def evaluate(self, expression):
        text = re.sub(r'\bdefined\s*\(\s*(\w+)\s*\)|\bdefined\s+(\w+)',
                      lambda m: '1' if (m.group(1) or m.group(2)) in self.macros else '0', expression)
        text = self.expand(text)
        text = re.sub(r'\b[A-Za-z_]\w*\b', '0', text)       # an unknown identifier is 0
        text = re.sub(r'\b(\d+)[uU]\b', r'\1', text)
        text = text.replace('&&', ' and ').replace('||', ' or ')
        text = re.sub(r'!(?!=)', ' not ', text)
        text = text.replace('/', '//')
        try:
            return bool(eval(text, {'__builtins__': {}}, {}))
        except Exception as e:
            raise Error('cannot evaluate #if %s (%s)' % (expression, e))

    def process(self, path):
        with open(path, encoding='utf-8') as f:
            text = strip_comments(f.read()).replace('\\\n', ' ')
        # (taken, any branch taken so far, parent active)
        stack = []

        def active():
            return all(frame[0] for frame in stack)

        for line in text.split('\n'):
            stripped = line.strip()
            m = re.match(r'#\s*(\w+)\s*(.*)', stripped)
            if not m:
                if active():
                    self.lines.append(line)
                continue
            directive, rest = m.group(1), m.group(2).strip()
            if directive in ('if', 'ifdef', 'ifndef'):
                if not active():
                    stack.append([False, True])
                    continue
                if directive == 'if':
                    value = self.evaluate(rest)
                else:
                    value = (rest.split()[0] in self.macros) == (directive == 'ifdef')
                stack.append([value, value])
            elif directive == 'elif':
                frame = stack[-1]
                parent_active = all(f[0] for f in stack[:-1])
                if frame[1] or not parent_active:
                    frame[0] = False
                else:
                    frame[0] = self.evaluate(rest)
                    frame[1] = frame[0]
            elif directive == 'else':
                frame = stack[-1]
                parent_active = all(f[0] for f in stack[:-1])
                frame[0] = parent_active and not frame[1]
                frame[1] = True
            elif directive == 'endif':
                stack.pop()
            elif not active():
                continue
            elif directive == 'include':
                m = re.match(r'"([^"]+)"', rest)
                if not m:
                    raise Error('%s: only #include "file" is supported: %s' % (path, stripped))
                self.process(os.path.normpath(os.path.join(os.path.dirname(path), m.group(1))))
            elif directive == 'define':
                m = re.match(r'(\w+)(\()?\s*(.*)', rest)
                name = m.group(1)
                if m.group(2):
                    self.function_macros.add(name)
                    self.macros[name] = ''
                else:
                    self.function_macros.discard(name)
                    self.macros[name] = m.group(3).strip()
            elif directive == 'undef':
                self.macros.pop(rest.split()[0], None)
                self.function_macros.discard(rest.split()[0])
            elif directive == 'error':
                raise Error('%s: #error %s' % (path, rest))
        if stack:
            raise Error('%s: unterminated #if' % path)


# ---- resource declarations ---------------------------------------------------------------------------------

BUFFER_RE = re.compile(
    r'\b(RW)?(StructuredBuffer|ByteAddressBuffer|Buffer|Texture3D)\s*(?:<\s*(\w+)\s*>)?\s+(\w+)\s*:\s*'
    r'register\(\s*([a-z])(\d+)\s*\)')
CBUFFER_RE = re.compile(r'\bcbuffer\s+(\w+)\s*:\s*register\(\s*b(\d+)\s*\)')
SAMPLER_RE = re.compile(r'\bSamplerState\s+(\w+)\s*:\s*register\(\s*s(\d+)\s*\)')
NUMTHREADS_RE = re.compile(r'\[\s*numthreads\s*\(([^)]*)\)\s*\]')

TYPED_FORMATS = {'uint': 'R32_UINT', 'uint4': 'RGBA32_UINT'}

CBUFFER_BODY_RE = re.compile(r'\bcbuffer\s+(\w+)\s*:\s*register\(\s*b(\d+)\s*\)\s*\{([^{}]*)\}')
STRUCT_RE = re.compile(r'\bstruct\s+(\w+)\s*\{([^{}]*)\}\s*;')
MEMBER_RE = re.compile(r'(\w+)\s+(\w+)\s*((?:\[[^\]]+\]\s*)*)$')
SCALAR_TYPES = ('int', 'uint', 'float', 'bool')


def element_size(type_name, structs, source):
    """The size in bytes of a structured buffer element of an HLSL type."""
    m = re.fullmatch(r'(int|uint|float|bool)([1-4])?(?:x([1-4]))?', type_name)
    if m:
        return 4 * int(m.group(2) or 1) * int(m.group(3) or 1)
    if type_name in structs:
        return structs[type_name]
    raise Error('%s: the size of type %s is not known' % (source, type_name))


def parse_members(body, pp, source, where):
    """The members of a struct or cbuffer body: [(type, [array dimensions])]."""
    members = []
    for statement in body.split(';'):
        statement = statement.strip()
        if not statement:
            continue
        member = MEMBER_RE.match(statement)
        if not member:
            raise Error('%s: %s: cannot parse member "%s"' % (source, where, statement))
        dimensions = []
        for dimension in re.findall(r'\[([^\]]+)\]', member.group(3)):
            value = pp.expand(dimension.strip())
            if not re.fullmatch(r'\d+', value):
                raise Error('%s: %s: array size %s is not a number' % (source, where, dimension))
            dimensions.append(int(value))
        members.append((member.group(1), dimensions))
    return members


def cbuffer_size(members, struct_members, source):
    """The size of a constant buffer with these members, by the packing rules of HLSL."""
    def align(offset):
        return (offset + 15) // 16 * 16

    def place(offset, type_name, dimensions):
        if dimensions:
            count = 1
            for dimension in dimensions:
                count *= dimension
            for _ in range(count):
                offset = place(align(offset), type_name, [])
            return offset
        m = re.fullmatch(r'(int|uint|float|bool)([1-4])?', type_name)
        if m:
            size = 4 * int(m.group(2) or 1)
            if offset % 16 + size > 16:
                offset = align(offset)
            return offset + size
        if type_name in struct_members:
            offset = align(offset)
            for member_type, member_dimensions in struct_members[type_name]:
                offset = place(offset, member_type, member_dimensions)
            return offset
        raise Error('%s: the cbuffer layout of type %s is not known' % (source, type_name))

    offset = 0
    for type_name, dimensions in members:
        offset = place(offset, type_name, dimensions)
    return align(offset)


def struct_sizes(text, pp, source):
    """The sizes of the structs a source defines, in definition order (a struct may use an earlier one)."""
    structs = {}
    for m in STRUCT_RE.finditer(text):
        name, body = m.group(1), m.group(2)
        size = 0
        for statement in body.split(';'):
            statement = statement.strip()
            if not statement:
                continue
            member = MEMBER_RE.match(statement)
            if not member:
                raise Error('%s: struct %s: cannot parse member "%s"' % (source, name, statement))
            count = 1
            for dimension in re.findall(r'\[([^\]]+)\]', member.group(3)):
                value = pp.expand(dimension.strip())
                if not re.fullmatch(r'\d+', value):
                    raise Error('%s: struct %s: array size %s is not a number' % (source, name, dimension))
                count *= int(value)
            size += count * element_size(member.group(1), structs, source)
        structs[name] = size
    return structs

# (kind, register class) in the order the bindings are listed in a layout
KIND_ORDER = ['ConstantBuffer', 'StructuredSRV', 'RawSRV', 'TypedSRV', 'TextureSRV',
              'StructuredUAV', 'RawUAV', 'TypedUAV', 'Sampler']
CLASS_ORDER = {'b': 0, 't': 1, 'u': 2, 's': 3}


def declarations(source, defines):
    pp = Preprocessor(defines)
    pp.process(os.path.join(SHADER_DIR, source))
    text = '\n'.join(pp.lines)

    structs = struct_sizes(text, pp, source)

    bindings = []
    for m in BUFFER_RE.finditer(text):
        rw, kind, element, name, reg_class, slot = m.groups()
        slot = int(slot)
        if reg_class != ('u' if rw else 't'):
            raise Error('%s: %s is declared in register class %s' % (source, name, reg_class))
        fmt = 'UNKNOWN'
        stride = 0
        if kind == 'StructuredBuffer':
            view = 'Structured'
            stride = element_size(element or 'float4', structs, source)
        elif kind == 'ByteAddressBuffer':
            view = 'Raw'
        elif kind == 'Buffer':
            view = 'Typed'
            if element not in TYPED_FORMATS:
                raise Error('%s: %s: no format for Buffer<%s>' % (source, name, element))
            fmt = TYPED_FORMATS[element]
        else:
            if rw:
                raise Error('%s: %s: texture UAVs are not expected' % (source, name))
            view = 'Texture'
        bindings.append((reg_class, slot, view + ('UAV' if rw else 'SRV'), fmt, stride, name))
    struct_members = {m.group(1): parse_members(m.group(2), pp, source, 'struct ' + m.group(1))
                      for m in STRUCT_RE.finditer(text)}
    cbuffers = 0
    for m in CBUFFER_BODY_RE.finditer(text):
        size = cbuffer_size(parse_members(m.group(3), pp, source, 'cbuffer ' + m.group(1)), struct_members, source)
        if size == 0 or size > MAX_CB_BYTES:
            raise Error('%s: cbuffer %s is %d bytes (1..%d expected)' % (source, m.group(1), size, MAX_CB_BYTES))
        bindings.append(('b', int(m.group(2)), 'ConstantBuffer', 'UNKNOWN', size, m.group(1)))
        cbuffers += 1
    if cbuffers != len(CBUFFER_RE.findall(text)):
        raise Error('%s: a cbuffer declaration was not understood' % source)
    for m in SAMPLER_RE.finditer(text):
        bindings.append(('s', int(m.group(2)), 'Sampler', 'UNKNOWN', 0, m.group(1)))

    if len(bindings) != len(re.findall(r'register\s*\(', text)):
        raise Error('%s: a register() declaration was not understood' % source)

    seen = {}
    for reg_class, slot, kind, fmt, stride, name in bindings:
        limit = {'t': MAX_SRV_SLOTS, 'u': MAX_UAV_SLOTS, 'b': MAX_CB_SLOTS, 's': 1}[reg_class]
        if slot >= limit:
            raise Error('%s: %s: register %s%d is out of range' % (source, name, reg_class, slot))
        if (reg_class, slot) in seen:
            raise Error('%s: register %s%d is declared twice (%s, %s)'
                        % (source, reg_class, slot, seen[(reg_class, slot)], name))
        seen[(reg_class, slot)] = name
    bindings.sort(key=lambda b: (CLASS_ORDER[b[0]], b[1]))

    groups = NUMTHREADS_RE.findall(text)
    if len(groups) != 1:
        raise Error('%s: expected exactly one numthreads, found %d' % (source, len(groups)))
    threads = []
    for part in groups[0].split(','):
        value = pp.expand(part.strip())
        if not re.fullmatch(r'\d+', value):
            raise Error('%s: numthreads argument %s is not a number' % (source, part))
        threads.append(int(value))
    if len(threads) != 3:
        raise Error('%s: numthreads needs three arguments' % source)

    return [b[:5] for b in bindings], [b[5] for b in bindings], threads


def enum_name(name):
    return name[0].upper() + name[1:]


def build():
    kernels = []
    for row in read_manifest():
        has_wave = WAVE_AXIS in row['axes']
        xpbd_values = row['axes'].get(XPBD_AXIS, [None])
        for xpbd in xpbd_values:
            per_wave = []
            for wave in ([0, 1] if has_wave else [None]):
                defines = {}
                if wave is not None:
                    defines[WAVE_AXIS] = wave
                if xpbd is not None:
                    defines[XPBD_AXIS] = xpbd
                per_wave.append(declarations(row['source'], defines))
            if any(p[0] != per_wave[0][0] or p[2] != per_wave[0][2] for p in per_wave):
                raise Error('%s: the %s permutations declare different resources or group sizes'
                            % (row['source'], WAVE_AXIS))
            name = row['entry'] + ('XPBD' if xpbd == 1 else '')
            stem = os.path.splitext(row['source'])[0]
            kernels.append({
                'name': name,
                'enum': enum_name(name),
                'entry': row['entry'],
                'blob': '%s_%s.bin' % (stem, row['entry']),
                'has_wave': has_wave,
                'xpbd': -1 if xpbd is None else xpbd,
                'bindings': per_wave[0][0],
                'names': per_wave[0][1],
                'threads': per_wave[0][2],
            })
    names = [k['enum'] for k in kernels]
    if len(set(names)) != len(names):
        raise Error('duplicate kernel names in the manifest')
    return kernels


def generate(kernels):
    layouts = []
    for kernel in kernels:
        key = tuple(kernel['bindings'])
        if key not in layouts:
            layouts.append(key)
        kernel['layout'] = layouts.index(key)

    out = []
    out.append('// Generated by engine/shaders/gen_kernel_table.py from engine/shaders/ethereal_phys.cfg and the')
    out.append('// resource declarations of the kernel sources. Do not edit: change the manifest or the source and')
    out.append('// run the script (the ethereal_phys_kernel_table test fails when this file is stale).')
    out.append('//')
    out.append('// %d kernels, %d binding layouts.' % (len(kernels), len(layouts)))
    out.append('')
    out.append('// Included by kernel_library.cpp inside namespace ephys::detail, after the declarations of')
    out.append('// KernelBindingDesc, KernelLayoutDesc and KernelDesc.')
    out.append('')
    out.append('static const KernelBindingDesc c_KernelBindings[] = {')
    first = []
    index = 0
    for number, layout in enumerate(layouts):
        users = [k['name'] for k in kernels if k['layout'] == number]
        out.append('    // layout %d: %s' % (number, ', '.join(users)))
        first.append(index)
        for reg_class, slot, kind, fmt, stride in layout:
            out.append('    {KernelBindingKind::%s, %d, nvrhi::Format::%s, %d},  // %s%d'
                       % (kind, slot, fmt, stride, reg_class, slot))
            index += 1
        if not layout:
            raise Error('a kernel declares no resources')
    out.append('};')
    out.append('')
    out.append('static const KernelLayoutDesc c_KernelLayouts[] = {')
    for number, layout in enumerate(layouts):
        out.append('    {%d, %d},' % (first[number], len(layout)))
    out.append('};')
    out.append('')
    out.append('// name, blob (below <shaderPath>/<dxil|spirv|dxbc>/), entry point, layout, wave axis, XPBD axis value')
    out.append('// (-1: the blob has no such axis), thread group size')
    out.append('static const KernelDesc c_KernelTable[] = {')
    for kernel in kernels:
        out.append('    {"%s", "%s", "%s", %d, %s, %d, {%d, %d, %d}},'
                   % (kernel['name'], kernel['blob'], kernel['entry'], kernel['layout'],
                      'true' if kernel['has_wave'] else 'false', kernel['xpbd'], *kernel['threads']))
    out.append('};')
    out.append('')
    out.append('// The table is indexed by KernelId.')
    for number, kernel in enumerate(kernels):
        out.append('static_assert(uint16_t(KernelId::%s) == %d);' % (kernel['enum'], number))
    out.append('static_assert(uint16_t(KernelId::Count) == %d);' % len(kernels))
    out.append('')
    return '\n'.join(out)


def header_enumerators():
    with open(PUBLIC_HEADER, encoding='utf-8') as f:
        text = strip_comments(f.read())
    m = re.search(r'enum\s+class\s+KernelId\s*:\s*uint16_t\s*\{(.*?)\}', text, re.S)
    if not m:
        raise Error('%s: enum class KernelId : uint16_t not found' % PUBLIC_HEADER)
    names = []
    for item in m.group(1).split(','):
        item = item.strip()
        if item:
            names.append(re.sub(r'\s*=.*', '', item))
    return names


def audit(kernels):
    per_register = {}
    for kernel in kernels:
        for reg_class, slot, kind, fmt, stride in kernel['bindings']:
            label = kind if fmt == 'UNKNOWN' else '%s(%s)' % (kind, fmt)
            per_register.setdefault((CLASS_ORDER[reg_class], slot, reg_class), {}).setdefault(label, []).append(
                kernel['name'])
    for (_, slot, reg_class), kinds in sorted(per_register.items()):
        summary = ', '.join('%s x%d' % (kind, len(users)) for kind, users in sorted(kinds.items()))
        print('%s%-3d %s' % (reg_class, slot, summary))


def main():
    try:
        kernels = build()
        text = generate(kernels)
        if '--enum' in sys.argv:
            for number, kernel in enumerate(kernels):
                print('    %s = %d,' % (kernel['enum'], number))
            print('    Count = %d,' % len(kernels))
            return 0
        if '--audit' in sys.argv:
            audit(kernels)
            return 0
        if '--check' in sys.argv:
            problems = 0
            on_disk = None
            if os.path.exists(OUTPUT):
                with open(OUTPUT, encoding='utf-8') as f:
                    on_disk = f.read().replace('\r\n', '\n')
            if on_disk != text:
                print('%s is stale: run engine/shaders/gen_kernel_table.py' % OUTPUT)
                problems += 1
            expected = [k['enum'] for k in kernels] + ['Count']
            actual = header_enumerators()
            if actual != expected:
                print('%s: the enumerators of KernelId differ from the manifest:' % PUBLIC_HEADER)
                for a, b in zip(actual + [''] * len(expected), expected + [''] * len(actual)):
                    if a != b:
                        print('    header %-40s manifest %s' % (a or '<none>', b or '<none>'))
                problems += 1
            if problems:
                return 1
            print('kernel table: %d kernels, up to date' % len(kernels))
            return 0
        with open(OUTPUT, 'w', encoding='utf-8', newline='\n') as f:
            f.write(text)
        print('wrote %s (%d kernels)' % (OUTPUT, len(kernels)))
        return 0
    except Error as e:
        print('gen_kernel_table.py: error: %s' % e)
        return 1


if __name__ == '__main__':
    sys.exit(main())
