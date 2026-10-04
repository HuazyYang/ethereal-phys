#!/usr/bin/env python3
"""ABI lint for the public headers of ethereal_phys (engine/include/ethereal/phys).

The rules are those of donut/nvrhi/doc/com-interface-abi.md. This script does not reimplement them: it loads
nvrhi's own lint (donut/nvrhi/tests/abi_lint.py, unmodified) as a module and points it at the ethereal_phys
headers. That file hard-codes nvrhi's header names and export macros in module globals, which are replaced
here before it runs:

  - every header under the include directory is a public API header (R4 and T1 apply to all of them),
  - none is excluded,
  - ETHEREAL_PHYS_API / ETHEREAL_PHYS_C_API join the export macros,
  - the enums and typedefs of the nvrhi headers are known, so that nvrhi::FRESULT or nvrhi::CommandQueue in
    a signature is a scalar and not "a class by value".

Checked by nvrhi's lint (see its docstring): R1 no class by value in a virtual, R2 no virtual destructor,
R3 no overloaded virtual, R5 every virtual noexcept, R4 no base class on a data struct, T1 no module-private
type, E no export macro in a class and no member function without a body.

Added here, for rules of the document that nvrhi checks by review or by its own static_asserts:

  I1  every interface has NVRHI_IID(Name, "...") in front of it and NVRHI_DECLARE_UUID_TRAITS(Name) inside,
      and derives from exactly one interface (section 3.1).
  D1  every enum has a fixed underlying type (section 4.2).
  D2  no alignas, #pragma pack, [[no_unique_address]], long, wchar_t (section 4.4).
  D3  every data struct has a static_assert on its sizeof (section 4.6).

usage: abi_lint.py <nvrhi tests dir> <nvrhi include dir> <ethereal/phys include dir>
       abi_lint.py --self-test <nvrhi tests dir> <nvrhi include dir> <ethereal/phys include dir>

The self test plants one violation of each rule in a copy of the headers and expects the message of that
rule (not just any message) to be reported.
"""
import importlib.util
import os
import re
import shutil
import sys
import tempfile


class Everything:
    """A set that contains every header."""

    def __contains__(self, item):
        return True


def load_nvrhi_lint(nvrhi_tests_dir, nvrhi_include_dir):
    path = os.path.join(nvrhi_tests_dir, 'abi_lint.py')
    sys.dont_write_bytecode = True  # no __pycache__ beside nvrhi's script: the nvrhi tree stays untouched
    spec = importlib.util.spec_from_file_location('nvrhi_abi_lint', path)
    lint = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(lint)

    lint.EXCLUDED = set()
    lint.API_HEADERS = Everything()
    lint.R4_EXEMPT = set()
    lint.EXPORT_MACROS = list(lint.EXPORT_MACROS) + ['ETHEREAL_PHYS_API', 'ETHEREAL_PHYS_C_API']

    # The types of nvrhi that the ethereal_phys signatures mention.
    nvrhi_sources = {}
    for dirpath, _, files in os.walk(nvrhi_include_dir):
        for name in sorted(files):
            if name.endswith('.h'):
                with open(os.path.join(dirpath, name), encoding='utf-8', errors='replace') as fh:
                    rel = os.path.relpath(os.path.join(dirpath, name), nvrhi_include_dir).replace(os.sep, '/')
                    nvrhi_sources['nvrhi:' + rel] = lint.strip_source(fh.read())
    if not nvrhi_sources:
        raise SystemExit(f'abi_lint: no nvrhi headers under {nvrhi_include_dir}')

    collect = lint.collect_types

    def collect_with_nvrhi(state, sources):
        collect(state, nvrhi_sources)
        collect(state, sources)

    lint.collect_types = collect_with_nvrhi
    return lint


def read_headers(root):
    headers = {}
    for dirpath, _, files in os.walk(root):
        for name in sorted(files):
            if name.endswith('.h'):
                rel = os.path.relpath(os.path.join(dirpath, name), root).replace(os.sep, '/')
                with open(os.path.join(dirpath, name), encoding='utf-8', errors='replace') as fh:
                    headers[rel] = fh.read()
    return headers


def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    return re.sub(r'//[^\n]*', '', text)


CLASS_HEAD = re.compile(r'(?<!enum )\b(struct|class)\s+(\w+)\s*(?:final\s*)?(?::\s*([^{;]*?))?\s*\{')


def extra_checks(root):
    """I1, D1, D2, D3 on the raw headers (the macros are still visible there)."""
    errors = []
    for header, raw in sorted(read_headers(root).items()):
        code = strip_comments(raw)

        for m in CLASS_HEAD.finditer(code):
            name, bases = m.group(2), (m.group(3) or '').strip()
            if re.match(r'^I[A-Z]\w*$', name):
                if not re.search(r'\bNVRHI_IID\s*\(\s*%s\s*,\s*"[^"]*"\s*\)\s*(struct|class)\s+%s\b' % (name, name), code):
                    errors.append(f'{header}: {name}: I1 no NVRHI_IID({name}, "...") in front of the interface')
                if not re.search(r'\bNVRHI_DECLARE_UUID_TRAITS\s*\(\s*%s\s*\)' % name, code):
                    errors.append(f'{header}: {name}: I1 no NVRHI_DECLARE_UUID_TRAITS({name})')
                base_list = [b.strip() for b in bases.split(',') if b.strip()]
                base_names = [re.sub(r'^(public|protected|private|virtual)\s+', '', b).split('::')[-1] for b in base_list]
                if len(base_names) != 1 or not re.match(r'^I[A-Z]\w*$', base_names[0]):
                    errors.append(f'{header}: {name}: I1 an interface derives from exactly one interface ({bases or "none"})')
            else:
                if not re.search(r'\bstatic_assert\s*\(\s*sizeof\s*\(\s*%s\s*\)\s*==' % name, code):
                    errors.append(f'{header}: {name}: D3 no static_assert on sizeof({name})')

        for m in re.finditer(r'\benum\s+(?:class\s+|struct\s+)?(\w+)\s*([^{;]*)[{;]', code):
            if ':' not in m.group(2):
                errors.append(f'{header}: enum {m.group(1)}: D1 no fixed underlying type')

        for pattern, what in ((r'\balignas\b', 'alignas'), (r'#\s*pragma\s+pack\b', '#pragma pack'),
                              (r'\bno_unique_address\b', '[[no_unique_address]]'), (r'\blong\b', 'long'),
                              (r'\bwchar_t\b', 'wchar_t')):
            if re.search(pattern, code):
                errors.append(f'{header}: D2 {what} in a public header')
    return errors


def run(lint, root):
    """(errors, summary) of both sets of checks on the headers under root."""
    result = lint.lint_headers(root)
    errors = list(result.errors) + extra_checks(root)
    return errors, f'{result.interfaces} interfaces, {result.virtuals} virtual functions, {result.classes} classes'


# (name, the message expected, header, anchor, replacement)
DEVICE = 'struct IPhysDevice : nvrhi::IObject {'
REGION = 'struct CopyRegion {'
PLANTED = [
    ('R1 return', 'R1 returns', 'device.h', DEVICE, DEVICE + '\n    virtual PhysDeviceInfo planted() noexcept = 0;'),
    ('R1 parameter', 'R1 parameter', 'device.h', DEVICE, DEVICE + '\n    virtual void planted(CopyRegion region) noexcept = 0;'),
    ('R1 handle', 'R1 returns', 'device.h', DEVICE, DEVICE + '\n    virtual PhysBufferHandle planted() noexcept = 0;'),
    ('R1 nvrhi struct', 'R1 parameter', 'device.h', DEVICE, DEVICE + '\n    virtual void planted(nvrhi::BufferDesc desc) noexcept = 0;'),
    ('R2', 'R2 virtual destructor', 'device.h', DEVICE, DEVICE + '\n    virtual ~IPhysDevice() = default;'),
    ('R3', 'R3 overloaded', 'device.h', DEVICE,
     DEVICE + '\n    virtual void planted(uint32_t a) noexcept = 0;\n    virtual void planted(float a) noexcept = 0;'),
    ('R5', 'R5 virtual function is not noexcept', 'device.h', DEVICE, DEVICE + '\n    virtual void planted() = 0;'),
    ('R4', 'R4 data struct with a base class', 'params.h', REGION, 'struct PlantedBase { int a; };\nstatic_assert(sizeof(PlantedBase) == 4);\n'
                               'struct CopyRegion : PlantedBase {'),
    ('T1', 'T1 module-private type MonoPtr', 'params.h', REGION, REGION + '\n    MonoPtr<int> planted;'),
    ('E export macro', 'ETHEREAL_PHYS_API in a class scope', 'params.h', REGION, REGION + '\n    ETHEREAL_PHYS_API void planted();'),
    ('E bodiless member', 'E member function declared without a body', 'params.h', REGION, REGION + '\n    void planted() const;'),
    ('I1 no IID', 'I1 no NVRHI_IID', 'device.h', 'NVRHI_IID(IPhysDevice, ', '// NVRHI_IID(IPhysDevice, '),
    ('I1 no traits', 'I1 no NVRHI_DECLARE_UUID_TRAITS', 'device.h', 'NVRHI_DECLARE_UUID_TRAITS(IPhysDevice)', ''),
    ('I1 two bases', 'I1 an interface derives from exactly one interface', 'device.h', DEVICE, 'struct IPhysDevice : nvrhi::IObject, IPhysMessageCallback {'),
    ('D1', 'D1 no fixed underlying type', 'params.h', 'enum class FeatureMode : uint8_t {', 'enum class FeatureMode {'),
    ('D2 alignas', 'D2 alignas', 'params.h', REGION, 'struct alignas(16) CopyRegion {'),
    ('D2 pragma pack', 'D2 #pragma pack', 'params.h', REGION, '#pragma pack(push, 1)\n' + REGION),
    ('D2 long', 'D2 long', 'params.h', REGION, REGION + '\n    long planted = 0;'),
    ('D3', 'D3 no static_assert on sizeof(Planted)', 'params.h', REGION, 'struct Planted { int a; };\n' + REGION),
]


def self_test(lint, root):
    errors, _ = run(lint, root)
    if errors:
        print('self-test: the real headers do not pass:')
        for e in errors:
            print('  ' + e)
        return 1
    failures = 0
    for name, expect, header, anchor, replacement in PLANTED:
        tmp = tempfile.mkdtemp(prefix='ethereal_phys_abi_lint_')
        try:
            copy = os.path.join(tmp, 'phys')
            shutil.copytree(root, copy)
            path = os.path.join(copy, header)
            with open(path, encoding='utf-8', newline='') as fh:
                text = fh.read()
            newline = '\r\n' if '\r\n' in text else '\n'
            text = text.replace('\r\n', '\n')
            if anchor not in text:
                print(f'self-test {name}: anchor not found in {header}')
                failures += 1
                continue
            with open(path, 'w', encoding='utf-8', newline='') as fh:
                fh.write(text.replace(anchor, replacement, 1).replace('\n', newline))
            found = [e for e in run(lint, copy)[0] if expect in e]
            if found:
                print(f'self-test {name}: detected: {found[0]}')
            else:
                print(f'self-test {name}: NOT detected')
                failures += 1
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
    print('self-test: ' + ('FAILED' if failures else f'all {len(PLANTED)} planted violations detected'))
    return 1 if failures else 0


def main(argv):
    args = argv[1:]
    selftest = bool(args) and args[0] == '--self-test'
    if selftest:
        args = args[1:]
    if len(args) != 3:
        print(__doc__)
        return 2
    lint = load_nvrhi_lint(args[0], args[1])
    if selftest:
        return self_test(lint, args[2])
    errors, summary = run(lint, args[2])
    for e in errors:
        print(e)
    if errors:
        print(f'abi_lint: {len(errors)} violation(s)')
        return 1
    print(f'abi_lint: OK ({summary} checked)')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
