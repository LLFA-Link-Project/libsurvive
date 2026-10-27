"""Isolate Linux replay snapshots and verify the libraries actually loaded.

A copied executable can retain an absolute build RUNPATH. Hashing its adjacent
libraries alone does not prove they were used. Pin library lookup to the snapshot
and inspect the running process's module list before accepting replay results.
"""
import hashlib
import os
from pathlib import Path
import subprocess


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def build_probe(destination):
    destination = Path(destination).resolve()
    source = Path(__file__).with_name('lighthouse_runtime_probe.c')
    subprocess.run(['cc', '-shared', '-fPIC', '-O2', str(source), '-o', str(destination), '-ldl'], check=True)
    return destination


def isolated_environment(binary, manifest, probe, extra_preloads=()):
    binary = Path(binary).resolve()
    if not (binary.parent / 'libsurvive.so.0').is_file():
        raise ValueError('Replay snapshot must contain libsurvive.so.0')
    environment = dict(os.environ)
    # Plugins can depend on another plugin by its bare DT_NEEDED name. Pin
    # that lookup too: otherwise an outer snapshot plugin's absolute build
    # RUNPATH can load a mutable dependency before the loader visits its copy.
    environment['LD_LIBRARY_PATH'] = ':'.join(map(str, (binary.parent, binary.parent / 'plugins')))
    # The plugin loader treats this as a filename whose sibling directories
    # contain plugins. Its first search root is the now-isolated core library.
    environment['SURVIVE_PLUGINS'] = str(binary)
    environment['LH_RUNTIME_MANIFEST'] = str(Path(manifest).resolve())
    preloads = [str(Path(probe).resolve()), *(str(Path(p).resolve()) for p in extra_preloads)]
    if environment.get('LD_PRELOAD'):
        preloads.append(environment['LD_PRELOAD'])
    environment['LD_PRELOAD'] = ':'.join(preloads)
    return environment


def verify_runtime(binary, manifest, expected_hashes=None, expected_extra_libraries=None):
    binary = Path(binary).resolve()
    expected_core = (binary.parent / 'libsurvive.so.0').resolve(strict=True)
    expected_plugins = {p.resolve() for p in (binary.parent / 'plugins').glob('*.so')}
    for path in [expected_core, *expected_plugins]:
        try:
            path.relative_to(binary.parent)
        except ValueError:
            raise ValueError(f'Snapshot library resolves outside its directory: {path}') from None
    rows = [line.split('\t', 1) for line in Path(manifest).read_text().splitlines()]
    executables = [Path(path).resolve() for kind, path in rows if kind == 'executable']
    modules = {Path(path).resolve() for kind, path in rows if kind == 'module'}
    cores = {p for p in modules if p.name.startswith('libsurvive.so')}
    plugins = {p for p in modules if 'plugins' in p.parts}
    if executables != [binary] or cores != {expected_core} or plugins != expected_plugins:
        raise ValueError(f'Replay loaded unexpected runtime: executable={executables}, '
                         f'core={sorted(map(str, cores))}, plugins={sorted(map(str, plugins))}; '
                         f'expected core={expected_core}, plugins={sorted(map(str, expected_plugins))}')
    files = {binary, expected_core, *expected_plugins}
    hashes = {str(p): sha256(p) for p in sorted(files)}
    if expected_hashes is not None:
        for path in files:
            relative = str(path.relative_to(binary.parent))
            if hashes[str(path)] != expected_hashes.get(relative):
                raise ValueError(f'Loaded runtime hash differs from snapshot proof: {relative}')
    result = dict(verified=True, executable=str(binary), core=str(expected_core),
                  plugins=sorted(map(str, plugins)), sha256=hashes,
                  manifest_sha256=sha256(manifest))
    if expected_extra_libraries is not None:
        extras = {}
        for filename, digest in expected_extra_libraries.items():
            path = Path(filename).resolve(strict=True)
            if path not in modules:
                raise ValueError(f'Expected extra library was not loaded: {path}')
            actual = sha256(path)
            if actual != digest:
                raise ValueError(f'Loaded extra library hash differs from proof: {path}')
            extras[str(path)] = actual
        result['extra_libraries'] = extras
    return result
