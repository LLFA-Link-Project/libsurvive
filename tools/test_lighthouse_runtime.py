from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import lighthouse_runtime as runtime


@unittest.skipUnless(sys.platform.startswith('linux') and shutil.which('cc'), 'Linux ELF and cc required')
class RuntimeIsolationTests(unittest.TestCase):
    def test_plugin_dependency_uses_snapshot_even_when_loaded_first(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            live, snapshot = root/'mutable-build', root/'snapshot'
            for path in (live, snapshot):
                (path/'plugins').mkdir(parents=True)
            library, dependency = root/'library.c', root/'dependency.c'
            plugin, main = root/'plugin.c', root/'main.c'
            library.write_text('int answer(void) { return 11; }\n')
            dependency.write_text('int dependency(void) { return 31; }\n')
            plugin.write_text('extern int dependency(void); int plugin(void) { return dependency(); }\n')
            main.write_text('''#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdio.h>
#include <string.h>
extern int answer(void);
int main(void) {
 Dl_info info; dladdr(answer,&info); char root[4096],path[4096];
 snprintf(root,sizeof(root),"%s",info.dli_fname); *strrchr(root,'/')=0;
 snprintf(path,sizeof(path),"%s/plugins/driver_first.so",root);
 void *first=dlopen(path,RTLD_NOW); if(!first)return 2;
 snprintf(path,sizeof(path),"%s/plugins/driver_dependency.so",root);
 void *second=dlopen(path,RTLD_NOW); if(!second)return 3;
 int (*one)(void)=dlsym(first,"plugin"),(*two)(void)=dlsym(second,"dependency");
 printf("%d %d %d\\n",answer(),one(),two()); return 0;
}
''')
            subprocess.run(['cc', '-shared', '-fPIC', str(library), '-Wl,-soname,libsurvive.so.0',
                            '-o', str(live/'libsurvive.so.0')], check=True)
            subprocess.run(['cc', '-shared', '-fPIC', str(dependency),
                            '-o', str(live/'plugins/driver_dependency.so')], check=True)
            subprocess.run(['cc', '-shared', '-fPIC', str(plugin), '-L'+str(live/'plugins'),
                            '-l:driver_dependency.so', '-Wl,-rpath,'+str(live/'plugins'),
                            '-o', str(live/'plugins/driver_first.so')], check=True)
            subprocess.run(['cc', str(main), '-L'+str(live), '-l:libsurvive.so.0', '-ldl',
                            '-Wl,-rpath,'+str(live), '-o', str(live/'survive-cli')], check=True)
            for name in ['survive-cli', 'libsurvive.so.0', 'plugins/driver_first.so',
                         'plugins/driver_dependency.so']:
                shutil.copyfile(live/name, snapshot/name)
            (snapshot/'survive-cli').chmod(0o755)
            dependency.write_text('int dependency(void) { return 72; }\n')
            subprocess.run(['cc', '-shared', '-fPIC', str(dependency),
                            '-o', str(live/'plugins/driver_dependency.so')], check=True)
            binary = snapshot/'survive-cli'
            probe = runtime.build_probe(root/'probe.so')
            manifest = root/'runtime.tsv'
            env = runtime.isolated_environment(binary, manifest, probe)
            self.assertEqual(subprocess.check_output([binary], env=env, text=True).strip(), '11 31 31')
            verified = runtime.verify_runtime(binary, manifest)
            self.assertEqual(set(verified['plugins']),
                             {str(snapshot/'plugins/driver_first.so'),
                              str(snapshot/'plugins/driver_dependency.so')})
            # The old root-only lookup can load both a mutable dependency and
            # its snapshot copy; checking only the outer plugin is insufficient.
            env['LD_LIBRARY_PATH'] = str(snapshot)
            self.assertEqual(subprocess.check_output([binary], env=env, text=True).strip(), '11 72 31')
            with self.assertRaises(ValueError):
                runtime.verify_runtime(binary, manifest)

    def test_absolute_build_runpath_cannot_override_snapshot_or_its_plugins(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            live, snapshot = root/'mutable-build', root/'snapshot'
            for path in (live, snapshot):
                (path/'plugins').mkdir(parents=True)
            library, plugin, main = root/'library.c', root/'plugin.c', root/'main.c'
            library.write_text('int answer(void) { return 11; }\n')
            plugin.write_text('extern int answer(void); int plugin(void) { return answer()+50; }\n')
            main.write_text('''#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdio.h>
#include <string.h>
extern int answer(void);
int main(void) {
 Dl_info info; dladdr(answer,&info); char path[4096];
 snprintf(path,sizeof(path),"%s",info.dli_fname); *strrchr(path,'/')=0;
 strcat(path,"/plugins/poser_fixture.so"); void *p=dlopen(path,RTLD_NOW);
 if(!p)return 2; int (*function)(void)=dlsym(p,"plugin");
 printf("%d %d\\n",answer(),function()); return 0;
}
''')
            subprocess.run(['cc', '-shared', '-fPIC', str(library), '-Wl,-soname,libsurvive.so.0',
                            '-o', str(live/'libsurvive.so.0')], check=True)
            subprocess.run(['cc', '-shared', '-fPIC', str(plugin), '-L'+str(live), '-l:libsurvive.so.0',
                            '-Wl,-rpath,'+str(live), '-o', str(live/'plugins/poser_fixture.so')], check=True)
            subprocess.run(['cc', str(main), '-L'+str(live), '-l:libsurvive.so.0', '-ldl',
                            '-Wl,-rpath,'+str(live), '-o', str(live/'survive-cli')], check=True)
            for name in ['survive-cli', 'libsurvive.so.0', 'plugins/poser_fixture.so']:
                shutil.copyfile(live/name, snapshot/name)
            (snapshot/'survive-cli').chmod(0o755)
            library.write_text('int answer(void) { return 22; }\n')
            subprocess.run(['cc', '-shared', '-fPIC', str(library), '-Wl,-soname,libsurvive.so.0',
                            '-o', str(live/'libsurvive.so.0')], check=True)
            binary = snapshot/'survive-cli'
            self.assertEqual(subprocess.check_output([binary], text=True).strip(), '22 72')
            probe = runtime.build_probe(root/'runtime-probe.so')
            manifest = root/'runtime.tsv'
            env = runtime.isolated_environment(binary, manifest, probe)
            self.assertEqual(subprocess.check_output([binary], env=env, text=True).strip(), '11 61')
            proof = runtime.verify_runtime(binary, manifest)
            self.assertEqual(proof['core'], str(snapshot/'libsurvive.so.0'))
            self.assertEqual(proof['plugins'], [str(snapshot/'plugins/poser_fixture.so')])
            hashes = {str(Path(p).relative_to(snapshot)): h for p, h in proof['sha256'].items()}
            runtime.verify_runtime(binary, manifest, hashes)
            # Additional preload implementations need their own path/hash
            # proof; an unchanged core alone says nothing about those edits.
            extra = root/'extra.so'
            extra_source = root/'extra.c'
            extra_source.write_text('int preload_fixture(void) { return 7; }\n')
            subprocess.run(['cc', '-shared', '-fPIC', str(extra_source), '-o', str(extra)], check=True)
            extras = {str(extra): runtime.sha256(extra)}
            with self.assertRaises(ValueError):
                runtime.verify_runtime(binary, manifest, hashes, extras)
            env = runtime.isolated_environment(binary, manifest, probe, extra_preloads=[extra])
            subprocess.run([binary], env=env, check=True, stdout=subprocess.DEVNULL)
            with_extra = runtime.verify_runtime(binary, manifest, hashes, extras)
            self.assertEqual(with_extra['extra_libraries'], extras)
            with self.assertRaises(ValueError):
                runtime.verify_runtime(binary, manifest, hashes, {str(extra): '0' * 64})
            hashes['libsurvive.so.0'] = '0' * 64
            with self.assertRaises(ValueError):
                runtime.verify_runtime(binary, manifest, hashes)
            # An unisolated run must be rejected even when all snapshot files
            # exist and have unchanged hashes.
            env['LD_LIBRARY_PATH'] = str(live)
            subprocess.run([binary], env=env, check=True, stdout=subprocess.DEVNULL)
            with self.assertRaises(ValueError):
                runtime.verify_runtime(binary, manifest)
            (snapshot/'libsurvive.so.0').unlink()
            (snapshot/'libsurvive.so.0').symlink_to(live/'libsurvive.so.0')
            with self.assertRaises(ValueError):
                runtime.verify_runtime(binary, manifest)


if __name__ == '__main__':
    unittest.main()
