import io
import fcntl
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch

from run_act_policy import ChunkPlayback,clamp_action,JOINTS
from provenance import select_policy
from sync_benchmark import unpack,install,REQUIRED,main as sync_main,download_bundle

HERE=Path(__file__).resolve().parent


def archive(files):
    stream=io.BytesIO()
    with tarfile.open(fileobj=stream,mode='w') as tar:
        for name,payload in files.items():
            entry=tarfile.TarInfo(name);entry.size=len(payload)
            tar.addfile(entry,io.BytesIO(payload))
    return stream.getvalue()


class Benchmark(unittest.TestCase):
    def test_complete_chunks_use_same_limits(self):
        calibration={j:{'range_min':0,'range_max':4095} for j in JOINTS}
        self.assertEqual(clamp_action([1000]*6,calibration),[180,180,180,180,12,100])
        self.assertEqual(clamp_action([-1000]*6,calibration),[-180,-180,-180,-180,-95,0])
        player=ChunkPlayback();player.load([[i]*6 for i in range(100)],10.)
        for i in range(100):
            now=player.next_due;self.assertFalse(player.exhausted(now))
            self.assertEqual(player.take(now),(i,[i]*6))
        self.assertAlmostEqual(player.next_due,10+100/30)
        self.assertFalse(player.exhausted(player.next_due-.001))
        self.assertTrue(player.exhausted(player.next_due))

    def test_policy_identity_comes_from_weights_not_robot_name(self):
        catalog=json.loads((HERE/'policies.json').read_text())
        with patch('provenance.sha256',return_value=catalog['models']['arm004-act-100k']['sha256']):
            path,model=select_policy(Path('/station'),Path('/custom/arm004-on-arm003'),hostname='arm-003')
            self.assertEqual(model['name'],'arm004-act-100k')
            self.assertEqual(path.name,'arm004-on-arm003')
        with patch('provenance.sha256',return_value='unknown'):
            with self.assertRaises(ValueError):select_policy(Path('/station'),Path('/unknown'))

    def test_identical_install_and_drift_repair_preserve_original_files(self):
        data=archive({'benchmark/'+name:(HERE/name).read_bytes() for name in REQUIRED})
        files,hashes,bundle=unpack(data)
        with tempfile.TemporaryDirectory() as temp:
            a=Path(temp)/'arm003';b=Path(temp)/'arm004'
            for station in (a,b):
                (station/'workspace/tools').mkdir(parents=True)
                (station/'workspace/tools/run_act_policy.py').write_text('old runner')
                install(station,files,hashes,bundle,'a'*40)
            self.assertEqual((a/'workspace/tools/run_act_policy.py').read_bytes(),(b/'workspace/tools/run_act_policy.py').read_bytes())
            (a/'workspace/tools/benchmark/current/run_act_policy.py').write_text('local drift')
            outcome=install(a,files,hashes,bundle,'b'*40)
            self.assertEqual(outcome['status'],'updated')
            self.assertEqual(outcome['git_commit'],'a'*40)
            self.assertEqual((a/'workspace/tools/benchmark/current/run_act_policy.py').read_bytes(),files['run_act_policy.py'])
            self.assertEqual(len(list((a/'state/benchmark-backups').glob('*/run_act_policy.py'))),1)

    def test_archive_cannot_escape_benchmark(self):
        with self.assertRaises(ValueError):unpack(archive({'benchmark/../../unexpected':b'x'}))
        with self.assertRaises(ValueError):unpack(archive({'benchmark/run_act_policy.py':b'print(1)'}))

    def test_download_requests_only_benchmark_blobs(self):
        blobs={str(i).zfill(40):(name,(HERE/name).read_bytes()) for i,name in enumerate(sorted(REQUIRED),1)}
        def fake_git(repo,*args,data=None):
            if args[0]=='ls-tree':return b''.join(f'100644 blob {oid}\tbenchmark/{name}\0'.encode() for oid,(name,_) in blobs.items())
            if args[0]=='-c':
                self.assertEqual(set(data.decode().splitlines()),set(blobs));return b''
            if args[0]=='cat-file':return blobs[args[2]][1]
            self.fail('Unexpected Git operation: '+str(args))
        with patch('sync_benchmark.git',side_effect=fake_git):
            files,_,_=download_bundle(Path('/repo'),'revision')
        self.assertEqual(set(files),REQUIRED)

    def test_sync_defers_without_touching_an_active_run(self):
        data=archive({'benchmark/'+name:(HERE/name).read_bytes() for name in REQUIRED})
        def fake_git(repo,*args):
            if args[0]=='rev-parse':return b'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n'
            if args[0]=='archive':return data
            return b''
        with tempfile.TemporaryDirectory() as temp:
            station=Path(temp);(station/'state').mkdir()
            with (station/'state/act-benchmark.lock').open('a') as lock:
                fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                with patch('sys.argv',['sync','--station',temp]),patch('sync_benchmark.git',side_effect=fake_git),patch('sync_benchmark.download_bundle',return_value=unpack(data)),patch('builtins.print'):
                    sync_main()
            result=json.loads((station/'state/benchmark-sync.json').read_text())
            self.assertEqual(result['status'],'deferred_active_run')
            self.assertFalse((station/'workspace/tools').exists())


if __name__=='__main__':unittest.main()
