"""Pull the canonical GitHub benchmark; activate verified releases only when idle."""
import argparse
import fcntl
import hashlib
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import tarfile
import time

REMOTE='https://github.com/BradleyBrewington/ArmFarm_Agent_Tools.git'
REQUIRED={'run_act_policy.py','act_runtime.py','motor_bus.py','provenance.py','policies.json',
          'launch_policy.py','run_act_loop.sh','sync_benchmark.py'}


def digest(data):return hashlib.sha256(data).hexdigest()


def atomic(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_name(path.name+'.tmp-'+str(os.getpid()))
    temporary.write_bytes(data);temporary.replace(path)


def unpack(data):
    files={}
    with tarfile.open(fileobj=io.BytesIO(data)) as archive:
        for member in archive.getmembers():
            if member.isdir():continue
            parts=Path(member.name).parts
            if len(parts)!=2 or parts[0]!='benchmark' or not member.isfile():
                raise ValueError('Unexpected benchmark archive member: '+member.name)
            payload=archive.extractfile(member).read()
            if parts[1].endswith('.py'):compile(payload,parts[1],'exec')
            files[parts[1]]=payload
    if not REQUIRED<=files.keys():raise ValueError('Incomplete benchmark release')
    json.loads(files['policies.json'])
    hashes={name:digest(data) for name,data in sorted(files.items())}
    return files,hashes,digest(json.dumps(hashes,sort_keys=True).encode())


def git(repo,*args,data=None):
    process=subprocess.Popen(['git','-C',str(repo),*args],stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE,stderr=subprocess.PIPE,start_new_session=True,
                             env={**os.environ,'GIT_TERMINAL_PROMPT':'0'})
    try:
        stdout,stderr=process.communicate(data,timeout=90)
        if process.returncode:raise RuntimeError(stderr.decode(errors='replace')[-1500:])
        return stdout
    finally:
        # Git's lazy-fetch helpers must not survive a timeout.
        try:os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        process.communicate()


def download_bundle(repo,commit):
    entries=[]
    for row in git(repo,'ls-tree','-r','-z',commit,'--','benchmark').split(b'\0'):
        if not row:continue
        metadata,name=row.split(b'\t',1);mode,kind,oid=metadata.split()
        parts=Path(name.decode()).parts
        if kind!=b'blob' or mode not in (b'100644',b'100755') or len(parts)!=2:
            raise ValueError('Unexpected benchmark tree entry: '+name.decode())
        entries.append((name.decode(),oid.decode()))
    if not entries:raise ValueError('Git revision has no benchmark folder')
    # Fetch these blobs explicitly. `git archive` on a partial clone can fetch
    # every missing blob in the fleet repository, even with a path argument.
    git(repo,'-c','fetch.negotiationAlgorithm=noop','fetch','--no-tags','--no-write-fetch-head',
        '--filter=blob:none','origin','--stdin',data=('\n'.join(oid for _,oid in entries)+'\n').encode())
    stream=io.BytesIO()
    with tarfile.open(fileobj=stream,mode='w') as archive:
        for name,oid in entries:
            payload=git(repo,'cat-file','blob',oid)
            member=tarfile.TarInfo(name);member.size=len(payload)
            archive.addfile(member,io.BytesIO(payload))
    return unpack(stream.getvalue())


def install(station,files,hashes,bundle,commit):
    tools=station/'workspace/tools';base=tools/'benchmark';release=base/'releases'/bundle
    current=base/'current'
    existing={}
    if current.exists() and (current/'release.json').exists():
        existing=json.loads((current/'release.json').read_text())
    # Reuse the code's original Git snapshot when unrelated fleet commits advance HEAD.
    if existing.get('bundle_sha256')==bundle:commit=existing['git_commit']
    release.mkdir(parents=True,exist_ok=True)
    changed=not current.exists() or current.resolve()!=release.resolve()
    for name,payload in files.items():
        path=release/name
        if not path.exists() or digest(path.read_bytes())!=hashes[name]:
            atomic(path,payload);changed=True
        if name.endswith('.sh'):path.chmod(0o755)
    metadata={'git_commit':commit,'bundle_sha256':bundle,'files':hashes,'remote':REMOTE}
    atomic(release/'release.json',(json.dumps(metadata,indent=2)+'\n').encode())
    for source,destination in [('launch_policy.py','run_act_policy.py'),('run_act_loop.sh','run_act_loop.sh'),('sync_benchmark.py','sync_act_benchmark.py')]:
        target=tools/destination;payload=files[source]
        if target.exists() and target.read_bytes()!=payload:
            backup=station/'state/benchmark-backups'/digest(target.read_bytes())/destination
            if not backup.exists():atomic(backup,target.read_bytes())
        if not target.exists() or target.read_bytes()!=payload:atomic(target,payload);changed=True
        if destination.endswith('.sh'):target.chmod(0o755)
    temporary=base/('current.tmp-'+str(os.getpid()))
    temporary.symlink_to(Path('releases')/bundle,target_is_directory=True)
    temporary.replace(current)
    return {'status':'updated' if changed else 'unchanged',**metadata,'release':str(release)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--station',type=Path,default=Path('/var/lib/armfarm/stations/armfarm'))
    parser.add_argument('--revision',default='main',help='Git branch or commit; use a commit to pin/roll back')
    a=parser.parse_args();state=a.station/'state';state.mkdir(parents=True,exist_ok=True)
    with (state/'benchmark-sync.lock').open('a') as sync_lock:
        fcntl.flock(sync_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:
            repo=state/'benchmark-git'
            if not repo.exists():
                repo.mkdir();git(repo,'init','--bare');git(repo,'remote','add','origin',REMOTE)
                git(repo,'config','remote.origin.promisor','true')
                git(repo,'config','remote.origin.partialclonefilter','blob:none')
            git(repo,'fetch','--depth=1','--filter=blob:none','--no-tags','origin',a.revision)
            commit=git(repo,'rev-parse','FETCH_HEAD').decode().strip()
            files,hashes,bundle=download_bundle(repo,commit)
            with (state/'act-benchmark.lock').open('a') as policy_lock:
                try:fcntl.flock(policy_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                except BlockingIOError:
                    result={'status':'deferred_active_run','available_git_commit':commit,'available_bundle_sha256':bundle}
                else:result=install(a.station,files,hashes,bundle,commit)
        except Exception as error:
            atomic(state/'benchmark-sync.json',json.dumps({'status':'error','error':str(error),'checked_at':time.time()}).encode())
            raise
        result['checked_at']=time.time();atomic(state/'benchmark-sync.json',json.dumps(result,indent=2).encode())
        print(json.dumps({k:v for k,v in result.items() if k!='files'}))


if __name__=='__main__':main()
