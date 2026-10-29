#!/usr/bin/env python3
"""Fetch the eight pinned ROBOTIS OMX-F meshes and verify Git blob identities."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / 'src/omx_project/omx_description'


def fetch(asset):
    path = DESCRIPTION / asset['destination']
    def valid(data):
        digest = hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()
        return digest == asset['git_blob_sha1']
    if path.exists() and valid(path.read_bytes()):
        return path.name + ': verified cache'
    data = urlopen(Request(asset['url'], headers={'User-Agent': 'OMX-simulation-setup'}), timeout=60).read()
    if not valid(data):
        raise RuntimeError('Mesh does not match pinned upstream Git object: ' + path.name)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.download')
    temporary.write_bytes(data)
    temporary.replace(path)
    return path.name + ': downloaded and verified'


def main():
    manifest = json.loads((DESCRIPTION / 'config/omx_f_assets.json').read_text())
    with ThreadPoolExecutor(max_workers=4) as pool:
        for result in pool.map(fetch, manifest['assets']):
            print(result)


if __name__ == '__main__':
    main()
