"""Replace only the Waffle camera subtree with the supplied Waffle Pi subtree.

Default is prepare/validate only. --apply backs up and atomically replaces the
resolved target. Installed symlinks are preserved. All non-camera elements must
remain identical. Run in a ROS environment providing xacro and check_urdf.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET


def camera_element(element):
    return element.tag in ('joint', 'link') and element.attrib.get('name', '').replace('${namespace}', '').startswith('camera_')


def structure(element):
    return (element.tag, sorted(element.attrib.items()), (element.text or '').strip(),
            [structure(child) for child in element])


def replace_camera(original, reference):
    pattern = r'  <joint name="\$\{namespace\}camera_joint" type="fixed">[\s\S]*?(?=\n</robot>)'
    matches = list(re.finditer(pattern, reference))
    before = list(re.finditer(pattern, original))
    if len(matches) != 1 or len(before) != 1:
        raise ValueError('Expected one camera subtree at the end of each URDF')
    replacement = ('  <!-- IMX219: standard Waffle Pi camera mounting geometry.\n'
                   '       Preserve the Waffle chassis and site-specific LiDAR calibration. -->\n'
                   + matches[0].group())
    candidate = original[:before[0].start()] + replacement + original[before[0].end():]
    old_tree, new_tree, ref_tree = map(ET.fromstring, (original, candidate, reference))
    if old_tree.attrib != new_tree.attrib:
        raise ValueError('Robot metadata changed')
    if [structure(e) for e in old_tree if not camera_element(e)] != [structure(e) for e in new_tree if not camera_element(e)]:
        raise ValueError('Non-camera robot elements changed')
    if [structure(e) for e in new_tree if camera_element(e)] != [structure(e) for e in ref_tree if camera_element(e)]:
        raise ValueError('Camera subtree differs from Waffle Pi reference')
    if any('camera_depth' in e.attrib.get('name','') for e in new_tree):
        raise ValueError('Depth camera frames remain in candidate')
    return candidate


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--target', type=Path, required=True)
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--apply', action='store_true')
    a = p.parse_args()
    target = a.target.resolve(strict=True)
    original = target.read_bytes()
    candidate = replace_camera(original.decode(), a.reference.read_text())
    a.output_dir.mkdir(parents=True, exist_ok=False)
    staged = a.output_dir/'candidate.urdf'
    staged.write_text(candidate)
    expanded = subprocess.check_output(['xacro', str(staged), 'namespace:='], text=True)
    expanded_path = a.output_dir/'expanded.urdf'
    expanded_path.write_text(expanded)
    checked = subprocess.run(['check_urdf',str(expanded_path)],text=True,capture_output=True)
    (a.output_dir/'check_urdf.txt').write_text(checked.stdout+checked.stderr)
    checked.check_returncode()
    (a.output_dir/'original.urdf').write_bytes(original)
    report = {'target':str(target),'reference':str(a.reference.resolve()),
              'before_sha256':hashlib.sha256(original).hexdigest(),
              'after_sha256':hashlib.sha256(candidate.encode()).hexdigest(),
              'non_camera_elements_unchanged':True,'camera_matches_waffle_pi':True,
              'applied':False}
    if a.apply:
        if target.read_bytes() != original:
            raise RuntimeError('Target changed after preparation; refusing to overwrite')
        with tempfile.NamedTemporaryFile(dir=target.parent,delete=False) as f:
            f.write(candidate.encode());tmp=Path(f.name)
        tmp.chmod(target.stat().st_mode & 0o777)
        tmp.replace(target)
        report['applied']=True
    (a.output_dir/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
