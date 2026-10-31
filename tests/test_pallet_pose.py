"""Offline failure-path coverage for learned perception and dataset conversion."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'),str(ROOT/'scripts')]
from pallet_pose_schema import KEYPOINT_NAMES, labelme_rows
from pallet_pose_detector import detection_from_keypoints
from pallet_vision import load_config, opening_model
from prepare_pallet_pose import prepare


def annotation():
    return dict(imageWidth=640,imageHeight=480,shapes=[
        dict(label='pallet',shape_type='rectangle',points=[[100,200],[300,300]],group_id=1),
        *[dict(label=name,shape_type='point',points=[[120+i*10,250]],group_id=1)
          for i,name in enumerate(KEYPOINT_NAMES)]])


class DatasetTests(unittest.TestCase):
    def test_semantic_order_and_missing_visibility(self):
        data=annotation();data['shapes'].reverse()
        row=labelme_rows(data)[0]
        self.assertEqual(len(row),29)
        self.assertEqual(row[5:8],[120/640,250/480,2])
        data['shapes']=[s for s in data['shapes'] if s['label']!='left_top_right']
        self.assertEqual(labelme_rows(data)[0][8:11],[0,0,0])

    def test_background_must_be_explicit(self):
        data=annotation();data['shapes']=[]
        with self.assertRaises(ValueError):labelme_rows(data)
        data['flags']={'background':True}
        self.assertEqual(labelme_rows(data),[])

    def test_duplicate_invalid_and_ungrouped_points_rejected(self):
        for change in ('duplicate','nan','outside','no_group'):
            data=annotation()
            if change=='duplicate':data['shapes'].append(copy.deepcopy(data['shapes'][1]))
            if change=='nan':data['shapes'][1]['points'][0][0]=float('nan')
            if change=='outside':data['shapes'][1]['points'][0][0]=700
            if change=='no_group':data['shapes'][1]['group_id']=None
            with self.subTest(change=change),self.assertRaises(ValueError):labelme_rows(data)

    def test_session_partition_and_original_preservation(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'source';raw=root/'raw'
            (source/'images').mkdir(parents=True);(source/'annotations').mkdir()
            originals={}
            for i,session in enumerate(('a','b','c')):
                directory=raw/session/'frames';directory.mkdir(parents=True)
                path=directory/f'{i}.jpg';cv2.imwrite(str(path),np.full((480,640,3),i*50,np.uint8))
                originals[path]=path.read_bytes()
                (source/'images'/path.name).write_bytes(path.read_bytes())
                (source/'annotations'/f'{i}.json').write_text(json.dumps(annotation()))
            report=prepare(source,raw,root/'export',['b'],['c'])
            self.assertEqual(report['splits'],{'train':1,'val':1,'test':1})
            manifest=json.loads((root/'export/manifest.json').read_text())
            self.assertEqual({r['session']:r['split'] for r in manifest},{'a':'train','b':'val','c':'test'})
            for path,content in originals.items():self.assertEqual(path.read_bytes(),content)
            with self.assertRaises(ValueError):prepare(source,raw,root/'bad',['b'],['b'])


class PoseTests(unittest.TestCase):
    def setUp(self):
        self.cfg=load_config(ROOT/'config/pallet_alignment.yaml')
        self.k=np.array([[500.,0,320],[0,500.,240],[0,0,1]])
        self.d=np.zeros(5)
        self.xy=cv2.projectPoints(opening_model(self.cfg),np.array([0.,.2,0.]),
                                  np.array([.01,.04,.3]),self.k,self.d)[0].reshape(8,2)

    def detect(self,xy=None,scores=None):
        return detection_from_keypoints(self.xy if xy is None else xy,
                                       np.ones(8) if scores is None else scores,
                                       (480,640,3),self.k,self.d,self.cfg)

    def test_consistent_points_recover_pose(self):
        result=self.detect();self.assertIsNotNone(result)
        np.testing.assert_allclose(result.translation,[.01,.04,.3],atol=1e-5)

    def test_low_confidence_nonfinite_wrong_order_and_bad_geometry(self):
        scores=np.ones(8);scores[0]=.2
        self.assertIsNone(self.detect(scores=scores))
        bad=self.xy.copy();bad[0,0]=np.nan
        self.assertIsNone(self.detect(bad))
        self.assertIsNone(self.detect(self.xy[::-1]))
        bad=self.xy.copy();bad[3]+=[20,30]
        self.assertIsNone(self.detect(bad))
        self.assertIsNone(self.detect(self.xy+[-500,0]))


if __name__=='__main__':unittest.main()
