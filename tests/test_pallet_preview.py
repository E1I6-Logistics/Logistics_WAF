"""Offline diagnostic-preview checks; no ROS node or trained model needed."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import json
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src')]
from preview_pallet_pose import analyze_object,base_estimate,camera_matrix,raw_candidates,accepted_refined_candidates
from pallet_vision import load_config,opening_model


class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.cfg=load_config(ROOT/'config/pallet_alignment_yolo.yaml')
        self.k=np.array([[500.,0,320],[0,500.,240],[0,0,1]])
        self.d=np.zeros(5)
        self.points=cv2.projectPoints(opening_model(self.cfg),np.array([0.,.2,0.]),
                                    np.array([.01,.04,.3]),self.k,self.d)[0].reshape(8,2)

    def test_synthetic_pose_and_units(self):
        result=analyze_object(self.points,np.ones(8),(480,640,3),self.k,self.d,self.cfg)
        self.assertTrue(result['geometry_pass'])
        np.testing.assert_allclose(result['candidates'][0]['translation_m'],[.01,.04,.3],atol=1e-5)
        self.assertNotIn('base',result)

    def test_rejected_points_still_show_raw_estimate(self):
        points=self.points.copy();points[3]+=[12,9]
        result=analyze_object(points,np.ones(8),(480,640,3),self.k,self.d,self.cfg)
        self.assertFalse(result['geometry_pass'])
        self.assertTrue(result['candidates'])
        self.assertGreater(result['candidates'][0]['rms_px'],self.cfg['max_reprojection_error_px'])
        self.assertIn('Preview geometry filter REJECTED',result['warnings'])

    def test_low_confidence_and_invalid_points_hide_pose(self):
        scores=np.ones(8);scores[0]=.2
        result=analyze_object(self.points,scores,(480,640,3),self.k,self.d,self.cfg)
        self.assertFalse(result['candidates'])
        points=self.points.copy();points[0,0]=np.nan
        self.assertEqual(raw_candidates(points,self.k,self.d,self.cfg),[])

    def test_saved_frame_refinement_preserves_inputs_and_resolves_seeds(self):
        fixture=json.loads((ROOT/'tests/fixtures/pallet_ippe_refinement.json').read_text())
        points=np.array(fixture['points']);original=points.copy()
        k=np.array(fixture['K']);d=np.array(fixture['D'])
        result=analyze_object(points,np.array(fixture['confidence']),(480,640,3),k,d,self.cfg)
        np.testing.assert_array_equal(points,original)
        self.assertTrue(result['geometry_pass'])
        self.assertEqual(result['pose_method'],'IPPE + LM')
        self.assertEqual(len(result['candidates']),2)
        for candidate in result['candidates']:
            self.assertTrue(candidate['refinement_ok'])
            self.assertGreater(candidate['ippe_rms_px'],2.)
            self.assertAlmostEqual(candidate['rms_px'],.79176,places=4)
            self.assertAlmostEqual(candidate['translation_m'][2],.28488,places=4)
        self.assertLess(result['candidate_normal_difference_deg'],.1)

    def test_controller_lm_opt_in_matches_preview(self):
        from pallet_pose_detector import detection_from_keypoints
        fixture=json.loads((ROOT/'tests/fixtures/pallet_ippe_refinement.json').read_text())
        points=np.array(fixture['points']);scores=np.array(fixture['confidence'])
        k=np.array(fixture['K']);d=np.array(fixture['D'])
        self.assertIsNone(detection_from_keypoints(points,scores,(480,640,3),k,d,self.cfg))
        cfg=dict(self.cfg,pose_refine_lm=True)
        det=detection_from_keypoints(points,scores,(480,640,3),k,d,cfg)
        self.assertIsNotNone(det)
        self.assertAlmostEqual(det.error,.79176,places=4)
        with patch('pallet_pose_refinement.cv2.solvePnPRefineLM',side_effect=cv2.error('failure')):
            self.assertIsNone(detection_from_keypoints(points,scores,(480,640,3),k,d,cfg))

    def test_refinement_failure_never_passes(self):
        with patch('preview_pallet_pose.cv2.solvePnPRefineLM',side_effect=cv2.error('failure')):
            result=analyze_object(self.points,np.ones(8),(480,640,3),self.k,self.d,self.cfg)
        self.assertFalse(result['geometry_pass'])
        self.assertTrue(result['candidates'])
        self.assertTrue(all(not c['refinement_ok'] for c in result['candidates']))

    def test_refinement_does_not_bypass_roi(self):
        cfg=dict(self.cfg,roi=[.05,.9,.95,.95])
        result=analyze_object(self.points,np.ones(8),(480,640,3),self.k,self.d,cfg)
        self.assertFalse(result['geometry_pass'])
        self.assertIn('Keypoint ROI / shape / size check failed',result['warnings'])

    def test_refined_pose_gates_and_ambiguity_remain(self):
        a={'refinement_ok':True,'rms_px':.5,'translation_m':[0,0,.3],
           'rotation':np.eye(3).tolist()}
        b=dict(a,rotation=cv2.Rodrigues(np.array([0.,.3,0.]))[0].tolist(),rms_px=.51)
        self.assertEqual(accepted_refined_candidates([a,b],self.cfg),[])
        self.assertEqual(accepted_refined_candidates([dict(a,rms_px=2)],self.cfg),[])
        self.assertEqual(accepted_refined_candidates([dict(a,translation_m=[0,0,2])],self.cfg),[])
        self.assertEqual(accepted_refined_candidates([dict(a,rotation=np.diag([1,-1,-1]).tolist())],self.cfg),[])

    def test_body_gap_uses_camera_offset_and_measured_front(self):
        candidate={'translation_m':[0,0,.162],'rotation':np.eye(3).tolist()}
        transform={'rotation':[[0,0,1],[-1,0,0],[0,-1,0]],'translation_m':[.076,0,.093]}
        result=base_estimate(candidate,transform,self.cfg)
        self.assertAlmostEqual(result['body_to_face_normal_gap_m'],.170)
        self.assertAlmostEqual(result['yaw_deg'],0)
        self.assertFalse(result['mounting_verified'])
        cfg=dict(self.cfg,body_front_x=None)
        self.assertIsNone(base_estimate(candidate,transform,cfg)['body_to_face_normal_gap_m'])

    def test_wrong_calibration_rejected(self):
        info={'k':self.k.ravel().tolist(),'d':self.d.tolist(),'width':640,'height':480}
        camera_matrix(info,(480,640,3))
        with self.assertRaises(ValueError):camera_matrix(info,(240,320,3))
        with self.assertRaises(ValueError):camera_matrix(dict(info,roi={'x_offset':10}),(480,640,3))
        with self.assertRaises(ValueError):camera_matrix(dict(info,distortion_model='equidistant'),(480,640,3))


if __name__=='__main__':unittest.main()
