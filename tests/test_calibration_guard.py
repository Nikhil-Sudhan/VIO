"""Synthetic configuration rejection tests; fixtures are not measured calibration."""
import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from calibration_guard import checked_bundle


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.config=self.root/'estimator.yaml';self.camcfg=self.root/'camera.json'
        self.camcfg.write_text(json.dumps({'main':{'size':[820,616]}}))
        self.contents={
            'estimator.yaml':{'max_cameras':1,'use_mask':False,'relative_config_imu':'imu.yaml','relative_config_imucam':'imucam.yaml'},
            'imu.yaml':{'imu0':{k:1. for k in ['gyroscope_noise_density','gyroscope_random_walk','accelerometer_noise_density','accelerometer_random_walk','update_rate']}},
            'imucam.yaml':{'cam0':{'T_cam_imu':[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],'timeshift_cam_imu':0.001,
                'camera_model':'pinhole','distortion_model':'radtan','intrinsics':[650,650,410,308],
                'distortion_coeffs':[0,0,0,0],'resolution':[820,616]}}}
        self.review={'accepted_for_hardware_estimation':True,'mounting_id':'SYNTHETIC-TEST-NOT-A-RIG',
            'camera_config_sha256':hashlib.sha256(self.camcfg.read_bytes()).hexdigest(),
            'imu_configured_registers':{'16':88},'supporting_reports':{'test-report.txt':hashlib.sha256(b'test fixture').hexdigest()}}
        (self.root/'test-report.txt').write_bytes(b'test fixture');self.save()

    def tearDown(self):self.temp.cleanup()

    def save(self):
        for name,data in self.contents.items():(self.root/name).write_text(yaml.safe_dump(data))
        self.review['files_sha256']={name:hashlib.sha256((self.root/name).read_bytes()).hexdigest() for name in self.contents}
        (self.root/'review.json').write_text(json.dumps(self.review))

    def test_exact_reviewed_fixture_loads(self):
        self.assertEqual(checked_bundle(self.config,self.camcfg)['mounting_id'],'SYNTHETIC-TEST-NOT-A-RIG')

    def test_camera_only_review_rejected(self):
        self.review['accepted_for_hardware_estimation']=False;self.save()
        with self.assertRaisesRegex(ValueError,'not been accepted'):checked_bundle(self.config,self.camcfg)

    def test_manual_demo_remains_rejected_without_explicit_mode(self):
        self.review.update(accepted_for_hardware_estimation=False,approved_for_manual_development_demo=True,
                           scope='manual development demo; NOT validated navigation');self.save()
        with self.assertRaisesRegex(ValueError,'not been accepted'):checked_bundle(self.config,self.camcfg)
        self.assertFalse(checked_bundle(self.config,self.camcfg,experimental=True)['accepted_for_hardware_estimation'])

    def test_experimental_mode_does_not_accept_camera_only_or_changed_files(self):
        self.review['accepted_for_hardware_estimation']=False;self.save()
        with self.assertRaisesRegex(ValueError,'not been accepted'):checked_bundle(self.config,self.camcfg,experimental=True)
        self.review.update(approved_for_manual_development_demo=True,scope='manual development demo; NOT validated navigation');self.save()
        with self.config.open('a') as f:f.write('\n# unreviewed edit\n')
        with self.assertRaisesRegex(ValueError,'reviewed hash'):checked_bundle(self.config,self.camcfg,experimental=True)

    def test_changed_file_rejected(self):
        with self.config.open('a') as f:f.write('\n# changed after review\n')
        with self.assertRaisesRegex(ValueError,'reviewed hash'):checked_bundle(self.config,self.camcfg)

    def test_mount_change_rejects_even_explicit_manual_demo(self):
        self.review.update(invalidated=True,invalidation_reason='Sensor mount shifted',
                           approved_for_manual_development_demo=True,
                           scope='manual development demo; NOT validated navigation');self.save()
        for experimental in (False,True):
            with self.assertRaisesRegex(ValueError,'Calibration invalidated: Sensor mount shifted'):
                checked_bundle(self.config,self.camcfg,experimental=experimental)

    def test_reflection_is_not_rotation(self):
        self.contents['imucam.yaml']['cam0']['T_cam_imu'][0][0]=-1;self.save()
        with self.assertRaisesRegex(ValueError,'proper rotation'):checked_bundle(self.config,self.camcfg)

    def test_nonfinite_noise_rejected(self):
        self.contents['imu.yaml']['imu0']['gyroscope_random_walk']=float('nan');self.save()
        with self.assertRaisesRegex(ValueError,'noise model'):checked_bundle(self.config,self.camcfg)

    def test_changed_geometry_rejected(self):
        self.camcfg.write_text(json.dumps({'main':{'size':[640,480]}}))
        with self.assertRaisesRegex(ValueError,'configuration changed'):checked_bundle(self.config,self.camcfg)


if __name__=='__main__':unittest.main()
