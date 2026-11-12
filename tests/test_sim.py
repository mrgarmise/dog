import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dog_sim import (CHANNELS, NOMINAL, ik, starting_config, joint_targets,
                     save_config, load_config, parse_triplets, read_freenove_point)

class DogSimulationTests(unittest.TestCase):
    def test_channels_unique_and_exact(self):
        self.assertEqual(CHANNELS, ((4,3,2),(7,6,5),(8,9,10),(11,12,13)))
        self.assertEqual(sorted(sum((list(x) for x in CHANNELS), [])), list(range(2,14)))

    def test_ik_nominal(self):
        for point in NOMINAL:
            self.assertEqual(len(ik(*point)), 3)

    def test_pose_has_twelve_outputs(self):
        rows = joint_targets(starting_config(), [list(x) for x in NOMINAL])
        self.assertEqual(len(rows),12)
        self.assertEqual(set(r['channel'] for r in rows), set(range(2,14)))
        self.assertTrue(all(102 <= r['pwm_count_at_50hz'] <= 512 for r in rows))

    def test_unreachable_rejected(self):
        for point in ((0,200,0),(0,1,0),(130,0,0)):
            with self.assertRaises(ValueError):
                ik(*point)

    def test_bad_data_rejected(self):
        with self.assertRaises(ValueError):
            parse_triplets('0,99,10;0,99,10')
        with self.assertRaises(ValueError):
            parse_triplets('0,nan,10;0,99,10;0,99,-10;0,99,-10')

    def test_save_load_and_backup(self):
        with tempfile.TemporaryDirectory() as td:
            dest = Path(td)/'calibration.json'
            cfg = starting_config()
            save_config(dest,cfg)
            self.assertEqual(load_config(dest),cfg)
            cfg['notes'] = 'second version'
            save_config(dest,cfg)
            self.assertEqual(load_config(dest)['notes'], 'second version')
            self.assertTrue((Path(td)/'calibration.json.bak').is_file())

    def test_bad_schema_rejected(self):
        cfg = starting_config()
        cfg['mode'] = 'hardware'
        with tempfile.TemporaryDirectory() as td:
            with self.assertRaises(ValueError):
                save_config(Path(td)/'x.json',cfg)

    def test_reference_point_import(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td)/'point.txt'
            path.write_text('0\t80\t10\n0\t80\t10\n0\t80\t-10\n0\t80\t-10\n')
            pts = read_freenove_point(path)
            self.assertEqual(pts[0], [0,80,10])

if __name__ == '__main__':
    unittest.main()
