import importlib.util
from pathlib import Path
import unittest

p=Path(__file__).resolve().parents[2]/'tools/analysis/contact_velocity_bound.py'
if not p.exists():p=Path(__file__).with_name('contact_velocity_bound.py')
s=importlib.util.spec_from_file_location('bound',p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)

class BoundTests(unittest.TestCase):
    def test_pure_translation(self):
        self.assertEqual(m.point_speed_lower_bound([1.,0.,0.],[0.,0.,0.],.2),1.)
    def test_rolling_can_have_stationary_point(self):
        self.assertEqual(m.point_speed_lower_bound([1.,0.,0.],[0.,5.,0.],.2),0.)
    def test_rotation_cannot_explain_large_translation(self):
        self.assertAlmostEqual(m.point_speed_lower_bound([1.2,0.,0.],[0.,1.,0.],.2),1.)
    def test_invalid(self):
        for radius in (0.,-1.,float('nan'),True):
            with self.assertRaises(ValueError):m.point_speed_lower_bound([0.,0.,0.],[0.,0.,0.],radius)
        with self.assertRaises(ValueError):m.point_speed_lower_bound([float('nan'),0.,0.],[0.,0.,0.],.2)
    def test_contact_segments_and_gap(self):
        row=[0.]*16;row[7]=1.;row[15]=30.
        out=m.contact_summary([(0.,row),(.005,row),(.010,row)],.2,20.)
        self.assertAlmostEqual(out['material_point_travel_lower_bound_m'],.01)
        self.assertEqual(m.contact_summary([(0.,row),(.005,row)],.2,50.)['qualified_samples'],0)
        with self.assertRaises(ValueError):m.contact_summary([(0.,row),(.02,row)],.2,20.)

if __name__=='__main__':unittest.main()
