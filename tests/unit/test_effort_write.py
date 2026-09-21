import sys,types,unittest
import torch
from types import SimpleNamespace as NS
from motion_pipeline.effort_write import install_zero_gain_effort_writer
class ImplicitActuator:pass
class EffortWriteTest(unittest.TestCase):
 def setUp(self):
  self.previous=sys.modules.get('isaaclab.actuators');sys.modules['isaaclab.actuators']=NS(ImplicitActuator=ImplicitActuator)
 def tearDown(self):
  if self.previous is None:sys.modules.pop('isaaclab.actuators',None)
  else:sys.modules['isaaclab.actuators']=self.previous
 def robot(self):
  a=ImplicitActuator();a.stiffness=torch.zeros(1,2);a.damping=torch.zeros(1,2);a.joint_indices=slice(None);a.effort_limit=torch.tensor([[10.,20.]]);a.velocity_limit=torch.tensor([[5.,6.]])
  data=NS(joint_stiffness=torch.zeros(1,2),joint_damping=torch.zeros(1,2),joint_effort_target=torch.tensor([[12.,-25.]]),computed_torque=torch.zeros(1,2),applied_torque=torch.zeros(1,2),soft_joint_vel_limits=torch.zeros(1,2))
  self.forces=[];self.wrenches=[]
  return NS(actuators={'all':a},data=data,_data=data,_joint_effort_target_sim=torch.zeros(1,2),has_external_wrench=True,_ALL_INDICES=torch.tensor([0]),_external_force_b=torch.ones(1,1,3),_external_torque_b=torch.zeros(1,1,3),write_data_to_sim=lambda:None,root_physx_view=NS(set_dof_actuation_forces=lambda x,i:self.forces.append(x.clone()),apply_forces_and_torques_at_position=lambda **kw:self.wrenches.append(kw)))
 def test_preserves_efforts_external_forces_and_diagnostic_clipping(self):
  r=self.robot();install_zero_gain_effort_writer(r);r.write_data_to_sim()
  torch.testing.assert_close(self.forces[0],torch.tensor([[12.,-25.]]))
  torch.testing.assert_close(r.data.computed_torque,torch.tensor([[12.,-25.]]))
  torch.testing.assert_close(r.data.applied_torque,torch.tensor([[10.,-20.]]))
  self.assertEqual(len(self.wrenches),1)
 def test_refuses_nonzero_gains(self):
  r=self.robot();r.data.joint_stiffness[0,0]=1
  with self.assertRaises(ValueError):install_zero_gain_effort_writer(r)
 def test_refuses_custom_actuator(self):
  r=self.robot();r.actuators={'custom':NS()}
  with self.assertRaises(ValueError):install_zero_gain_effort_writer(r)
if __name__=='__main__':unittest.main()
