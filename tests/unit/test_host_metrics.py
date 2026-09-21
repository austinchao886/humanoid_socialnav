"""Compare the exact runner safety function on CPU and GPU, including NaNs."""
import ast,time,unittest
from pathlib import Path
import torch
from motion_pipeline.host_metrics import on_host
class HostMetricsTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  tree=ast.parse(Path('/sim/run_motion_pipeline_sim.py').read_text())
  fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='scripted_critical_metrics');fn.decorator_list=[]
  scope={'torch':torch};exec(compile(ast.Module(body=[fn],type_ignores=[]),'metrics','exec'),scope);cls.function=staticmethod(scope[fn.name])
 def test_cpu_gpu_equivalence_and_nonfinite_detection(self):
  self.assertTrue(torch.cuda.is_available(),'GPU equivalence test requires a GPU')
  for seed in range(5):
   torch.manual_seed(seed)
   tensors=[torch.randn(13),torch.randn(1,53),torch.randn(1,53)]+[torch.randn(29) for _ in range(8)]
   tensors[8]=torch.ones(29)*20;tensors[10]=torch.ones(29)*50
   for command,tracking in [(False,False),(True,False),(True,True)]:
    for nonfinite in [False,True]:
     values=[x.clone().cuda() for x in tensors]
     if nonfinite:values[1][0,0]=float('nan')
     direct=self.function(*values,command,tracking).cpu()
     host=on_host(self.function,*values,command,tracking)
     torch.testing.assert_close(host,direct,rtol=1e-5,atol=1e-6,equal_nan=True)
     self.assertEqual(host[7].item(),0. if nonfinite else 1.)
if __name__=='__main__':unittest.main()
