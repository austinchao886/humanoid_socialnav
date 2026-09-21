"""Evaluate unchanged safety reductions after one packed device-to-host copy."""
def on_host(function, *arguments):
    import torch
    tensors=arguments[:-2]
    sizes=[x.numel() for x in tensors]
    packed=torch.cat([x.reshape(-1) for x in tensors]).detach().cpu()
    values=[part.reshape(original.shape) for part,original in zip(packed.split(sizes),tensors)]
    return function(*values,*arguments[-2:])
