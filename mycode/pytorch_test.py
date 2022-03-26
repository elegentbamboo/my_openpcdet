import torch
print(torch.__version__)
t=torch.ones(1,1,2,2)
t=t.cuda()
t