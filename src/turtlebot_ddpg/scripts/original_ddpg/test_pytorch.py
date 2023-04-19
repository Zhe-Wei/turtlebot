#! /usr/bin/env python

import torch
import sys

print(sys.version)
print(torch.__version__)

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(device)

print(torch.cuda.current_device())
print(torch.rand(3,3).cuda())


# # 创建一个 3x2 矩阵，未初始化
# x = torch.empty(3, 2)
# print(x)
# # 创建一个0填充的矩阵，指定数据类型
# x = torch.zeros(3, 2, dtype=torch.long)
# print(x)
# # 使用指定数据创建tensor
# x = torch.tensor([3.2, 3])
# print(x)