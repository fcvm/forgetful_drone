import torch
from typing import Tuple
import math




class View (torch.nn.Module):
    def __init__ (self, sample_shape: Tuple[int], *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.tensor_shape = sample_shape
        
    def __repr__(self):
        return f'View{self.tensor_shape}'

    def forward(self, x: torch.Tensor):
        batch_size = x.size(0)
        batch_shape = (batch_size, *self.tensor_shape)
        return x.view(batch_shape)



class Conv2dSame (torch.nn.Conv2d):

    def calc_same_pad(self, i: int, k: int, s: int, d: int) -> int:
        return max((math.ceil(i / s) - 1) * s + (k - 1) * d + 1 - i, 0)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        ih, iw = x.size()[-2:]

        pad_h = self.calc_same_pad(i=ih, k=self.kernel_size[0], s=self.stride[0], d=self.dilation[0])
        pad_w = self.calc_same_pad(i=iw, k=self.kernel_size[1], s=self.stride[1], d=self.dilation[1])

        if pad_h > 0 or pad_w > 0:
            x = torch.nn.functional.pad(
                x, [pad_w // 2, pad_w - pad_w // 2, pad_h // 2, pad_h - pad_h // 2]
            )
        return torch.nn.functional.conv2d(
            x,
            self.weight,
            self.bias,
            self.stride,
            self.padding,
            self.dilation,
            self.groups,
        )





class ResidualBlock (torch.nn.Module):
    def __init__ (self, in_channels: int, width: int, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.activation_function = torch.nn.ReLU()

        self.convolution2d_0a = torch.nn.Conv2d(
            in_channels=in_channels, 
            out_channels=width, 
            kernel_size=(3, 3), 
            stride=(2, 2),
            padding=(1, 1)
        )
        
        self.convolution2d_0b = torch.nn.Conv2d(
            in_channels=width, 
            out_channels=width, 
            kernel_size=(3, 3), 
            stride=(1, 1),
            padding=(1, 1)
        )

        self.convolution2d_1 = torch.nn.Conv2d(
            in_channels=in_channels, 
            out_channels=width, 
            kernel_size=(1, 1), 
            stride=(2, 2),
            padding=(0, 0)
        )
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x_0 = self.activation_function(x)
        x_0 = self.convolution2d_0a(x_0)
        x_0 = self.activation_function(x_0)
        x_0 = self.convolution2d_0b(x_0)

        x_1 = self.convolution2d_1(x)
        
        return x_0 + x_1

    def get_regularization_term (self):
        return 1e-4 * (
            torch.norm(self.convolution2d_0a.weight, p=2) 
            + torch.norm(self.convolution2d_0b.weight, p=2)
        )



class ResNet8 (torch.nn.Module):

    def __init__ (self, width_scale: float, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.input_layer = torch.nn.Sequential(
            torch.nn.Conv2d(
                in_channels=3,
                out_channels=int(32*width_scale), 
                kernel_size=(5, 5), 
                stride=(2, 2),
                padding=(2, 2)
            ),
            torch.nn.MaxPool2d(
                kernel_size=(3,3),
                stride=(2, 2)
            )
        )

        self.residual_block_0 = ResidualBlock(
            in_channels=int(32*width_scale),
            width=int(32*width_scale)
        )

        self.residual_block_1 = ResidualBlock(
            in_channels=int(32*width_scale),
            width=int(64*width_scale)
        )

        self.residual_block_2 = ResidualBlock(
            in_channels=int(64*width_scale),
            width=int(128*width_scale)
        )

        self.output_layer = torch.nn.AdaptiveAvgPool2d((1,1))


    def forward (self, x:torch.Tensor) -> torch.Tensor:
        x = self.input_layer(x)
        x = self.residual_block_0(x)
        x = self.residual_block_1(x)
        x = self.residual_block_2(x)
        x = self.output_layer(x)
        #x = torch.flatten(x, start_dim=1)

        return x

    def get_regularization_term (self):
        return (
            self.residual_block_0.get_regularization_term()
            + self.residual_block_1.get_regularization_term()
            + self.residual_block_2.get_regularization_term()
        )