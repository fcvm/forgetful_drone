
from pathlib import Path
import torch
import config
import typing

import torchvision.models
from typing import Callable, Tuple
from typing import Dict
import json
import copy
import resnet8



class CAT (torch.nn.Module):
    
    def __init__ (self, dim: int, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.dim = dim

    def forward (self, x_0: torch.Tensor, x_1: torch.Tensor) -> torch.Tensor:
        return torch.cat(tensors=(x_0, x_1), dim=self.dim)




class CATIdentity (torch.nn.Identity):
    
    def __init__ (self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def forward (self, x_0: torch.Tensor, x_1: torch.Tensor) -> torch.Tensor:
        return x_0




class GRUIdentity (torch.nn.Identity):
    
    def __init__ (self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def forward (self, x: torch.Tensor, h: torch.Tensor) -> typing.Tuple[torch.Tensor, torch.Tensor]:
        return x, h




class ForgetfulANN (torch.nn.Module):

    @staticmethod
    def asDict(
        cnn_torchvision_model_id: str,
        cnn_pretrained: bool,
        cnn_trainable: bool,
        cat_enabled: bool,
        cat_input_size: int,
        gru_enabled: bool,
        gru_hidden_size: int,
        gru_num_layers: int,
        gru_dropout: float,
        fc_enabled: bool,
        fc_width: int,
        fc_num_layers: int,
        fc_activation_function_id: str,
        fc_dropout: float,
        head_output_size: int
    ) -> Dict[str, 
            Dict[str, str or bool] 
            or Dict[str, bool or int] 
            or Dict[str, bool or int or float] 
            or Dict[str, bool or int or str or float] 
            or Dict[str, int]]:
        return {
            'cnn': {
                'torchvision_model_id': cnn_torchvision_model_id,
                'pretrained': cnn_pretrained,
                'trainable': cnn_trainable
            },
            'cat': {
                'enabled': cat_enabled,
                'input_size': cat_input_size
            },
            'gru': {
                'enabled': gru_enabled,
                'hidden_size': gru_hidden_size,
                'num_layers': gru_num_layers,
                'dropout': gru_dropout
            },
            'fc': {
                'enabled': fc_enabled,
                'width': fc_width,
                'num_layers': fc_num_layers,
                'activation_function_id': fc_activation_function_id,
                'dropout': fc_dropout
            },
            'head': {
                'output_size': head_output_size
            }
        }


    def __init__ (
        self, 
        config: Dict[str,
            Dict[str, str or bool]
            or Dict[str, bool or int]
            or Dict[str, bool or int or float]
            or Dict[str, bool or int or str or float]
            or Dict[str, int]],
        *args, 
        **kwargs
    ):
        super().__init__(*args, **kwargs)

        self.config = config
        self.hard_coded = {
            'gru': {
                'bias': True,
                'batch_first': False,
                'bidirectional': False
            },
            'head': {
                'bias': True
            }
        }
        self.initParameters()
        self.cnn_output_size = self.params['cnn']['output_size']

        

        self.addToConsoleRepresentation('\n\n.--- FORGETFUL ANN\n|')
        self.initCNN(
            torchvision_model_id=self.params['cnn']['torchvision_model_id'],
            pretrained=self.params['cnn']['pretrained'],
            trainable=self.params['cnn']['trainable'])
        self.initCAT(
            enabled=self.params['cat']['enabled'],
            input_size=self.params['cat']['input_size'],
            cnn_output_size=self.params['cnn']['output_size'])
        self.initGRU(
            enabled=self.params['gru']['enabled'],
            input_size=self.params['cat']['output_size'],
            hidden_size=self.params['gru']['hidden_size'],
            num_layers=self.params['gru']['num_layers'],
            bias=self.params['gru']['bias'],
            batch_first=self.params['gru']['batch_first'],
            dropout=self.params['gru']['dropout'],
            bidirectional=self.params['gru']['bidirectional'])
        self.initFC(
            enabled=self.params['fc']['enabled'],
            input_size=self.params['gru']['output_size'],
            width=self.params['fc']['width'],
            num_layers=self.params['fc']['num_layers'],
            activation_function_id=self.params['fc']['activation_function_id'],
            dropout=self.params['fc']['dropout'])
        self.initHEAD(
            input_size=self.params['fc']['output_size'],
            output_size=self.params['head']['output_size'],
            activation_function_id=self.params['head']['activation_function_id'],
            bias=self.params['head']['bias'])
        self.addToConsoleRepresentation('|_________________________________________________')
        print(self.console_representation)



    

    def initParameters (self) -> None:
        derived: Dict[str, int] = {
            'cnn': {
                'output_size': -1
            },
            'cat': {
                'output_size': -1
            },
            'gru': {
                'output_size': -1
            },
            'fc': {
                'output_size': -1
            }
        }

        self.params = copy.deepcopy(self.config)
        for key_0 in self.hard_coded:
            for key_1 in self.hard_coded[key_0]:
                self.params[key_0][key_1] = self.hard_coded[key_0][key_1]
        for key_0 in derived:
            for key_1 in derived[key_0]:
                self.params[key_0][key_1] = derived[key_0][key_1]
        
        #print(json.dumps(self.summary, sort_keys=True, indent=4))




    def addToConsoleRepresentation (self, text: str) -> None:
        try:
            self.console_representation += text + '\n'
        except:
            self.console_representation = text + '\n'



    def saveConfigurationAsJSON (self, dirpath: Path) -> None:
        filename = "ForgetfulANN.config.json"
        with open(dirpath/filename, 'w') as file:
            file.write(json.dumps(self.config, sort_keys=True, indent=4))

    def saveParametersAsJSON (self, dirpath: Path) -> None:
        filename = "ForgetfulANN.params.json"
        with open(dirpath/filename, 'w') as file:
            file.write(json.dumps(self.params, sort_keys=True, indent=4))
    


    def initCNN (self, torchvision_model_id: str, pretrained: bool, trainable: bool) -> None:
        
        backbone: Callable[[torch.nn.Module], torch.nn.Sequential]\
            = lambda tm : torch.nn.Sequential(*list(tm.children())[:-1])

        tm = torchvision.models
        pt = pretrained
        cnn_model_output_size_dict: dict[str, typing.Tuple[torch.nn.Sequential, int]] = {
            'resnet8': (resnet8.ResNet8(width_scale=1), 1 * 128 *1),
            'resnet18': (backbone(tm.resnet18(pretrained=pt)), 512),
            'resnet50': (backbone(tm.resnet50(pretrained=pt)), 2048),
            'efficientnet_b0': (backbone(tm.efficientnet_b0(pretrained=pt)), 1280),
            'efficientnet_b5': (backbone(tm.efficientnet_b5(pretrained=pt)), 2048),
            'mobilenet_v3_small': (backbone(tm.mobilenet_v3_small(pretrained=pt)), 576),
            #"convnext_tiny": (backbone(tm..convnext_tiny(pretrained=pt), 576),
        }

        try:
            self.CNN, self.params['cnn']['output_size'] = cnn_model_output_size_dict[torchvision_model_id]
        except:
            raise ValueError(f"Unknown torchvision model ID: {torchvision_model_id}.")

         # Set the model parameters to trainable or frozen
        for p in self.CNN.parameters():
            p.requires_grad = trainable

        self.addToConsoleRepresentation('|  CNN  |'\
            + f'    - Model: {torchvision_model_id}'\
            + ('  - Pretrained' if pretrained else '  - Not pretrained')\
            + ('  - Trainable' if trainable else '  - Not trainable')\
            + f"  - Output size: {self.params['cnn']['output_size']}")


    def initCAT (self, enabled: bool, input_size: int, cnn_output_size: int) -> None:
        if enabled:
            self.CAT = CAT(dim=2)
            self.params['cat']['output_size'] = input_size + cnn_output_size
            self.addToConsoleRepresentation('|  CAT  |'\
                + f'    - Input sizes: {cnn_output_size}|{input_size}'\
                + f"  - Output size: {self.params['cat']['output_size']}")
        else:
            self.CAT = CATIdentity()
            self.params['cat']['output_size'] = cnn_output_size
            self.addToConsoleRepresentation('|  CAT  |    - None')




    def initGRU (self, enabled: bool, input_size: int, hidden_size: int, 
                num_layers: int, bias: bool, batch_first: bool, 
                dropout: float, bidirectional: bool) -> None:
        if enabled:
            self.GRU = torch.nn.GRU(input_size, hidden_size, num_layers, 
                                bias, batch_first, dropout, bidirectional)
            self.addToConsoleRepresentation('|  GRU  |'\
                + f'    - Input size: {input_size}'\
                + f'  - Hidden size: {hidden_size}'\
                + f'  - # Layers: {num_layers}'\
                + f'  - Dropout: {dropout}')
            self.params['gru']['output_size'] = hidden_size
        else:
            self.GRU = GRUIdentity()
            self.addToConsoleRepresentation('|  GRU  |    - None')
            self.params['gru']['output_size'] = input_size

    
    def initFC (self, enabled: bool, input_size: int, width: int, 
                num_layers: int, activation_function_id: str, dropout: float) -> None:
        if enabled:
            activation_function = self.getActivationFunction(activation_function_id)
            
            layers = [
                activation_function,
                torch.nn.Dropout(p=dropout, inplace=False),
                torch.nn.Linear(input_size, width),
            ]

            layers += [
                activation_function,
                torch.nn.Dropout(p=dropout, inplace=False),
                torch.nn.Linear(width, width),
            ] * (num_layers - 1)
                        
            self.FC = torch.nn.Sequential(*layers)
            self.params['fc']['output_size'] = width
            self.addToConsoleRepresentation('|  FC   |'\
                + f'    - Input size: {input_size}'\
                + f'  - Width: {width}'\
                + f'  - # Layers: {num_layers}'\
                + f'  - Act. function: {activation_function_id}'\
                + f'  - Dropout: {dropout}')
            
        else:
            self.FC = torch.nn.Identity()
            self.addToConsoleRepresentation('|  FC   |    - None')
            self.params['fc']['output_size'] = input_size



    def initHEAD (self, input_size: int, output_size: int, activation_function_id: str, bias: bool) -> None:
        activation_function = self.getActivationFunction(activation_function_id)

        self.HEAD = torch.nn.Sequential(
            activation_function,
            torch.nn.Linear(in_features=input_size, out_features=output_size,
                                            bias=bias, device=None, dtype=None)
        )
        self.addToConsoleRepresentation('|  HEAD |'\
            + f'    - Input size: {input_size}'\
            + f'  - Output size: {output_size}'\
            + f'  - Act. function: {activation_function_id}')


    def getActivationFunction (self, id: str) -> torch.nn.Module:
        activation_functions: dict[str, torch.nn.Module] = {
            'ReLU': torch.nn.ReLU(inplace=False),
            'PReLU': torch.nn.PReLU()
        }
        try:
            return activation_functions[id]
        except:
            raise ValueError(f"Unknown activation function ID: {id}.")


    def getZeroInitializedHiddenState (self, batch_size: int, device: str) -> torch.Tensor:
        weight = next(self.parameters()).data
        return weight.new(
            self.params['gru']['num_layers'],
            batch_size, 
            self.params['gru']['hidden_size']).zero_().to(device)


    
    def forward (self, x_img: torch.Tensor, x_cat: torch.Tensor, h: torch.Tensor
                                            ) -> typing.Tuple[torch.Tensor, torch.Tensor]:

        batch_size, seq_len, C, H, W = x_img.shape

        # CNN
        x_img = x_img.view(batch_size*seq_len, C, H, W)
        x_img = self.CNN(x_img)
        #x_img = x_img.squeeze(dim=3) # Remove dimensions caused by 3 channel img
        #x_img = x_img.squeeze(dim=2)
        x_img = x_img.view(batch_size, seq_len, self.cnn_output_size)

        # CAT | Return x_img
        x = self.CAT(x_img, x_cat)

        # GRU | Identity
        x = x.transpose(0, 1) # [sequence length, batch size, input size]
        x, h = self.GRU(x, h)
        x = x[-1] # Take only last element of sequence -> [batch size, input size]


        x = self.FC(x) # Fully connected | Identity
        x = self.HEAD(x) # Head
        return x, h



    def export_as_annotated_torch_script_module (self, fpath: Path) -> None:
        model_scripted = torch.jit.script(self)
        model_scripted.save(fpath)

    def export_as_traced_torch_script_module (
        self,
        x_img: torch.Tensor, 
        x_cat: torch.Tensor,
        fpath: Path
    ) -> None:
        batch_size = x_img.shape[0]
        h = torch.rand(self.params['gru']['num_layers'], batch_size, self.params['gru']['hidden_size']).to(config.TORCH_DEVICE)
        traced_input = [x_img, x_cat, h]
        model_scripted = torch.jit.trace(self, traced_input)
        model_scripted.save(fpath)


    def get_regularization_term (self) -> float:
        reg_term: float = 0.0
        
        try: reg_term += self.CNN.get_regularization_term()
        except: pass
        try: reg_term += self.CAT.get_regularization_term()
        except: pass
        try: reg_term += self.GRU.get_regularization_term()
        except: pass
        try: reg_term += self.FC.get_regularization_term()
        except: pass
        try: reg_term += self.HEAD.get_regularization_term()
        except: pass

        return reg_term






if __name__ == '__main__':
    
    cnn_torchvision_model_id = 'resnet18'
    cnn_pretrained = False
    cnn_trainable = False
    
    cat_enabled = True
    cat_input_size = 6
    
    gru_enabled = True
    gru_hidden_size = 1024
    gru_num_layers = 3
    gru_dropout = 0.25
    
    fc_enabled = True
    fc_width = 2024
    fc_num_layers = 3
    fc_activation_function_id = 'PReLU'
    fc_dropout = 0.4
    
    head_output_size = 3

    forgetful_ann = ForgetfulANN(
        ForgetfulANN.asDict(
        cnn_torchvision_model_id=cnn_torchvision_model_id,
        cnn_pretrained=cnn_pretrained,
        cnn_trainable=cnn_trainable,
        cat_enabled=cat_enabled,
        cat_input_size=cat_input_size,
        gru_enabled=gru_enabled,
        gru_hidden_size=gru_hidden_size,
        gru_num_layers=gru_num_layers,
        gru_dropout=gru_dropout,
        fc_enabled=fc_enabled,
        fc_width=fc_width,
        fc_num_layers=fc_num_layers,
        fc_activation_function_id=fc_activation_function_id,
        fc_dropout=fc_dropout,
        head_output_size=head_output_size
        )
    )

    forgetful_ann.saveConfigurationAsJSON(dirpath=config.SRC_DIR)





