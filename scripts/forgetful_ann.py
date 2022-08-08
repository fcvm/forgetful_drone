import torch
import torchvision
from typing import Dict, Tuple, Callable, Any
import copy
import pathlib
import json
import resnet8

TORCH_DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

def dict2json (dictionary: Dict) -> str:
    return json.dumps(dictionary, sort_keys=True, indent=4)


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

    def forward (self, x: torch.Tensor, h: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        return x, h




class ForgetfulANN (torch.nn.Module):

    @staticmethod
    def asDict(
        cnn_torchvisionModelId: str,
        cnn_pretrained: bool,
        cnn_trainable: bool,
        cat_inputSize: int or None,
        gru_numLayers: int or None,
        gru_hiddenSize: int or None,
        gru_dropout: float or None,
        fc_numLayers: int or None,
        fc_width: int or None,
        fc_activationFunctionId: str or None,
        fc_dropout: float or None,
        head_outputSize: int,
        head_activationFunctionId: str
    ) -> Dict [str, Dict [str, Any]]:
        
        return {
            'cnn': {
                'torchvision_model_id': cnn_torchvisionModelId,
                'pretrained': cnn_pretrained,
                'trainable': cnn_trainable
            },
            'cat': {
                'input_size': cat_inputSize
            },
            'gru': {
                'hidden_size': gru_hiddenSize,
                'num_layers': gru_numLayers,
                'dropout': gru_dropout
            },
            'fc': {
                'width': fc_width,
                'num_layers': fc_numLayers,
                'activation_function_id': fc_activationFunctionId,
                'dropout': fc_dropout
            },
            'head': {
                'activation_function_id': head_activationFunctionId,
                'output_size': head_outputSize
            }
        }


    def __init__ (
        self, 
        input_config: Dict [str, Dict [str, Any]],
        log_on : bool = True,
        *args, **kwargs
    ) -> None:
    
        super().__init__(*args, **kwargs)

        self.input_config = input_config
        self.config = self.initConfiguration(input_config)
        

        self.addToConsoleRepresentation('\n.--- INIT FORGETFUL ANN\n|')
        self.initCNN(
            torchvision_model_id=self.config['cnn']['torchvision_model_id'],
            pretrained=self.config['cnn']['pretrained'],
            trainable=self.config['cnn']['trainable'])
        self.cnn_output_size = self.config['cnn']['output_size'] # Necessary for torch scripting
        self.initCAT(
            cat_input_size=self.config['cat']['input_size'],
            cnn_output_size=self.config['cnn']['output_size'])
        self.initGRU(
            input_size=self.config['cat']['output_size'],
            hidden_size=self.config['gru']['hidden_size'],
            num_layers=self.config['gru']['num_layers'],
            bias=self.config['gru']['bias'],
            batch_first=self.config['gru']['batch_first'],
            dropout=self.config['gru']['dropout'],
            bidirectional=self.config['gru']['bidirectional'])
        self.initFC(
            input_size=self.config['gru']['output_size'],
            width=self.config['fc']['width'],
            num_layers=self.config['fc']['num_layers'],
            activation_function_id=self.config['fc']['activation_function_id'],
            dropout=self.config['fc']['dropout'])
        self.initHEAD(
            input_size=self.config['fc']['output_size'],
            output_size=self.config['head']['output_size'],
            activation_function_id=self.config['head']['activation_function_id'],
            bias=self.config['head']['bias'])
        self.addToConsoleRepresentation('|_________________________________________________')
        if log_on: print(self.console_representation)


    def initConfiguration (self, input_config: Dict [str, Dict [str, Any]]) -> Dict [str, Dict [str, Any]]:
        c = copy.deepcopy(self.input_config)
        
        gru_enabled = c['gru']['num_layers'] is not None
        c['gru']['bias'] = True if gru_enabled else None
        c['gru']['batch_first'] = False if gru_enabled else None
        c['gru']['bidirectional'] = False if gru_enabled else None
        c['head']['bias'] = False if gru_enabled else None

        c['cnn']['output_size'] = None
        c['cat']['output_size'] = None
        c['gru']['output_size'] = None
        c['fc']['output_size'] = None

        return c



    def addToConsoleRepresentation (self, text: str) -> None:
        try: self.console_representation += text + '\n'
        except: self.console_representation = text + '\n'

    def exportInputConfigAsJSON (self, dirpath: pathlib.Path) -> None:
        filename = "forgetfulAnn_inputConfig.json"
        with open(dirpath/filename, 'w') as file:
            file.write(dict2json(self.input_config))

    def exportConfigAsJSON (self, dirpath: pathlib.Path) -> None:
        filename = "forgetfulAnn_config.json"
        with open(dirpath/filename, 'w') as file:
            file.write(dict2json(self.config))

    def printInputConfig (self) -> None:
        print(f"INPUT CONFIG >> {dict2json(self.input_config)}")

    def printConfig (self) -> None:
        print(f"CONFIG >> {dict2json(self.config)}")
    


    def initCNN (self, torchvision_model_id: str, pretrained: bool, trainable: bool) -> None:
        
        backbone: Callable[[torch.nn.Module], torch.nn.Sequential]\
            = lambda tm : torch.nn.Sequential(*list(tm.children())[:-1])

        tm = torchvision.models
        pt = pretrained
        cnn_model_output_size_dict: dict[str, Tuple[torch.nn.Sequential, int]] = {
            'resnet8': (resnet8.ResNet8(width_scale=1), 1 * 128 *1),
            'resnet18': (backbone(tm.resnet18(pretrained=pt)), 512),
            'resnet50': (backbone(tm.resnet50(pretrained=pt)), 2048),
            'efficientnet_b0': (backbone(tm.efficientnet_b0(pretrained=pt)), 1280),
            'efficientnet_b5': (backbone(tm.efficientnet_b5(pretrained=pt)), 2048),
            'mobilenet_v3_small': (backbone(tm.mobilenet_v3_small(pretrained=pt)), 576),
            #"convnext_tiny": (backbone(tm..convnext_tiny(pretrained=pt), 576),
        }

        try:
            self.CNN, self.config['cnn']['output_size'] = cnn_model_output_size_dict[torchvision_model_id]
        except:
            raise ValueError(f"Unknown torchvision model ID: {torchvision_model_id}.")

         # Set the model parameters to trainable or frozen
        for p in self.CNN.parameters():
            p.requires_grad = trainable

        self.addToConsoleRepresentation('|  CNN  |'\
            + f'    - Model: {torchvision_model_id}'\
            + ('  - Pretrained' if pretrained else '  - Not pretrained')\
            + ('  - Trainable' if trainable else '  - Not trainable')\
            + f"  - Output size: {self.config['cnn']['output_size']}")


    def initCAT (self, cat_input_size: int or None, cnn_output_size: int) -> None:
        if cat_input_size is not None:
            self.CAT = CAT(dim=2)
            self.config['cat']['output_size'] = cat_input_size + cnn_output_size
            self.addToConsoleRepresentation('|  CAT  |'\
                + f'    - Input sizes: {cnn_output_size}|{cat_input_size}'\
                + f"  - Output size: {self.config['cat']['output_size']}")
        else:
            self.CAT = CATIdentity()
            self.config['cat']['output_size'] = cnn_output_size
            self.addToConsoleRepresentation('|  CAT  |    - None')




    def initGRU (self, input_size: int, hidden_size: int or None, 
                num_layers: int or None, bias: bool or None, batch_first: bool or None, 
                dropout: float or None, bidirectional: bool or None) -> None:
        if num_layers is not None:
            self.GRU = torch.nn.GRU(input_size, hidden_size, num_layers, 
                                bias, batch_first, dropout, bidirectional)
            self.addToConsoleRepresentation('|  GRU  |'\
                + f'    - Input size: {input_size}'\
                + f'  - Hidden size: {hidden_size}'\
                + f'  - # Layers: {num_layers}'\
                + f'  - Dropout: {dropout}')
            self.config['gru']['output_size'] = hidden_size
        else:
            self.GRU = GRUIdentity()
            self.addToConsoleRepresentation('|  GRU  |    - None')
            self.config['gru']['output_size'] = input_size

    
    def initFC (self, input_size: int, width: int or None, 
                num_layers: int or None, activation_function_id: str or None, dropout: float or None) -> None:
        
        if num_layers is not None:
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
            self.config['fc']['output_size'] = width
            self.addToConsoleRepresentation('|  FC   |'\
                + f'    - Input size: {input_size}'\
                + f'  - Width: {width}'\
                + f'  - # Layers: {num_layers}'\
                + f'  - Act. function: {activation_function_id}'\
                + f'  - Dropout: {dropout}')
            
        else:
            self.FC = torch.nn.Identity()
            self.addToConsoleRepresentation('|  FC   |    - None')
            self.config['fc']['output_size'] = input_size



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
        if self.config['gru']['num_layers'] is not None:
            return weight.new(
                self.config['gru']['num_layers'],
                batch_size, 
                self.config['gru']['hidden_size']).zero_().to(device)
        else:
            return weight.new(1, 1, 1).zero_().to(device) # Not in use anyways


    
    def forward (self, x_img: torch.Tensor, x_cat: torch.Tensor, h: torch.Tensor
                                            ) -> Tuple[torch.Tensor, torch.Tensor]:

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



    def exportAsAnnotatedTorchScriptModule (self, fpath: pathlib.Path) -> None:
        if self.training:
            self.eval()
            changed = True
        model_scripted = torch.jit.script(self)
        model_scripted.save(fpath)
        if changed: self.train()

    def exportAsTracedTorchScriptModule (
        self,
        x_img: torch.Tensor, 
        x_cat: torch.Tensor,
        fpath: pathlib.Path
    ) -> None:
        changed = False
        if self.training:
            self.eval()
            changed = True
        batch_size = x_img.shape[0]
        h = self.getZeroInitializedHiddenState(batch_size, TORCH_DEVICE).random_()
        traced_input = [x_img, x_cat, h]
        model_scripted = torch.jit.trace(self, traced_input)
        model_scripted.save(fpath)
        if changed: self.train()


    def getRegularizationTerm (self) -> float:
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

    forgetful_ann = ForgetfulANN(
        ForgetfulANN.asDict(
            cnn_torchvisionModelId='resnet18',
            cnn_pretrained=False,
            cnn_trainable=False,
            cat_inputSize=8,
            gru_hiddenSize=1024,
            gru_numLayers=3,
            gru_dropout=0.25,
            fc_width=2024,
            fc_numLayers=5,
            fc_activationFunctionId='PReLU',
            fc_dropout=0.5,
            head_outputSize=7,
            head_activationFunctionId='ReLU'
        )
    )

    forgetful_ann.printInputConfig()
    forgetful_ann.printConfig()
