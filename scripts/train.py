import torch
import matplotlib.pyplot as plt
from tqdm import tqdm
import shutil
import config
import utils


from typing import Dict, Tuple




#from model import ForgetfulANN, ForgetfulANN_seq
from model import ForgetfulANN

import dataset
import random

import itertools
import operator









'''def _Img2Wp(forward_method, batch, batch_hidden):
    batch_image = batch['image'].to(config.TORCH_DEVICE)
    batch_waypoint = batch['waypoint'].to(config.TORCH_DEVICE)

    batch_outputs, batch_hidden = forward_method(x_img=batch_image, x_imu=None, h=batch_hidden)
    #batch_outputs, batch_hidden = model(x_img=batch_image, x_imu=None, h=batch_hidden)
    batch_loss = loss(batch_outputs, batch_waypoint)
    
    return batch_outputs, batch_hidden, batch_loss

def _Img2WpSpeed(forward_method, batch, batch_hidden):
    batch_image = batch['image'].to(config.TORCH_DEVICE)
    batch_waypoint = batch['waypoint'].to(config.TORCH_DEVICE)
    batch_speed = batch['speed'].to(config.TORCH_DEVICE)

    batch_outputs, batch_hidden = forward_method(x_img=batch_image, x_imu=None, h=batch_hidden)
    #batch_outputs, batch_hidden = model(x_img=batch_image, x_imu=None, h=batch_hidden)
    batch_loss\
        = loss(batch_outputs[:, :2], batch_waypoint)\
        + loss(batch_outputs[:, 2:], batch_speed) * config.LOSS_SPEED2WAYPOINT_RATIO
    
    return batch_outputs, batch_hidden, batch_loss

def _Img2Ctrlcmd(forward_method, batch, batch_hidden):
    batch_image = batch['image'].to(config.TORCH_DEVICE)
    batch_ctrlcmd = batch['ctrl_cmd'].to(config.TORCH_DEVICE)

    batch_outputs, batch_hidden = forward_method(x_img=batch_image, x_imu=None, h=batch_hidden)
    #batch_outputs, batch_hidden = model(x_img=batch_image, x_imu=None, h=batch_hidden)
    batch_loss= loss(batch_outputs, batch_ctrlcmd)
    
    return batch_outputs, batch_hidden, batch_loss

def _ImgImu2Wp(forward_method, batch, batch_hidden):
    batch_image = batch['image'].to(config.TORCH_DEVICE)
    batch_imu = batch['imu'].to(config.TORCH_DEVICE)
    batch_waypoint = batch['waypoint'].to(config.TORCH_DEVICE)

    batch_outputs, batch_hidden = forward_method(x_img=batch_image, x_imu=batch_imu, h=batch_hidden)
    #batch_outputs, batch_hidden = model(x_img=batch_image, x_imu=batch_imu, h=batch_hidden)
    batch_loss = loss(batch_outputs, batch_waypoint)
    
    return batch_outputs, batch_hidden, batch_loss

def _ImgImu2WpSpeed(forward_method, batch, batch_hidden):
    batch_image = batch['image'].to(config.TORCH_DEVICE)
    batch_imu = batch['imu'].to(config.TORCH_DEVICE)
    batch_waypoint = batch['waypoint'].to(config.TORCH_DEVICE)
    batch_speed = batch['speed'].to(config.TORCH_DEVICE)

    batch_outputs, batch_hidden = forward_method(x_img=batch_image, x_imu=batch_imu, h=batch_hidden)
    #batch_outputs, batch_hidden = model(x_img=batch_image, x_imu=batch_imu, h=batch_hidden)
    batch_loss\
        = loss(batch_outputs[:, :2], batch_waypoint)\
        + loss(batch_outputs[:, 2:], batch_speed) * config.LOSS_SPEED2WAYPOINT_RATIO
    
    return batch_outputs, batch_hidden, batch_loss

def _ImgImu2Ctrlcmd(forward_method, batch, batch_hidden):
    batch_image = batch['image'].to(config.TORCH_DEVICE)
    batch_imu = batch['imu'].to(config.TORCH_DEVICE)
    batch_ctrlcmd = batch['ctrl_cmd'].to(config.TORCH_DEVICE)

    batch_outputs, batch_hidden = forward_method(x_img=batch_image, x_imu=batch_imu, h=batch_hidden)
    #batch_outputs, batch_hidden = model(x_img=batch_image, x_imu=batch_imu, h=batch_hidden)
    batch_loss= loss(batch_outputs, batch_ctrlcmd)
    
    return batch_outputs, batch_hidden, batch_loss'''










'''# training function
def train(train_x2x):

    model.train()
    batch_loss_sum = 0.0
    batch_cnt = 0

    # Get random-order list of keys to training run data loaders
    keys = list(dataset.dataloaders["train"].keys())
    keys = random.sample(keys, len(keys))

    # For every training run...
    with tqdm(enumerate(keys), leave=False) as pbar_runs:
        pbar_runs.set_description(f"|    Training")
        
        for _, key in pbar_runs:
            
            dl = dataset.dataloaders["train"][key]
            #h = model.init_hidden(dl.batch_size)
            h = model.getZeroInitializedHiddenState(dl.batch_size, config.TORCH_DEVICE)

            # For every batch of the training run...
            with tqdm(enumerate(dl), leave=False) as pbar_batches:
                pbar_batches.set_description(f"|      {key[:-4]}")

                for _, batch in pbar_batches:
                    
                    batch_cnt += 1
                    h = h.data
                    optimizer.zero_grad()


                    _, h, batch_loss = train_x2x(model.forward, batch, h)
                    batch_loss_sum += batch_loss.item()
                    
                    batch_loss.backward()
                    optimizer.step()

            learn_rate_this_run = learn_rate_scheduler.get_last_lr()[0]
            learn_rate_scheduler.step()

    
    train_loss_this_epoch = batch_loss_sum/batch_cnt
    return train_loss_this_epoch, learn_rate_this_run'''


'''
def train_withRNN(train_x2x):

    model.train()
    train_loss_sum_over_batches = 0.0
    batch_count = 0

    # Get random-order list of keys to training run data loaders
    train_run_keys = list(dataset.dataloaders["train"].keys())
    train_run_n = len(train_run_keys)
    train_run_keys = random.sample(train_run_keys, train_run_n)

    # For every training run...
    with tqdm(enumerate(train_run_keys), total=train_run_n, leave=False) as TQDM1:
        for key_i, key in TQDM1:
            TQDM1.set_description(f"|    Training")

            # Get the data loader
            run_dataloader = dataset.dataloaders["train"][key]
            run_batch_n = len(run_dataloader)

            # Init the hidden state of the RNN
            batch_hidden = model.init_hidden(run_dataloader.batch_size)
            optimizer.zero_grad()

            # For every batch of the training run...
            with tqdm(enumerate(run_dataloader), total=run_batch_n, leave=False) as TQDM2:
                for batch_j, batch in TQDM2:
                    TQDM2.set_description(f"|      {key[:-4]}")

                    batch_count += 1
                    batch_hidden = batch_hidden.data
                    
                    _, batch_hidden, batch_loss = train_x2x(batch, batch_hidden)
                    #batch_image = batch['image'].to(config.TORCH_DEVICE)
                    #batch_labels = batch['labels'].to(config.TORCH_DEVICE)
                    #batch_outputs, batch_hidden = model(batch_image, batch_hidden)
                    #batch_loss = criterion(batch_outputs, batch_labels)



                    train_loss_sum_over_batches += batch_loss.item()
                    
                    batch_loss.backward()
                    optimizer.step()


            learn_rate_this_run = learn_rate_scheduler.get_last_lr()
            learn_rate_scheduler.step()

    
    train_loss_this_epoch = train_loss_sum_over_batches/batch_count
    return train_loss_this_epoch, learn_rate_this_run

'''


"""def validate(valid_x2x, epoch_i, lr):

    model.eval()
    valid_loss_sum_over_batches = 0.0
    batch_count = 0

    # Sorted order of datasets
    valid_run_keys = sorted(list(dataset.dataloaders["valid"].keys()))

    with torch.no_grad():
        # For every training run data loader...
        with tqdm(enumerate(valid_run_keys), total=len(valid_run_keys), leave=False) as TQDM1:
            for key_i, key in TQDM1:
                TQDM1.set_description(f"|    Validation")

                run_dataloader = dataset.dataloaders["valid"][key]
                run_batch_n = len(run_dataloader)
    
                batch_hidden = model.init_hidden(run_dataloader.batch_size)

                run_outputs_over_samples = []
                run_loss_over_batches = []
                run_batch_count = 0


                with tqdm(enumerate(run_dataloader), total=run_batch_n, leave=False) as TQDM2:
                    for batch_j, batch in TQDM2:
                        TQDM2.set_description(f"|      {key[:-4]}")

                        batch_count += 1
                        run_batch_count += 1

                        batch_hidden = batch_hidden.data

                        
                        batch_outputs, batch_hidden, batch_loss = valid_x2x(model.forward_valid, batch, batch_hidden)
                        #batch_image = batch['image'].to(config.TORCH_DEVICE)
                        #batch_labels = batch['labels'].to(config.TORCH_DEVICE)
                        #batch_outputs, batch_hidden = model(batch_image, batch_hidden)
                        #batch_loss = loss(batch_outputs, batch_labels)


                        valid_loss_sum_over_batches += batch_loss.item()
                        
                        '''
                        # plot the predicted validation keypoints after every...
                        # ... predefined number of epochs
                        if batch_j == 0:
                            utils.valid_keypoints_plot(batch_image, batch_outputs, batch_labels, epoch_i, ds_name[:-4], batch_j, lr)
                        #if batch_j == int(batch_n/2):
                        #    utils.valid_keypoints_plot(batch_image, batch_outputs, batch_labels, epoch_i, ds_name, batch_j)
                        if batch_j == batch_n - 1:
                            utils.valid_keypoints_plot(batch_image, batch_outputs, batch_labels, epoch_i, ds_name[:-4], batch_j, lr)
                        '''
                        
                        run_outputs_over_samples += list(batch_outputs.cpu().numpy())
                        run_loss_over_batches += [batch_loss.item()]
                
                run_loss_over_batches = list(itertools.accumulate(run_loss_over_batches, func=operator.add))
                run_loss_over_batches = [x / run_batch_count for x in run_loss_over_batches]

                if epoch_i % 5 == 0:
                    utils.generate_validation_run_video(
                        epoch_i,
                        key[:-4],
                        run_dataloader,
                        run_outputs_over_samples,
                        run_loss_over_batches
                        )


    valid_loss_this_epoch = valid_loss_sum_over_batches / batch_count
    return valid_loss_this_epoch"""






'''summary = {
    'ANN': {
        'CNN': {
            'Model': config.CNN_MODEL_ID,
            'Pretrained': config.CNN_PRETRAINED,
            'Frozen': config.CNN_FROZEN,
            'Dropout': config.CNN_DROPOUT,
        },
        'RNN': {
            'Type': config.RNN_TYPE if config.USE_RNN else "-",
            'Hidden Size': config.GRU_HIDDEN_SIZE if config.USE_RNN else "-",
            '#Layers': config.GRU_NUM_LAYERS if config.USE_RNN else "-",
            'Dropout': config.GRU_DROPOUT if config.USE_RNN else "-",
        },
        'FC': {
            'Output Size': config.FC_OUTPUT_SIZE if config.USE_FC else "-",
            '#Layers': config.FC_NUM_LAYERS if config.USE_FC else "-",
            'Act Fct': config.FC_ACTIVATION_FCT_ID if config.USE_FC else "-",
            'Dropout': config.FC_DROPOUT,
        },
    },
    'DATA': {
        'RUNS': {
            'Mode': config.RUN_SELECTION_MODE,
            'FIG8': 'Y' if config.RUN_USE_FIG8 else 'N',
            'ITL': 'Y' if config.RUN_USE_ITL else 'N',
            #'#Train/Sc': config.SCENE_TRAIN_RUN_N,
            #'#Valid/Sc': config.SCENE_VALID_RUN_N,
            '#Images': config.RUN_IMAGE_N,
        },
        'INPUTS': {
            'Img Resize': config.IMAGE_RESIZE_FACTOR,
            'IMU': 'Y' if config.INPUT_USE_IMU else 'N',
            'Seq Length': config.RNN_SEQUENCE_LEN if config.USE_RNN else "-",
            'Seq Step': config.RNN_SEQUENCE_STEP if config.USE_RNN else "-",
        },
        'LABELS': {
            'Waypoint': config.LABELS_USE_WAYPOINT,
            'Speed': config.LABELS_USE_SPEED,
            'Ctrl Cmd': config.LABELS_USE_CTRL_CMD,
        },
    },
    'LEARN': {
        'Batch Size': {
            '': config.BATCH_SIZE
        },
        '#Epochs': {
            '': config.EPOCH_N
        },
        'Optimizer': {
            '':config.OPTIMIZER_TYPE
        },
        'L RATE': {
            'Init': config.LEARN_RATE_INITIAL,
            'Scheduler': config.LEARN_RATE_SCHEDULER_TYPE,
            'Gamma': config.LEARN_RATE_GAMMA,
        },
        'LOSS': {
            'Type': config.LOSS_TYPE,
            'Sp/Wp Ratio': config.LOSS_SPEED2WAYPOINT_RATIO,
        }
        
    },
    'RESULTS': {
        'Epochs': [],
        'T Loss': [],
        'V Loss': [],
        'L Rate': [],
    }
}'''





def get_optimizer (id: str, model: torch.nn.Module, lr: float) -> torch.optim.Optimizer:
    try:
        return {
            'Adam': torch.optim.Adam(model.parameters(), lr=lr),
            'SGD': torch.optim.SGD(model.parameters(), lr=lr),
        }[id]
    except:
        raise ValueError(f"Unknown optimizer id: {id}.")


def get_learning_rate_scheduler (id: str, optimizer: torch.optim.Optimizer, 
                                    gamma: float) -> torch.optim.lr_scheduler._LRScheduler:
    try:
        return {
            'ExponentialLR': torch.optim.lr_scheduler.ExponentialLR(optimizer, gamma=gamma)
        }[id]
    except:
        raise ValueError(f"Unknown learning rate scheduler id: {id}.")


def get_loss (id: str) -> torch.nn.Module:
    try:
        return {
            'SmoothL1Loss': torch.nn.SmoothL1Loss(),
            'MSELoss': torch.nn.MSELoss()
        }[id]
    except:
        raise ValueError(f"Unknown loss id: {id}.")




def train (
    model: torch.nn.Module,
    optimizer: torch.optim.Optimizer,
    lr_scheduler: torch.optim.lr_scheduler._LRScheduler,
    loss: torch.nn.Module,
    dataloaders: Dict[str, torch.utils.data.DataLoader]
) -> Tuple[float, float]:

    # Init
    model.train()
    loss_batch_sum = 0.0
    batch_cnt = 0

    # Get random-order list of keys to training data loaders
    keys = list(dataloaders.keys())
    keys = random.sample(keys, len(keys))

    # For every training data loader
    with tqdm(keys, leave=False) as pbar_dl:
        pbar_dl.set_description(f"|    Training")
        
        for key in pbar_dl:
            
            dloader = dataloaders[key]
            #h = model.init_hidden(dl.batch_size)
            h = model.getZeroInitializedHiddenState(dloader.batch_size, config.TORCH_DEVICE)

            # For every batch
            with tqdm(dloader, leave=False) as pbar_b:
                pbar_b.set_description(f"|      {key}")

                for batch in pbar_b:
                    batch_cnt += 1
                    h = h.data
                    optimizer.zero_grad()


                    output, h = model.forward(
                        x_img=batch['inputs']['image'].to(config.TORCH_DEVICE), 
                        x_cat=batch['inputs']['imu'].to(config.TORCH_DEVICE), 
                        h=h
                    )

                    #batch_loss\
                    #    = loss(batch_outputs[:, :2], batch_waypoint)\
                    #    + loss(batch_outputs[:, 2:], batch_speed) * config.LOSS_SPEED2WAYPOINT_RATIO
                    batch_loss = loss(output, batch['label'].to(config.TORCH_DEVICE))
                    try:
                        batch_loss += model.get_regularization_term()
                    except:
                        pass
                    loss_batch_sum += batch_loss.item()

                    batch_loss.backward()
                    optimizer.step()

    
    lr_scheduler.step()
    
    # Return this epoch's training loss and learning rate
    return loss_batch_sum / batch_cnt, lr_scheduler.get_last_lr()[0]



def valid (
    model: torch.nn.Module,
    loss: torch.nn.Module,
    dataloaders: Dict[str, torch.utils.data.DataLoader]
) -> float:

    # Init
    model.eval()
    valid_loss_sum_over_batches = 0.0
    valid_batch_cnt = 0

    # Get sorted list of keys to validation data loaders
    keys = sorted(list(dataloaders.keys()))


    with torch.no_grad():
        # For every validation data loader...
        with tqdm(keys, leave=False) as pbar_dl:
            for key in pbar_dl:
                pbar_dl.set_description(f"|    Validation")

                dloader = dataloaders[key]

    
                batch_hidden = model.getZeroInitializedHiddenState(dloader.batch_size, config.TORCH_DEVICE)

                dloader_output_over_samples = []
                dloader_loss_over_batches = []
                dloader_batch_cnt = 0


                with tqdm(dloader, leave=False) as pbar_b:
                    pbar_b.set_description(f"|      {key}")

                    for batch in pbar_b:
                        valid_batch_cnt += 1
                        dloader_batch_cnt += 1
                        batch_hidden = batch_hidden.data

                        batch_output, batch_hidden = model.forward(
                            x_img=batch['inputs']['image'].to(config.TORCH_DEVICE), 
                            x_cat=batch['inputs']['imu'].to(config.TORCH_DEVICE), 
                            h=batch_hidden
                        )

                        batch_loss = loss(batch_output, batch['label'].to(config.TORCH_DEVICE))
                        #try:
                        #    batch_loss += model.get_regularization_term()
                        #except:
                        #    pass
                        valid_loss_sum_over_batches += batch_loss.item()


                        dloader_output_over_samples += list(batch_output.cpu().numpy())
                        dloader_loss_over_batches += [batch_loss.item()]
                
                dloader_loss_over_batches = list(itertools.accumulate(dloader_loss_over_batches, func=operator.add))
                dloader_loss_over_batches = [x / dloader_batch_cnt for x in dloader_loss_over_batches]

                if epoch_i % 5 == 0:
                    utils.generate_validation_run_video(
                        epoch_i,
                        key,
                        dloader,
                        dloader_output_over_samples,
                        dloader_loss_over_batches,
                        configuration['data']['loaded']['image']['height'],
                        configuration['data']['loaded']['image']['width'],
                        configuration['data']['image']['frame_rate'],
                        config.EXPERIMENT_DPATH
                    )

    # Return this epoch's validation loss
    return valid_loss_sum_over_batches / valid_batch_cnt



if __name__ == "__main__":

    plt.style.use('ggplot')

    configuration = config.get_configuration(config.user_input)
    


    model = ForgetfulANN(
        #ForgetfulANN.asDict(
        #    cnn_torchvision_model_id=params['ann']['cnn']['torchvision_model_id'],
        #    cnn_pretrained=params['ann']['cnn']['pretrained'],
        #    cnn_trainable=params['ann']['cnn']['trainable'],
        #    cat_enabled=params['ann']['cat']['enabled'],
        #    cat_input_size=params['ann']['cat']['input_size'],
        #    gru_enabled=params['ann']['gru']['enabled'],
        #    gru_hidden_size=params['ann']['gru']['hidden_size'],
        #    gru_num_layers=params['ann']['gru']['num_layers'],
        #    gru_dropout=params['ann']['gru']['dropout'],
        #    fc_enabled=params['ann']['fc']['enabled'],
        #    fc_width=params['ann']['fc']['width'],
        #    fc_num_layers=params['ann']['fc']['num_layers'],
        #    fc_activation_function_id=params['ann']['fc']['activation_function_id'],
        #    fc_dropout=params['ann']['fc']['dropout'],
        #    head_output_size=params['ann']['head']['output_size']
        #)
        configuration['ann']
    ).to(config.TORCH_DEVICE)

    optimizer = get_optimizer(
        id=configuration['learn']['optimizer_id'],
        model=model,
        lr=configuration['learn']['learning_rate']['initial_value']
    )

    lr_scheduler = get_learning_rate_scheduler(
        id=configuration['learn']['learning_rate']['scheduler_id'],
        optimizer=optimizer,
        gamma=configuration['learn']['learning_rate']['gamma']
    )

    loss = get_loss(id=configuration['learn']['loss']['id'])

    dataloaders = dataset.get_data_loaders()

    recordings = {
        'loss': {
            'id': configuration['learn']['loss']['id'],    
            'train': [],
            'valid': [],
        },
        'learn_rate': {
            'id': configuration['learn']['learning_rate']['scheduler_id'],
            'gamma': configuration['learn']['learning_rate']['gamma'],
            'values': [],
        }
    }

    
    print('\n\n.--- TRAIN FORGETFUL ANN\n|')
    with tqdm(range(configuration['learn']['num_epochs'])) as pbar:
        for epoch_i in pbar:
            pbar.set_description(f"|  Epochs")
            
            train_loss_i, learn_rate_i = train(
                model,
                optimizer,
                lr_scheduler,
                loss,
                dataloaders['train']
            )

            valid_loss_i = valid(
                model,
                loss,
                dataloaders['valid'],
                #epoch_i=epoch_i, 
                #lr=learn_rate_i
            )

            pbar.write(f"|    [{epoch_i:04d}] Loss:  {train_loss_i:.10f} (train)  |  {valid_loss_i:.10f} (valid)    //    LR:  {learn_rate_i:.10f}")
            if epoch_i + 1 == configuration['learn']['num_epochs']: pbar.write("", end='\n')

            recordings['loss']['train'].append(train_loss_i)
            recordings['loss']['valid'].append(valid_loss_i)
            recordings['learn_rate']['values'].append(learn_rate_i)
            



            utils.export_to_json(config.user_input, config.EXPERIMENT_DPATH/'user_input.json')

            # --- Update plot ---
            utils.update_recordings_plot(recordings)
            utils.export_to_json(recordings, config.EXPERIMENT_DPATH/'recordings.json')
            
            

            # --- Update recordings ---
            #with open(config.EXP_DIR/'recordings.json', 'w') as file:
            #    file.write(json.dumps(summary))

            #summary['RESULTS']['Epochs'].append(epoch_i + 1)
            #summary['RESULTS']['T Loss'] = train_loss_over_epochs
            #summary['RESULTS']['V Loss'] = valid_loss_over_epochs
            #summary['RESULTS']['L Rate'] = learn_rate_over_epochs
            #utils.save_summary(summary)

            
            # --- Save model ---
            utils.save_checkpoint(
                epoch_i, model, optimizer, loss, config.EXPERIMENT_DPATH/'checkpoint.pt')
            
            model.export_as_annotated_torch_script_module(
                config.EXPERIMENT_DPATH/'model_scripted_with_annotation.pt'
            )

            batch = next(iter(next(iter(dataloaders['valid'].values()))))
            model.export_as_traced_torch_script_module(
                batch['inputs']['image'].to(config.TORCH_DEVICE),
                batch['inputs']['imu'].to(config.TORCH_DEVICE),
                config.EXPERIMENT_DPATH/'model_scripted_with_tracing.pt'
            )
    print("|_________________________________________________")








"""if __name__ == "__main__":

    plt.style.use('ggplot')

    #if config.IMG__2__WP:
    #    _x2x = _Img2Wp
    #elif config.IMG__2__WP_SPEED:
    #    _x2x = _Img2WpSpeed
    #elif config.IMG__2__CTRLCMD:
    #    _x2x = _Img2Ctrlcmd
    #elif config.IMG_IMU__2__WP:
    #    _x2x = _ImgImu2Wp
    #elif config.IMG_IMU__2__WP_SPEED:
    #    _x2x = _ImgImu2WpSpeed
    #elif config.IMG_IMU__2__CTRLCMD:
    #    _x2x = _ImgImu2Ctrlcmd


    model = ForgetfulANN_seq() if config.USE_RNN else ForgetfulANN()
    model.to(config.TORCH_DEVICE)


    if config.OPTIMIZER_TYPE == 'Adam':
        optimizer = torch.optim.Adam(model.parameters(), lr=config.LEARN_RATE_INITIAL)
    elif config.OPTIMIZER_TYPE == 'SGD':
        optimizer = torch.optim.SGD(model.parameters(), lr=config.LEARN_RATE_INITIAL)
    else:
        raise ValueError(f"No implementation provided for 'OPTIMIZER_TYPE={config.OPTIMIZER_TYPE}'.")


    if config.LEARN_RATE_SCHEDULER_TYPE == 'ExponentialLR':
        learn_rate_scheduler = torch.optim.lr_scheduler.ExponentialLR(optimizer, gamma=config.LEARN_RATE_GAMMA)
    else:
        raise ValueError(f"No implementation provided for 'LEARNING_RATE_SCHEDULER_TYPE={config.LEARN_RATE_SCHEDULER_TYPE}'.")


    if config.LOSS_TYPE == 'SmoothL1Loss':
        loss = torch.nn.SmoothL1Loss()
    elif config.LOSS_TYPE == 'MSELoss':
        loss = torch.nn.MSELoss()
    else:
        raise ValueError(f"No implementation provided for 'LOSS_TYPE={config.LOSS_TYPE}'.")

    train_loss_over_epochs = []
    valid_loss_over_epochs = []
    learn_rate_over_epochs = []


    print('')
    print('')
    print(".--- TRAIN FORGETFUL ANN")
    print("|")

    # For every epoch...
    with tqdm(range(config.EPOCH_N), total=config.EPOCH_N) as pbar:
        for epoch_i in pbar:
            pbar.set_description(f"|  Epochs")
            
            train_loss_i, learn_rate_i = train(train_x2x=_x2x)
            valid_loss_i = validate(valid_x2x=_x2x, epoch_i=epoch_i, lr=learn_rate_i)

            pbar.write(f"|    [{epoch_i:04d}] Loss:  {train_loss_i:.10f} (train)  |  {valid_loss_i:.10f} (valid)    //    LR:  {learn_rate_i:.10f}")
            if epoch_i + 1 == config.EPOCH_N: pbar.write("", end='\n')

            train_loss_over_epochs.append(train_loss_i)
            valid_loss_over_epochs.append(valid_loss_i)
            learn_rate_over_epochs.append(learn_rate_i)
            

            # --- Update plot ---
            utils.plot_losses_and_learn_rate_over_epochs(
                train_loss_over_epochs,
                valid_loss_over_epochs,
                learn_rate_over_epochs
            )

            # --- Update summary ---
            summary['RESULTS']['Epochs'].append(epoch_i + 1)
            summary['RESULTS']['T Loss'] = train_loss_over_epochs
            summary['RESULTS']['V Loss'] = valid_loss_over_epochs
            summary['RESULTS']['L Rate'] = learn_rate_over_epochs
            utils.save_summary(summary)
            
            # --- Save model ---
            utils.save_model(epoch_i, model, optimizer, loss)
            utils.save_model_as_script_module(model, script_mode='annotation')
            utils.update_experiment_overview()

    print("|_________________________________________________")"""
