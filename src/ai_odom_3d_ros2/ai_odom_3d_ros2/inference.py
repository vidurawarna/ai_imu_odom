from .model_resnet1d import *
import numpy as np
import quaternion

class ResNet:
    def __init__(self,model_path):

        # Parameters for the model
        self._input_channel, self._output_channel = 6, 4
        self.window_size = 200
        self._fc_config = {'fc_dim': 512, 'in_dim': 7, 'dropout': 0.4, 'trans_planes': 128}
        self._fc_config['in_dim'] = self.window_size // 32 + 1
        self.model_path = model_path
        self.device = torch.device('cpu')

        # Variables for calculations
        self.gyro_bias = None

        # Variables for the estimation
        self.pre_t = 0
        self.pre_pos = np.zeros((1,3))
        
        self.load_model()

    def load_model(self):

        checkpoint = torch.load(
            self.model_path,
            map_location=lambda storage, location: storage,
            weights_only=False
        )

        state_dict = checkpoint['model_state_dict']

        out_dim = None

        for key in reversed(list(state_dict.keys())):
            tensor = state_dict[key]
            if hasattr(tensor, "ndim") and tensor.ndim == 2:
                out_dim = tensor.shape[0]
                print(f"[INFO] Detected output dim from {key}: {out_dim}")
                break

        if out_dim is None:
            raise ValueError("Could not infer output dimension from checkpoint")

        self._output_channel = out_dim

        self.network = ResNet1D(
            self._input_channel,
            self._output_channel,
            BasicBlock1D,
            [2, 2, 2, 2],
            base_plane=64,
            output_block=FCOutputModule,
            kernel_size=3,
            **self._fc_config
        )

        self.network.load_state_dict(state_dict)
        self.network.eval().to(self.device)

        print(f"Model {self.model_path} loaded to device {self.device} with output dim {out_dim}.")
    
    def set_gyro_bias(self, bias, timestamp):
        '''
            Gyro bias as a numpy array of shape (3, )  -> [vx, vy, vz]
            timestamp:  Time of bias calculation
        '''
        self.gyro_bias = bias
        self.pre_t = timestamp
    
    def get_estimate(self,ori,gyro,acce,time_stamp):

        # deduct gyro_bias from every gyro reading
        gyro = gyro - self.gyro_bias  

        # rotate the accelaration, gyroscope readings to the correct orientation using quaternions
        ori_q = quaternion.from_float_array(ori)

        gyro_q = quaternion.from_float_array(np.concatenate([np.zeros([gyro.shape[0], 1]), gyro], axis=1))
        acce_q = quaternion.from_float_array(np.concatenate([np.zeros([acce.shape[0], 1]), acce], axis=1))
        glob_gyro = quaternion.as_float_array(ori_q * gyro_q * ori_q.conj())[:, 1:]
        glob_acce = quaternion.as_float_array(ori_q * acce_q * ori_q.conj())[:, 1:]

        # Input features for the model
        # must have 200 [gyro,acce] rows of data
        features = np.concatenate([glob_gyro, glob_acce], axis=1)[0:]
        # reshape to size 6->rows and 200->columns
        features = features.astype(np.float32).T
        features = torch.from_numpy(features)

        # use these these 3 lines as-it-is for the correct output
        index = torch.tensor([i for i in range(200)])
        y = features.index_select(1, index)
        y = y.resize_((1,6,200))

        # get the velocity prediction in (x,y) directions from the model
        # 200 readings --> 1 velocity estimation
        # vel = self.network(y.to(self.device)).cpu().detach().numpy()

        # output variances also
        output = self.network(y.to(self.device)).cpu().detach().numpy()
        
        if output.shape[1] == 4:
            # 2D model output
            vel = output[:,:2]
            cov = np.exp(output[:,2:])
            vel = np.hstack((vel, np.zeros((vel.shape[0], 1))))
            cov = np.hstack((cov, np.zeros((cov.shape[0], 1))))
        elif output.shape[1] == 6:
            # 3D model output
            vel = output[:,:3]
            cov = np.exp(output[:,3:])

        # calculate the position using velocity, previous position and timestamp
        dt = round(time_stamp-self.pre_t,3)  
        pos = vel*dt + self.pre_pos

        self.pre_pos = pos
        self.pre_t = time_stamp
        
        return pos.reshape(3,), vel.reshape(3,), cov.reshape(3,)
