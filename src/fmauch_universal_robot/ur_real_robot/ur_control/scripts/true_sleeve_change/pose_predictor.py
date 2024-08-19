import torch
import numpy as np
from model import LSTMModel
import transforms3d

class MyModelPredictor:
    def __init__(self, model_path, mean_path, std_path):
    # def __init__(self, model_path):  
        # 加载模型
        self.model = LSTMModel(14, 100, 2, 7)
        self.model.load_state_dict(torch.load(model_path))
        self.model.eval()

        # 加载归一化参数
        mean = np.load(mean_path)
        std = np.load(std_path)
        self.mean_input, self.mean_output = mean[:-7], mean[-7:]
        self.std_input, self.std_output = std[:-7], std[-7:]
        print("model is loaded")

    def combine_poses(self, robot_pose, end_effector_pose):
        # 转换四元数从 xyzw 到 wxyz
        robot_position, robot_orientation_xyzw = robot_pose[:3], robot_pose[3:]
        robot_orientation = np.roll(robot_orientation_xyzw, shift=1)

        ee_position, ee_orientation_xyzw = end_effector_pose[:3], end_effector_pose[3:]
        ee_orientation = np.roll(ee_orientation_xyzw, shift=1)

        # 将四元数转换为旋转矩阵
        robot_rotation_matrix = transforms3d.quaternions.quat2mat(robot_orientation)
        ee_rotation_matrix = transforms3d.quaternions.quat2mat(ee_orientation)

        # 计算世界坐标系下的末端执行器旋转矩阵
        combined_rotation_matrix = np.dot(robot_rotation_matrix, ee_rotation_matrix)

        # 计算世界坐标系下的末端执行器位置
        combined_position = robot_position + np.dot(robot_rotation_matrix, ee_position)

        # 将组合的旋转矩阵转换回四元数（wxyz）
        combined_orientation_wxyz = transforms3d.quaternions.mat2quat(combined_rotation_matrix)

        # 转换四元数从 wxyz 到 xyzw
        combined_orientation = np.roll(combined_orientation_wxyz, shift=-1)

        return np.concatenate([combined_position, combined_orientation])
    
    def relative_transform(self,pose2, pose1):
        # 解包世界坐标系下的末端执行器位姿，并将四元数从 xyzw 转换为 wxyz
        world_ee_position, world_ee_orientation = pose2[:3], pose2[3:]
        world_ee_orientation = np.roll(world_ee_orientation, shift=1)

        # 解包机器人在世界坐标系下的位姿，并将四元数从 xyzw 转换为 wxyz
        robot_position, robot_orientation = pose1[:3], pose1[3:]
        robot_orientation = np.roll(robot_orientation, shift=1)

        # 将四元数转换为旋转矩阵
        robot_rotation_matrix = transforms3d.quaternions.quat2mat(robot_orientation)
        world_ee_rotation_matrix = transforms3d.quaternions.quat2mat(world_ee_orientation)

        # 计算相对于机器人的末端执行器旋转矩阵
        robot_rotation_matrix_inv = np.linalg.inv(robot_rotation_matrix)
        ee_relative_rotation_matrix = np.dot(robot_rotation_matrix_inv, world_ee_rotation_matrix)

        # 计算相对于机器人的末端执行器位置
        ee_relative_position = np.dot(robot_rotation_matrix_inv, world_ee_position - robot_position)

        # 将相对旋转矩阵转换回四元数，并将四元数从 wxyz 转换为 xyzw
        ee_relative_orientation = transforms3d.quaternions.mat2quat(ee_relative_rotation_matrix)
        ee_relative_orientation = np.roll(ee_relative_orientation, shift=-1)

        return np.concatenate([ee_relative_position, ee_relative_orientation])

    def preprocess(self, data):
        timestamps = data[:, 0] - data[0, 0]
        timestamps = np.expand_dims(timestamps, axis=1)  # 重新调整形状为 (n_steps, 1)

        # 工具位姿处理
        tool_poses = data[:, 1:8]
        relative_poses = np.array([self.relative_transform(pose, tool_poses[-1]) for pose in tool_poses])
        # 力保持不变
        forces = data[:, 8:14]
        # 组合新的输入数据
        data_processed = np.hstack((timestamps, relative_poses, forces))

        data_flattened = data_processed.reshape(-1)

        data_normalized = (data_flattened - self.mean_input) / self.std_input
        # data_normalized = data_flattened

        return data_normalized.reshape(data.shape)

    def postprocess(self, data):
        return data * self.std_output + self.mean_output
        # return data
    

    def predict(self, input_data):
        input_data_normalized = self.preprocess(input_data)
        input_tensor = torch.tensor(input_data_normalized, dtype=torch.float32).unsqueeze(0)
        print('predict finished')

        with torch.no_grad():
            output_tensor = self.model(input_tensor)

        output_data_normalized = output_tensor.squeeze(0).numpy()
        output_data = self.postprocess(output_data_normalized)
        return output_data

if __name__ == '__main__':
    # 使用示例
    model_predictor = MyModelPredictor('lstm_model.pth', 'data_mean.npy', 'data_std.npy')
    input_data = np.array([
        [1704788671.2319071,0.2275933102738367,1.0035500316952775,0.07236056170836991,0.7121635621237326,-0.7020074679518247,-0.002827137348297973,0.0007635558382040826,35.67690177402725,-15.734819019703192,-4.32374989166141,3.0875489966090988,11.861159000623708,0.11607465779141318],
        [1704788671.430529,0.227048189637417,1.0024059003118277,0.07416832174667398,0.7094017330117888,-0.7047484001501388,-0.007913858237906024,0.004030450778858896,-1.6043642258158588,-11.769966817560668,-3.6963176481075455,3.530524847427528,-0.4574123006941424,-0.23291136525883313],
        [1704788671.6323469,0.2286747227768049,1.00165138306132,0.07523199718488283,0.7085062414819849,-0.705643729565001,-0.008341813386078865,0.004030738902380481,1.1718644527348852,5.146024016773949,-2.5224448745923875,-0.8619881385505875,-0.5034642978098978,-0.23674171640150388],
        [1704788672.0405433,0.2293195191627219,1.0009142345364264,0.07424039972783714,0.7063417094980304,-0.7078558521797544,0.00031745572348981603,0.004623978821057737,0.7130575181679086,-0.3185316745133884,0.5101544119701353,-0.5108151058135728,-1.22086554392079,-0.2513951929372239],
        [1704788672.1320727,0.2293013834751652,1.0018914057939252,0.07450810421749265,-0.703014068422761,0.711150710511768,-0.005471582460865282,0.0024389185429443882,5.52974104302909,6.85642660356188,-1.3594982525504704,-1.6265703274170131,0.628572757516431,-0.2542592127760082],
        [1704788672.4401107,0.22841874159941483,1.0031905299642576,0.07434134292460104,-0.7035917661872803,0.7105607134076548,-0.0032618645626885396,0.007173517592011457,22.001012926385048,-12.007927633332319,-4.3765256228580975,2.6755393002805574,6.351216366780921,0.0874807213015884],
        [1704788672.5361626,0.2274850055125648,1.0037120819313325,0.07320530115502,-0.7058442967638647,0.7083191474517121,0.0013087987766742454,0.008130259833931288,29.20774224037956,-18.505358535069263,-5.482844865503764,5.124231510914729,9.630562642282781,0.030020906809828607],
        [1704788672.840308,0.2278990877638848,1.0024532120925187,0.0750574299098441,-0.7063505235680251,0.7078524760565753,-0.003012325217894893,0.002176210590480641,-6.262268392020993,0.2125949358291095,-2.3303635395759867,0.6964035855918748,-1.328824633391171,-0.10642962266432653],
        [1704788673.021792,0.22873987574299817,1.001161581772042,0.07580789403225341,0.7063984235016145,-0.7077944458838381,0.005311138454791563,0.00028540977205547335,-3.166038797082709,2.0380318954169288,-6.146792589865248,-0.015515960193313028,-2.0997876994564715,-0.11194969121270096],
        [1704788673.2324293,0.22879091274162322,1.00245797081475,0.07538583783788888,0.7033444936418941,-0.7107759138735014,0.0034732180306723116,0.009594804710246393,11.548663087746544,2.5400888023717823,-11.074565008656414,-0.9481759584719801,3.030886901014658,-0.24288207062778455]
    ])  # 用您的输入数据替换这里
    tool_pose=np.array([0.22879091274162322,1.00245797081475,0.07538583783788888,0.7033444936418941,-0.7107759138735014,0.0034732180306723116,0.009594804710246393])
    screw_pose = np.array([0.22887, 1.00167, 0.07144, 0, 0, 0, 1])
    prediction = model_predictor.predict(input_data)
    pred_pose = model_predictor.combine_poses(tool_pose, prediction)
    print(pred_pose)