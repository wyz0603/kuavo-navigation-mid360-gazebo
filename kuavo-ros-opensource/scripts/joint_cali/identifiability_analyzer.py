#!/opt/miniconda3/envs/joint_cali/bin/python
# -*- coding: utf-8 -*-

from arm_kinematics import ArmKinematics, get_package_path

import pinocchio as pin
import numpy as np
import os
import nlopt
import time
import matplotlib.pyplot as plt

def create_objective_function(fk, data_dict):
    def objective_function(delta, grad):
        total_error = 0.0
        if grad.size > 0:
            grad[:] = 0.0
        delta = np.array(delta).flatten()
        # true_bias = np.array(true_bias).flatten()

        for q, true_pos, true_rot in zip(data_dict['q'], data_dict['true_pos'], data_dict['true_rot']):
            q = np.array(q).flatten()
            pred_pos, pred_rot, J = fk(q + delta)
            
            error_pos = pred_pos - true_pos
            error_rot = pin.log3(pred_rot @ true_rot.T)
            error = np.concatenate([error_pos, error_rot])
            total_error += np.sum(error**2)

            if grad.size > 0:
                grad += 2 * J.T @ error
        return float(total_error)
    return objective_function



class identifiability_analyzer:
    def __init__(self, fk, q_list):
        self.fk = fk
        self.q_list = q_list

    def get_cond_num_i(self, idx=-1):
        q_list = self.q_list[:idx]
        return self.condition_number(q_list)

    def get_cond_num_all(self, start_idx):
        return [self.get_cond_num_i(i) for i in range(start_idx, len(self.q_list))]

    def draw_cond_num_all(self, start_idx):
        if start_idx >= len(self.q_list):
            print(f"start_idx: {start_idx} is out of range")
            return
        cond_num_list = self.get_cond_num_all(start_idx)
        x = np.arange(start_idx, len(self.q_list))
        plt.plot(x, cond_num_list)
        plt.title('condition num')
        plt.show()
        
    def condition_number(self, q_list):
        # 正规矩阵
        n_vars = 7
        A = np.zeros((n_vars, n_vars))
        for q in q_list:
            q = np.array(q).flatten()
            pred_pos, pred_rot, J = self.fk(q)
            A += J.T @ J
        # 计算条件数
        cond_num = np.linalg.cond(A)
        return cond_num

    def singular_values(self, q_list):
        n_vars = 7
        A = np.zeros((n_vars, n_vars))
        for q in q_list:
            q = np.array(q).flatten()
            pred_pos, pred_rot, J = self.fk(q)
            A += J.T @ J
        return np.linalg.svd(A)

    def get_singular_values_list(self, start_idx=-1):
        if start_idx >= len(self.q_list):
            print(f"start_idx: {start_idx} is out of range")
            return
        singular_values_list = []
        U_list = []
        Vt_list = []
        for i in range(start_idx, len(self.q_list)):
            U, s, Vt = self.singular_values(self.q_list[i:])
            singular_values_list.append(s)
            U_list.append(U)
            Vt_list.append(Vt)
        return singular_values_list, U_list, Vt_list
    
    def locate_hard_to_identify_joints(self):
        s_list, U_list, Vt_list = self.get_singular_values_list()
        s = s_list[0]
        U = U_list[0]
        Vt = Vt_list[0]
        print(f"singular_values: {s}")
        # print(f"U: {U}")
        print(f"Vt: {Vt}")
        max_idx = np.argmax(s)
        print(f"max_idx: {max_idx}")

        for i in range(len(s)):
            ratio = s[i] / s[max_idx]
            if ratio < 1e-3:
                print(f"idx {i} is hard to identify")
                Vt_i = Vt[i, :]
                print(f"Vt_i: {Vt_i}")
        sensitivity_ratios = self.parameter_sensitivity_ratio(s, Vt, 1e-12)
        # print(f"sensitivity_ratios: {sensitivity_ratios}")
        print(f"灵敏度: {sensitivity_ratios/max(sensitivity_ratios)}")

    @staticmethod
    def parameter_sensitivity_ratio(s, Vh, epsilon=1e-12):
        V = Vh.T.conj()  # 转换为右奇异向量的列向量 (实数矩阵可简化为 V = Vh.T)
        N = V.shape[0]   # 关节数
        
        sensitivity_ratios = np.zeros(N)
        
        for i in range(N):
            sum_val = 0.0
            for k in range(N):
                v_ki = V[i, k]          # 第k个右奇异向量的第i个分量
                sigma_k = s[k]          # 第k个奇异值（降序排列）
                if sigma_k < epsilon:   # 忽略过小的奇异值
                    continue
                sum_val += (v_ki / (sigma_k + epsilon))** 2
            sensitivity_ratios[i] = np.sqrt(sum_val)
        return sensitivity_ratios

    def draw_singular_values(self, start_idx):
        s_list, U_list, Vt_list = self.get_singular_values_list(start_idx)
        s = np.asarray(s_list)
        # print(f"singular_values.type: {type(s)}")
        # print(f"singular_values.shape: {s.shape}")
        # print(f"singular_values: {s}")
        x = np.arange(len(s))
        plt.plot(x, s[:,0], label='q1')
        plt.plot(x, s[:,1], label='q2')
        plt.plot(x, s[:,2], label='q3')
        plt.plot(x, s[:,3], label='q4')
        plt.plot(x, s[:,4], label='q5')
        plt.plot(x, s[:,5], label='q6')
        plt.plot(x, s[:,6], label='q7')
        plt.title("singular values of q")
        plt.legend()
        plt.show()
        plt.clf()

def main():
    asset_path = get_package_path("kuavo_assets")
    # print(f"asset_path: {asset_path}")
    robot_version = os.environ.get('ROBOT_VERSION', '40')
    urdf_path = os.path.join(asset_path, f"models/biped_s{robot_version}/urdf/biped_s{robot_version}.urdf")
    print(f"urdf_path: {urdf_path}")
    T_et = np.eye(4)
    R_mod = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=np.float32)
    T_et[:3, :3] = R_mod
    T_et[:3, 3] = np.array([0.15, 0.0, 0.0])
    arm_kinematics = ArmKinematics(urdf_path, T_et)

    n_poses = 50
    start_idx = int(0.1*n_poses)
    n_joints = 14
    np.random.seed(0)
    ths = np.array([90, 90, 90, 90, 90, 90, 40])
    ths = np.deg2rad(ths)
    q_list = []
    for i in range(n_poses):
        q_single = np.zeros(7)
        for j in range(7):
            q_single[j] = np.random.uniform(-ths[j], ths[j])
        q_list.append(np.concatenate([q_single, q_single]))
    
    true_bias = np.array([0.01, -0.02, 0.03, 0.04, 0.05, -0.06, 0.07, 
                          0.01, -0.02, 0.03, 0.04, 0.05, -0.06, 0.07])
    data_dict = {}
    n_vars = 7
    for i in range(1):
        data_dict['q'] = [q[i*n_vars:(i+1)*n_vars] for q in q_list]
        mean = 0
        std = 0.01
        measure_noise = np.random.normal(mean, std, 3)
        # print(f"measure_noise: {measure_noise}")
        
        fk = arm_kinematics.FK_l if i == 0 else arm_kinematics.FK_r
        id_analyzer = identifiability_analyzer(fk, data_dict['q'])
        id_analyzer.draw_cond_num_all(start_idx=start_idx)
        id_analyzer.locate_hard_to_identify_joints()
        # print(f"singular_values: {s}")
        # id_analyzer.draw_singular_values(start_idx=start_idx)
        print(f"cond_num: {id_analyzer.get_cond_num_i()}")
        data_dict['true_pos'] = [fk((q + true_bias)[i*n_vars:(i+1)*n_vars])[0] + np.random.normal(0.0, 0.001, 3) for q in q_list]
        data_dict['true_rot'] = [fk((q + true_bias)[i*n_vars:(i+1)*n_vars])[1] for q in q_list]
        # # opt = nlopt.opt(nlopt.LD_SLSQP, n_vars)
        # # opt = nlopt.opt(nlopt.LD_LBFGS, n_vars)
        # # opt = nlopt.opt(nlopt.LD_LBFGS, n_vars)
        # # opt = nlopt.opt(nlopt.GN_ISRES, n_vars)
        opt = nlopt.opt(nlopt.LD_MMA, n_vars)
        opt.set_lower_bounds([-np.deg2rad(15)]*n_vars)
        opt.set_upper_bounds([+np.deg2rad(15)]*n_vars)
        
        
        objective = create_objective_function(fk, data_dict)
        opt.set_min_objective(objective)
        
        opt.set_ftol_rel(1e-8)
        opt.set_maxeval(1000)
        
        initial_delta = np.zeros(n_vars)
        
        try:
            time_start = time.time()
            result = opt.optimize(initial_delta)
            time_end = time.time()
            print("最终误差：", opt.last_optimum_value())
            print("找到的偏差：", result)
            # print("找到的偏差与真实偏差的模：", np.linalg.norm(result - true_bias[i*n_vars:(i+1)*n_vars]))
            print(f"优化时间：{1e3*(time_end - time_start):.2f} 毫秒")
        except Exception as e:
            print("优化错误：", e)

if __name__ == "__main__":
    main()
