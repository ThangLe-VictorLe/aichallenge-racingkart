from typing import Tuple
import numpy as np
import osqp
from scipy import sparse
import matplotlib.pyplot as plt

# Colors
PREDICTION = '#BA4A00'

##################
# MPC Controller #
##################

class MPC:
    def __init__(self, model, N, Q, R, QN, StateConstraints, InputConstraints,
                 ay_max, max_steering_rate, wp_id_offset, use_obstacle_avoidance, use_path_constraints_topic, use_max_kappa_pred=True):
        """
        Constructor for the Model Predictive Controller.
        :param model: bicycle model object to be controlled
        :param N: time horizon | int
        :param Q: state cost matrix
        :param R: input cost matrix
        :param QN: final state cost matrix
        :param StateConstraints: dictionary of state constraints
        :param InputConstraints: dictionary of input constraints
        :param ay_max: maximum allowed lateral acceleration in curves
        :param wp_id_offset: offset for waypoint id to consider control delay
        :param use_obstacle_avoidance: flag to enable obstacle avoidance
        :param use_path_constraints_topic: flag to use path constraints from topic
        :param max_steering_rate: maximum allowed steering rate in rad/s
        """
        # 既存の初期化パラメータ
        self.N = N
        self.Q = Q
        self.R = R
        self.QN = QN
        self.wp_id_offset = wp_id_offset
        self.use_obstacle_avoidance = use_obstacle_avoidance
        self.use_path_constraints_topic = use_path_constraints_topic
        self.model = model
        self.nx = self.model.n_states
        self.nu = 2
        self.state_constraints = StateConstraints
        self.input_constraints = InputConstraints
        self.ay_max = ay_max

        # 追加: ステアリングレート制限関連のパラメータ
        self.max_steering_rate = max_steering_rate
        self.previous_steering = 0.0  # 前回のステア角

        # 追加: ay_maxによる速度制限の方式切り替え
        self.use_max_kappa_pred = use_max_kappa_pred
        # 既存の初期化
        self.current_prediction = None
        self.infeasibility_counter = 0
        self.last_solved_wp_id = 0
        self.current_control = np.zeros((self.nu*self.N))
        self.current_ub = None
        self.current_lb = None
        self.optimizer = osqp.OSQP()

        if not self.use_obstacle_avoidance:
            self.model.reference_path.update_simple_path_constraints(
                N,
                self.model.safety_margin)

        self._precompute_static_matrices()

    def _precompute_P(self):
        from scipy import sparse
        self._P_cached = sparse.block_diag([
            sparse.kron(sparse.eye(self.N), self.Q),
            self.QN,
            sparse.kron(sparse.eye(self.N), self.R)
        ], format='csc')

    def _precompute_static_matrices(self):
        from scipy import sparse
        N = self.N
        
        # Precompute P
        self._precompute_P()
        
        # Precompute Ax static base
        self._Ax_static_cached = sparse.kron(sparse.eye(N + 1), -sparse.eye(self.nx))
        
        # Precompute Zero blocks for equality constraints
        self._Z_top_A = sparse.csc_matrix((self.nx, N * self.nx))
        self._Z_right_A = sparse.csc_matrix(((N + 1) * self.nx, self.nx))
        self._Z_top_B = sparse.csc_matrix((self.nx, N * self.nu))
        
        # Precompute steering rate matrix & A_inequality
        nx_N = self.nx * (N + 1)
        nu_N = self.nu * N
        n_rate_constraints = N - 1
        steering_rate_matrix = np.zeros((n_rate_constraints, nx_N + nu_N))
        for i in range(n_rate_constraints):
            steering_rate_matrix[i, nx_N + self.nu*i + 1] = -1
            steering_rate_matrix[i, nx_N + self.nu*(i+1) + 1] = 1
            
        self._A_inequality_cached = sparse.vstack([
            sparse.eye(nx_N + nu_N),
            sparse.csc_matrix(steering_rate_matrix)
        ])

    def update_v_max(self, v_max: float):
        self.input_constraints['umax'][0] = v_max

    def update_ay_max(self, ay_max: float):
        self.ay_max = ay_max

    def update_wp_id_offset(self, wp_id_offset: int):
        self.wp_id_offset = wp_id_offset

    def update_Q(self, Q: np.ndarray):
        self.Q = Q
        if hasattr(self, '_P_cached'): self._precompute_P()

    def update_R(self, R: np.ndarray):
        self.R = R
        if hasattr(self, '_P_cached'): self._precompute_P()

    def update_QN(self, QN: np.ndarray):
        self.QN = QN
        if hasattr(self, '_P_cached'): self._precompute_P()

    def update_max_steering_rate(self, max_steering_rate: float):
        self.max_steering_rate = max_steering_rate


    def _init_problem(self, N, umin, umax, safety_margin, dynamic_obstacles=None, launch_N=None, lateral_offset=0.0, nearest_vid=None):
        import time
        t_start = time.time()
        
        """
        Initialize optimization problem for current time step with steering rate constraints.
        """
        # 既存の制約設定
        xmin = self.state_constraints['xmin']
        xmax = self.state_constraints['xmax']

        # Precompute common terms
        nx_N = self.nx * (N + 1)
        nu_N = self.nu * N

        # Reference vector
        ur = np.zeros(nu_N)
        xr = np.zeros(nx_N)
        uq = np.zeros(N * self.nx)

        # Collect values for vectorized linearize
        v_ref_list = []
        kappa_ref_list = []
        delta_s_list = []

        # Predict realistic velocity profile from current state
        sim_v = max(self.current_control[0], 1.0)
        
        for n in range(N):
            current_waypoint = self.model.reference_path.get_waypoint(self.model.wp_id + n)
            next_waypoint = self.model.reference_path.get_waypoint(self.model.wp_id + n + 1)
            delta_s = next_waypoint - current_waypoint
            
            # Kart accelerates at roughly 3.0 m/s^2.
            # dv = a * dt = a * (ds / v)
            sim_v = min(sim_v + 3.0 * (delta_s / sim_v), current_waypoint.v_ref)
            
            v_ref_list.append(np.clip(sim_v, umin[0], umax[0]))
            kappa_ref_list.append(current_waypoint.kappa)
            delta_s_list.append(delta_s)
        
        f_arr, A_arr, B_arr = self.model.linearize_vectorized(
            np.array(v_ref_list), np.array(kappa_ref_list), np.array(delta_s_list)
        )

        # Dynamic constraints
        xmin_dyn = np.kron(np.ones(N + 1), xmin)
        xmax_dyn = np.kron(np.ones(N + 1), xmax)
        umax_dyn = np.kron(np.ones(N), umax)

        # Get curvature predictions
        kappa_pred = np.tan(np.append(np.array(self.current_control[3::self.nu]), self.current_control[-1])) / self.model.length

        # Consider control delay
        offset_wp_id = self.model.wp_id + self.wp_id_offset

        # Iterate over horizon
        for n in range(N):
            # Set reference
            ur[n*self.nu:(n+1)*self.nu] = [v_ref_list[n], kappa_ref_list[n]]
            uq[n * self.nx:(n+1)*self.nx] = B_arr[n].dot([v_ref_list[n], kappa_ref_list[n]]) - f_arr[n]

            # Constrain maximum speed based on curvature
            if self.use_max_kappa_pred:
                max_kappa_pred = np.max(np.abs(kappa_pred[n:]))
                vmax_dyn = np.sqrt(self.ay_max / (np.abs(max_kappa_pred) + 1e-12))
            else:
                vmax_dyn = np.sqrt(self.ay_max / (np.abs(kappa_pred[n]) + 1e-12))
            umax_dyn[self.nu*n] = min(vmax_dyn, umax_dyn[self.nu*n])

        if self.use_obstacle_avoidance and not self.use_path_constraints_topic:
            ub, lb, _ = self.model.reference_path.update_path_constraints(
                offset_wp_id + 1,
                [self.model.temporal_state.x, self.model.temporal_state.y, self.model.temporal_state.psi],
                N, self.model.length, self.model.width, safety_margin, dynamic_obstacles=dynamic_obstacles, launch_N=launch_N, nearest_vid=nearest_vid)
        else:
            ref_wp_id = (offset_wp_id + 1) % len(self.model.reference_path.path_constraints[0])
            ub = self.model.reference_path.path_constraints[0][ref_wp_id]
            lb = self.model.reference_path.path_constraints[1][ref_wp_id]
            self.model.reference_path.border_cells.current_wp_id = ref_wp_id

            # Update safety margin if provided as argument and different from current value
            if self.model.safety_margin != safety_margin:
                safety_margin_diff = safety_margin - self.model.safety_margin
                ub -= safety_margin_diff
                lb += safety_margin_diff

                infeasible_index = ub < lb
                ub[infeasible_index] = 0.0
                lb[infeasible_index] = 0.0

        self.current_ub = ub
        self.current_lb = lb
        t_path_end = time.time()

        # Update dynamic state constraints
        xmin_dyn[0] = xmax_dyn[0] = self.model.spatial_state.e_y
        
        # FUNNEL LOGIC: Ensure bounds are physically reachable from current position
        # Maximum lateral shift per prediction step (tuning parameter)
        max_dy_per_step = 0.5 
        reachable_lb = self.model.spatial_state.e_y
        reachable_ub = self.model.spatial_state.e_y
        for n in range(N):
            reachable_lb -= max_dy_per_step
            reachable_ub += max_dy_per_step
            # Relax the bounds if they are tighter than what the kart can physically reach
            lb[n] = min(lb[n], reachable_ub)
            ub[n] = max(ub[n], reachable_lb)
            
        xmin_dyn[self.nx::self.nx] = lb
        xmax_dyn[self.nx::self.nx] = ub
        
        target_ey = ((lb + ub) / 2.0) + lateral_offset
        target_ey = np.clip(target_ey, lb, ub)
        xr[self.nx::self.nx] = target_ey

        # Get equality matrix
        A_blocks = [A_arr[n] for n in range(N)]
        B_blocks = [B_arr[n] for n in range(N)]
        
        A_block_diag = sparse.block_diag(A_blocks)
        A_left = sparse.vstack([self._Z_top_A, A_block_diag])
        A_sparse = sparse.hstack([A_left, self._Z_right_A])

        B_block_diag = sparse.block_diag(B_blocks)
        B_sparse = sparse.vstack([self._Z_top_B, B_block_diag])

        Ax = self._Ax_static_cached + A_sparse
        Bu = B_sparse
        Aeq = sparse.hstack([Ax, Bu])

        # 完全な制約行列
        A_full = sparse.vstack([Aeq, self._A_inequality_cached], format='csc')

        # 境界制約の構築
        x0 = np.array(self.model.spatial_state[:])
        leq = np.hstack([-x0, uq])
        ueq = leq

        # 入力と状態の制約境界
        lineq_basic = np.hstack([xmin_dyn, np.kron(np.ones(N), umin)])
        uineq_basic = np.hstack([xmax_dyn, umax_dyn])

        # ステアリングレート制約の境界
        n_rate_constraints = N - 1
        lineq_rate = np.zeros(n_rate_constraints)
        uineq_rate = np.zeros(n_rate_constraints)
        for i in range(n_rate_constraints):
            v_i = max(v_ref_list[i], 1.0)
            dt_i = delta_s_list[i] / v_i
            max_delta_change = self.max_steering_rate * dt_i
            lineq_rate[i] = -max_delta_change
            uineq_rate[i] = max_delta_change

        # 全ての境界を結合
        l = np.hstack([leq, lineq_basic, lineq_rate])
        u = np.hstack([ueq, uineq_basic, uineq_rate])

        len_eq = len(leq)
        l_ey = l[len_eq + self.nx : len_eq + self.nx*(N+1) : self.nx]
        u_ey = u[len_eq + self.nx : len_eq + self.nx*(N+1) : self.nx]

        # コスト行列
        P = self._P_cached

        q = np.hstack([
            -np.tile(np.diag(self.Q.toarray()), N) * xr[:-self.nx],
            -self.QN.dot(xr[-self.nx:]),
            -np.tile(np.diag(self.R.toarray()), N) * ur
        ])

        # オプティマイザの設定
        t_matrix_end = time.time()
        self.optimizer = osqp.OSQP()
        self.optimizer.setup(P=P, q=q, A=A_full, l=l, u=u, verbose=False)
        t_setup_end = time.time()
        return P, q, A_full, l, u

    def get_control(self, dynamic_obstacles=None, launch_N=None, lateral_offset=0.0, nearest_vid=None) -> Tuple[np.ndarray, float]:
        """
        Get control signal given the current position of the car.
        """
        nx = self.nx
        nu = self.nu
        import time
        t_start = time.time()

        self.model.get_current_waypoint()

        N = min(self.N, self.model.reference_path.n_waypoints - self.model.wp_id) \
            if not self.model.reference_path.circular else self.N

        t_t2s_start = time.time()
        self.model.spatial_state = self.model.t2s(
            reference_state=self.model.temporal_state,
            reference_waypoint=self.model.current_waypoint)
        t_t2s_end = time.time()

        t_init_start = time.time()
        P, q, A, l, u = self._init_problem(N, self.input_constraints['umin'], self.input_constraints['umax'], self.model.safety_margin, dynamic_obstacles=dynamic_obstacles, launch_N=launch_N, lateral_offset=lateral_offset, nearest_vid=nearest_vid)
        t_init_end = time.time()



        try:
            t_solve_start = time.time()
            dec = self.optimizer.solve()
            t_solve_end = time.time()

            control_signals = np.array(dec.x[-N*nu:])
            use_control_signals = control_signals[1::2]

            if not np.all(use_control_signals):
                for i in range(1, 6):
                    relaxed_safety_margin = self.model.safety_margin * ((5-i) / 5.0)
                    P, q, A, l, u = self._init_problem(N, self.input_constraints['umin'], self.input_constraints['umax'], relaxed_safety_margin, dynamic_obstacles=dynamic_obstacles, launch_N=launch_N, lateral_offset=lateral_offset, nearest_vid=nearest_vid)
                    dec = self.optimizer.solve()
                    control_signals = np.array(dec.x[-N*nu:])
                    use_control_signals = control_signals[1::2]

                    if self.infeasibility_counter == 0 and np.all(use_control_signals):
                        break

            # ステア角の計算と保存
            control_signals[1::2] = np.arctan(control_signals[1::2] * self.model.length)
            v = control_signals[0]
            delta = control_signals[1]

            # ステアレートの制限を適用
            max_delta_change = self.max_steering_rate * self.model.Ts
            delta = np.clip(
                delta,
                self.previous_steering - max_delta_change,
                self.previous_steering + max_delta_change
            )
            self.previous_steering = delta

            # 予測の更新
            self.current_control = control_signals
            x = np.reshape(dec.x[:(N+1)*nx], (N+1, nx))
            self.current_prediction_raw = x
            self.current_prediction = self.update_prediction(x, N)

            u = np.array([v, delta])
            max_delta = np.max(np.abs(control_signals[1:len(control_signals)//3*2:2]))

            self.infeasibility_counter = 0
            self.last_solved_wp_id = self.model.wp_id

        except TypeError or ValueError:
            id = nu * (self.infeasibility_counter + 1)
            if id + 2 < len(self.current_control):
                u = np.array(self.current_control[id:id+2])
                max_delta = np.abs(u[1])
            else:
                u = np.array([0.0, 0.0])
                max_delta = 0.0

            self.infeasibility_counter += 1

        if self.infeasibility_counter > (N - 1) and self.infeasibility_counter % 100 == 0:
            print('No control signal computed!')

        return u, max_delta

    def update_prediction(self, spatial_state_prediction, N):
        """
        Transform the predicted states to predicted x and y coordinates.
        Mainly for visualization purposes.
        :param spatial_state_prediction: list of predicted state variables
        :return: lists of predicted x and y coordinates
        """

        # Containers for x and y coordinates of predicted states
        x_pred, y_pred = [], []

        # Iterate over prediction horizon starting from n=0 (ego position)
        # so prediction dots always begin at the kart, not 2 waypoints ahead.
        for n in range(0, N):
            # Get associated waypoint
            associated_waypoint = self.model.reference_path.\
                get_waypoint(self.model.wp_id+n)
            # Transform predicted spatial state to temporal state
            predicted_temporal_state = self.model.s2t(associated_waypoint,
                                            spatial_state_prediction[n, :])

            # Save predicted coordinates in world coordinate frame
            x_pred.append(predicted_temporal_state.x)
            y_pred.append(predicted_temporal_state.y)

        return x_pred, y_pred

    def show_prediction(self, ax):
        """
        Display predicted car trajectory on the provided axis.
        :param ax: Matplotlib axis object to plot on
        """

        if self.current_prediction is not None:
            # ax.scatter(self.current_prediction[0], self.current_prediction[1],
            #            c=PREDICTION, s=5)
            ax.plot(self.current_prediction[0], self.current_prediction[1], c=PREDICTION)
