with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/MPC.py", "r") as f:
    content = f.read()

content = content.replace(
    "def get_control(self, dynamic_obstacles=None) -> Tuple[np.ndarray, float]:",
    "def get_control(self, dynamic_obstacles=None, launch_N=None) -> Tuple[np.ndarray, float]:"
)

content = content.replace(
    "    def _init_problem(self, N: int, xmin: np.ndarray, xmax: np.ndarray, umin: np.ndarray, umax: np.ndarray, safety_margin: float, dynamic_obstacles=None):",
    "    def _init_problem(self, N: int, xmin: np.ndarray, xmax: np.ndarray, umin: np.ndarray, umax: np.ndarray, safety_margin: float, dynamic_obstacles=None, launch_N=None):"
)

old_call = """        t_init_start = time.time()
        self._init_problem(N, self.input_constraints['umin'], self.input_constraints['umax'], self.model.safety_margin, dynamic_obstacles=dynamic_obstacles)
        t_init_end = time.time()"""
new_call = """        t_init_start = time.time()
        self._init_problem(N, self.input_constraints['umin'], self.input_constraints['umax'], self.model.safety_margin, dynamic_obstacles=dynamic_obstacles, launch_N=launch_N)
        t_init_end = time.time()"""
content = content.replace(old_call, new_call)

old_retry_call = """                    self._init_problem(N, self.input_constraints['umin'], self.input_constraints['umax'], relaxed_safety_margin, dynamic_obstacles=dynamic_obstacles)"""
new_retry_call = """                    self._init_problem(N, self.input_constraints['umin'], self.input_constraints['umax'], relaxed_safety_margin, dynamic_obstacles=dynamic_obstacles, launch_N=launch_N)"""
content = content.replace(old_retry_call, new_retry_call)

old_ref_call = """        if self.use_obstacle_avoidance and not self.use_path_constraints_topic:
            ub, lb, _ = self.model.reference_path.update_path_constraints(
                self.model.wp_id + 1,
                [self.model.temporal_state.x, self.model.temporal_state.y, self.model.temporal_state.psi],
                N, self.model.length, self.model.width, safety_margin, dynamic_obstacles=dynamic_obstacles)"""
new_ref_call = """        if self.use_obstacle_avoidance and not self.use_path_constraints_topic:
            ub, lb, _ = self.model.reference_path.update_path_constraints(
                self.model.wp_id + 1,
                [self.model.temporal_state.x, self.model.temporal_state.y, self.model.temporal_state.psi],
                N, self.model.length, self.model.width, safety_margin, dynamic_obstacles=dynamic_obstacles, launch_N=launch_N)"""
content = content.replace(old_ref_call, new_ref_call)


with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/MPC.py", "w") as f:
    f.write(content)
