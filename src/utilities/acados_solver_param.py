from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from math import atan2
import numpy as np
from casadi import SX, vertcat, dot, sqrt

def acados_solver_model(N_players,robot_locations,x_goals,theta_0):
    "Specify the Initial Condition"
    x0=vertcat(*[
            val
            for i in range(N_players)
            for val in (robot_locations[i][0], robot_locations[i][1], theta_0[i])
        ])
    # print(f"x0: {x0}")
    "Initialize the solver model"
    model_name="multi_agent_setting"
    ocp=AcadosOcp()
    ocp.model.name=model_name
    x, y, theta, v, omega = [], [], [], [], []
    states_list, controls_list, xdot = [], [], []
    for i in range(N_players):
        x_i = SX.sym(f'x_{i}')
        y_i = SX.sym(f'y_{i}')
        theta_i = SX.sym(f'theta_{i}')
        v_i = SX.sym(f'v_{i}')
        omega_i = SX.sym(f'omega_{i}')
        states_list += [x_i, y_i, theta_i]
        controls_list += [v_i, omega_i]
        xdot += [v_i * SX.cos(theta_i), v_i * SX.sin(theta_i), omega_i]
    states=vertcat(*states_list)
    controls=vertcat(*controls_list)
    f_expl = vertcat(*xdot)
    ocp.model.x=states
    ocp.model.u=controls
    ocp.model.f_expl_expr=f_expl
    ocp.constraints.x0=np.array(x0).astype(np.float64).flatten()
    params_list=[]
    for i in range(N_players):
        p_x_i = SX.sym(f'p_x_{i}')
        p_y_i = SX.sym(f'p_y_{i}')
        params_list += [p_x_i, p_y_i]
    lambda_par=SX.sym('lambda_par')
    params = vertcat(*params_list,lambda_par)
    ocp.model.p = params
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'
    o_radius = 0.35                                         #1 m for Jackal
    beta_radius = 1.0
    epsilon = 1e-6  # small positive to avoid division by zero
    collision_constraints = []
    for i in range(N_players):
        xi_k = ocp.model.x[3*i:3*i+2]
        for j in range(N_players):
            if j == i:
                continue
            xj_k = ocp.model.x[3*j:3*j+2]
            homotope_i = ocp.model.p[2*i:2*i+2]
            diff = xj_k - homotope_i
            norm_diff = sqrt(dot(diff, diff) + epsilon)
            A_ij = diff / norm_diff
            b_ij = dot(A_ij, xj_k) - beta_radius * o_radius
            constraint = dot(A_ij, xi_k) - b_ij
            collision_constraints.append(constraint)
    con_h = vertcat(*collision_constraints)
    num_collision_constraints = len(collision_constraints)
    s = SX.sym('s', num_collision_constraints)  # slack variables
    ocp.model.u = vertcat(controls, s)
    ocp.model.con_h_expr = con_h - 0.00* s
    Tf, N = 5, 50
    ocp.solver_options.tf = Tf
    ocp.solver_options.N_horizon = N
    lambda_x, lambda_y = 100, 100
    lambda_u, lambda_omega =1, 40
    lambda_slack = 0 # high penalty on slack
    stage_cost, terminal_cost = 0, 0
    for i in range(N_players):
        x_idx = 3 * i
        y_idx = 3 * i + 1
        v_idx = 2 * i
        omega_idx = 2 * i + 1
        ref_x = ocp.model.p[2 * i]
        ref_y = ocp.model.p[2 * i + 1]
        pos_error_x = ocp.model.x[x_idx] - ref_x
        pos_error_y = ocp.model.x[y_idx] - ref_y
        stage_cost += lambda_x * pos_error_x**2 + lambda_y * pos_error_y**2
        stage_cost += lambda_u * ocp.model.u[v_idx]**2 + lambda_omega * ocp.model.u[omega_idx]**2
        terminal_cost += lambda_x * pos_error_x**2 + lambda_y * pos_error_y**2
    stage_cost += lambda_slack * dot(s, s)
    stage_cost += lambda_par*10
    ocp.constraints.lh = -1e9 * np.ones(num_collision_constraints)
    ocp.constraints.uh = np.zeros(num_collision_constraints)
    nu = 2 * N_players  # number of original controls (v, omega) per player
    ocp.constraints.idxbu = np.arange(nu)
    v_min, v_max = 0.0, 0.22                                                # 1.5 m/s for Jackal
    omega_min, omega_max = -0.5, 0.5
    ocp.constraints.lbu = np.array([v_min, omega_min] * N_players)
    ocp.constraints.ubu = np.array([v_max, omega_max] * N_players)
    ocp.model.cost_expr_ext_cost = stage_cost
    ocp.model.cost_expr_ext_cost_e = terminal_cost
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.print_level = 0
    ocp.solver_options.nlp_solver_max_iter = 2
    ocp.solver_options.qp_solver_iter_max = 2
    ocp.solver_options.qp_solver_warm_start = 1
    ocp.solver_options.nlp_solver_warm_start = 1
    ocp.solver_options.hessian_approx = 'EXACT'
    tol_limit = 5E-6
    ocp.solver_options.nlp_solver_tol_stat = tol_limit
    ocp.solver_options.nlp_solver_tol_eq = tol_limit
    ocp.solver_options.nlp_solver_tol_ineq = tol_limit
    ocp.solver_options.nlp_solver_tol_comp = tol_limit
    ocp.solver_options.qp_solver_tol_stat = tol_limit
    ocp.solver_options.qp_solver_tol_eq = tol_limit
    ocp.solver_options.qp_solver_tol_ineq = tol_limit
    ocp.solver_options.qp_solver_tol_comp = tol_limit
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps =4
    ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
    ocp.solver_options.regularize_method = "GERSHGORIN_LEVENBERG_MARQUARDT"
    ocp.solver_options.reg_perturb = 1E-6
    ocp.solver_options.with_adaptive_levenberg_marquardt = True
    ocp.solver_options.adaptive_levenberg_marquardt_mu0 = 1e-6
    ocp.solver_options.adaptive_levenberg_marquardt_mu_min = 1e-6
    ocp.solver_options.adaptive_levenberg_marquardt_lam = 1.01
    ocp.solver_options.qp_regularization = 1e-6
    ocp.solver_options.qp_solver_ric_alg = 1
    ocp.parameter_values = np.array([2] * (2 * N_players) + [10])   # 2 per player, last value is lambda_par
    nx=ocp.model.x.shape
    nu=ocp.model.u.shape
    return ocp, Tf, N, o_radius,ocp.constraints.x0, nx, nu