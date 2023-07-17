import math
import numpy as np
import g2o
from colorist import red


def bundle_adjustment(keyframes, points, K, local_window=None, fixed_points=False, verbose=False, rounds=10,
                      use_robust_kernel=False):
    if local_window is None:
        local_frames = keyframes
    else:
        local_frames = keyframes[-local_window:]

    # create g2o optimizer
    opt = g2o.SparseOptimizer()
    block_solver = g2o.BlockSolverX(g2o.LinearSolverEigenX())
    solver = g2o.OptimizationAlgorithmLevenberg(block_solver)
    opt.set_algorithm(solver)

    thHuberMono = math.sqrt(5.991)  # chi-square 2 DOFS

    graph_keyframes, graph_points = {}, {}

    # add frame vertices to graph
    for kf in (
            local_frames if fixed_points else keyframes):  # if points are fixed then consider just the local frames, otherwise we need all frames or at least two frames for each point
        R = kf.pose[:3, :3]
        t = kf.pose[:3, 3]
        se3 = g2o.SE3Quat(R, t)
        v_se3 = g2o.VertexSE3Expmap()
        v_se3.set_estimate(se3)
        v_se3.set_id(kf.id * 2)  # even ids  (use f.kid here!)
        v_se3.set_fixed(kf.id == 0 or kf not in local_frames)  # (use f.kid here!)
        opt.add_vertex(v_se3)

        # confirm pose correctness
        # est = v_se3.estimate()
        # assert np.allclose(pose[0:3, 0:3], est.rotation().matrix())
        # assert np.allclose(pose[0:3, 3], est.translation())

        graph_keyframes[kf] = v_se3

    num_edges = 0

    # add point vertices to graph
    for (idx, p) in enumerate(points):
        assert (p is not None)
        v_p = g2o.VertexPointXYZ()
        v_p.set_id(idx * 2 + 1)
        v_p.set_estimate(p.pt_3d)
        v_p.set_marginalized(True)
        v_p.set_fixed(fixed_points)
        opt.add_vertex(v_p)
        graph_points[idx] = v_p

        # add edges
        for kf_id in p.kf_observations:
            kf = [frame for frame in keyframes if frame.id == kf_id][0]
            if kf not in graph_keyframes:
                continue
            # print('adding edge between point ', p.id,' and frame ', f.id)
            edge = g2o.EdgeSE3()
            edge.set_vertex(0, v_p)
            edge.set_vertex(1, graph_keyframes[kf])
            edge.set_measurement(kf.kpu[p.kf_observations[kf_id]])
            # invSigma2 = Frame.feature_manager.inv_level_sigmas2[kf.octaves[p.kf_observations[kf_id]]]
            edge.set_information(np.eye(2))
            if use_robust_kernel:
                edge.set_robust_kernel(g2o.RobustKernelHuber(thHuberMono))

            edge.fx = K.A[0][0]
            edge.fy = K.A[1][1]
            edge.cx = K.A[0][2]
            edge.cy = K.A[1][2]

            opt.add_edge(edge)
            num_edges += 1

    if verbose:
        opt.set_verbose(True)
    opt.initialize_optimization()
    opt.optimize(rounds)

    # put frames back
    for kf in graph_keyframes:
        est = graph_keyframes[kf].estimate()
        # R = est.rotation().matrix()
        # t = est.translation()
        # f.update_pose(poseRt(R, t))
        kf.update_pose(g2o.Isometry3d(est.orientation(), est.position()))

    # put points back
    if not fixed_points:
        for p in graph_points:
            p.update_position(np.array(graph_points[p].estimate()))
            p.update_normal_and_depth(force=True)

    mean_squared_error = opt.active_chi2() / max(num_edges, 1)

    return mean_squared_error
