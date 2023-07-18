import math
import numpy as np
import sys
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
            edge = g2o.EdgeSE3ProjectXYZ()
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
        rawpose = g2o.Isometry3d(est.orientation(), est.position())
        newpose = np.eye(4)
        newpose[:3, :3] = rawpose.R
        newpose[:3, 3] = rawpose.t
        kf.set_pose(newpose)

    # put points back
    if not fixed_points:
        for p in graph_points:
            points[p].pt_3d = np.array(graph_points[p].estimate())
            # p.update_normal_and_depth(force=True)

    mean_squared_error = opt.active_chi2() / max(num_edges, 1)

    return mean_squared_error


def pose_optimization(frame, verbose=False, rounds=10):
    is_ok = True

    # create g2o optimizer
    opt = g2o.SparseOptimizer()
    # block_solver = g2o.BlockSolverSE3(g2o.LinearSolverCSparseSE3())
    # block_solver = g2o.BlockSolverSE3(g2o.LinearSolverDenseSE3())
    block_solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
    solver = g2o.OptimizationAlgorithmLevenberg(block_solver)
    opt.set_algorithm(solver)

    # robust_kernel = g2o.RobustKernelHuber(np.sqrt(5.991))  # chi-square 2 DOFs
    thHuberMono = math.sqrt(5.991);  # chi-square 2 DOFS

    point_edge_pairs = {}
    num_point_edges = 0

    v_se3 = g2o.VertexSE3Expmap()
    v_se3.set_estimate(g2o.SE3Quat(frame.Rcw, frame.tcw))
    v_se3.set_id(0)
    v_se3.set_fixed(False)
    opt.add_vertex(v_se3)

    with MapPoint.global_lock:
        # add point vertices to graph
        for idx, p in enumerate(frame.points):
            if p is None:
                continue

            # reset outlier flag
            frame.outliers[idx] = False

            # add edge
            # print('adding edge between point ', p.id,' and frame ', frame.id)
            edge = g2o.EdgeSE3ProjectXYZOnlyPose()

            edge.set_vertex(0, opt.vertex(0))
            edge.set_measurement(frame.kpsu[idx])
            invSigma2 = Frame.feature_manager.inv_level_sigmas2[frame.octaves[idx]]
            edge.set_information(np.eye(2) * invSigma2)
            edge.set_robust_kernel(g2o.RobustKernelHuber(thHuberMono))

            edge.fx = frame.camera.fx
            edge.fy = frame.camera.fy
            edge.cx = frame.camera.cx
            edge.cy = frame.camera.cy
            edge.Xw = p.pt[0:3]

            opt.add_edge(edge)

            point_edge_pairs[p] = (edge, idx)  # one edge per point
            num_point_edges += 1

    if num_point_edges < 3:
        Printer.red('pose_optimization: not enough correspondences!')
        is_ok = False
        return 0, is_ok, 0

    if verbose:
        opt.set_verbose(True)

    # perform 4 optimizations:
    # after each optimization we classify observation as inlier/outlier;
    # at the next optimization, outliers are not included, but at the end they can be classified as inliers again
    chi2Mono = 5.991  # chi-square 2 DOFs
    num_bad_point_edges = 0

    for it in range(4):
        v_se3.set_estimate(g2o.SE3Quat(frame.Rcw, frame.tcw))
        opt.initialize_optimization()
        opt.optimize(rounds)

        num_bad_point_edges = 0

        for p, edge_pair in point_edge_pairs.items():
            edge, idx = edge_pair
            if frame.outliers[idx]:
                edge.compute_error()

            chi2 = edge.chi2()

            if chi2 > chi2Mono:
                frame.outliers[idx] = True
                edge.set_level(1)
                num_bad_point_edges += 1
            else:
                frame.outliers[idx] = False
                edge.set_level(0)

            if it == 2:
                edge.set_robust_kernel(None)

        if len(opt.edges()) < 10:
            Printer.red('pose_optimization: stopped - not enough edges!')
            is_ok = False
            break

    print('pose optimization: available ', num_point_edges, ' points, found ', num_bad_point_edges, ' bad points')
    if num_point_edges == num_bad_point_edges:
        Printer.red('pose_optimization: all the available correspondences are bad!')
        is_ok = False

        # update pose estimation
    if is_ok:
        est = v_se3.estimate()
        # R = est.rotation().matrix()
        # t = est.translation()
        # frame.update_pose(poseRt(R, t))
        frame.update_pose(g2o.Isometry3d(est.orientation(), est.position()))

    # since we have only one frame here, each edge corresponds to a single distinct point
    num_valid_points = num_point_edges - num_bad_point_edges

    mean_squared_error = opt.active_chi2() / max(num_valid_points, 1)

    return mean_squared_error, is_ok, num_valid_points
