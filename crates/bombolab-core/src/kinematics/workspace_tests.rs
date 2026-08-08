use super::*;

#[test]
fn test_drawing_plane_constraint() {
    let mut sampler = WorkspaceSampler::new(42, WorkspaceMode::DrawingPlane);
    let mut accepted = 0;
    for _ in 0..200 {
        if let Some(q) = sampler.next_q() {
            accepted += 1;
            assert_eq!(q[3], 0.0, "q4 must be fixed to zero in drawing plane");
            assert!(
                (q[4] + q[1] + q[2]).abs() < 1e-12,
                "q5 must equal -(q2+q3), got q={q:?}"
            );
            for i in 0..3 {
                let (lo, hi) = sampler.joint_limits(i);
                assert!(
                    q[i] >= lo && q[i] <= hi,
                    "q{i} must stay within joint limits, q={q:?}"
                );
            }
            let (lo, hi) = sampler.joint_limits(4);
            assert!(
                q[4] >= lo && q[4] <= hi,
                "accepted q5 must stay within joint limits, q={q:?}"
            );
        }
    }
    assert!(accepted > 0, "expected at least one accepted sample");
}

#[test]
fn test_drawing_plane_rejection() {
    let mut sampler = WorkspaceSampler::new(42, WorkspaceMode::DrawingPlane);
    let batch = sampler.sample_batch(1000);
    let stats = sampler.stats();
    assert!(
        stats.n_rejected > 0,
        "expected rejections for seed 42, got {}",
        stats.n_rejected
    );
    assert_eq!(stats.n_valid + stats.n_rejected, 1000);
    assert_eq!(batch.len(), STRIDE * stats.n_valid);
}

#[test]
fn test_full_5dof_no_rejection() {
    let mut sampler = WorkspaceSampler::new(42, WorkspaceMode::Full5Dof);
    let batch = sampler.sample_batch(1000);
    let stats = sampler.stats();
    assert_eq!(stats.n_rejected, 0);
    assert_eq!(stats.n_valid, 1000);
    assert_eq!(batch.len(), STRIDE * 1000);
    let mut seen = 0;
    for _ in 0..100 {
        if let Some(q) = sampler.next_q() {
            seen += 1;
            for i in 0..5 {
                let (lo, hi) = sampler.joint_limits(i);
                assert!(
                    q[i] >= lo && q[i] <= hi,
                    "q{i} must stay within limits in full-5dof, q={q:?}"
                );
            }
        }
    }
    assert_eq!(seen, 100, "full-5dof must never reject");
}

#[test]
fn test_sample_batch_zero() {
    let mut sampler = WorkspaceSampler::new(42, WorkspaceMode::DrawingPlane);
    let batch = sampler.sample_batch(0);
    assert!(batch.is_empty());
    let stats = sampler.stats();
    assert_eq!(stats.n_valid, 0);
    assert_eq!(stats.n_rejected, 0);
    assert_eq!(stats.bounds_min, None);
    assert_eq!(stats.bounds_max, None);
    assert_eq!(stats.centroid, None);
    assert_eq!(stats.reach, None);
}

#[test]
fn test_seed_replay_bit_identical() {
    let mut a = WorkspaceSampler::new(42, WorkspaceMode::DrawingPlane);
    let mut b = WorkspaceSampler::new(42, WorkspaceMode::DrawingPlane);
    let batch_a = a.sample_batch(300);
    let batch_b = b.sample_batch(300);
    assert_eq!(batch_a.len(), batch_b.len());
    for (x, y) in batch_a.iter().zip(&batch_b) {
        assert_eq!(
            x.to_bits(),
            y.to_bits(),
            "same seed must replay bit-identical samples"
        );
    }
    assert_eq!(a.stats(), b.stats());
}

#[test]
fn test_different_seeds_differ() {
    let mut a = WorkspaceSampler::new(42, WorkspaceMode::DrawingPlane);
    let mut b = WorkspaceSampler::new(43, WorkspaceMode::DrawingPlane);
    let batch_a = a.sample_batch(50);
    let batch_b = b.sample_batch(50);
    assert!(
        batch_a
            .iter()
            .zip(&batch_b)
            .any(|(x, y)| x.to_bits() != y.to_bits()),
        "different seeds must produce different sequences"
    );
}

#[test]
fn test_stats_match_brute_recompute() {
    let mut sampler = WorkspaceSampler::new(7, WorkspaceMode::DrawingPlane);
    let batch = sampler.sample_batch(500);
    let stats = sampler.stats();
    assert!(stats.n_valid > 0);
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    let mut sum = [0.0; 3];
    let mut reach: f64 = 0.0;
    for chunk in batch.chunks_exact(STRIDE) {
        let p = [chunk[0], chunk[1], chunk[2]];
        for (v, (mn, (mx, sm))) in p
            .iter()
            .zip(min.iter_mut().zip(max.iter_mut().zip(sum.iter_mut())))
        {
            *mn = (*mn).min(*v);
            *mx = (*mx).max(*v);
            *sm += *v;
        }
        let norm = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        reach = reach.max(norm);
    }
    let eps = 1e-9;
    for i in 0..3 {
        assert!(
            (stats.bounds_min.unwrap()[i] - min[i]).abs() < eps,
            "min axis {i}"
        );
        assert!(
            (stats.bounds_max.unwrap()[i] - max[i]).abs() < eps,
            "max axis {i}"
        );
        assert!(
            (stats.centroid.unwrap()[i] - sum[i] / stats.n_valid as f64).abs() < eps,
            "centroid axis {i}"
        );
    }
    assert!((stats.reach.unwrap() - reach).abs() < eps, "reach");
}

#[test]
fn test_stats_accumulate_across_batches() {
    let mut sampler = WorkspaceSampler::new(11, WorkspaceMode::Full5Dof);
    let a = sampler.sample_batch(100);
    let b = sampler.sample_batch(200);
    let stats = sampler.stats();
    assert_eq!(stats.n_valid, 300);
    assert_eq!(a.len() / STRIDE + b.len() / STRIDE, 300);
}
