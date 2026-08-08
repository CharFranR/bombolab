use super::*;

#[test]
fn test_rng_deterministic_sequence() {
    let mut a = Xoshiro256StarStar::new(42);
    let mut b = Xoshiro256StarStar::new(42);
    for _ in 0..100 {
        assert_eq!(
            a.next_u64(),
            b.next_u64(),
            "same seed must replay bit-identical u64s"
        );
    }
}

#[test]
fn test_rng_uniform_deterministic() {
    let mut a = Xoshiro256StarStar::new(42);
    let mut b = Xoshiro256StarStar::new(42);
    for _ in 0..100 {
        assert_eq!(
            a.next_f64_uniform().to_bits(),
            b.next_f64_uniform().to_bits(),
            "same seed must replay bit-identical floats"
        );
    }
}

#[test]
fn test_rng_different_seeds_differ() {
    let mut a = Xoshiro256StarStar::new(42);
    let mut b = Xoshiro256StarStar::new(43);
    let seq_a: Vec<u64> = (0..8).map(|_| a.next_u64()).collect();
    let seq_b: Vec<u64> = (0..8).map(|_| b.next_u64()).collect();
    assert!(
        seq_a.iter().zip(&seq_b).any(|(x, y)| x != y),
        "different seeds must produce different sequences"
    );
}

#[test]
fn test_rng_uniform_range() {
    let mut rng = Xoshiro256StarStar::new(7);
    for _ in 0..100_000 {
        let u = rng.next_f64_uniform();
        assert!(
            (0.0..1.0).contains(&u),
            "uniform value {u} must be in [0,1)"
        );
    }
}

#[test]
fn test_rng_uniform_mean() {
    let mut rng = Xoshiro256StarStar::new(7);
    let n = 1_000_000u64;
    let sum: f64 = (0..n).map(|_| rng.next_f64_uniform()).sum();
    let mean = sum / n as f64;
    assert!(
        (mean - 0.5).abs() < 0.01,
        "mean {mean} must be within 0.5 ± 0.01"
    );
}

#[test]
fn test_rng_state_nonzero() {
    for seed in [0u64, 1, 42, u64::MAX] {
        let rng = Xoshiro256StarStar::new(seed);
        assert!(
            rng.state.iter().all(|&s| s != 0),
            "seed {seed} must expand to a nonzero state"
        );
    }
}
