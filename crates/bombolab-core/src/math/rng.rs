const SPLITMIX_GOLDEN: u64 = 0x9E37_79B9_7F4A_7C15;
const SPLITMIX_M1: u64 = 0xBF58_476D_1CE4_E5B9;
const SPLITMIX_M2: u64 = 0x94D0_49BB_1331_11EB;

pub struct Xoshiro256StarStar {
    state: [u64; 4],
}

impl Xoshiro256StarStar {
    pub fn new(seed: u64) -> Self {
        let mut sm = seed;
        let mut state = [0u64; 4];
        for word in state.iter_mut() {
            sm = sm.wrapping_add(SPLITMIX_GOLDEN);
            let mut z = sm;
            z = (z ^ (z >> 30)).wrapping_mul(SPLITMIX_M1);
            z = (z ^ (z >> 27)).wrapping_mul(SPLITMIX_M2);
            z ^= z >> 31;
            *word = z;
        }
        Self { state }
    }

    pub fn next_u64(&mut self) -> u64 {
        let result = self.state[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = self.state[1] << 17;
        self.state[2] ^= self.state[0];
        self.state[3] ^= self.state[1];
        self.state[1] ^= self.state[2];
        self.state[0] ^= self.state[3];
        self.state[2] ^= t;
        self.state[3] = self.state[3].rotate_left(45);
        result
    }

    pub fn next_f64_uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }
}

#[cfg(test)]
#[path = "rng_tests.rs"]
mod rng_tests;
