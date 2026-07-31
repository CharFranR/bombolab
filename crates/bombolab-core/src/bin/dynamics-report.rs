use bombolab_core::kinematics::forward::forward_kinematics;
use bombolab_core::math::jacobian::{JointKind, geometric_jacobian};
use bombolab_core::math::{Iso3, Mat4, Vec3};
use bombolab_core::robot::fabri_creator::fabri_creator;

fn main() {
    let robot = fabri_creator();
    let base = Iso3::identity();

    // Home pose: all q = 0
    let (frames, _ee) = forward_kinematics(base, &robot);

    // Convert Iso3 frames to Mat4 for geometric_jacobian
    let mats: Vec<Mat4> = frames.iter().map(|iso| iso.to_matrix()).collect();
    let final_mat = _ee.to_matrix();

    let kinds = [
        JointKind::Revolute,
        JointKind::Revolute,
        JointKind::Revolute,
        JointKind::Twist,
        JointKind::Revolute,
    ];

    // ============================================================
    println!("╔══════════════════════════════════════════════════════════╗");
    println!("║     FABRI CREATOR — forward_kinematics() EN HOME        ║");
    println!("║     q = [0, 0, 0, 0, 0]                                ║");
    println!("╚══════════════════════════════════════════════════════════╝\n");

    // ============================================================
    // 1. FRAMES (T_0i)
    // ============================================================
    println!("=== 1. POSICIONES DE FRAMES (sin base ni tool) ===\n");
    for (i, m) in mats.iter().enumerate() {
        let p = m.fixed_view::<3, 1>(0, 3);
        println!(
            "  p_{:<1} = ({:>8.2}, {:>8.2}, {:>8.2}) mm",
            i + 1,
            p[(0, 0)],
            p[(1, 0)],
            p[(2, 0)]
        );
    }
    let pee = final_mat.fixed_view::<3, 1>(0, 3);
    println!(
        "\n  p_ee = ({:>8.2}, {:>8.2}, {:>8.2}) mm\n",
        pee[(0, 0)],
        pee[(1, 0)],
        pee[(2, 0)]
    );

    // ============================================================
    // 2. EJES DE ROTACIÓN
    // ============================================================
    println!("=== 2. EJES DE ROTACIÓN EN HOME ===\n");
    println!("  z_0 = (0, 0, 1)");
    for i in 0..4 {
        let z = mats[i].fixed_view::<3, 1>(0, 2);
        let x = mats[i].fixed_view::<3, 1>(0, 0);
        let kind = kinds[i + 1];
        let axis = if kind == JointKind::Twist { x } else { z };
        let label = if kind == JointKind::Twist { "x" } else { "z" };
        println!(
            "  {}_{:<1} = ({:>8.4}, {:>8.4}, {:>8.4})  ({})",
            label,
            i + 1,
            axis[(0, 0)],
            axis[(1, 0)],
            axis[(2, 0)],
            if kind == JointKind::Twist {
                "Twist"
            } else {
                "Revolute"
            }
        );
    }
    println!();

    // ============================================================
    // 3. J_omega,i
    // ============================================================
    println!("=== 3. J_ω,i (3×5, columnas i+1..5 nulas) ===\n");
    let axes: [Vec3; 6] = [
        Vec3::new(0.0, 0.0, 1.0),                      // z0
        mats[0].fixed_view::<3, 1>(0, 2).into_owned(), // z1
        mats[1].fixed_view::<3, 1>(0, 2).into_owned(), // z2
        mats[2].fixed_view::<3, 1>(0, 0).into_owned(), // x3 (Twist)
        mats[3].fixed_view::<3, 1>(0, 2).into_owned(), // z4
        mats[4].fixed_view::<3, 1>(0, 2).into_owned(), // z5 (not used)
    ];

    for i in 1..=5 {
        print!("  J_ω,{} = [", i);
        for (j, ax) in axes.iter().enumerate() {
            if j >= i {
                print!(" (0,0,0)");
            } else {
                print!(" ({:.4},{:.4},{:.4})", ax[0], ax[1], ax[2]);
            }
            if j < 4 {
                print!(" ;");
            }
        }
        println!(" ]");
    }
    println!();

    // ============================================================
    // 4. J_ee
    // ============================================================
    println!("=== 4. JACOBIANO DEL EFECTOR (6×5) ===\n");
    let j_ee = geometric_jacobian(&mats, &kinds, &final_mat).unwrap();
    println!("  J_home =");
    for row in 0..6 {
        print!("    [");
        for col in 0..5 {
            print!(" {:>10.4}", j_ee[(row, col)]);
        }
        println!(" ]");
    }
    println!();

    // ============================================================
    // 5. J_com,i
    // ============================================================
    println!("=== 5. J_com,i^(0) (3×5, COM en origen de frame) ===\n");
    for i in 0..5 {
        let partial: Vec<Mat4> = mats[..=i].to_vec();
        let partial_kinds: Vec<JointKind> = kinds[..=i].to_vec();
        let frame_i = mats[i];
        let jp = geometric_jacobian(&partial, &partial_kinds, &frame_i).unwrap();

        println!("  Eslabón {} — parte lineal (3×{}):", i + 1, i + 1);
        for row in 0..3 {
            print!("    [");
            for col in 0..=i {
                print!(" {:>10.4}", jp[(row, col)]);
            }
            println!(" ]");
        }
        println!();
    }

    // ============================================================
    // 6. ALTURAS
    // ============================================================
    println!("=== 6. ALTURAS DE FRAMES (z, sin base) ===\n");
    for (i, m) in mats.iter().enumerate() {
        let z = m[(2, 3)];
        println!("  z_{:<1} = {:>8.2} mm", i + 1, z);
    }
    println!();
}
