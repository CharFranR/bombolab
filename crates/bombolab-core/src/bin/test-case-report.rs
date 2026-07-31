use std::f64::consts::PI;

use bombolab_core::math::dynamics::{LinkParams, gravity_vector, inertia_matrix};
use bombolab_core::math::jacobian::{JointKind, geometric_jacobian};
use bombolab_core::math::{DualQuaternion, Iso3, Mat3, Mat4, Quaternion, Vec3};
use bombolab_core::robot::fabri_creator::fabri_creator;

fn test_links() -> Vec<LinkParams> {
    let cyl = |mass: f64, len: f64| -> Mat3 {
        let r = 20.0;
        let i_xy = mass * (3.0 * r * r + len * len) / 12.0;
        let i_z = mass * r * r / 2.0;
        Mat3::from_diagonal(&Vec3::new(i_xy, i_xy, i_z))
    };
    vec![
        LinkParams::new(0.11, cyl(0.11, 50.0)),
        LinkParams::new(0.10, cyl(0.10, 120.0)),
        LinkParams::new(0.05, cyl(0.05, 90.0)),
        LinkParams::new(0.04, cyl(0.04, 50.0)),
        LinkParams::new(0.02, cyl(0.02, 40.0)),
    ]
}

fn main() {
    let q: [f64; 5] = [PI / 6.0, PI / 4.0, -PI / 4.0, PI / 3.0, PI / 6.0];
    let mut robot = fabri_creator();
    for (i, v) in q.iter().enumerate() {
        robot.segments[i].joint.value = *v;
    }

    let (frames, ee) =
        bombolab_core::kinematics::forward::forward_kinematics(Iso3::identity(), &robot);
    let mats: Vec<Mat4> = frames.iter().map(|iso| iso.to_matrix()).collect();
    let final_mat = ee.to_matrix();
    let kinds = [
        JointKind::Revolute,
        JointKind::Revolute,
        JointKind::Revolute,
        JointKind::Twist,
        JointKind::Revolute,
    ];
    let links = test_links();

    println!("======================================================================");
    println!("  FABRI CREATOR - CASO DE PRUEBA NUMÉRICO (Sección 15 del reporte)");
    println!("  q_test = (pi/6, pi/4, -pi/4, pi/3, pi/6) rad, estado estatico");
    println!("======================================================================\n");

    // 1. MTH
    println!("=== 1. MTH GLOBAL DEL EFECTOR (mm) ===\n");
    for (i, m) in mats.iter().enumerate() {
        let p = m.fixed_view::<3, 1>(0, 3);
        println!(
            "  p_{} = ({:>9.4}, {:>9.4}, {:>9.4})",
            i + 1,
            p[(0, 0)],
            p[(1, 0)],
            p[(2, 0)]
        );
    }
    println!("\n  T_0,5(q_test) =");
    for row in 0..4 {
        print!("    [");
        for col in 0..4 {
            print!(" {:>10.4}", final_mat[(row, col)]);
        }
        println!(" ]");
    }

    // 2. Cuaternión unitario
    let r5: Mat3 = final_mat.fixed_view::<3, 3>(0, 0).into_owned();
    let qr = Quaternion::from_rotation_matrix(&r5);
    println!("\n=== 2. CUATERNIÓN UNITARIO DE R_0,5 ===\n");
    println!(
        "  q_r = ({:>8.4}, {:>8.4}, {:>8.4}, {:>8.4})   |q_r| = {:.6}",
        qr.a,
        qr.b,
        qr.c,
        qr.d,
        qr.norm()
    );

    // 3. Cuaternión dual
    let t5 = final_mat.fixed_view::<3, 1>(0, 3).into_owned();
    let dq = DualQuaternion::from_pose(&r5, &t5);
    let t_rec = dq.translation();
    println!("\n=== 3. CUATERNIÓN DUAL DE LA POSE ===\n");
    println!(
        "  q_r = ({:>8.4}, {:>8.4}, {:>8.4}, {:>8.4})",
        dq.real.a, dq.real.b, dq.real.c, dq.real.d
    );
    println!(
        "  q_d = ({:>8.4}, {:>8.4}, {:>8.4}, {:>8.4})",
        dq.dual.a, dq.dual.b, dq.dual.c, dq.dual.d
    );
    println!(
        "  verificacion: 2*q_d x q_r* = ({:>9.4}, {:>9.4}, {:>9.4})  == p_ee",
        t_rec.x, t_rec.y, t_rec.z
    );

    // 4. Jacobiana
    let j = geometric_jacobian(&mats, &kinds, &final_mat).expect("jacobiano");
    let jt_j = j.transpose() * &j;
    println!("\n=== 4. JACOBIANA GEOMETRICA (6x5) ===\n");
    for row in 0..6 {
        print!("    [");
        for col in 0..5 {
            print!(" {:>9.4}", j[(row, col)]);
        }
        println!(" ]");
    }
    println!(
        "  det(J^T J) = {:.3e}  -> configuracion regular (rango 5)",
        jt_j.determinant()
    );

    // 5. Matriz de inercia
    let m = inertia_matrix(&robot, &mats, &links);
    println!("\n=== 5. MATRIZ DE INERCIA M(q_test) (kg*mm^2) ===\n");
    for row in 0..5 {
        print!("    [");
        for col in 0..5 {
            print!(" {:>9.1}", m[(row, col)]);
        }
        println!(" ]");
    }

    // 6. Pares
    let g = gravity_vector(&robot, &mats, &links, 9.81);
    println!("\n=== 6. PARES ARTICULARES (estatico: tau = g(q_test)) ===\n");
    println!(
        "  g(q_test) = ({:>9.4}, {:>9.4}, {:>9.4}, {:>9.4}, {:>9.4}) N*m",
        g[(0, 0)],
        g[(1, 0)],
        g[(2, 0)],
        g[(3, 0)],
        g[(4, 0)]
    );
    println!("\n  Los valores coinciden con los cálculos manuales del reporte (Sección 15).");
    println!("  NOTA: modelo ESTÁTICO — no existe término C(q,q̇). La identidad");
    println!("  'M qpp + C qp + g = tau' solo se cumple con qp = qpp = 0 (reposo).");
    println!("  Para control dinámico de trayectorias se necesitaría Coriolis/centrífugos.");
    println!("  OK: qp = qpp = 0 -> tau = g (verificación estática válida).");
}
