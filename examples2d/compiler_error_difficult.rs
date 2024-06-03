use nalgebra::OPoint;
use rapier2d::prelude::*;
use rapier_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    let origin = vector![100_000.0, -80_000.0];
    let friction = 0.6;

    /*
     * Ground
     */
    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -1.0] + origin);
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(100.0, 1.0).friction(friction);
    colliders.insert_with_parent(collider, ground_handle, &mut bodies);

    testbed.add_callback(move |_, physics, _, run_state| {
        static WARMUP_SECONDS: f32 = 2_f32;
        if WARMUP_SECONDS + 5f32 <= run_state.time {
            let rb = physics.bodies.get_mut(ground_handle).unwrap();
            if rb.body_type() == RigidBodyType::Fixed {
                rb.set_body_type(RigidBodyType::Dynamic, false);
            }
        }
        if WARMUP_SECONDS + 5f32 + 3f32 <= run_state.time {
            let rb = physics.bodies.get_mut(ground_handle).unwrap();
            rb.set_linvel(Point::new(1f32, -1f32), false);
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 2.5] + origin, 20.0);
}
