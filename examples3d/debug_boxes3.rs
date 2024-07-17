use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Ground
     */
    let ground_size = 100.1;
    let ground_height = 0.1;

    let ground_height = 0.1;
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(100.1, ground_height, 100.1).friction(0.5),
        bodies.insert(RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0])),
        &mut bodies,
    );

    let (collider_mass, rigidbody_additional_mass) = (0.0, 0.5);

    // NOTE: #666: uncomment this to test "expected" behaviour.
    //let (collider_mass, rigidbody_additional_mass) = (0.5, 0.0);

    colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.2, 5., 1.5)
            .friction(0.5)
            .mass(collider_mass),
        bodies.insert(
            RigidBodyBuilder::dynamic()
                .additional_mass(rigidbody_additional_mass)
                .translation(vector![-10., 6.0, 0.0])
                .linvel(vector![20., 0., 0.])
                .can_sleep(false),
        ),
        &mut bodies,
    );

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![10.0, 10.0, 10.0], Point::origin());
}
