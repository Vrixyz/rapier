use nalgebra::Vector3;
use rand::thread_rng;
use rand::Rng;
use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;
use rapier_testbed3d::TestbedApp;
use std::collections::VecDeque;
use std::error::Error;
use std::io::Write;
use std::path::PathBuf;
use tracing_log::LogTracer;
use tracing_subscriber::filter::FromEnvError;
use tracing_subscriber::filter::ParseError;
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::registry::Registry;
use tracing_subscriber::EnvFilter;

pub fn main() {
    init_tracing();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut bf = BroadPhaseMultiSap::new();
    let mut nf = NarrowPhase::new();
    let mut islands = IslandManager::new();
    let mut query_pipeline = QueryPipeline::new();
    let mut ccd_solver = CCDSolver::new();

    let mut bodies = RigidBodySet::new();

    let mut rng = thread_rng();
    let mut handles = VecDeque::new();

    for i in 0..4 {
        pipeline.step(
            &Vector3::new(0.0, -9.81, 0.0),
            &IntegrationParameters::default(),
            &mut islands,
            &mut bf,
            &mut nf,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd_solver,
            Some(&mut query_pipeline),
            &(),
            &(),
        );

        if i % 100 == 0 || true {
            //assert!(bodies.len() <= 10);
            //assert!(colliders.len() <= 10);
            let incremental_bytes = bincode::serialized_size(&query_pipeline).unwrap();

            let mut new_qp = QueryPipeline::new();
            new_qp.update(&colliders);
            let new_bytes = bincode::serialized_size(&new_qp).unwrap();
            let new_bytes = bincode::serialized_size(&new_qp.qbvh.raw_nodes()).unwrap();

            //println!("{i}, {incremental_bytes}, {new_bytes}");
            /*
            let incremental_bytes =
                bincode::serialized_size(&query_pipeline.qbvh.raw_nodes()).unwrap();
            println!("{i}, {incremental_bytes}, {new_bytes}");
             */
            println!(
                "iteration:{i} \
                raw nodes size (incremental): {} \
                raw nodes size (fresh): {} \
                nb bodies: {}",
                query_pipeline.qbvh.raw_nodes().len(),
                new_qp.qbvh.raw_nodes().len(),
                bodies.len()
            );
            let file_path = PathBuf::from(format!(
                "./data_{}/{i}.ron",
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_secs()
            ));
            let prefix = file_path.parent().unwrap();
            std::fs::create_dir_all(prefix).unwrap();
            let mut file = std::fs::File::create(file_path).unwrap();
            file.write_fmt(format_args!("{:#?}", query_pipeline.qbvh.raw_nodes()))
                .unwrap();
        }

        if i % 2 == 0 {
            let rigid_body = RigidBodyBuilder::dynamic().translation(vector![
                rng.gen_range(0.0..100.0),
                rng.gen_range(0.0..100.0),
                rng.gen_range(0.0..100.0)
            ]);
            let handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(1.0, 1.0, 1.0);
            colliders.insert_with_parent(collider, handle, &mut bodies);

            handles.push_back(handle);
        } else {
            let remove = handles.pop_front().unwrap();
            bodies.remove(
                remove,
                &mut islands,
                &mut colliders,
                &mut impulse_joints,
                &mut multibody_joints,
                true,
            );
        }
    }
}

/*
/// Global allocator for tracy to support memory profiling.
#[global_allocator]
static GLOBAL: tracy_client::ProfiledAllocator<std::alloc::System> =
    tracy_client::ProfiledAllocator::new(std::alloc::System, 100);
*/
pub fn init_tracing() {
    // While not testing with tracy or tracing, for example through bytehound, it's best to return here to avoid useless noise.
    return;

    let default_filter = "WARN";
    let filter_layer = EnvFilter::try_from_default_env()
        .or_else(|from_env_error| {
            _ = from_env_error
                .source()
                .and_then(|source| source.downcast_ref::<ParseError>())
                .map(|parse_err| {
                    // we cannot use the `error!` macro here because the logger is not ready yet.
                    eprintln!("LogPlugin failed to parse filter from env: {}", parse_err);
                });

            Ok::<EnvFilter, FromEnvError>(EnvFilter::builder().parse_lossy(&default_filter))
        })
        .unwrap();
    let subscriber = Registry::default();
    let subscriber = subscriber.with(filter_layer);

    let tracy_layer = tracing_tracy::TracyLayer::default();
    // note: the implementation of `Default` reads from the env var NO_COLOR
    // to decide whether to use ANSI color codes, which is common convention
    // https://no-color.org/
    let fmt_layer = tracing_subscriber::fmt::Layer::default().with_writer(std::io::stderr);
    let subscriber = subscriber.with(fmt_layer);
    let subscriber = subscriber.with(tracy_layer);

    assert!(LogTracer::init().is_ok());
    assert!(tracing::subscriber::set_global_default(subscriber).is_ok());
}
