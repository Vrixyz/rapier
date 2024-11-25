use parry::bounding_volume;
use parry::math::{Isometry, Point, Real};
use parry::shape::{Cuboid, SharedShape, TriMeshBuilderError, TriMeshFlags};

#[cfg(feature = "dim3")]
use parry::transformation::vhacd::VHACDParameters;

/*
 *
 * TODO: should all this be part of parry instead?
 *
 */

/// Error that can be generated by the [`MeshConverter`].
#[derive(thiserror::Error, Debug)]
pub enum MeshConverterError {
    /// The convex hull calculation carried out by the [`MeshConverter::ConvexHull`] failed.
    #[error("convex-hull computation failed")]
    ConvexHullFailed,
    /// The TriMesh building failed.
    #[error("TriMesh building failed")]
    TriMeshBuilderError(TriMeshBuilderError),
}

/// Determines how meshes (generally when loaded from a file) are converted into Rapier colliders.
// TODO: implement Copy once we add a Copy implementation for VHACDParameters.
#[derive(Clone, Debug, PartialEq, Default)]
pub enum MeshConverter {
    /// The mesh is loaded as-is without any particular processing.
    #[default]
    TriMesh,
    /// The mesh is loaded with the specified flags.
    TriMeshWithFlags(TriMeshFlags),
    /// The mesh is replaced by its Oriented Bounding Box (represented as
    /// a rotated cuboid).
    ///
    /// With this option, the mesh’s index buffer is ignored.
    Obb,
    /// The mesh is replaced by its AABB.
    ///
    /// With this option, the mesh’s index buffer is ignored.
    Aabb,
    /// The mesh is replaced by its convex-hull.
    ///
    /// With this option, the mesh’s index buffer is ignored.
    ConvexHull,
    /// The mesh is replaced by its convex decomposition.
    #[cfg(feature = "dim3")]
    ConvexDecomposition,
    /// The mesh is replaced by its convex decomposition with parameters specified to adjust
    /// the convex decomposition algorithm.
    #[cfg(feature = "dim3")]
    ConvexDecompositionWithParams(VHACDParameters),
}

impl MeshConverter {
    /// Applies the conversion rule described by this [`MeshConverter`] to build a shape from
    /// the given vertex and index buffers.
    #[profiling::function]
    pub fn convert(
        &self,
        vertices: Vec<Point<Real>>,
        indices: Vec<[u32; 3]>,
    ) -> Result<(SharedShape, Isometry<Real>), MeshConverterError> {
        let mut transform = Isometry::identity();
        let shape = match self {
            MeshConverter::TriMesh => SharedShape::trimesh(vertices, indices)
                .map_err(MeshConverterError::TriMeshBuilderError)?,
            MeshConverter::TriMeshWithFlags(flags) => {
                SharedShape::trimesh_with_flags(vertices, indices, *flags)
                    .map_err(MeshConverterError::TriMeshBuilderError)?
            }
            MeshConverter::Obb => {
                let (pose, cuboid) = parry::utils::obb(&vertices);
                transform = pose;
                SharedShape::new(cuboid)
            }
            MeshConverter::Aabb => {
                let aabb = bounding_volume::details::local_point_cloud_aabb(&vertices);
                let cuboid = Cuboid::new(aabb.half_extents());
                transform = Isometry::from(aabb.center().coords);
                SharedShape::new(cuboid)
            }
            MeshConverter::ConvexHull => {
                SharedShape::convex_hull(&vertices).ok_or(MeshConverterError::ConvexHullFailed)?
            }
            #[cfg(feature = "dim3")]
            MeshConverter::ConvexDecomposition => {
                SharedShape::convex_decomposition(&vertices, &indices)
            }
            #[cfg(feature = "dim3")]
            MeshConverter::ConvexDecompositionWithParams(params) => {
                SharedShape::convex_decomposition_with_params(&vertices, &indices, params)
            }
        };
        Ok((shape, transform))
    }
}
