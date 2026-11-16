SurgAware anatomy exports (binary PLY, world frame, metres)

Per run:
  mesh_tNNN.ply           anatomy surfaces at frame NNN (all organs merged), vertex colour = photo texture (de-lit albedo)
  mesh_semantic_tNNN.ply  same mesh, vertex colour = semantic class
  pointcloud_left_t000.ply  what the left camera sees at frame 0: one point per anatomy pixel (tools removed),
                            colour = clean render

Every vertex also carries a `label` property = semantic class id (the id -> name map is in each file's header comments).
The meshes extend past the camera view (the scene geometry is padded beyond the photo), so they are larger than the
point clouds. Vertex colours: use "vertex colour" shading in MeshLab / CloudCompare / Blender.

  v08_grasp     variety2/v08: two instruments, one grasps and retracts tissue (t000 vs t047 shows the deformation)
  v03_hero      variety2/v03
  b_real_0002   b_real_0002
  c01_cut       cut/c01_blood_on: scissors incision (t000 intact, t104 after the cut)

Textures derive from Cholec80 / CholecSeg8k frames (CC BY-NC-SA 4.0): non-commercial use only.
