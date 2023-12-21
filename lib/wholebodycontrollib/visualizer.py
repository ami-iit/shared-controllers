import idyntree.bindings as iDynTree

class Visualizer:

    def __init__(self, color_palette = "meshcat", force_scale_factor = 100):

        self.idyntree_visualier = iDynTree.Visualizer()
        super().__init__
        visualizer_options = iDynTree.VisualizerOptions()
        self.idyntree_visualier.init(visualizer_options)
        self.idyntree_visualier.setColorPalette(color_palette)
        self.optimization_models = {}
        self.force_scale_factor = force_scale_factor
        self.is_running = False

    
    def add_model(self, model_name, model, urdf_path, base_link, model_color = None):

        self.idyntree_visualier.addModel(model.kindyn.model(), model_name)
        self.optimization_models[model_name] = model

        if not model_color is None:
            self.idyntree_visualier.modelViz(model_name).setModelColor(iDynTree.ColorViz(iDynTree.Vector4.FromPython(model_color)))

    def update_model(self, model_name, H_B, s, draw = False):
        s_idyntree_opti =iDynTree.VectorDynSize.FromPython(s)
        T_b_opti = iDynTree.Transform()
        T_b_opti.fromHomogeneousTransform(iDynTree.Matrix4x4(H_B))
        self.idyntree_visualier.modelViz(model_name).setPositions(T_b_opti,s_idyntree_opti)

        if draw and self.idyntree_visualier.run():
            self.idyntree_visualier.draw()
        
        # TODO: add force visualization
        # for contact in self.optimization_models[model_name].contact_constraints.values():
        #     T_c = self.optimization_models[model_name].model.forward_kinematics_fun(contact.constraint_object.frame_name)(H_B, s)
        #     T_c_idyn = iDynTree.Transform()
        #     T_c_idyn.fromHomogeneousTransform(iDynTree.Matrix4x4(T_c))
        #     force = iDynTree.Direction().FromPython(f_c.ravel()[contact.wrench_vector_indices][:3] / self.force_scale_factor)
        #     self.idyntree_visualier.vectors().addVector(T_c_idyn.getPosition(), force)
        #     # self.idyntree_visualier.frames().addFrame(T_c_idyn, 0.5)
    
    def draw(self):
        if self.idyntree_visualier.run():
            self.idyntree_visualier.draw()

    def run(self):
        self.idyntree_visualier.camera().animator().enableMouseControl()
       
        if(self.idyntree_visualier.run()):
            self.is_running = True
            

    def close(self):
        self.idyntree_visualier.close()
