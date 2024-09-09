'''
    create_tree.py
    
    Hannah Gillespie
    Imperial College London
    MSc in Computing

    Notes:
    print() displays to the System Console
'''

import bpy


### parameters
N_TREES = 1
NODE_NAME = "test1"

def enable_mtree_addon():
    print("Enabling MTree add-on.")
    addon_name = 'modular_tree-blender_28'  # The module name of the MTree add-on
    if addon_name not in bpy.context.preferences.addons:
        bpy.ops.preferences.addon_enable(module=addon_name)
        print("MTree add-on enabled.")
    else:
        print("MTree add-on is already enabled.")

def delete_all_objects():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False, confirm=False)
    
def delete_object(object_name):
    obj = bpy.data.objects[object_name]
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.delete()

# Function to append an existing material to a node in an MTree
def append_material_to_mtree(node_group_name, node_name, material_name):
    # Get the MTree node tree
    node_group = bpy.data.node_groups.get(node_group_name)
    if node_group is None:
        print(f"MTree node group '{node_group_name}' not found.")
        return
    
    # Get the node from the MTree node tree
    node = node_group.nodes.get("tree")
    if node is None:
        print(f"Node 'tree' not found in MTree '{node_group_name}'.")
        return
    
    # Get the existing material
    material = bpy.data.materials.get(material_name)
    if material is None:
        print(f"Material '{material_name}' not found.")
        return
    
    # Check if the node has a material input
    if 'Material' not in node.inputs:
        print(f"Node '{node_name}' does not have a 'Material' input.")
        return
    
    # Create a material output node if not exists
    if 'Material Output' not in mtree.nodes:
        material_output_node = mtree.nodes.new(type='ShaderNodeOutputMaterial')
        material_output_node.location = (400, 0)
    else:
        material_output_node = mtree.nodes.get('Material Output')

    # Create a new material node
    material_node = mtree.nodes.new(type='ShaderNodeMaterial')
    material_node.material = material
    material_node.location = (0, 0)
    
    # Link the material node to the material output node
    mtree.links.new(material_node.outputs['Shader'], material_output_node.inputs['Surface'])

    print(f"Material '{material.name}' appended to node '{node_name}' in MTree '{mtree_name}'.")


def create_tree(node_group_name, bark_type):
    print("Creating tree group", node_group_name)

    # Create a new tree
    bpy.ops.node.new_node_tree(type="mtree_node_tree", name=node_group_name)

    # Load the type of tree ("old oak copy")
    bpy.data.node_groups[node_group_name].preset_to_load = 'old oak copy'
    bpy.ops.mtree.save_preset(node_group_name=node_group_name, load=True)

    #tree = bpy.context.active_object
    # Create Tree
    bpy.ops.object.mtree_execute_tree(node_group_name=node_group_name)
    new_tree = bpy.data.objects[-1]
    new_tree.name = "tree"
    print("Created tree")
    
    # Create Twig Branch
    bpy.ops.object.mtree_twig(node_group_name=node_group_name, node_name="Twig Node")
    new_twig = bpy.data.objects[-1]
    new_twig.name = "twig"
    
    # Link Leaf to Tree
    bpy.data.node_groups[node_group_name].nodes["Tree parameters"].leaf_dupli_object = bpy.data.objects["twig"]
    bpy.ops.object.mtree_execute_tree(node_group_name=node_group_name)
    
    # Delete Twig Node
    #delete_object("twig")
    print("Added leaves")
    
    # Add bark
    bpy.context.scene.render.engine = "CYCLES"
    bpy.context.scene.cycles.feature_set = "EXPERIMENTAL"
    bpy.context.scene.cycles.device = "GPU"
    bpy.ops.mtree.append_bark_materials()
    
    append_material_to_mtree(node_group_name, "tree", 'oak')
    #print("Applied bark")
    
    # Add leaf via twig node
    print("Tree creation complete.")
    
    #bpy.ops.mesh.treemesh_add()

    # Get the tree object
    #tree = bpy.context.active_object

    # Optionally, name the tree
    #tree.name = "MyTree"

    # Optionally, set the tree's location, rotation, and scale
    #tree.location = (0, 0, 0)
    #tree.rotation_euler = (0, 0, 0)
    #tree.scale = (1, 1, 1)

    # Optionally, apply transformations (location, rotation, scale)
    #bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

    #print("Tree created successfully!")

def create_n_random_trees(number_of_trees):
    for i in range(0, number_of_trees):
        node_group_name = NODE_NAME + "_" + str(i)
        delete_all_objects()
        # Bark types: healthy, diseased
        create_tree(node_group_name, "healthy")
        

# Run the function to create the tree
if __name__ == "__main__":
    enable_mtree_addon()
    create_n_random_trees(number_of_trees=N_TREES)


# import bpy
# bpy.ops.node.new_node_tree(type="mtree_node_tree", name="tree4")
# bpy.data.node_groups["tree"].preset_to_load = 'old oak'
# bpy.ops.mtree.save_preset(node_group_name="tree4", load=True)
# bpy.ops.mtree_randomize_tree
# bpy.data.node_groups["tree"].nodes["Branch Node.001"].seed = 30

# import bpy
# tree_name = "test3"

# # Clear all existing objects
# bpy.ops.object.select_all(action='SELECT')
# bpy.ops.object.delete(use_global=False, confirm=False)


# print("beginning...")
# bpy.ops.node.new_node_tree(type="mtree_node_tree", name=tree_name)
# bpy.data.node_groups[tree_name].preset_to_load = 'old oak'
# bpy.ops.mtree.save_preset(node_group_name=tree_name, load=True)
# tree = bpy.context.active_object
# bpy.ops.object.mtree_execute_tree(node_group_name=tree_name)
# print("complete...")