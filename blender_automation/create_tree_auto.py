import bpy
import bpy_extras
from mathutils import Vector
import random
import os
from datetime import datetime

### Parameters
N_TREES = 1
NODE_NAME = "tree_node"
OUTPUT_IMG_DIRECTORY = "/Users/hannahgillespie/aod_detection/blender_automation/bark/renderings/images"  # images composed of trunk + background
OUTPUT_LBL_DIRECTORY = "/Users/hannahgillespie/aod_detection/blender_automation/bark/renderings/labels" # YOLO formatted labels
BACKGROUND_DIRECTORY = "/Users/hannahgillespie/aod_detection/blender_automation/bark/backgrounds"  # collection of random backgrounds

BARK_TEXTURE_FOLDERS = {
    'low_risk_oak': "/Users/hannahgillespie/aod_detection/blender_automation/bark/types/low_risk_oak/",
    'medium_risk_oak': "/Users/hannahgillespie/aod_detection/blender_automation/bark/types/medium_risk_oak/",
    'high_risk_oak': "/Users/hannahgillespie/aod_detection/blender_automation/bark/types/high_risk_oak/",
    'non_oak': "/Users/hannahgillespie/aod_detection/blender_automation/bark/types/non_oak/"
}

# Map class names to their YOLO format labels
YOLO_LABELS = {
    'low_risk_oak': 0,
    'medium_risk_oak': 1,
    'high_risk_oak': 2,
    'non_oak': 3
}

def enable_mtree_addon():
    """Enable the Modular Tree (MTree) add-on, version compatible with Blender 2.8."""
    print("Enabling MTree add-on.")
    addon_name = 'modular_tree-blender_28'
    if addon_name not in bpy.context.preferences.addons:
        bpy.ops.preferences.addon_enable(module=addon_name)
        print("MTree add-on enabled.")
    else:
        print("MTree add-on is already enabled.")

def delete_all_objects():
    """Delete all objects (including camera and light) in active screen."""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False, confirm=False)

def choose_random_bark_texture():
    """
    Choose bark from any of the four classes (low, medium, and high risk oak or non-oak).
    First the class is chosen randomly and then the bark is chosen randomly.

    Returns:
    - texture_path: Path for specific bark image file.
    - category: Class distinction, i.e. 'low_risk_oak'.
    """
    # Choose one of the classes
    category = random.choice(list(BARK_TEXTURE_FOLDERS.keys()))
    folder_path = BARK_TEXTURE_FOLDERS[category]
    
    # Get all image files in the selected folder
    textures = [f for f in os.listdir(folder_path) if f.endswith(('.jpg', '.jpeg', '.png'))]
    if not textures:
        print(f"No textures found in {folder_path}.")
        return None, None
    
    # Randomly select a texture file
    texture_file = random.choice(textures)
    texture_path = os.path.join(folder_path, texture_file)
    
    return texture_path, category

def create_tree(node_group_name):
    """
    Creates new tree object using the MTree add-on. Currently disables the Branch Nodes.

    Args:
    - node_group_name: Unique name for MTree node group.

    Returns:
    - new_tree: Blender Tree object consisting of Trunk Node and Tree Parameters.
    - category: Class distinction, i.e. 'low_risk_oak'.
    """
    print("Creating tree group", node_group_name)
    bpy.ops.node.new_node_tree(type="mtree_node_tree", name=node_group_name)
    
    # Base model is derived off the "old oak" json that ships with MTree
    bpy.data.node_groups[node_group_name].preset_to_load = 'old oak copy'
    bpy.ops.mtree.save_preset(node_group_name=node_group_name, load=True)
    
    # Removing branches for simplification
    mtree_nodes = bpy.data.node_groups[node_group_name].nodes
    if "Branch Node" in mtree_nodes:
        mtree_nodes.remove(mtree_nodes["Branch Node"])  # Remove the branch node
    if "Branch Node.001" in mtree_nodes:
        mtree_nodes.remove(mtree_nodes["Branch Node.001"])  # Remove the branch node
    
    # Set a random seed for the tree
    random_seed = random.randint(0, 1000)
    bpy.data.node_groups[node_group_name].nodes["Trunk Node"].seed = random_seed
    print(f"Assigned random seed {random_seed} to tree.")

    bpy.ops.object.mtree_execute_tree(node_group_name=node_group_name)
    new_tree = bpy.data.objects[-1]
    new_tree.name = "tree"
    print("Created tree")

    # Choose random bark texture and apply it
    bark_texture_path, category = choose_random_bark_texture()
    if bark_texture_path:
        apply_bark_texture(new_tree, bark_texture_path)
    
    return new_tree, category

def apply_bark_texture(tree_object, texture_path):
    """
    Applies randomly selected bark to tree object.
    
    Args:
    - tree_object: The Blender object representing the tree.
    - texture_path: Filepath to randomly selected bark texture. 
    """
    # Set up new material for the bark
    bark_material = bpy.data.materials.new(name="BarkMaterial")
    bark_material.use_nodes = True
    nodes = bark_material.node_tree.nodes
    links = bark_material.node_tree.links

    # Clear existing nodes
    nodes.clear()

    # Set new node parameters
    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    output_node.location = (400, 0)

    principled_node = nodes.new(type='ShaderNodeBsdfPrincipled')
    principled_node.location = (0, 0)
    links.new(principled_node.outputs['BSDF'], output_node.inputs['Surface'])

    # Load the image texture
    texture_node = nodes.new(type='ShaderNodeTexImage')
    texture_node.image = bpy.data.images.load(texture_path)
    texture_node.location = (-300, 0)
    links.new(texture_node.outputs['Color'], principled_node.inputs['Base Color'])

    # Adjust shader properties
    principled_node.inputs['Specular'].default_value = 0.1
    principled_node.inputs['Roughness'].default_value = 0.8

    # Assign the material to the tree object
    if tree_object.data.materials:
        tree_object.data.materials[0] = bark_material
    else:
        tree_object.data.materials.append(bark_material)

    print(f"Applied texture from {texture_path} to the tree.")

def create_random_camera(name_suffix):
    """
    Create a camera at a random location pointing towards trunk at random target point.
    
    Args:
    - name_suffix: Index of camera depending on number of cameras generated for given tree.
    
    Returns:
    - camera: New camera object created at random location pointing towards tree trunk 
              in a random specific spot. 
    """
    # Choose random location of camera (boundary of 2 in x, y direction from trunk)
    camera_x = random.uniform(2, 10) * random.choice([-1, 1])
    camera_y = random.uniform(2, 10) * random.choice([-1, 1])
    camera_z = random.uniform(1, 4)
    
    # Add camera
    bpy.ops.object.camera_add(location=(camera_x, camera_y, camera_z))
    camera = bpy.context.object
    camera.name = f"Camera_{name_suffix}"

    # Select random target location towards trunk of tree
    target_x = random.uniform(-1, 1)
    target_y = random.uniform(-1, 1)
    target_z = random.uniform(1, 3)
    target_location = Vector((target_x, target_y, target_z))

    # Set the camera's orientation towards the target location
    camera_location = camera.location
    direction = target_location - camera_location
    direction.normalize()
    rot_quat = direction.to_track_quat('-Z', 'Y')
    camera.rotation_euler = rot_quat.to_euler()

    print(f"Camera '{camera.name}' created at ({camera_x:.2f}, {camera_y:.2f}, {camera_z:.2f}) and pointing to ({target_x:.2f}, {target_y:.2f}, {target_z:.2f}).")
    
    # Each camera will have a little light behind it aimed at the trunk to help with shadows in Blender
    create_light_behind_camera(camera)

    return camera

def create_light_behind_camera(camera):
    """Set light source behind each camera of type 'point' with low and variable intensity"""
    light_location = camera.location - camera.matrix_world.to_3x3() @ Vector((0, 0, 2))
    bpy.ops.object.light_add(type='POINT', location=light_location)
    light = bpy.context.object
    light.name = f"Light_{camera.name}"
    light.data.energy = random.uniform(250, 750)

    print(f"Light '{light.name}' created behind {camera.name} at intensity {light.data.energy}.")

def create_sun_light():
    """Set light source for scene of type 'sun" with a random intensity."""
    bpy.ops.object.light_add(type='SUN', location=(10, -10, 10))
    sun = bpy.context.object
    sun.name = "Sun"
    sun.data.energy = random.uniform(3, 8)

    print(f"Sun light created at intensity {sun.data.energy}.")

def set_day_environment_texture():
    """Set a generic day background using Blender's environment nodes with a sky texture."""
    # Ensure the world uses nodes
    bpy.context.scene.world.use_nodes = True
    world_nodes = bpy.context.scene.world.node_tree.nodes
    world_links = bpy.context.scene.world.node_tree.links

    # Clear existing nodes
    world_nodes.clear()

    # Set default Sky Texture
    sky_texture_node = world_nodes.new(type='ShaderNodeTexSky')
    sky_texture_node.location = (-300, 0)
    sky_texture_node.sky_type = 'HOSEK_WILKIE'  

    # Sun direction parameters
    sky_texture_node.sun_direction = (0.0, 0.0, 1.0)
    
    # Add a Background node
    background_node = world_nodes.new(type='ShaderNodeBackground')
    background_node.location = (0, 0)
    background_node.inputs['Strength'].default_value = 0.3

    # Add an Output node
    output_node = world_nodes.new(type='ShaderNodeOutputWorld')
    output_node.location = (400, 0)

    # Link the Sky Texture to the Background node
    world_links.new(sky_texture_node.outputs['Color'], background_node.inputs['Color'])
    world_links.new(background_node.outputs['Background'], output_node.inputs['Surface'])

    print("Set sky texture environment background.")

def set_random_background(camera):
    # Set a generic day environment texture first
    set_day_environment_texture()
    
    # Use a random image from the backgrounds folder
    background_images = [f for f in os.listdir(BACKGROUND_DIRECTORY) if f.endswith(('.jpg', '.jpeg', '.png'))]
    if not background_images:
        print("No background images found in the folder.")
        return

    random_image_path = os.path.join(BACKGROUND_DIRECTORY, random.choice(background_images))
    
    # Create a new plane to use as a background
    bpy.ops.mesh.primitive_plane_add()
    background_plane = bpy.context.object
    background_plane.name = "BackgroundPlane"

    # Set the plane's location to be behind the camera
    plane_distance = 25  # Distance behind the camera
    camera_direction = camera.matrix_world.to_3x3() @ Vector((0.0, 0.0, -1.0))
    background_plane.location = camera.location + camera_direction * plane_distance

    # Rotate the plane to face the camera
    background_plane.rotation_euler = camera.rotation_euler

    # Scale the plane to fill the camera's view
    cam_data = camera.data
    cam_aspect_ratio = cam_data.sensor_width / cam_data.sensor_height
    scale_factor = plane_distance * (cam_data.angle / 2.0)
    background_plane.scale = (scale_factor * cam_aspect_ratio, scale_factor, 1.0)

    # Create a new material with the image texture
    mat = bpy.data.materials.new(name="BackgroundMaterial")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    # Clear existing nodes
    nodes.clear()

    # Add new nodes for the image texture
    texture_node = nodes.new(type='ShaderNodeTexImage')
    
    # Load the image texture and check for errors
    try:
        image = bpy.data.images.load(random_image_path)
        texture_node.image = image
    except RuntimeError:
        print(f"Failed to load image: {random_image_path}")
        bpy.data.objects.remove(background_plane, do_unlink=True)
        return

    texture_node.location = (-300, 0)

    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    output_node.location = (400, 0)

    principled_node = nodes.new(type='ShaderNodeBsdfPrincipled')
    principled_node.location = (0, 0)

    links.new(texture_node.outputs['Color'], principled_node.inputs['Base Color'])
    links.new(principled_node.outputs['BSDF'], output_node.inputs['Surface'])

    # Assign the material to the background plane
    background_plane.data.materials.append(mat)

    print(f"Set random image background: {random_image_path}")

def render_from_cameras(cameras, tree_object, category):
    """
    Render images from multiple camera perspectives and generate bounding boxes.

    Args:
    - cameras: A list of camera objects.
    - tree_object: The Blender object representing the tree.
    - category: The category label of the tree (e.g., non_oak, low_risk_oak, etc.)
    """
    for idx, camera in enumerate(cameras):
        set_random_background(camera)  # Apply a random background
        bpy.context.scene.camera = camera
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        bpy.context.scene.render.filepath = f"{OUTPUT_IMG_DIRECTORY}/render_camera_{idx + 1}_{timestamp}.png"
        bpy.ops.render.render(write_still=True)
        
        print(f"Rendered image from {camera.name} and saved to {bpy.context.scene.render.filepath}")
        
        # Generate bounding box text file in YOLO format
        generate_yolo_label(camera, tree_object, category, idx, timestamp)
        
        # Delete background plane and image
        bpy.data.objects.remove(bpy.data.objects["BackgroundPlane"], do_unlink=True)
        bpy.data.images.remove(bpy.data.images[-1], do_unlink=True)  # Removes the last loaded image

def calculate_trunk_boundaries(tree_object, camera):
    """
    Calculate the bounding box of the tree trunk within the image pane.

    Args:
    - tree_object: The Blender object representing the tree.
    - camera: The Blender camera object.

    Returns:
    - normalized_bbox: A list containing the normalized bounding box [x_center, y_center, width, height].
    """
    # Get the scene and its dimensions
    scene = bpy.context.scene
    render = scene.render
    resolution_x = render.resolution_x
    resolution_y = render.resolution_y

    # Get the vertices of the tree trunk in world coordinates
    tree_vertices = [tree_object.matrix_world @ v.co for v in tree_object.data.vertices]
    
    # Project the vertices to the 2D image space using the camera perspective
    vertices_2d = [bpy_extras.object_utils.world_to_camera_view(scene, camera, v) for v in tree_vertices]

    # Filter vertices that are in front of the camera
    vertices_2d = [v for v in vertices_2d if 0.0 <= v.x <= 1.0 and 0.0 <= v.y <= 1.0 and v.z >= 0.0]
    
    # Find min and max x and y coordinates from the projected 2D vertices
    if not vertices_2d:  # If no vertices are in front of the camera, return an empty bounding box
        print("No vertices in front of the camera.")
        return [0.5, 0.5, 0, 0]

    min_x = min([v.x for v in vertices_2d])
    max_x = max([v.x for v in vertices_2d])
    min_y = min([v.y for v in vertices_2d])
    max_y = max([v.y for v in vertices_2d])

    # Convert to screen space
    min_x_screen = min_x * resolution_x
    max_x_screen = max_x * resolution_x
    min_y_screen = (1 - max_y) * resolution_y  # Invert Y-axis for Blender
    max_y_screen = (1 - min_y) * resolution_y

    # Ensure the bounding box is within image bounds
    min_x_screen = max(0, min_x_screen)
    max_x_screen = min(resolution_x, max_x_screen)
    min_y_screen = max(0, min_y_screen)
    max_y_screen = min(resolution_y, max_y_screen)

    # Calculate the normalized bounding box
    x_center = (min_x_screen + max_x_screen) / (2 * resolution_x)
    y_center = (min_y_screen + max_y_screen) / (2 * resolution_y)
    width = (max_x_screen - min_x_screen) / resolution_x
    height = (max_y_screen - min_y_screen) / resolution_y

    # Return the normalized bounding box
    normalized_bbox = [x_center, y_center, width, height]
    
    print(f"Calculated bounding box: {normalized_bbox}")
    
    return normalized_bbox

def generate_yolo_label(camera, tree_object, category, idx, timestamp):
    """
    Assigns the YOLO label (x, y, width, height) to rendered image.
    
    Args:
    - camera: A Blender camera object pointing towards the trunk.
    - tree_object: A Blender Tree object with random bark.
    - category: Class distinction, i.e. low_risk_oak
    - idx: Integer corresponding to camera number.
    - timestamp: Datetime timestamp of when image was rendered.
    """
    # Calculate the bounding box of the tree trunk
    bbox = calculate_trunk_boundaries(tree_object, camera)
    
    # Get the YOLO label index for the category
    label = YOLO_LABELS[category]

    # Save the label to a text file
    label_path = f"{OUTPUT_LBL_DIRECTORY}/render_camera_{idx + 1}_{timestamp}.txt"
    with open(label_path, 'w') as f:
        f.write(f"{label} {' '.join(map(str, bbox))}\n")
    print(f"Generated YOLO label file for {camera.name} at {label_path}")
    
def create_n_random_trees(number_of_trees):
    """
    For each tree, will create renderings from the viewpoint of 1, 2, or 3 cameras.
    
    Args:
    - number_of_trees: Integer with the number of trees to generate in given run.
    """
    for i in range(number_of_trees):
        node_group_name = NODE_NAME + "_" + str(i)
        delete_all_objects()
        random.seed()
        tree, category = create_tree(node_group_name)
        n_cameras = int(random.uniform(1, 4))
        cameras = [create_random_camera(i) for i in range(n_cameras)]
        create_sun_light()
        render_from_cameras(cameras, tree, category)
        
if __name__ == "__main__":
    """Run in Blender 2.8."""
    enable_mtree_addon()
    create_n_random_trees(number_of_trees=N_TREES)