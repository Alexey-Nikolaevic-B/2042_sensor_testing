import xml.etree.ElementTree as ET

def generate_world( world_path, camera_model_path, base_world_path):
    try:
        tree = ET.parse(world_path)
        root = tree.getroot()
        world = root.find('world')
        
        camera_tree = ET.parse(camera_model_path)
        camera_root = camera_tree.getroot()
        
        camera_models = camera_root.findall('model')
        
        if not camera_models:
            print("Error: No models found in camera SDF file")
            return
        
        for i, camera_model in enumerate(camera_models):
            model_name = camera_model.get('name', f'unknown_{i}')
            world.append(camera_model)
        
        tree.write(base_world_path, encoding='utf-8', xml_declaration=True)
        
    except Exception as e:
        print(f"Error generating world: {e}")

def save_data():
    pass