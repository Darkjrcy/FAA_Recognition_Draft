from asyncio import new_event_loop
import xml.etree.ElementTree as ET
import os
import re

def patch_dae_with_textures(dae_path,textures_dir,output_path):
    dae_tree = ET.parse(dae_path)
    root = dae_tree.getroot()

    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    ET.register_namespace('', ns['c'])  # Keep namespaces clean

    # 1. Add <library_images> with all textures
    lib_images = root.find('c:library_images', ns)
    if lib_images is None:
        lib_images = ET.SubElement(root, 'library_images')
    else:
        lib_images.clear()
    
    list_of_needed_images = []
    # Check which effects are inside the dae file:
    for effect in root.findall('.//c:effect', ns):
        effect_id = effect.attrib['id'].replace('-effect', '')
        needed_image = re.split(r'_png',effect_id)
        # ELiminate the rist _ int he effecto combination:
        if len(needed_image) >= 3:
            if needed_image[1].startswith('_'):
                needed_image[1] = needed_image[1][1:]
        # Take out the _jpg letters:
        for i in range(len(needed_image)):
            if needed_image[i].endswith('_jpg'):
                needed_image[i] = re.split(r'_jpg',needed_image[i])[0]
            # Add the needed image to the list in case is not repeated:
            if needed_image[i] not in list_of_needed_images:
                list_of_needed_images.append(needed_image[i])
    
    # Add all the textures images into the dae file:
    texture_files = [f for f in os.listdir(textures_dir) if f.endswith(('.jpg', '.png'))]
    for tex in texture_files:
        tex_name = re.split(r'.jpg',tex)[0]
        tex_name = re.split(r'.png',tex_name)[0]
        if tex_name in list_of_needed_images:
            image = ET.SubElement(lib_images,'image',id=os.path.splitext(tex)[0])
            init_from = ET.SubElement(image, 'init_from')
            init_from.text = f'materials/textures/{tex}'
    # Add the image ovberlay:
    image_overlay = ET.SubElement(lib_images,'image',id="overlay_png",name="overlay_png")
    init_from = ET.SubElement(image_overlay,'init_from')
    init_from.text = f'overlay.png'
    
    # Modify the effect to add the images:
    for effect in root.findall('.//c:effect', ns):
        profile = effect.find('c:profile_COMMON', ns)
        if profile is None:
            continue
        technique = profile.find('c:technique', ns)
        if technique is None:
            continue
        lambert = technique.find('c:lambert', ns)
        if lambert is None:
            continue
        diffuse = lambert.find('c:diffuse', ns)
        if diffuse is None:
            continue
        # Restart the needed textures:
        needed_textures = []
        # Check the needed textures for each effect:
        effect_id = effect.attrib['id'].replace('-effect', '')
        needed_image = re.split(r'_png',effect_id)
        # ELiminate the rist _ int he effecto combination:
        if len(needed_image) >= 3:
            if needed_image[1].startswith('_'):
                needed_image[1] = needed_image[1][1:]
        # Take out the _jpg letters:
        for i in range(len(needed_image)):
            if needed_image[i].endswith('_jpg'):
                needed_image[i] = re.split(r'_jpg',needed_image[i])[0]
            # Add the needed image to the list in case is not repeated:
            if needed_image[i] not in needed_textures:
                needed_textures.append(needed_image[i])
        needed_textures = [item for item in needed_textures if item]
        # Add the textures:
        if len(needed_textures) == 1:
            # Remove <color> and <texture>:
            color = diffuse.find('c:color', ns)
            if color is not None:
                diffuse.remove(color)
                sampler_id = f'{needed_textures[0]}_sampler'
                # Add the surface and Sampler 2D:
                surface = ET.SubElement(profile,'newparam',sid=f'{needed_textures[0]}_surface')
                surface_elem = ET.SubElement(surface, 'surface', type='2D')
                init_from = ET.SubElement(surface_elem, 'init_from')
                init_from.text = needed_textures[0]
                # Add the source of the model
                sampler = ET.SubElement(profile, 'newparam', sid=sampler_id)
                sampler2D = ET.SubElement(sampler, 'sampler2D')
                source = ET.SubElement(sampler2D, 'source')
                source.text = f'{needed_textures[0]}_surface'
                # Replace diffuse with texture
                texture = ET.SubElement(diffuse, 'texture', texture=sampler_id, texcoord='UVSET0')
        elif len(needed_textures) == 2:
            # Remove <color> and <texture>:
            color = diffuse.find('c:color', ns)
            if color is not None:
                # Add the winsows
                sampler_id = f'{needed_textures[0]}_sampler'
                # Add the surface and Sampler 2D:
                surface_1 = ET.SubElement(profile,'newparam',sid=f'{needed_textures[0]}_surface')
                surface_elem_1 = ET.SubElement(surface_1, 'surface', type='2D')
                init_from_1 = ET.SubElement(surface_elem_1, 'init_from')
                init_from_1.text = needed_textures[0]
                # Add the source of the model
                sampler_1 = ET.SubElement(profile, 'newparam', sid=sampler_id)
                sampler2D_1 = ET.SubElement(sampler_1, 'sampler2D')
                source_1 = ET.SubElement(sampler2D_1, 'source')
                source_1.text = f'{needed_textures[0]}_surface'
                
                # Add the material texture:
                material_id = f'{needed_textures[1]}_sampler'
                # Add the surface and Sampler 2D:
                surface_2 = ET.SubElement(profile,'newparam',sid=f'{needed_textures[1]}_surface')
                surface_elem_2 = ET.SubElement(surface_2, 'surface', type='2D')
                init_from_2 = ET.SubElement(surface_elem_2, 'init_from')
                init_from_2.text = needed_textures[1]
                # Add the source of the model
                sampler_2 = ET.SubElement(profile, 'newparam', sid=material_id)
                sampler2D_2 = ET.SubElement(sampler_2, 'sampler2D')
                source_2 = ET.SubElement(sampler2D_2, 'source')
                source_2.text = f'{needed_textures[1]}_surface'

                # --- Update the diffuse element in the lambert technique ---
                technique = profile.find("c:technique", ns)
                lambert = technique.find("c:lambert", ns)
                diffuse = lambert.find("c:diffuse", ns)

                # Remove color
                diffuse.remove(color)

                # Replace diffuse with texture
                texture = ET.SubElement(diffuse, 'texture', texture=sampler_id, texcoord='UVSET0')
                # Replace diffuse with texture
                texture = ET.SubElement(diffuse, 'texture', texture=material_id, texcoord='UVSET1')
    
    # 3. Write updated file
    dae_tree.write(output_path, encoding='utf-8', xml_declaration=True)
    print(f"Updated DAE saved to: {output_path}")








patch_dae_with_textures(
    dae_path = "/home/adcl/AirplanePathFollower/src/plane_bringup/models/MIT_city/Section3/Section3.dae",
    textures_dir = "/home/adcl/AirplanePathFollower/src/plane_bringup/models/MIT_city/Section3/materials/textures",
    output_path = "/home/adcl/AirplanePathFollower/src/plane_bringup/models/MIT_city/Section3/Section3.dae"

)