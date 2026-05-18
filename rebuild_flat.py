import xml.etree.ElementTree as ET
import copy
import numpy as np
import os

def parse_pos(pos_str):
    if pos_str is None: return np.zeros(3)
    return np.fromstring(pos_str, sep=' ')

def merge_urdf():
    # Source files
    base_file = "assets/unitree/g1/g1_dual_arm.urdf"
    hand_file = "assets/unitree/g1/g1_29dof_with_hand.urdf"
    out_file = "assets/unitree/g1/g1_dual_arm_with_hand.urdf"

    tree = ET.parse(base_file)
    root = tree.getroot()
    h_tree = ET.parse(hand_file)
    h_root = h_tree.getroot()
    
    # 1. Add world base
    world = ET.Element('link', name='world')
    root.insert(0, world)
    world_joint = ET.Element('joint', name='world_to_waist_joint', type='fixed')
    ET.SubElement(world_joint, 'parent', link='world')
    ET.SubElement(world_joint, 'child', link='waist_yaw_link')
    ET.SubElement(world_joint, 'origin', xyz='0 0 0.4', rpy='0 0 0')
    root.insert(1, world_joint)
    
    # 2. Cleanup legacy links/joints from base URDF
    to_remove = []
    for child in list(root):
        name = child.get('name', '')
        if 'rubber_hand' in name:
            to_remove.append(child)
            continue
        if child.tag == 'joint':
            c_link = child.find('child')
            if c_link is not None and 'rubber_hand' in c_link.get('link', ''):
                to_remove.append(child)
    for e in to_remove: root.remove(e)

    # 3. Add dexterous components
    added_names = set()
    for e in h_root:
        name = e.get('name', '')
        if 'hand' in name and 'rubber' not in name:
            if name in added_names: continue
            new_e = copy.deepcopy(e)
            if e.tag == 'joint' and 'palm_joint' in name:
                side = 'left' if 'left' in name else 'right'
                new_e.find('parent').set('link', f'{side}_wrist_yaw_link')
            root.append(new_e)
            added_names.add(name)
            
    tree.write(out_file)
    print(f"Successfully generated URDF: {out_file}")

def merge_xml():
    tree = ET.parse("assets/unitree/g1/g1_dual_arm.xml")
    root = tree.getroot()
    h_tree = ET.parse("assets/unitree/g1/g1_29dof_with_hand.xml")
    h_root = h_tree.getroot()
    
    # 1. Asset setup
    asset = root.find('asset')
    for m in [m for m in list(asset) if 'rubber_hand' in m.get('name', '')]: asset.remove(m)
    for m in h_root.find('asset').findall('mesh'):
        if 'hand' in m.get('name') and 'rubber' not in m.get('name'): asset.append(copy.deepcopy(m))
    ET.SubElement(asset, 'texture', type="skybox", builtin="flat", rgb1="0 0 0", rgb2="0 0 0", width="512", height="3072")

    # 2. Worldbody Shift
    wb = root.find('worldbody')
    robot_root = ET.Element('body', name='robot_root', pos='0 0 0.4')
    to_move = [e for e in list(wb) if e.tag in ['geom', 'body'] and e.get('name') != 'floor']
    for e in to_move:
        wb.remove(e)
        robot_root.append(e)
    wb.append(robot_root)

    # Lock waist joints to ensure torso stays vertical
    for joint in robot_root.findall('.//joint'):
        if 'waist' in joint.get('name', ''):
            joint.set('type', 'fixed')

    # 3. Attach Fingers with corrected relative positions
    for side in ['left', 'right']:
        wrist = robot_root.find(f".//body[@name='{side}_wrist_yaw_link']")
        for g in [g for g in list(wrist) if g.tag == 'geom' and 'rubber_hand' in g.get('mesh', '')]: wrist.remove(g)
        
        palm_offset = np.array([0.0415, 0.003, 0]) if side == 'left' else np.array([0.0415, -0.003, 0])
        palm = ET.SubElement(wrist, 'body', name=f'{side}_hand_palm_link', pos=' '.join(map(str, palm_offset)))
        
        h_wrist = h_root.find(f".//body[@name='{side}_wrist_yaw_link']")
        for child in list(h_wrist):
            if child.tag == 'geom' and 'palm' in child.get('mesh', ''):
                new_c = copy.deepcopy(child)
                new_c.set('pos', ' '.join(map(str, parse_pos(child.get('pos')) - palm_offset)))
                palm.append(new_c)
            elif child.tag == 'body' and 'hand' in child.get('name', ''):
                new_c = copy.deepcopy(child)
                new_c.set('pos', ' '.join(map(str, parse_pos(child.get('pos')) - palm_offset)))
                palm.append(new_c)
                
    # 4. Environment
    table = ET.SubElement(wb, 'body', name='table', pos='0.5 0 0.4')
    ET.SubElement(table, 'geom', type='box', size='0.3 0.6 0.02', rgba='0.4 0.2 0.1 1', conaffinity='1')
    cube = ET.SubElement(wb, 'body', name='cube', pos='0.5 0 0.445')
    ET.SubElement(cube, 'freejoint', name='cube_joint')
    ET.SubElement(cube, 'geom', type='box', size='0.025 0.025 0.025', rgba='0 1 0 1', mass='0.1', friction='1 1 1')
    
    for side, y in [('left', '0.2'), ('right', '-0.2')]:
        t = ET.SubElement(wb, 'body', name=f'{side}_target', mocap='true', pos=f'0.4 {y} 0.6')
        # Thicker and longer RGB axes for better visibility
        ET.SubElement(t, 'geom', type='cylinder', size='0.006 0.06', pos='0.06 0 0', quat='0.7071 0 0.7071 0', rgba='1 0 0 0.8', contype='0', conaffinity='0') # X
        ET.SubElement(t, 'geom', type='cylinder', size='0.006 0.06', pos='0 0.06 0', quat='0.7071 -0.7071 0 0', rgba='0 1 0 0.8', contype='0', conaffinity='0') # Y
        ET.SubElement(t, 'geom', type='cylinder', size='0.006 0.06', pos='0 0 0.06', quat='1 0 0 0', rgba='0 0 1 0.8', contype='0', conaffinity='0') # Z
        
        tr = ET.SubElement(wb, 'body', name=f'{side}_tracker_target', mocap='true', pos=f'0.4 {y} 0.6')
        ET.SubElement(tr, 'geom', type='sphere', size='0.025', rgba='1 0.5 0 0.5' if side=='left' else '0 1 0.5 0.5', contype='0', conaffinity='0')

    # 5. High-Gain Position Actuators and Physics Stabilization
    default = root.find('default')
    if default is None: default = ET.SubElement(root, 'default')
    # Physics stabilization: armature and damping
    ET.SubElement(default, 'geom', contype='1', conaffinity='0')
    ET.SubElement(default, 'joint', damping="2.0", armature="0.2", frictionloss="0.2")
    
    act = root.find('actuator')
    for m in list(act): act.remove(m)
    for side in ['left', 'right']:
        # High gain for arms to maintain "torque" (stiffness)
        for j in ['shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw']:
            ET.SubElement(act, 'position', name=f'{side}_{j}_joint', joint=f'{side}_{j}_joint', kp='800', kv='30')
        # Fingers also need decent gains to hold items
        for f in ['thumb_0', 'thumb_1', 'thumb_2', 'middle_0', 'middle_1', 'index_0', 'index_1']:
            ET.SubElement(act, 'position', name=f'{side}_hand_{f}_joint', joint=f'{side}_hand_{f}_joint', kp='20', kv='1.0')

    tree.write("assets/unitree/g1/g1_dual_arm_with_hand.xml")
    print("Successfully generated XML with high-gain actuators.")

if __name__ == '__main__':
    merge_urdf()
    merge_xml()
