"""Shared static scene source, with no ROS dependency."""
import math
import xml.etree.ElementTree as ET
import yaml


def load_objects(path):
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get('objects'), list):
        raise ValueError('Scene must have an objects list')
    objects = []
    names = set()
    for item in data['objects']:
        name = item.get('id')
        size = item.get('size', [])
        position = item.get('position', [])
        if (not isinstance(name, str) or not name or name in names
                or len(size) != 3 or len(position) != 3
                or not all(isinstance(v, (int, float)) and math.isfinite(v)
                           for v in (*size, *position))
                or not all(v > 0 for v in size)):
            raise ValueError('Invalid or duplicate scene object: ' + str(name))
        names.add(name)
        objects.append({'id': name, 'size': list(map(float, size)),
                        'position': list(map(float, position))})
    return objects


def world_xml(objects):
    root = ET.Element('sdf', version='1.6')
    world = ET.SubElement(root, 'world', name='omx_workcell')
    ET.SubElement(world, 'gravity').text = '0 0 -9.81'
    physics = ET.SubElement(world, 'physics', name='default', type='ode')
    ET.SubElement(physics, 'max_step_size').text = '0.001'
    ET.SubElement(physics, 'real_time_update_rate').text = '1000'
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(light, 'diffuse').text = '0.8 0.8 0.8 1'
    ET.SubElement(light, 'direction').text = '-0.5 0.1 -0.9'
    for item in objects:
        model = ET.SubElement(world, 'model', name=item['id'])
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = ' '.join(map(str, (*item['position'], 0, 0, 0)))
        link = ET.SubElement(model, 'link', name='body')
        for kind in ('collision', 'visual'):
            node = ET.SubElement(link, kind, name=kind)
            geometry = ET.SubElement(node, 'geometry')
            box = ET.SubElement(geometry, 'box')
            ET.SubElement(box, 'size').text = ' '.join(map(str, item['size']))
    return ET.tostring(root, encoding='unicode')
