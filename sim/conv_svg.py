#!/usr/bin/env python2.7
import xml.etree.ElementTree as etree
import argparse
import collections
import gen_osg

def find_recursive(element, name, namespace):
    r = []
    for e in element:
        e.parent = element
        r.extend(find_recursive(e, name, namespace))
    r.extend(element.findall(name, namespace))
    return r

def chunk(l,n):
    for i in range(0,len(l),n):
        yield l[i:i+n]

def curr_next(l):
    for i in range(0, len(l) - 1):
        yield l[i], l[i+1]

def interleave(*args):
    for i in range(min((len(x) for x in args))):
        for l in args:
            yield l[i]

def rel_wrapper(func):
    def rel_func(self, *args):
        return func(self, *[(self.pos[0] + p[0],
                             self.pos[1] + p[1]) for p in args])
    return rel_func

def dist2(p1, p2):
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

class SVGPath:
    def __init__(self):
        self.pos = (0, 0)
        self.points = []
        self.depth = 0
        self.commands = {
            "M" : (1, self.move_abs),
            "m" : (1, self.move_rel),
            "L" : (1, self.lineto_abs),
            "l" : (1, self.lineto_rel),
            "C" : (3, self.curve_abs),
            "c" : (3, self.curve_rel),
            "A" : (5, self.arc_abs),
            "a" : (5, self.arc_rel)
        }


    def move_abs(self, pos):
        self.pos = pos
        self.points.append(pos)

    def lineto_abs(self, pos):
        self.move_abs(pos)

    def curve_abs(self, ctrl_1, ctrl_2 ,pos):
        self.move_abs(pos)

    def arc_abs(self, rad, x_rot, large_arc, sweep, pos):
        self.move_abs(pos)

    def arc_rel(self, rad, x_rot, large_arc, sweep, pos):
        self.move_rel(pos)

    def closest_distance(self, pos):
        curr_dist = float('+inf')
        for p in self.points:
            dist = dist2(p, pos)**0.5
            curr_dist = min(dist, curr_dist)
        for p1, p2 in curr_next(self.points):
            if p1 == p2:
                continue
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[0]
            u = (pos[0] - p1[0])*dx + (pos[1] - p1[1])*dy
            u /= (dx**2 +  dy**2)
            if u > 0 and u < 1:
                closest = (p1[0] + u*dx, p1[1] + u*dy)
                dist = dist2(pos, closest)**0.5
                curr_dist = min(dist, curr_dist)
        return curr_dist

    def apply_transform(self, transform):
        self.points = [transform.apply(x) for x in self.points]

    def __repr__(self):
        return repr(self.points)

    move_rel = rel_wrapper(move_abs) 
    lineto_rel = rel_wrapper(lineto_abs) 
    curve_rel = rel_wrapper(curve_abs) 

class SVGCircle:
    def __init__(self, x, y, rx, ry):
        self.pos = (x, y)
        self.rx = rx
        self.ry = ry
        self.depth = 0

    def closest_distance(self, pos):
        return dist2(pos, self.pos)**0.5
    
    def apply_transform(self, transform):
        sx_pos = transform.apply((0,1))
        sy_pos = transform.apply((1,0))
        null_pos = transform.apply((0,0))
        self.rx /= dist2(null_pos, sx_pos)**0.5
        self.ry /= dist2(null_pos, sy_pos)**0.5
        self.pos = transform.apply(self.pos)

    def __repr__(self):
        return repr(self.pos)

class MatrixTransform:
    def __init__(self, mat):
        self.mat = mat

    def apply(self, pos):
        pos = (pos[0], pos[1], 1)
        x = sum((pos[i] * self.mat[0][i] for i in range(3)))
        y = sum((pos[i] * self.mat[1][i] for i in range(3)))
        z = sum((pos[i] * (0,0,1)[i] for i in range(3)))
        return x/z,y/z
    
    def __repr__(self):
        return '\n'.join((' '.join((str(x) for x in self.mat[i])) for i in (0,1)))

class TranslateTransform:
    def __init__(self, dx, dy):
        self.dx = dx
        self.dy = dy

    def apply(self, pos):
        return pos[0] + self.dx, pos[1] + self.dy

class TransformGroup(list):
    def apply(self, pos):
        for t in self:
            pos = t.apply(pos)
        return pos

def parse_transform_str(string):
    values = [float(x) for x in string.split("(")[1].split(")")[0].split(",")]
    if string.startswith("matrix"):
        mat = zip(values[:2],values[2:4],values[4:])
        return MatrixTransform(mat)
    if string.startswith("translate"):
        return TranslateTransform(values[0], values[1])
    else:
        raise RuntimeError("Unknown transform type!")

def get_transforms(element):
    transforms = TransformGroup()
    if hasattr(element, "parent"):
        transforms.extend(get_transforms(element.parent))
    if element.get("transform"):
        transforms.append(parse_transform_str(element.get("transform")))
    return transforms

def parse_path_str(path_str):
    commands = []
    current_cmd = "M"
    current_list = []
    for p in path_str.split():
        if len(p) == 1 and p.isalpha():
            current_cmd = p
            current_list = []
            commands.append((p, current_list))
        else:
            current_list.append(tuple((float(x) for x in p.split(","))))
    path = SVGPath()
    for cmd in commands:
        if cmd[0] in path.commands:
            n_args, func = path.commands[cmd[0]]
            for args in chunk(cmd[1],n_args):
                func(*args)
    return path

def parse_style_str(style_str):
    return {x.split(":")[0]: x.split(":")[1] for x in style_str.split(";")}

def parse_path(element):
    s = "{{{}}}".format(n['sodipodi'])
    if element.get(s+"type"):
        ret = SVGCircle(*[float(element.get(s+x)) for x in ["cx", "cy", "rx" ,"ry"]])
    else:
        ret = parse_path_str(element.get("d"))
    ret.style = parse_style_str(element.get("style"))
    return ret

p = argparse.ArgumentParser(description="Convert an SVG into a osg file")

p.add_argument("--file", "-f", help="SVG file to convert", required = True)
p.add_argument("--out", "-o", help="osgt file to output")

args = p.parse_args()

with open(args.file) as svg_file:
    svg_tree = etree.fromstring(svg_file.read())

n = {"svg":"http://www.w3.org/2000/svg",
     "sodipodi": "http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"}

path_elems = find_recursive(svg_tree, "svg:path", n)

group_dict = collections.defaultdict(set)

paths = []

for p in path_elems:
    g_id = p.parent.get("id")
    path = parse_path(p)
    path.peers = group_dict[g_id]
    path.peers.add(path)
    paths.append(path)
    #print(path, get_transforms(p))
    path.apply_transform(get_transforms(p))

labels = find_recursive(svg_tree, "svg:text", n)

for t in labels:
    text = ''.join(t.itertext()).strip()
    try:
        if text.endswith('m'):
            transform = get_transforms(t)
            pos = (float(t.get('x')), float(t.get('y')))
            pos = transform.apply(pos)
            depth = float(text[:-1])
        else:
            raise ValueError()
    except ValueError:
        continue
    closest_path = min(((p.closest_distance(pos), p) for p in paths), key = lambda x: x[0])[1]
    closest_path.depth = depth
    print(depth, closest_path)

geometry_strings = []
for path in paths:
    if isinstance(path, SVGCircle):
        radius = (path.rx + path.ry)/2
        colour = path.style['fill']
        if colour.startswith('#'):
            colour = tuple((int(x,16)/255.0 for x in chunk(colour[1:],2)))
        else:
            colour = (0,0,0)
        geometry_strings.append(gen_osg.get_buoy_string((path.pos[0]/10, path.pos[1]/10, -path.depth), colour, radius/10))
    elif isinstance(path, SVGPath):
        peer_paths = [peer for peer in path.peers if isinstance(peer, SVGPath)]
        if len(peer_paths) == 2:
            p1 = peer_paths[0]
            p2 = peer_paths[1]
            if len(p1.points) != len(p2.points):
                continue
        else:
            continue
        p1_points = [(p[0]/10, p[1]/10, -p1.depth) for p in p1.points]
        p2_points = [(p[0]/10, p[1]/10, -p2.depth) for p in p2.points]
        path.peers.clear()
        geometry_strings.append(gen_osg.get_wall_string(list(interleave(p1_points, p2_points))))

output = gen_osg.file_template.format(
            len=1, geodes = gen_osg.indent(
                        gen_osg.get_geode_string(
                            geometry_strings)))

print(output)

with open(args.out,"w") as output_file:
    output_file.write(output)
