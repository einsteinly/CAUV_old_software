#!/usr/bin/env python2.7
import argparse

def indent(string, prefix = "    "):
    return '\n'.join((prefix + s for s in string.split("\n")))

buoy_template = \
"""osg::ShapeDrawable {{
    UniqueID {id} 
    DataVariance STATIC 
    Shape TRUE {{
        osg::Sphere {{
            UniqueID 10{id} 
            DataVariance STATIC 
            Center {x} {y} {z}
            Radius {R} 
        }}
    }}
  Color {r} {g} {b} 1 
}}"""

wall_template = \
"""osg::Geometry {{
    UniqueID {id}
    DataVariance STATIC
    PrimitiveSetList 1 {{
        DrawArrays GL_TRIANGLES 0 {len}
    }}
    VertexData {{
        Array TRUE ArrayID {id} Vec3fArray {len} {{
             {points}
        }}
        Indices FALSE 
        Binding BIND_PER_VERTEX 
        Normalize 0 
    }}
    NormalData {{
        Array TRUE ArrayID 1{id} Vec3fArray {nlen} {{
             {normals}
        }}
        Indices FALSE 
        Binding BIND_PER_VERTEX 
        Normalize 0 
    }}
    ColorData {{
        Array TRUE ArrayID 2{id} Vec4fArray {len} {{
            {colors}
        }}
        Indices FALSE 
        Binding BIND_PER_VERTEX 
        Normalize 0 
    }}
}}"""

asdf = """
      StateSet TRUE {{
        osg::StateSet {{
          UniqueID 10{id} 
          DataVariance STATIC 
          ModeList 1 {{
            GL_LIGHTING ON 
          }}
          AttributeList 2 {{
            osg::Material {{
              UniqueID 5 
              DataVariance STATIC 
              Ambient TRUE Front 0.0 0.2 0.2 1 Back 0.0 0.2 0.2 1 
              Diffuse TRUE Front 0.0 0.8 0.8 1 Back 0.0 0.8 0.8 1 
              Specular TRUE Front 0 0 0 1 Back 0 0 0 1 
              Emission TRUE Front 0 0 0 1 Back 0 0 0 1 
              Shininess TRUE Front 0 Back 0 
            }}
            Value OFF 
            osg::ShadeModel {{
              UniqueID 6 
              DataVariance STATIC 
            }}
            Value OFF 
          }}
          RenderingHint 1 
          BinName "RenderBin" 
        }}
      }}
"""

geode_template = \
"""osg::Geode {{
    UniqueID {id}
    DataVariance STATIC
    Drawables {len} {{
        {shapes}
    }}

}}"""

#'#' doesn't mean comment in osg file format
file_template = \
"""#Ascii Scene
#Version 78
#Generator OpenSceneGraph 2.9.17
osg::Group {{
    UniqueID 1
    Children {len} {{
        {geodes}
    }}
}}"""

ids = iter(xrange(10,100000))

def three_iter(l):
    for i in range(0, len(l) - 2):
        yield l[i], l[i+1], l[i+2]

def get_buoy_string(pos = (0,0,0), col = (0,0,0), radius = 10):
    return buoy_template.format(id = ids.next(),
            x = pos[0], y = pos[1], z = pos[2],
            r = col[0], g = col[1], b = col[2],
            R = radius)

def get_wall_string(wall_points):
    normals = []
    points = []
    for p1, p2, p3 in three_iter(wall_points):
        v1 = tuple((p3[i] - p2[i] for i in range(3)))
        v2 = tuple((p3[i] - p1[i] for i in range(3)))
        normal = (v1[1] * v2[2] - v1[2] * v2[1],
                  v1[2] * v2[0] - v1[0] * v2[2],
                  v1[0] * v2[1] - v1[1] * v2[0])
        normals.extend([normal] * 3)
        points.extend([p1, p2, p3])
    unit_normals = []
    for i,n in enumerate(normals):
        scale = 1/(sum((x**2 for x in n))**0.5)
        if not i/3 % 2:
            scale *= -1
        n = tuple((i * scale for i in n))
        unit_normals.append(n)
    return wall_template.format(id = ids.next(), len = len(points), nlen = len(unit_normals),
            points = '\n'.join((' '.join((str(y) for y in x)) for x in points)),
            normals = '\n'.join((' '.join((str(y) for y in x)) for x in unit_normals)),
            colors = '\n'.join(['0.2 0.2 0.2 1'] * len(points)))

def get_geode_string(shapes):
    return geode_template.format(id = ids.next(), len = len(shapes), 
            shapes = '\n'.join((indent(x,2*"    ") for x in shapes)))


if __name__ == "__main__":
    print(file_template.format(len=1, geodes = indent(get_geode_string([get_wall_string([(0,0,0), (0,1,0), (0,1,1), (1,1,1)])]))))
    #print(file_template.format(len=1, geodes = indent(get_geode_string([get_buoy_string()]))))
