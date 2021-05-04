from math import pi
import argparse

parser = argparse.ArgumentParser(description=
                                 'Create a box model with holes on 4 faces')
parser.add_argument('nx', type=int, help='number of holes in x direction')
parser.add_argument('ny', type=int, help='number of holes in y direction')
parser.add_argument('nz', type=int, help='number of holes in z direction')
parser.add_argument('spacing', type=float, help='spacing of the holes')

args = parser.parse_args()
print("nx={}, ny={}, nz={}, spacing={}".format(args.nx,args.ny,args.nz,
                                               args.spacing))
# Some constant values
epsilon = 0.01
clearance = 0.05

X = (args.nx+1) * args.spacing
Y = (args.ny+1) * args.spacing
Z = (args.nz+1) * args.spacing

urdf = """<?xml version="1.0"?>
<robot name="part"> 
  <link name="base_link">
    <visual>
      <geometry>
        <box size="{X} {Y} {Z}"/>
      </geometry>
      <origin xyz="0 0 {Z_2}"/>
      <material name="Red">
        <color rgba="1 0.1 0.1 0.6"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="{X} {Y} {Z}"/>
      </geometry>
      <origin xyz="0 0 {Z_2}"/>
    </collision>
    <visual>
      <geometry>
        <box size="{X} {Y} 0.8"/>
      </geometry>
      <origin xyz="0 0 -0.4"/>
      <material name="Grey">
        <color rgba="0.8 0.8 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="{X} {Y} 0.8"/>
      </geometry>
      <origin xyz="0 0 -0.4"/>
    </collision>
  </link>
</robot>
""".format(X=X, Y=Y, Z=Z, Z_2=.5*Z)

with open('./part.urdf', 'w') as f:
    f.write(urdf)

with open('./part.srdf', 'w') as f:
    iHandle=0
    f.write('<robot name="part">\n')
    # First face y = -Y/2
    y = -.5*Y - epsilon
    rpy = '"0 0 {}"'.format(pi/2)
    for i in range(args.nx):
        x = -.5*X + (i+1)*args.spacing
        for j in range(args.nz):
            z = (j+1)*args.spacing
            xyz = '"{} {} {}"'.format(x,y,z)
            f.write('<handle name="handle_{h:04}" clearance="{clearance}">\n'.\
                    format(h=iHandle, clearance=clearance))
            f.write('    <position xyz={} rpy={}/>\n'.format(xyz, rpy))
            f.write('    <link name="base_link"/>\n')
            f.write('    <mask>1 1 1 0 1 1</mask>\n')
            f.write('</handle>\n\n')
            iHandle += 1
            
    # Second face x = X/2
    x = .5*X + epsilon
    rpy = '"0 0 {}"'.format(pi)
    for i in range(args.ny):
        y = -.5*Y + (i+1)*args.spacing
        for j in range(args.nz):
            z = (j+1)*args.spacing
            xyz = '"{} {} {}"'.format(x,y,z)
            f.write('<handle name="handle_{h:04}" clearance="{clearance}">\n'.\
                    format(h=iHandle, clearance=clearance))
            f.write('    <position xyz={} rpy={}/>\n'.format(xyz, rpy))
            f.write('    <link name="base_link"/>\n')
            f.write('    <mask>1 1 1 0 1 1</mask>\n')
            f.write('</handle>\n\n')
            iHandle += 1

    # Third face y = Y/2
    y = .5*Y + epsilon
    rpy = '"0 0 {}"'.format(-pi/2)
    for i in range(args.nx):
        x = -.5*X + (i+1)*args.spacing
        for j in range(args.nz):
            z = (j+1)*args.spacing
            xyz = '"{} {} {}"'.format(x,y,z)
            f.write('<handle name="handle_{h:04}" clearance="{clearance}">\n'.\
                    format(h=iHandle, clearance=clearance))
            f.write('    <position xyz={} rpy={}/>\n'.format(xyz, rpy))
            f.write('    <link name="base_link"/>\n')
            f.write('    <mask>1 1 1 0 1 1</mask>\n')
            f.write('</handle>\n\n')
            iHandle += 1

    # Fourth face x = X/2
    x = -.5*X - epsilon
    rpy = '"0 0 {}"'.format(0)
    for i in range(args.ny):
        y = -.5*Y + (i+1)*args.spacing
        for j in range(args.nz):
            z = (j+1)*args.spacing
            xyz = '"{} {} {}"'.format(x,y,z)
            f.write('<handle name="handle_{h:04}" clearance="{clearance}">\n'.\
                    format(h=iHandle, clearance=clearance))
            f.write('    <position xyz={} rpy={}/>\n'.format(xyz, rpy))
            f.write('    <link name="base_link"/>\n')
            f.write('    <mask>1 1 1 0 1 1</mask>\n')
            f.write('</handle>\n\n')
            iHandle += 1

    f.write('</robot>\n')
