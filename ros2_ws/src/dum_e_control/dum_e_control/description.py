"""Generate a schematic URDF using the same standard-DH geometry as the solver."""
from math import radians
from .core.kinematics import Robot


def robot_description(path=''):
    robot = Robot.load(path) if path else Robot.load()
    parts = ['<robot name="dum_e">', '''<material name="arm"><color rgba="0.15 0.55 0.85 1"/></material>
    <link name="base_link"><visual><geometry><cylinder radius="0.04" length="0.015"/></geometry>
    <material name="arm"/></visual></link>''']
    parent = 'base_link'
    for index, (a, d, alpha, limits) in enumerate(zip(robot.a, robot.d, robot.alpha, robot.dh_limits), 1):
        a, d = a / 1000.0, d / 1000.0
        parts.append(f'''
        <link name="rotor_{index}"><visual><geometry><sphere radius="0.012"/></geometry>
        <material name="arm"/></visual></link>
        <joint name="joint_{index}" type="revolute">
          <parent link="{parent}"/><child link="rotor_{index}"/>
          <origin xyz="0 0 {d}"/><axis xyz="0 0 1"/>
          <limit lower="{radians(limits[0])}" upper="{radians(limits[1])}" effort="1" velocity="0.872665"/>
        </joint>
        <link name="link_{index}"><visual>
          <origin xyz="{-a/2} 0 0" rpy="0 1.5707963267948966 0"/>
          <geometry><cylinder radius="0.008" length="{max(a, 0.001)}"/></geometry>
          <material name="arm"/></visual></link>
        <joint name="dh_fixed_{index}" type="fixed">
          <parent link="rotor_{index}"/><child link="link_{index}"/>
          <origin xyz="{a} 0 0" rpy="{radians(alpha)} 0 0"/>
        </joint>''')
        parent = f'link_{index}'
    parts.append(f'<link name="tool0"/><joint name="tool_fixed" type="fixed"><parent link="{parent}"/><child link="tool0"/></joint></robot>')
    return '\n'.join(parts)
