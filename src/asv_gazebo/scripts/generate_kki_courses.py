#!/usr/bin/env python3
import math
from pathlib import Path


ROOT = Path("/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_gazebo")
WORLDS = ROOT / "worlds"
MODELS = ROOT / "models"
NAV_CONFIG = Path("/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_navigation/config")
COURSE_HALF_SIZE = 15.0
GATE_SPACING = 2.0


def file_uri(path):
    return f"file://{path}"


def include(name, model, pose):
    x, y, z, roll, pitch, yaw = pose
    return (
        f'    <include><name>{name}</name><pose>{x:.3f} {y:.3f} {z:.3f} '
        f'{roll:.6f} {pitch:.6f} {yaw:.6f}</pose>'
        f"<uri>{file_uri(MODELS / model)}</uri></include>\n"
    )


def dash_visuals(points):
    out = []
    dash = 0.55
    gap = 0.42
    width = 0.075
    index = 0
    for a, b in zip(points, points[1:]):
        ax, ay = a
        bx, by = b
        dx = bx - ax
        dy = by - ay
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            continue
        yaw = math.atan2(dy, dx)
        ux = dx / length
        uy = dy / length
        pos = 0.0
        while pos < length:
            seg_len = min(dash, length - pos)
            center = pos + seg_len / 2.0
            x = ax + ux * center
            y = ay + uy * center
            out.append(
                f'        <visual name="track_dash_{index:03d}">\n'
                f"          <cast_shadows>false</cast_shadows>\n"
                f"          <pose>{x:.3f} {y:.3f} 0.055 0 0 {yaw:.6f}</pose>\n"
                f"          <geometry><box><size>{seg_len:.3f} {width:.3f} 0.035</size></box></geometry>\n"
                f"          <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>\n"
                f"        </visual>\n"
            )
            index += 1
            pos += dash + gap
    return "".join(out)


def wave_crest_visuals():
    out = []
    index = 0
    x_start = -19.0
    x_end = 19.0
    step = 1.85
    rows = [value * 2.1 for value in range(-9, 10)]
    for row_index, row_y in enumerate(rows):
        x = x_start
        while x < x_end:
            phase = row_index * 0.55
            y = row_y + 0.24 * math.sin(0.72 * x + phase)
            dy_dx = 0.24 * 0.72 * math.cos(0.72 * x + phase)
            yaw = math.atan2(dy_dx, 1.0)
            seg_len = 1.15
            out.append(
                f'        <visual name="wave_crest_{index:03d}">\n'
                f"          <cast_shadows>false</cast_shadows>\n"
                f"          <pose>{x:.3f} {y:.3f} 0.018 0 0 {yaw:.6f}</pose>\n"
                f"          <geometry><box><size>{seg_len:.3f} 0.035 0.010</size></box></geometry>\n"
                f"          <material>\n"
                f"            <ambient>0.78 0.94 1.00 0.42</ambient>\n"
                f"            <diffuse>0.78 0.94 1.00 0.42</diffuse>\n"
                f"            <emissive>0.03 0.05 0.06 1</emissive>\n"
                f"          </material>\n"
                f"        </visual>\n"
            )
            index += 1
            x += step
    return "".join(out)


def base_world(name, title, path_points, includes):
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="kki_4ms" type="dart">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>1.470000</latitude_deg>
      <longitude_deg>102.110000</longitude_deg>
      <elevation>0.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
      <background_color>0.70 0.82 0.92</background_color>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <!-- The ASV is driven by a planar pose controller; wave water stays visual. -->

    <scene>
      <ambient>0.75 0.78 0.80 1</ambient>
      <background>0.70 0.82 0.92 1</background>
      <sky/>
    </scene>

    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.75 0.78 0.80</ambient_light>
        <background_color>0.70 0.82 0.92</background_color>
        <camera_pose>0 -33 32 0 0.78 1.5708</camera_pose>
        <camera_clip><near>0.1</near><far>500</far></camera_clip>
      </plugin>
      <plugin filename="EntityContextMenuPlugin" name="Entity context menu"/>
      <plugin filename="GzSceneManager" name="Scene Manager"/>
      <plugin filename="InteractiveViewControl" name="Interactive view control"/>
      <plugin filename="CameraTracking" name="Camera Tracking"/>
      <plugin filename="Spawn" name="Spawn Entities"/>
      <plugin filename="WorldControl" name="World control">
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>false</start_paused>
        <use_event>true</use_event>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <gz-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">true</property>
          <property type="bool" key="resizable">true</property>
          <property type="double" key="height">96</property>
          <property type="double" key="width">300</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
        </gz-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>false</real_time_factor>
        <iterations>true</iterations>
      </plugin>
    </gui>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 40 0 0 0</pose>
      <diffuse>0.95 0.92 0.85 1</diffuse>
      <specular>0.45 0.45 0.45 1</specular>
      <direction>-0.45 0.15 -0.88</direction>
    </light>

    <include>
      <name>kki_ocean_waves</name>
      <pose>0 0 0 0 0 0</pose>
      <uri>{file_uri(MODELS / "kki_ocean_waves")}</uri>
    </include>

{includes}
  </world>
</sdf>
'''


def grid_visuals():
    out = []
    values = [value for value in range(int(-COURSE_HALF_SIZE), int(COURSE_HALF_SIZE) + 1)]
    for i, value in enumerate(values):
        major = value % 5 == 0
        width = 0.048 if major else 0.020
        height = 0.028 if major else 0.014
        color = "0.96 0.98 1.00 1" if major else "0.72 0.78 0.80 1"
        out.append(
            f'        <visual name="grid_x_{i}"><cast_shadows>false</cast_shadows>'
            f'<pose>{value} 0 0.060 0 0 0</pose>'
            f'<geometry><box><size>{width:.3f} {COURSE_HALF_SIZE * 2:.3f} {height:.3f}</size></box></geometry>'
            f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>\n'
        )
        out.append(
            f'        <visual name="grid_y_{i}"><cast_shadows>false</cast_shadows>'
            f'<pose>0 {value} 0.060 0 0 0</pose>'
            f'<geometry><box><size>{COURSE_HALF_SIZE * 2:.3f} {width:.3f} {height:.3f}</size></box></geometry>'
            f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>\n'
        )
    boundary = [
        ("north", 0, COURSE_HALF_SIZE, COURSE_HALF_SIZE * 2, 0.090),
        ("south", 0, -COURSE_HALF_SIZE, COURSE_HALF_SIZE * 2, 0.090),
        ("east", COURSE_HALF_SIZE, 0, 0.090, COURSE_HALF_SIZE * 2),
        ("west", -COURSE_HALF_SIZE, 0, 0.090, COURSE_HALF_SIZE * 2),
    ]
    for name, x, y, sx, sy in boundary:
        out.append(
            f'        <visual name="course_boundary_{name}"><cast_shadows>false</cast_shadows>'
            f'<pose>{x:.3f} {y:.3f} 0.075 0 0 0</pose>'
            f'<geometry><box><size>{sx:.3f} {sy:.3f} 0.040</size></box></geometry>'
            f'<material><ambient>1.00 1.00 1.00 1</ambient><diffuse>1.00 1.00 1.00 1</diffuse></material></visual>\n'
        )
    return "".join(out)


def common_gate_includes():
    text = ""
    for idx, y in enumerate([0.0, 4.8, 8.8], start=1):
        text += include(f"left_gate_{idx}_green", "kki_buoy_green", (-13.0, y, 0.10, 0, 0, 0))
        text += include(f"left_gate_{idx}_red", "kki_buoy_red", (-13.0 + GATE_SPACING, y, 0.10, 0, 0, 0))
    for idx, x in enumerate([-5.2, -1.4, 2.4, 6.2], start=4):
        text += include(f"top_gate_{idx}_green", "kki_buoy_green", (x, 13.0, 0.10, 0, 0, 0))
        text += include(f"top_gate_{idx}_red", "kki_buoy_red", (x, 13.0 - GATE_SPACING, 0.10, 0, 0, 0))
    for idx, y in enumerate([0.0, 4.8, 8.8], start=8):
        text += include(f"right_gate_{idx}_red", "kki_buoy_red", (13.0 - GATE_SPACING, y, 0.10, 0, 0, 0))
        text += include(f"right_gate_{idx}_green", "kki_buoy_green", (13.0, y, 0.10, 0, 0, 0))
    return text


COURSE_A_PATH = [
    (12.0, -5.2),
    (12.0, 0.0),
    (12.0, 4.8),
    (12.0, 8.8),
    (6.2, 12.0),
    (2.4, 12.0),
    (-1.4, 12.0),
    (-5.2, 12.0),
    (-12.0, 8.8),
    (-12.0, 4.8),
    (-12.0, 0.0),
    (-11.2, -4.8),
    (-10.2, -5.4),
    (-11.0, -7.7),
    (-8.0, -9.0),
    (-2.0, -11.2),
    (4.0, -11.2),
    (10.8, -8.7),
    (12.15, -9.7),
]

COURSE_B_PATH = [
    (-12.0, -5.2),
    (-12.0, 0.0),
    (-12.0, 4.8),
    (-12.0, 8.8),
    (-5.2, 12.0),
    (-1.4, 12.0),
    (2.4, 12.0),
    (6.2, 12.0),
    (12.0, 8.8),
    (12.0, 4.8),
    (12.0, 0.0),
    (11.2, -4.8),
    (10.2, -5.4),
    (11.0, -7.7),
    (8.0, -9.0),
    (2.0, -11.2),
    (-4.0, -11.2),
    (-10.8, -8.7),
    (-12.15, -9.7),
]


def course_a_includes():
    text = ""
    text += include("start_finish_a", "kki_start_finish_marker", (12.2, -11.9, 0.02, 0, 0, 0))
    text += include("area_docking_a", "kki_dock", (13.5, -9.4, 0.02, 0, 0, 1.570796))
    for idx, y in enumerate([-10.0, -9.7, -9.4], start=1):
        text += include(f"docking_a_blue_{idx}", "kki_buoy_blue", (12.35, y, 0.10, 0, 0, 0))
    text += common_gate_includes()
    text += include("green_top_target_a", "kki_surface_target", (-10.2, -5.4, 0.0, 0, 0, 0.0))
    text += include("blue_top_target_a", "kki_underwater_target", (-11.0, -7.7, 0.0, 0, 0, 0.0))
    text += include("floating_obstacle_a_01", "kki_obstacle_yellow", (-3.0, -2.0, 0.35, 0, 0, 0))
    text += include("floating_obstacle_a_02", "kki_obstacle_yellow", (6.8, 2.4, 0.35, 0, 0, 0))
    return text


def course_b_includes():
    text = ""
    text += include("start_finish_b", "kki_start_finish_marker", (-12.2, -11.9, 0.02, 0, 0, 0))
    text += include("area_docking_b", "kki_dock", (-13.5, -9.4, 0.02, 0, 0, 1.570796))
    for idx, y in enumerate([-10.0, -9.7, -9.4], start=1):
        text += include(f"docking_b_blue_{idx}", "kki_buoy_blue", (-12.35, y, 0.10, 0, 0, 0))
    text += common_gate_includes()
    text += include("green_top_target_b", "kki_surface_target", (10.2, -5.4, 0.0, 0, 0, 0.0))
    text += include("blue_top_target_b", "kki_underwater_target", (11.0, -7.7, 0.0, 0, 0, 0.0))
    text += include("floating_obstacle_b_01", "kki_obstacle_yellow", (3.0, -2.0, 0.35, 0, 0, 0))
    text += include("floating_obstacle_b_02", "kki_obstacle_yellow", (-6.8, 2.4, 0.35, 0, 0, 0))
    return text


def write_waypoints(path, points, extras):
    lines = [
        "origin:",
        "  latitude: 1.470000",
        "  longitude: 102.110000",
        "  elevation: 0.0",
        "",
        "default_speed_mps: 0.65",
        "",
        "waypoints:",
    ]
    for name, x, y, speed in extras:
        lines.append(f"  - {{name: {name}, x: {x:.2f}, y: {y:.2f}, speed: {speed:.2f}}}")
    for idx, (x, y) in enumerate(points, start=1):
        name = f"path_{idx:02d}"
        speed = 0.45 if idx in (1, len(points)) else 0.65
        if abs(abs(x) - 10.2) < 0.05 and abs(y + 5.4) < 0.05:
            name = "surface_photo_green_box"
            speed = 0.25
        elif abs(abs(x) - 11.0) < 0.05 and abs(y + 7.7) < 0.05:
            name = "underwater_photo_blue_box"
            speed = 0.25
        elif abs(abs(x) - 10.8) < 0.05 and abs(y + 8.7) < 0.05:
            name = "return_to_start_finish"
            speed = 0.35
        elif abs(abs(x) - 12.15) < 0.05 and abs(y + 9.7) < 0.05:
            name = "dock_contact_blue_buoys"
            speed = 0.18
        lines.append(f"  - {{name: {name}, x: {x:.2f}, y: {y:.2f}, speed: {speed:.2f}}}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    WORLDS.mkdir(parents=True, exist_ok=True)
    a_world = base_world("kki_2026_lintasan_a", "lintasan_a", COURSE_A_PATH, course_a_includes())
    b_world = base_world("kki_2026_lintasan_b", "lintasan_b", COURSE_B_PATH, course_b_includes())
    (WORLDS / "kki_2026_lintasan_a.sdf").write_text(a_world, encoding="utf-8")
    (WORLDS / "kki_2026_lintasan_b.sdf").write_text(b_world, encoding="utf-8")
    (WORLDS / "kki_2026_asv_course.sdf").write_text(a_world, encoding="utf-8")

    write_waypoints(
        NAV_CONFIG / "kki_waypoints_a.yaml",
        COURSE_A_PATH,
        [("start_a", 10.8, -8.7, 0.30)],
    )
    write_waypoints(
        NAV_CONFIG / "kki_waypoints_b.yaml",
        COURSE_B_PATH,
        [("start_b", -10.8, -8.7, 0.30)],
    )
    (NAV_CONFIG / "kki_waypoints.yaml").write_text(
        (NAV_CONFIG / "kki_waypoints_a.yaml").read_text(encoding="utf-8"),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
