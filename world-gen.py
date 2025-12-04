import rospy
import random
import math
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

# --- КОНСТАНТЫ ЗАДАНИЯ ---
START_X, START_Y = 1.0, 1.0
PIPE_RADIUS = 0.1  # Толщина 20см = радиус 0.1м
TAP_RADIUS = 0.05  # Толщина 10см = радиус 0.05м
TAP_LENGTH = 0.5   # Длина врезки (визуально достаточно 0.5-1м, чтобы торчала)
MIN_DIST_BETWEEN_TAPS = 0.75

# Шаблон SDF файла (XML описание модели)
SDF_WRAPPER = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='pipeline_system'>
    <static>true</static>
    {links}
  </model>
</sdf>
"""

# Шаблон одного звена (цилиндра)
LINK_TEMPLATE = """
    <link name='{name}'>
      <pose frame=''>{x} {y} {z} {roll} {pitch} {yaw}</pose>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{length}</length>
          </cylinder>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/{color}</name>
          </script>
        </material>
      </visual>
      <collision name='collision'>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{length}</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
"""

def get_cylinder_pose(start_x, start_y, length, angle_rad):
    """
    Gazebo размещает цилиндр центром в (x,y).
    Нам нужно найти центр сегмента, зная начало, длину и угол.
    """
    center_x = start_x + (length / 2.0) * math.cos(angle_rad)
    center_y = start_y + (length / 2.0) * math.sin(angle_rad)
    # Цилиндр в Gazebo по умолчанию стоит вертикально. 
    # Поворачиваем его на 90 градусов (PI/2) по Pitch, чтобы положить, 
    # и затем на нужный угол по Yaw (Z).
    return center_x, center_y, 0.2, 0, 1.5708, angle_rad

def generate_pipeline():
    rospy.init_node('world_generator')
    
    # 1. Генерируем параметры основной трубы
    total_len = random.uniform(5.0, 10.0)
    # Разбиваем на 2 сегмента для изгиба (например, 40-60% длины на первый сегмент)
    len1 = total_len * random.uniform(0.4, 0.6)
    len2 = total_len - len1
    
    # Угол первого сегмента (направляем примерно в центр карты, чтобы не улететь)
    angle1 = random.uniform(0, 1.57) # 0 to 90 degrees
    
    # Угол второго сегмента (изгиб <= 30 градусов)
    bend = random.radians(random.uniform(-30, 30))
    angle2 = angle1 + bend
    
    links_xml = ""
    
    # --- СЕГМЕНТ 1 ---
    cx1, cy1, cz, cr, cp, cy = get_cylinder_pose(START_X, START_Y, len1, angle1)
    links_xml += LINK_TEMPLATE.format(name="pipe_seg1", x=cx1, y=cy1, z=cz, roll=cr, pitch=cp, yaw=cy, radius=PIPE_RADIUS, length=len1, color="Yellow")
    
    # Конец 1-го сегмента = Начало 2-го
    end_x1 = START_X + len1 * math.cos(angle1)
    end_y1 = START_Y + len1 * math.sin(angle1)
    
    # --- СЕГМЕНТ 2 ---
    cx2, cy2, cz, cr, cp, cy = get_cylinder_pose(end_x1, end_y1, len2, angle2)
    links_xml += LINK_TEMPLATE.format(name="pipe_seg2", x=cx2, y=cy2, z=cz, roll=cr, pitch=cp, yaw=cy, radius=PIPE_RADIUS, length=len2, color="Yellow")
    
    # --- ГЕНЕРАЦИЯ ВРЕЗОК ---
    taps_positions = []
    attempts = 0
    while len(taps_positions) < 5 and attempts < 100:
        attempts += 1
        # Случайная точка на всей длине трубы
        dist = random.uniform(0, total_len)
        
        # Проверка минимального расстояния
        valid = True
        for p in taps_positions:
            if abs(dist - p) < MIN_DIST_BETWEEN_TAPS:
                valid = False
                break
        if not valid: continue
            
        taps_positions.append(dist)
        
        # Определяем координаты врезки
        if dist <= len1:
            # На первом сегменте
            tap_x = START_X + dist * math.cos(angle1)
            tap_y = START_Y + dist * math.sin(angle1)
            tap_angle = angle1 + 1.5708 # Перпендикулярно (+90 град)
        else:
            # На втором сегменте
            rem_dist = dist - len1
            tap_x = end_x1 + rem_dist * math.cos(angle2)
            tap_y = end_y1 + rem_dist * math.sin(angle2)
            tap_angle = angle2 + 1.5708

        # Добавляем врезку (смещаем центр врезки чуть вбок, чтобы она торчала из трубы)
        # Смещение на (TAP_LENGTH/2), чтобы она начиналась от центра трубы
        offset = TAP_LENGTH / 2.0
        final_tap_x = tap_x + offset * math.cos(tap_angle)
        final_tap_y = tap_y + offset * math.sin(tap_angle)
        
        links_xml += LINK_TEMPLATE.format(
            name=f"tap_{len(taps_positions)}",
            x=final_tap_x, y=final_tap_y, z=cz, # Высота та же
            roll=0, pitch=1.5708, yaw=tap_angle,
            radius=TAP_RADIUS, length=TAP_LENGTH, color="Red"
        )
        
    # --- ОТПРАВКА В GAZEBO ---
    final_sdf = SDF_WRAPPER.format(links=links_xml)
    
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model("pipeline_system", final_sdf, "", Pose(), "world")
        rospy.loginfo("Трубопровод успешно сгенерирован!")
    except rospy.ServiceException as e:
        rospy.logerr("Ошибка спавна: %s", e)

if __name__ == '__main__':
    generate_pipeline()