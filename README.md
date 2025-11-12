# PAS_zadaca1 – Fanuc LRMate200iC ROS2 Project

Ovaj projekt sadrži kompletan ROS2 paket za rad s Fanuc LRMate200iC robotom, uključujući URDF, ros2_control konfiguraciju, kontrolere, RViz prikaz i Python nodeove za slanje komandi.

U nastavku se nalaze potpune upute za instalaciju, strukturu paketa, buildanje te pokretanje kontrolera i primjera.

---

## ✅ 1. Napraviti workspace (npr. imena zadaca1)

```bash
mdkir -p zadaca1
```

---

## ✅ 2. Kloniranje repozitorija

Ući u napravljeni folder `zadaca1` te pokrenuti:

```bash
git clone https://github.com/Gugo1507/PAS_zadaca1.git src
```

---

## ✅ 3. Struktura projekta

Unutar `src/fanuc` direktorija nalazi se:

```
fanuc
├── fanuc_lrmate200ic_support
│   ├── config              # Config fajlovi za kontrolere i publishere
│   ├── launch              #Launch fajlovi za pokretanje sustava
│   ├── meshes              # STL i DAE modeli robota
│   ├── ros2_control        # ros2_control xacro konfiguracija
│   ├── scripts             # Python nodeovi
│   ├── rviz                # RViz konfiguracija
│   ├── urdf                # URDF i Xacro modeli robota
│   ├── package.xml
│   └── CMakeLists.txt
└── fanuc_resources
    └── (materijali i boje za URDF)
```

---

## ✅ 4. Buildanje projekta

Vrati se u svoj workspace:

```bash
cd ~/zadaca1
colcon build --symlink-install
source install/setup.bash
```

---

## ✅ 5. Pokretanje vizualizacije robota u RViz-u

Pokretanje launch datoteke za vizualizaciju i upravljanje gui -em.

```bash
ros2 launch fanuc_lrmate200ic_support view_fanuc.launch.py gui:=true
```

Ovo pokreće:

* joint_state_publisher_gui
* robot_state_publisher
* RViz s konfiguriranim Fanuc modelom

---

## ✅ 6. Pokretanje ros2_control sustava i svih kontrolera

Otvoriti novi terminal, source -ati te pokrenuti: 

```bash
ros2 launch fanuc_lrmate200ic_support bringup_controllers.launch.py
```

Glavni bringup koji starta:

* ros2_control_node
* robot_state_publisher
* joint_trajectory_controller
* joint_state_broadcaster
* forward_position_controller


---

## ✅ 7. Slanje trajektrorije – Joint Trajectory Controller
Otvoriti novi terminal te source -ati te pokrenuti: 

```bash
ros2 launch fanuc_lrmate200ic_support joint_trajectory.launch.py
```
NAPOMENA: Ako se želi upravljati robota kontrolerima potrebno je prilikom vizualizacije robota isključiti gui:

```bash
ros2 launch fanuc_lrmate200ic_support view_fanuc.launch.py gui:=false
```

Konfiguracija ciljeva se nalazi u:

```
config/fanuc_joint_trajectory_publisher.yaml
```

---

## ✅ 8. Slanje komandi – Forward Position Controller

Za periodično slanje pozciija na Forward Position kontroler potrebno je prvo promijeniti kontroler komandom u novom terminalu: 


```bash
ros2 control switch_controllers \
  --activate forward_position_controller \
  --deactivate joint_trajectory_controller
```

Otvoriti novi terminal, source -ati te pokrenuti komandu: 

```bash
ros2 launch fanuc_lrmate200ic_support forward_position.launch.py
```

---


Ako želiš dodatni README za GitHub s badgeovima, slikama i animacijama robota, samo reci.

