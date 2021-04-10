<?xml version="1.0" encoding="utf-8"?>
<!-- This URDF was automatically created by SolidWorks to URDF Exporter! Originally created by Stephen Brawner (brawner@gmail.com) 
     Commit Version: 1.6.0-1-g15f4949  Build Version: 1.6.7594.29634
     For more information, please see http://wiki.ros.org/sw_urdf_exporter -->
<robot
  name="car">
  <link name="dummy">
    </link> 
  <link
    name="base_link">
    <inertial>
      <origin
        xyz="-0.0984180141164289 -4.24653808969242E-05 -0.0479216367248507"
        rpy="0 0 0" />
      <mass
        value="6.12417699747623" />
      <inertia
        ixx="0.0300418227285899"
        ixy="-1.31898495040203E-10"
        ixz="0.000115309225809925"
        iyy="0.0748141060332957"
        iyz="-2.70309349390935E-07"
        izz="0.0990159080937532" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/base_link.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.752941176470588 0.752941176470588 0.752941176470588 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/base_link.STL" />
      </geometry>
    </collision>
  </link>
  <joint name="dummy_joint" type="fixed">
    <parent link="dummy"/>
    <child link="base_link"/>
  </joint>
  <link
    name="link_1">
    <inertial>
      <origin
        xyz="0.00269150109526521 0.00643720654093697 0.0134014173539046"
        rpy="0 0 0" />
      <mass
        value="0.0798083224491903" />
      <inertia
        ixx="2.67162812954716E-05"
        ixy="-2.43210282464138E-06"
        ixz="3.12605441684161E-07"
        iyy="2.66369888305853E-05"
        iyz="-3.15010363972303E-07"
        izz="4.93317878172949E-05" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_1.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.752941176470588 0.752941176470588 0.752941176470588 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_1.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="joint_0"
    type="revolute">
    <origin
      xyz="0 0 0"
      rpy="0 0 0" />
    <parent
      link="base_link" />
    <child
      link="link_1" />
    <axis
      xyz="0 0 1" />
    <limit
      lower="-1.42"
      upper="3.14"
      effort="0"
      velocity="0" />
  </joint>
  <link
    name="link_2">
    <inertial>
      <origin
        xyz="4.66857455472258E-16 -0.0509999999999999 0.00196123940376829"
        rpy="0 0 0" />
      <mass
        value="0.0475993853834784" />
      <inertia
        ixx="4.53034392400334E-05"
        ixy="4.39248438114813E-20"
        ixz="4.91279109407495E-20"
        iyy="2.93414623184682E-05"
        iyz="2.63109835750389E-20"
        izz="2.08816059621121E-05" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_2.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.752941176470588 0.752941176470588 0.752941176470588 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_2.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="joint1"
    type="revolute">
    <origin
      xyz="0.00124058826901926 0.00124058826901925 0.0222"
      rpy="-1.5707963267949 0 -2.35619449019235" />
    <parent
      link="link_1" />
    <child
      link="link_2" />
    <axis
      xyz="0 0 1" />
    <limit
      lower="-1.57"
      upper="1.52"
      effort="0"
      velocity="0" />
  </joint>
  <link
    name="link_3">
    <inertial>
      <origin
        xyz="0.048654934891534 -0.0267541750207239 -0.00374536355010625"
        rpy="0 0 0" />
      <mass
        value="0.111344903565102" />
      <inertia
        ixx="1.48058334362821E-05"
        ixy="6.13521058640702E-06"
        ixz="-5.30481365743086E-07"
        iyy="2.07786943796875E-05"
        iyz="-6.77818543214601E-07"
        izz="2.27017654959212E-05" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_3.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.752941176470588 0.752941176470588 0.752941176470588 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_3.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="joint_2"
    type="revolute">
    <origin
      xyz="0 -0.102 0"
      rpy="0 0 0" />
    <parent
      link="link_2" />
    <child
      link="link_3" />
    <axis
      xyz="0 0 1" />
    <limit
      lower="-1.38"
      upper="1.726"
      effort="0"
      velocity="0" />
  </joint>
  <link
    name="link_4">
    <inertial>
      <origin
        xyz="0.0312862928250999 0.0162844264935197 0.00196741396525555"
        rpy="0 0 0" />
      <mass
        value="0.0508687963957953" />
      <inertia
        ixx="2.06185098194461E-05"
        ixy="-2.35495810123568E-06"
        ixz="-3.50200079311361E-08"
        iyy="2.39090019104651E-05"
        iyz="-1.82228134622045E-08"
        izz="9.9109950707106E-06" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_4.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.752941176470588 0.752941176470588 0.752941176470588 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_4.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="joint_3"
    type="revolute">
    <origin
      xyz="0.110641877280467 -0.0692166995578351 0"
      rpy="0.0330134851352876 0 -0.559025497866973" />
    <parent
      link="link_3" />
    <child
      link="link_4" />
    <axis
      xyz="0 0 1" />
    <limit
      lower="-0.977"
      upper="2.13"
      effort="0"
      velocity="0" />
  </joint>
  <link
    name="link_5">
    <inertial>
      <origin
        xyz="0.0200430658409259 0.015206821362248 -0.0628230629281958"
        rpy="0 0 0" />
      <mass
        value="0.0397478655309209" />
      <inertia
        ixx="6.83118920776812E-06"
        ixy="-8.25522508386529E-07"
        ixz="-1.11386372673254E-09"
        iyy="2.47605219937485E-06"
        iyz="-6.38856974010174E-09"
        izz="6.25600379880046E-06" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_5.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.752941176470588 0.752941176470588 0.752941176470588 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_5.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="joint_4"
    type="revolute">
    <origin
      xyz="0.0263869262023886 0.0137314418971836 0.0112859927951107"
      rpy="2.66176793053187 1.5707963267949 0" />
    <parent
      link="link_4" />
    <child
      link="link_5" />
    <axis
      xyz="0 0 1" />
    <limit
      lower="-1.55"
      upper="1.55"
      effort="0"
      velocity="0" />
  </joint>
  <link
    name="link_6">
    <inertial>
      <origin
        xyz="-0.00019273 -0.0027587 0.00081377"
        rpy="0 0 0" />
      <mass
        value="0.0034196" />
      <inertia
        ixx="2.621E-07"
        ixy="-2.3036E-08"
        ixz="-1.7205E-22"
        iyy="1.014E-07"
        iyz="2.6139E-22"
        izz="3.5849E-07" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_6.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.89412 0.91373 0.92941 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_6.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="joint_5"
    type="revolute">
    <origin
      xyz="-0.00125082871048847 0.0134873940524703 -0.0660040360555754"
      rpy="1.5707963267949 0 -1.74350141191543" />
    <parent
      link="link_5" />
    <child
      link="link_6" />
    <axis
      xyz="0 0 -1" />
    <limit
      lower="-0.01"
      upper="1.57"
      effort="0"
      velocity="0" />
  </joint>
  <link
    name="link_61">
    <inertial>
      <origin
        xyz="0.0065432 0.022668 0.0015"
        rpy="0 0 0" />
      <mass
        value="0.0014839" />
      <inertia
        ixx="4.324E-07"
        ixy="-7.8439E-08"
        ixz="-6.6174E-23"
        iyy="3.0753E-08"
        iyz="-5.6248E-22"
        izz="4.6093E-07" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_61.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_61.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="joint_51"
    type="revolute">
    <origin
      xyz="-0.0036089 -0.030839 0.00175"
      rpy="3.1416 0 0" />
    <parent
      link="link_6" />
    <child
      link="link_61" />
    <axis
      xyz="0 0 -1" />
    <limit
      lower="-0.01"
      upper="1.57"
      effort="0"
      velocity="0" />
    <mimic
      joint="joint_5"
      multiplier="1"
      offset="0" />
  </joint>
  <link
    name="link_62">
    <inertial>
      <origin
        xyz="0.00042559 0.0066665 -0.001"
        rpy="0 0 0" />
      <mass
        value="0.0014157" />
      <inertia
        ixx="2.1288E-07"
        ixy="-2.2306E-08"
        ixz="-5.3932E-22"
        iyy="4.9968E-08"
        iyz="-7.821E-22"
        izz="2.6191E-07" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_62.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_62.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="joint_52"
    type="revolute">
    <origin
      xyz="-0.007627 -0.012881 -0.066004"
      rpy="-1.5708 0 -1.7435" />
    <parent
      link="link_5" />
    <child
      link="link_62" />
    <axis
      xyz="0 0 -1" />
    <limit
      lower="-0.01"
      upper="1.57"
      effort="0"
      velocity="0" />
    <mimic
      joint="joint_5"
      multiplier="1"
      offset="0" />
  </joint>
  <link
    name="link_63">
    <inertial>
      <origin
        xyz="-0.006769 0.022602 0.0015"
        rpy="0 0 0" />
      <mass
        value="0.0014839" />
      <inertia
        ixx="4.308E-07"
        ixy="8.243E-08"
        ixz="-2.0514E-22"
        iyy="3.2358E-08"
        iyz="4.2641E-22"
        izz="4.6093E-07" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_63.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/link_63.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="joint_53"
    type="revolute">
    <origin
      xyz="0.003442 0.030858 0"
      rpy="0 0 0" />
    <parent
      link="link_62" />
    <child
      link="link_63" />
    <axis
      xyz="0 0 1" />
    <limit
      lower="-0.01"
      upper="1.57"
      effort="0"
      velocity="0" />
    <mimic
      joint="joint_5"
      multiplier="1"
      offset="0" />
  </joint>
  <link
    name="whfl">
    <inertial>
      <origin
        xyz="1.2467E-09 -5.4442E-09 -0.020213"
        rpy="0 0 0" />
      <mass
        value="0.072755" />
      <inertia
        ixx="4.1437E-05"
        ixy="2.2383E-13"
        ixz="-5.4568E-11"
        iyy="4.1437E-05"
        iyz="3.9178E-11"
        izz="6.0865E-05" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/whfl.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/whfl.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="fl"
    type="continuous">
    <origin
      xyz="0.0015 0.083487 -0.0592"
      rpy="1.5708 1.5708 0" />
    <parent
      link="base_link" />
    <child
      link="whfl" />
    <axis
      xyz="0 0 1" />
  </joint>
  <link
    name="whfr">
    <inertial>
      <origin
        xyz="1.24962969139109E-09 5.44436740785789E-09 0.0202134707653484"
        rpy="0 0 0" />
      <mass
        value="0.0727548969842859" />
      <inertia
        ixx="4.14370643491625E-05"
        ixy="-2.25286543002883E-13"
        ixz="5.45657502466087E-11"
        iyy="4.14370654220599E-05"
        iyz="3.91772712986478E-11"
        izz="6.08652668960254E-05" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/whfr.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/whfr.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="fr"
    type="continuous">
    <origin
      xyz="0.0015 -0.083487 -0.0592"
      rpy="1.5708 1.5708 0" />
    <parent
      link="base_link" />
    <child
      link="whfr" />
    <axis
      xyz="0 0 1" />
  </joint>
  <link
    name="whbl">
    <inertial>
      <origin
        xyz="1.24650349764277E-09 -5.44134765023685E-09 -0.020213470764244"
        rpy="0 0 0" />
      <mass
        value="0.0727548969730556" />
      <inertia
        ixx="4.14370643512431E-05"
        ixy="2.24951970406938E-13"
        ixz="-5.45685294714399E-11"
        iyy="4.14370654186999E-05"
        iyz="3.91804314061101E-11"
        izz="6.08652668941202E-05" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/whbl.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/whbl.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="bl"
    type="continuous">
    <origin
      xyz="-0.1985 0.083487 -0.0592"
      rpy="1.5708 1.5708 0" />
    <parent
      link="base_link" />
    <child
      link="whbl" />
    <axis
      xyz="0 0 1" />
  </joint>
  <link
    name="whbr">
    <inertial>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <mass
        value="0" />
      <inertia
        ixx="0"
        ixy="0"
        ixz="0"
        iyy="0"
        iyz="0"
        izz="0" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/whbr.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="1 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://car/meshes/whbr.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="br"
    type="continuous">
    <origin
      xyz="-0.198499999999999 -0.0834873571736593 -0.0592000000000001"
      rpy="1.57079632679489 1.5707963267949 0" />
    <parent
      link="base_link" />
    <child
      link="whbr" />
    <axis
      xyz="0 0 1" />
  </joint>
</robot>
