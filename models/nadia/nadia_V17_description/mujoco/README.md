These files were created by first editing the corresponding URDF to add mujoco compile options
```
<mujoco>
    <compiler meshdir="../.." discardvisual="false" strippath="false"/>
</mujoco>
```
and a floating base
```
<!-- worldbody for MuJoCo -->
<link name="world"/>
<joint name="pelvis" type="floating">
    <parent link="world"/>
    <child link="PELVIS_LINK"/>
</joint>
```
Then the models were converted to MJCF using the following in Python (needs to be done with MuJoCo version >= 3.2, can check using
`mujoco.mj_version() >= 320`)
```
import mujoco
model = mujoco.MjModel.from_xml_path("nadia_V17_description/urdf/nadiaV17.fullRobot.simpleKnees.cycloidArms_mj.urdf")
mujoco.mj_saveLastXML("nadiaV17.fullRobot.simpleKnees.cycloidArms.xml", model)
```
