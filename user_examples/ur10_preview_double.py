# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb.input
import omni.kit.commands
import omni.ext
import omni.ui as ui
import omni.kit.settings

import weakref

from omni.isaac.motion_planning import _motion_planning
from omni.isaac.dynamic_control import _dynamic_control
import omni.physx as _physx
from omni.physx.bindings._physx import SimulationEvent
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription

from omni.isaac.examples.user_examples.ur10_scenarios.scenario_v1 import Scenario
from omni.isaac.examples.user_examples.ur10_scenarios import bin_stack_v1
from omni.isaac.examples.user_examples.ur10_scenarios.scenario_v2 import Scenario2
from omni.isaac.examples.user_examples.ur10_scenarios import bin_stack_v2

import time
from collections import deque

import asyncio

EXTENSION_NAME = "BAE Demo Controller"


class Extension_Double(omni.ext.IExt):

    def on_startup(self):
        self._timeline = omni.timeline.get_timeline_interface()
        self._viewport = omni.kit.viewport.get_default_viewport_window()
        self._viewport1 = omni.kit.viewport.get_viewport_interface()

        self._viewport1_names = []
        self._num_viewports = 0
        self._usd_context = omni.usd.get_context()
        self._stage = self._usd_context.get_stage()
        self._first_step = True
        self._first_step2 = True

        self._is_playing = False
        self.hobe_flag = 1234
        self._mp = _motion_planning.acquire_motion_planning_interface()
        self._dc = _dynamic_control.acquire_dynamic_control_interface()

        self._mp = _motion_planning.acquire_motion_planning_interface()
        self._dc = _dynamic_control.acquire_dynamic_control_interface()

        self._physxIFace = _physx.acquire_physx_interface()

        self._settings = carb.settings.get_settings()

        self._appwindow = omni.appwindow.get_default_app_window()
        self._sub_stage_event = self._usd_context.get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event
        )
        self._physx_subs = _physx.get_physx_interface().subscribe_physics_step_events(self._on_simulation_step)
        self._scenario2 = Scenario2(self._dc, self._mp)
        self._scenario = Scenario(self._dc, self._mp)

        self._window = None
        # self._create_UR10_btn = None
        self._create_UR10_v1_btn = None
        self._create_UR10_v2_btn = None
        self._create_UR10_v3_btn = None
        self._create_UR10_v4_btn = None
        self._create_UR10_v5_btn = None


        self.action_result = False
        self._ros_service_btn = None
        self._add_new_bins_btn = None


        self.task1 = None
        self.task2 = None
        self.task3 = None
        self.task4 = None
        self.task5 = None
        self.task6 = None
        self.task7 = None
        self.rosserver1 = None
        self.rosserver2 = None

        self.start_bins = [12, 8, 4, 12, 12]  # Default number of cargo per environment

        self._menu_items = [
            MenuItemDescription(
                name="BAE Demo",
                sub_menu=[
                    MenuItemDescription(
                        name="BAE Demo", onclick_fn=lambda a=weakref.proxy(self): a._menu_callback()
                    )
                ],
            )
        ]
        add_menu_items(self._menu_items, "Isaac Examples")

    def _menu_callback(self):
        self._build_ui()

    def _build_ui(self):
        import rospy
        # rospy.init_node('KGU_Isaac_Node', anonymous=True)
        if not self._window:
            self._window = ui.Window(
                title=EXTENSION_NAME, width=300, height=300, dockPreference=ui.DockPreference.LEFT_BOTTOM
            )
            self._app_update_sub = (
                omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update_ui)
            )
            with self._window.frame:
                with ui.VStack():
                    with ui.HStack(height=0):
                        ui.Label("Selected Scenario", width=0)
                        ui.Spacer(width=5)

                    # self._create_UR10_btn = ui.Button("Create Scenario", clicked_fn=self._on_environment_setup)

                    # 0903 CHOI
                    self._create_UR10_v1_btn = ui.Button("Create Factory v1 Scenario", clicked_fn=self._on_environment_v1_setup)
                    self._create_UR10_v2_btn = ui.Button("Create Factory v2 Scenario", clicked_fn=self._on_environment_v2_setup)
                    self._create_UR10_v3_btn = ui.Button("Create Factory v3 Scenario", clicked_fn=self._on_environment_v3_setup)
                    self._create_UR10_v4_btn = ui.Button("Create Factory v4 Scenario", clicked_fn=self._on_environment_v4_setup)
                    self._create_UR10_v5_btn = ui.Button("Create Factory v5 Scenario", clicked_fn=self._on_environment_v5_setup)

                    self._ros_service_btn = ui.Button("Ros Start", clicked_fn=self._on_ros_service)
                    self._ros_service_btn.enabled = False

                    # self._perform_ur10v1_btn = ui.Button("Perform UR10v1", clicked_fn=self._on_perform_task)
                    # self._perform_ur10v1_btn.enabled = False
                    #
                    # self._stop_ur10v1_btn = ui.Button("Pause UR10_v1", clicked_fn=self._on_pause_tasks)
                    # self._stop_ur10v1_btn.enabled = False
                    #
                    # self._perform_ur10v2_btn = ui.Button("Perform UR10v2", clicked_fn=self._on_perform_task1)
                    # self._perform_ur10v2_btn.enabled = False
                    #
                    # self._stop_ur10v2_btn = ui.Button("Pause UR10_v2", clicked_fn=self._on_pause_tasks1)
                    # self._stop_ur10v2_btn.enabled = False

        self._window.visible = True

    def _on_environment_v1_setup(self):
        # wait for new stage before creating franka
        task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._on_create_UR10(task, 1))
        self._on_clear_all_setup()
        self._create_UR10_v1_btn.text = "Clear Scenario"
        self._create_UR10_v1_btn.set_clicked_fn(self._on_clear_v1_scenario)

    def _on_clear_v1_scenario(self):
        # wait for new stage before creating franka
        asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        self._create_UR10_v1_btn.text = "Create Factory v1 Scenario"
        self._create_UR10_v1_btn.set_clicked_fn(self._on_environment_v1_setup)

    def _on_environment_v2_setup(self):
        # wait for new stage before creating franka
        task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._on_create_UR10(task, 2))
        self._on_clear_all_setup()
        self._create_UR10_v2_btn.text = "Clear Scenario"
        self._create_UR10_v2_btn.set_clicked_fn(self._on_clear_v2_scenario)

    def _on_clear_v2_scenario(self):
        # wait for new stage before creating franka
        asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        self._create_UR10_v2_btn.text = "Create Factory v2 Scenario"
        self._create_UR10_v2_btn.set_clicked_fn(self._on_environment_v2_setup)

    def _on_environment_v3_setup(self):
        # wait for new stage before creating franka
        task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._on_create_UR10(task, 3))
        self._on_clear_all_setup()
        self._create_UR10_v3_btn.text = "Clear Scenario"
        self._create_UR10_v3_btn.set_clicked_fn(self._on_clear_v3_scenario)

    def _on_clear_v3_scenario(self):
        # wait for new stage before creating franka
        asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        self._create_UR10_v3_btn.text = "Create Factory v3 Scenario"
        self._create_UR10_v3_btn.set_clicked_fn(self._on_environment_v3_setup)

    def _on_environment_v4_setup(self):
        # wait for new stage before creating franka
        task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._on_create_UR10(task, 4))
        self._on_clear_all_setup()
        self._create_UR10_v4_btn.text = "Clear Scenario"
        self._create_UR10_v4_btn.set_clicked_fn(self._on_clear_v4_scenario)

    def _on_clear_v4_scenario(self):
        # wait for new stage before creating franka
        asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        self._create_UR10_v4_btn.text = "Create Factory v4 Scenario"
        self._create_UR10_v4_btn.set_clicked_fn(self._on_environment_v4_setup)

    def _on_environment_v5_setup(self):
        # wait for new stage before creating franka
        task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._on_create_UR10(task, 5))
        self._on_clear_all_setup()
        self._create_UR10_v5_btn.text = "Clear Scenario"
        self._create_UR10_v5_btn.set_clicked_fn(self._on_clear_v5_scenario)

    def _on_clear_v5_scenario(self):
        # wait for new stage before creating franka
        asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        self._create_UR10_v5_btn.text = "Create Factory v5 Scenario"
        self._create_UR10_v5_btn.set_clicked_fn(self._on_environment_v5_setup)

    def _on_clear_all_setup(self):
        if self._scenario.is_created() or self._scenario2.is_created():
            if self._create_UR10_v1_btn.text == "Clear Scenario":
                self._on_clear_v1_scenario()
            elif self._create_UR10_v2_btn.text == "Clear Scenario":
                self._on_clear_v2_scenario()
            elif self._create_UR10_v3_btn.text == "Clear Scenario":
                self._on_clear_v3_scenario()
            elif self._create_UR10_v4_btn.text == "Clear Scenario":
                self._on_clear_v4_scenario()
            elif self._create_UR10_v5_btn.text == "Clear Scenario":
                self._on_clear_v5_scenario()

    async def _on_create_UR10(self, task, version):
        done, pending = await asyncio.wait({task})
        if task not in done:
            await omni.kit.app.get_app().next_update_async()
            return

        self._stage = self._usd_context.get_stage()
        self._scenario = bin_stack_v1.BinStack(self._dc, self._mp, self.start_bins[version-1])
        self._scenario2 = bin_stack_v2.BinStack(self._dc, self._mp)

        viewports = self._viewport1.get_instance_list()
        self._viewport1_names = [self._viewport1.get_viewport_window_name(vp) for vp in viewports]
        self._num_viewports = len(self._viewport1_names)
        self._viewport.set_camera_position("/OmniverseKit_Persp", -650.928, -135.982, 3459.74,
                                           True)  # 12554.86909, 13767.31414, 453.73453, True)  # 370, 135, 60
                                           # 0811 : 101.66722, 243.55117, 226.34345
        self._viewport.set_camera_target("/OmniverseKit_Persp", -650.8, -309.63597, 11.0,
                                         True)  # 13248.0, 14190, 11.0, True)
                                         # 0811 : 774.83398, 309.63597, -0.33244
        self._first_step = True
        self._first_step2 = True
        self._timeline.stop()
        self._on_ros_stop_service()
        self._physxIFace.release_physics_objects()

        self._settings.set("/rtx/reflections/halfRes", True)
        self._settings.set("/rtx/shadows/denoiser/quarterRes", True)
        self._settings.set("/rtx/translucency/reflectionCutoff", 0.1)

        self._scenario.create_UR10(version)
        self._scenario2.create_UR10()

        self._physxIFace.release_physics_objects()
        self._physxIFace.force_load_physics_from_usd()

        self._physxIFace.release_physics_objects()
        self._physxIFace.force_load_physics_from_usd()
        self._ros_service_btn.enabled = True


    def _on_stop_tasks(self, *args):
        print("_on_stop_task")
        self._on_ros_stop_service()
        if self._scenario:
            self._scenario.stop_tasks()
        if self._scenario2:
            self._scenario2.stop_tasks()

    def _on_perform_task(self, *args):
        # perform = self._scenario2.perform_tasks()
        perform = self._scenario.perform_tasks()
        return perform

    def _on_pause_tasks(self, *args):
        # isPaused = self._scenario2.pause_tasks()
        isPaused = self._scenario.pause_tasks()
        return isPaused

    def _on_perform_task1(self, *args):
        perform = self._scenario2.perform_tasks()
        print("Perform button", perform)
        return perform

    def _on_pause_tasks1(self, *args):
        isPaused = self._scenario2.pause_tasks()
        print("Pause button", isPaused)
        return isPaused

    def _on_open_gripper(self, *args):
        self._scenario.open_gripper()

    def _on_add_bin(self, *args):
        self._scenario.add_bin()

    def _on_simulation_step(self, step):
        if self._first_step:
            self._scenario.register_assets()
            self._first_step = False
        self._scenario.step(step)
        if self._first_step2:
            self._scenario2.register_assets()
            self._first_step2 = False
        self._scenario2.step(step)

    def _on_stage_event(self, event):
        if self._window:
            self.stage = self._usd_context.get_stage()
            if event.type == int(omni.usd.StageEventType.OPENED):
                # self._create_UR10_btn.enabled = True
                self._create_UR10_v1_btn.enabled = True
                self._create_UR10_v2_btn.enabled = True
                self._create_UR10_v3_btn.enabled = True
                self._create_UR10_v4_btn.enabled = True
                self._create_UR10_v5_btn.enabled = True
                self._ros_service_btn.enabled = False
                self._on_stop_tasks()
                self._scenario = Scenario(self._dc, self._mp)
                self._scenario2 = Scenario2(self._dc, self._mp)
                # self._perform_ur10v2_btn.enabled = False
                # self._stop_ur10v2_btn.enabled = False
                # self._perform_ur10v1_btn.enabled = False
                # self._stop_ur10v1_btn.enabled = False


    # TODO: need to plus constraint every versions
    def _on_update_ui(self, step):
        is_stopped = self._timeline.is_stopped()
        if is_stopped and self._is_playing:
            self._on_stop_tasks()
        self._is_playing = not is_stopped

        if self._timeline.is_playing() or (self._scenario.is_created() and self._scenario2.is_created()):
            self._ros_service_btn.enabled = True
            # self._perform_ur10v1_btn.enabled = True
            # self._stop_ur10v1_btn.enabled = True
            # self._perform_ur10v2_btn.enabled = True
            # self._stop_ur10v2_btn.enabled = True

            # if not self._scenario.is_created() and not self._scenario2.is_created():
            # #     # self._on_ros_stop_service()

        if not self._timeline.is_playing():
            self._ros_service_btn.enabled = False

            if not self._scenario.is_created() and not self._scenario2.is_created():
                # self._create_UR10_btn.enabled = True
                self._create_UR10_v1_btn.enabled = True
                self._create_UR10_v2_btn.enabled = True
                self._create_UR10_v3_btn.enabled = True
                self._create_UR10_v4_btn.enabled = True
                self._create_UR10_v5_btn.enabled = True
                self._ros_service_btn.enabled = False


    def _on_ros_service(self):
        import rospy
        print('Ros start!!')
        rospy.init_node('KGU_Isaac_Node', anonymous=True)

        self.task1 = asyncio.ensure_future(self.listener1())
        self.task2 = asyncio.ensure_future(self.listener2())
        self.task3 = asyncio.ensure_future(self.ros_publish_packing())
        self.task4 = asyncio.ensure_future(self.ros_publish_gripper())
        self.task5 = asyncio.ensure_future(self.ros_publish_box())

        self._ros_service_btn.enabled = False
        self._ros_service_btn.text = "Stop Ros"
        self._ros_service_btn.set_clicked_fn(self._on_ros_stop_service)

    def myhook(self):
        print("     SHUTDOWN time!")

    def _on_ros_stop_service(self):
        import rospy
        rospy.on_shutdown(self.myhook)
        print(rospy.is_shutdown())
        if self.task1 is not None: self.task1.cancel()
        if self.task2 is not None: self.task2.cancel()
        if self.task3 is not None: self.task3.cancel()
        if self.task4 is not None: self.task4.cancel()
        if self.task5 is not None: self.task5.cancel()
        # self.task1 = None
        # self.task2 = None
        # self.task3 = None
        # self.task4 = None
        # self.task5 = None

        self._ros_service_btn.text = "Ros Start"
        self._ros_service_btn.set_clicked_fn(self._on_ros_service)

    def handle_paletting_cb1(self, req):
        import rospy
        # string robotname, string action, string parameter
        result = False
        print(req)
        self._scenario.reset_step_start()
        while True:
            if rospy.is_shutdown():
                break
            if req.palletizer == "Palletizer2":
                if req.action == "PalletizerStart":
                    result = self._scenario.perform_tasks()
                elif req.action == "PalletizerStop":
                    result = self._scenario.pause_tasks()
                else:
                    result = False
                if result is not None:
                    print("result: ", result)
                    return result
            else:
                return False
            time.sleep(0.1)

    def handle_paletting_cb2(self, req):
        import rospy
        # string robotname, string action, string parameter
        result = False
        print(req)
        self._scenario2.reset_step_start()
        while True :
            if rospy.is_shutdown():
                break
            if req.palletizer == "Palletizer3":
                if req.action == "PalletizerStart":
                    result = self._scenario2.perform_tasks()
                elif req.action == "PalletizerStop":
                    result = self._scenario2.pause_tasks()
                else:
                    result = False
                if result is not None:
                    print("result: ", result)
                    return result
            else:
                return False
            time.sleep(0.1)

    async def listener1(self):
        import rospy
        import rosservice
        # from tf.srv import PalletService_kgu
        from tf.srv import PalletService

        self.rosserver1 = None

        if not hasattr(self, "rosserver1") or self.rosserver1 is None:
            print("Rosservice Palletizer2")
            self.rosserver1 = rospy.Service("/Palletizer2_control", PalletService, self.handle_paletting_cb1)


    async def listener2(self):
        import rospy
         # from tf.srv import PalletService_kgu
        from tf.srv import PalletService
        self.rosserver2 = None

        if not hasattr(self, "rosserver2") or self.rosserver2 is None:
            print("Rosservice Palletizer3")
            self.rosserver2 = rospy.Service("/Palletizer3_control", PalletService, self.handle_paletting_cb2)

    async def ros_publish_packing(self):
        import rospy
        from tf.msg import PalletizerPackingFinish

        pub_packing = rospy.Publisher("/packingFinish", PalletizerPackingFinish, queue_size=10)
        msg = PalletizerPackingFinish()
        print("Ros Publish Palletizer Packing!!")

        while True:
            if rospy.is_shutdown() or not hasattr(self, "_scenario") or not hasattr(self, "_scenario2"):
                break
            result1 = self._scenario.packing_result()
            # print("Result_1:", result1)
            # print("Palletizer2 packing finished:", result1)
            if result1:
                print("Palletizer2 packing finished:", result1)
                msg.palletizer = "Palletizer2"
                msg.node = "1"
                pub_packing.publish(msg)
                self._scenario.reset_packingFinished()

            result2 = self._scenario2.packing_result()
            # print("Result_2:", result2)
            # print("Palletizer3 packing finished:", result2)
            if result2:
                print("Palletizer3 packing finished:", result2)
                msg.palletizer = "Palletizer3"
                msg.node = "4"
                pub_packing.publish(msg)
                self._scenario2.reset_packingFinished()
            await asyncio.sleep(0.2)

    async def ros_publish_gripper(self):
        import rospy
        from tf.msg import Gripper
        from tf2_msgs.msg import TFMessage
        from geometry_msgs.msg import TransformStamped

        print('Ros Publish Gripper!!')
        pub_gripper_state = rospy.Publisher("/Palletizer_gripper_state", Gripper, queue_size=10)
        pub_gripper1_pose = rospy.Publisher("/Palletizer2_gripper_pose", TFMessage, queue_size=10)
        pub_gripper2_pose = rospy.Publisher("/Palletizer3_gripper_pose", TFMessage, queue_size=10)

        gripper_msg = Gripper()
        tf = TransformStamped()

        while True:
            if rospy.is_shutdown() or not hasattr(self, "_scenario") or not hasattr(self, "_scenario2"):
                break

            # Palletizer2
            palletizer_id = "Palletizer2"
            gripper_state = self._scenario.pick_and_place._closed # open : False / close : True
            gripper_msg.state = gripper_state
            gripper_msg.gripper_name = palletizer_id

            # Gripper pose
            prim_path = "/environments/env/ur10/ee_link"
            frame_id = self._stage.GetPrimAtPath(prim_path)
            position, rotation = self._scenario.getObjectPose(frame_id)
            tf.header.frame_id = palletizer_id
            tf.child_frame_id = palletizer_id
            tf.header.stamp = rospy.Time.now()
            tf.transform.translation.x = position[0] / 100
            tf.transform.translation.y = position[1] / 100
            tf.transform.translation.z = position[2] / 100
            tf.transform.rotation.x = -float(rotation.split(',')[0].split('(')[-1])
            tf.transform.rotation.y = -float(rotation.split(',')[1].split(' ')[-1])
            tf.transform.rotation.z = -float(rotation.split(',')[2].split(' ')[-1])
            tf.transform.rotation.w = -float(rotation.split(',')[3].split(')')[0])
            tfm = TFMessage([tf])

            pub_gripper1_pose.publish(tfm)
            pub_gripper_state.publish(gripper_msg)


            ## Palletizer3
            palletizer_id = "Palletizer3"
            gripper_state = self._scenario2.pick_and_place._closed  # open : False / close : True
            gripper_msg.gripper_name = palletizer_id
            gripper_msg.state = gripper_state

            # Gripper pose
            prim_path = "/environments/env1/ur10/ee_link"
            frame_id = self._stage.GetPrimAtPath(prim_path)
            position, rotation = self._scenario.getObjectPose(frame_id)
            tf.header.frame_id = palletizer_id
            tf.child_frame_id = palletizer_id
            tf.header.stamp = rospy.Time.now()
            tf.transform.translation.x = position[0] / 100
            tf.transform.translation.y = position[1] / 100
            tf.transform.translation.z = position[2] / 100
            tf.transform.rotation.x = -float(rotation.split(',')[0].split('(')[-1])
            tf.transform.rotation.y = -float(rotation.split(',')[1].split(' ')[-1])
            tf.transform.rotation.z = -float(rotation.split(',')[2].split(' ')[-1])
            tf.transform.rotation.w = -float(rotation.split(',')[3].split(')')[0])
            tfm = TFMessage([tf])

            pub_gripper2_pose.publish(tfm)
            pub_gripper_state.publish(gripper_msg)

            await asyncio.sleep(0.2)

    async def ros_publish_box(self):
        import rospy
        from tf.msg import Mass
        from tf2_msgs.msg import TFMessage
        from geometry_msgs.msg import TransformStamped


        pub_box_mass = rospy.Publisher("/Box_weight", Mass, queue_size=10)
        pub_box_pose = rospy.Publisher("/Box_pose", TFMessage, queue_size=10)

        mass_msg = Mass()
        t = TransformStamped()


        while True:
            if rospy.is_shutdown() or not hasattr(self, "_scenario"):
                break
            for i in range(self._scenario.max_bins):
                frame_id = self._stage.GetPrimAtPath("/background/Box{0:02d}".format(i+1))
                weight = self._scenario.getObjectMass(frame_id)
                position, rotation = self._scenario.getObjectPose(frame_id)

                header_id = str(frame_id).split('/')[-1].split('>')[0].split('_')[-1]  # num of bins
                frame_id = str(frame_id).split('/')[-1].split('>')[0]  # name of bins

                # Mass publish
                mass_msg.child_frame_id = frame_id
                mass_msg.mass = round(weight, 5)
                pub_box_mass.publish(mass_msg)

                # Pose publish
                t.header.frame_id = header_id
                t.header.stamp = rospy.Time.now()
                t.transform.translation.x = position[0] / 100
                t.transform.translation.y = position[1] / 100
                t.transform.translation.z = position[2] / 100

                t.transform.rotation.x = -float(rotation.split(',')[0].split('(')[-1])
                t.transform.rotation.y = -float(rotation.split(',')[1].split(' ')[-1])
                t.transform.rotation.z = -float(rotation.split(',')[2].split(' ')[-1])
                t.transform.rotation.w = -float(rotation.split(',')[3].split(')')[0])
                pose_msg = TFMessage([t])

                pub_box_pose.publish(pose_msg)

            await asyncio.sleep(1.0)
