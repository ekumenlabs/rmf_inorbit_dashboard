#!/usr/bin/env python3

# Copyright 2023 InOrbit, Inc.


import rclpy
from rclpy.parameter import Parameter
from rmf_task_msgs.msg import Loop, TaskType
from rmf_task_msgs.srv import SubmitTask

from .models.SubmissionError import SubmissionError
from .models.SubmissionResponse import SubmissionResponse
from .models.TaskDescription import TaskDescription, LoopDescription

# from server.models.SubmissionError import SubmissionError
# from .models.SubmissionResponse import SubmissionResponse
# from .models.TaskDescription import TaskDescription

# This class has been adapted from open_rmf/rmf_demos package:
# https://github.com/open-rmf/rmf_demos/blob/6533f32a5e5387ddc06d402957eb7adfc8e33d11/rmf_demos_tasks/rmf_demos_tasks/dispatch_loop.py
# The following is a summary of the introduced changes:
# - All logging is done via the node's logger, and some logging messages were changed
# - main() was renamed to make_request(), which now returns the results
# - Timeouts inside TaskRequester.main() were extracted as valued parameters of the method and their values were changed
# - It takes the task parameters from a data class
#
# The following is a copy of the original license note:
#
# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


class TaskRequester:
    """Generates and sends a request to /submit_task service of a loop action task"""

    def __init__(self, use_sim_time=False):
        self.node = rclpy.create_node('task_requester')
        self.submit_task_srv = self.node.create_client(
            SubmitTask, '/submit_task')

        # enable ros sim time
        if use_sim_time:
            self.node.get_logger().info("Using Sim Time")
            param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
            self.node.set_parameters([param])

    def generate_task_req_msg(self, task_description: TaskDescription):
        req_msg = SubmitTask.Request()

        assert isinstance(task_description.description,
                          LoopDescription), "Only loop tasks are allowed"

        req_msg.description.task_type.type = TaskType.TYPE_LOOP

        loop = Loop()
        loop.num_loops = task_description.description.loop_num
        loop.start_name = task_description.description.start
        loop.finish_name = task_description.description.finish
        req_msg.description.loop = loop

        ros_start_time = self.node.get_clock().now().to_msg()
        ros_start_time.sec += task_description.start_time
        req_msg.description.start_time = ros_start_time
        req_msg.description.priority.value = task_description.priority
        return req_msg

    def make_request(self, description: TaskDescription, service_timeout=3.0, response_timeout=3.0, task_delay_timeout=1.0) -> SubmissionResponse:
        req_msg = self.generate_task_req_msg(description)
        req_msg_str = str(req_msg)

        if not self.submit_task_srv.wait_for_service(timeout_sec=service_timeout):
            message = f'Could not reach /submit_task srv after {service_timeout} seconds'
            self.node.get_logger().error(message)
            return SubmissionResponse(
                error=SubmissionError.SERVICE_TIMEOUT,
                message=message,
                description=req_msg_str
            )

        rclpy.spin_once(self.node, timeout_sec=task_delay_timeout)
        self.node.get_logger().info(
            f"\nGenerated loop request: \n {req_msg}\n")
        self.node.get_logger().info("Submitting Loop Request")

        try:
            future = self.submit_task_srv.call_async(req_msg)
            rclpy.spin_until_future_complete(
                self.node, future, timeout_sec=response_timeout)
            call_response = future.result()
            if call_response is None:
                message = '/submit_task srv call failed'
                self.node.get_logger().error(message)
                return SubmissionResponse(
                    message=message,
                    error=SubmissionError.CALL_FAILED,
                    description=req_msg_str
                )
            elif not call_response.success:
                message = 'Dispatcher node failed to accept task'
                self.node.get_logger().error(message)
                return SubmissionResponse(
                    error=SubmissionError.TASK_REJECTED,
                    message=message,
                    description=req_msg_str
                )
            else:
                message = f'Request was successfully submitted and assigned task_id: [{call_response.task_id}]'
                self.node.get_logger().info(message)
                return SubmissionResponse(
                    message=message,
                    task_id=call_response.task_id,
                    description=req_msg_str
                )
        except Exception as e:
            message = 'Error! Submit Srv failed %r' % (e,)
            self.node.get_logger().error(message)
            return SubmissionResponse(
                message=message,
                error=SubmissionError.OTHER,
                description=req_msg_str
            )
