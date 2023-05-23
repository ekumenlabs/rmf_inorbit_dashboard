from pydantic import BaseModel


# Schemas for task description: https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter/schemas
class LoopDescription(BaseModel):
    start: str
    finish: str
    loop_num: int = 1

# If other task types were implemented:
# class CleanDescription(BaseModel):
#     start: str
#     etc
# The description field in TaskDescription would look like `description: LoopDescription | CleanDescription`


class TaskDescription(BaseModel):
    start_time: int = 0
    priority: int = 0
    description: LoopDescription
