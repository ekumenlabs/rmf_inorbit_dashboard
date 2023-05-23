from enum import Enum


class SubmissionError(Enum):
    SERVICE_TIMEOUT = "Service timeout"
    CALL_FAILED = "Service call failed"
    TASK_REJECTED = "Task was rejected"
    OTHER = "Unknown exception"
