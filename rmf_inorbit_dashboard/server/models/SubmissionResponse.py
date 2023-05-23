from pydantic import BaseModel

from .SubmissionError import SubmissionError


class SubmissionResponse(BaseModel):
    message: str
    error: SubmissionError | None = None
    task_id: str | None = None
    description: str
