#!/usr/bin/env python3

# Copyright 2023 InOrbit, Inc.

import os
import sys

from ament_index_python.packages import get_package_share_directory
from contextlib import asynccontextmanager
from fastapi import FastAPI, Response, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
import rclpy

from .models.SubmissionError import SubmissionError
from .models.SubmissionResponse import SubmissionResponse
from .models.TaskDescription import TaskDescription
from .TaskRequester import TaskRequester


class AppStaticFilesConfig:
    """App static files configurations"""
    # This is the root URI.
    SOURCE_DIRECTORY="/"
    # This holds the directory where static files are deployed.
    DIRECTORY=os.path.join(get_package_share_directory('rmf_inorbit_dashboard'), 'static')
    # This helps to identify the static assets
    NAME="app_static_assets"

class State:
    """App state"""
    task_requester: TaskRequester


error_to_status_code = {
    SubmissionError.SERVICE_TIMEOUT: status.HTTP_503_SERVICE_UNAVAILABLE,
    SubmissionError.CALL_FAILED: status.HTTP_500_INTERNAL_SERVER_ERROR,
    SubmissionError.TASK_REJECTED: status.HTTP_400_BAD_REQUEST,
    SubmissionError.OTHER: status.HTTP_500_INTERNAL_SERVER_ERROR,
}


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    rclpy.init(args=sys.argv)
    State.task_requester = TaskRequester()
    # Receive requests
    yield
    # Cleanup
    rclpy.shutdown()


app = FastAPI(
    title="InOrbit/RMF task request API",
    description="REST API server designed to be consumed from the InOrbit/Ekumen RMF dashboard",
    version="0.1.0",
    license_info={
        "name": "3-Clause BSD License"
    },
    openapi_url="/api/openapi.json",
    openapi_tags=[
        {
            "name": "Tasks"
        }],
    lifespan=lifespan
)

'''
CORS configuration.
TODO: issue #112 remove CORS allowance if not needed.
'''
origins = [
    "http://localhost:8001",
    "http://localhost:5173",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=['POST', 'GET'],
    allow_headers=['*'],
    max_age=3600,
)

@app.post("/submit_task/", tags=['Tasks'], status_code=status.HTTP_201_CREATED,
          summary="Send a task request to the /submit_task service",
          response_description="Includes de generated task request, a description of the outcome of the operation and an error message if there is any")
def create_loop_request(description: TaskDescription, response: Response) -> SubmissionResponse:
    """
    Create a **Loop** task request

    - **start_time**: Number of seconds of delay after the task starts. Default: 0
    - **priority**: Task priority. Default: 0
    - **description**: Task description. Available options:
      - **Type `LoopDescription`**:
        - *start*: Vertex name of the start of the loop
        - *finish*: Vertex name of the end of the loop
        - *loop_num*: Number of loops to perform. Default: 1

    In the *Parameters* page, click *Schema* for the full schema
    """

    submission_response = State.task_requester.make_request(description)

    if submission_response.error:
        response.status_code = error_to_status_code[submission_response.error]

    return submission_response

# Configure the app to serve static files.
# See this StackOverflow comment: https://stackoverflow.com/a/73113792
# In a nutshell, *order matters*. By placing this mount point at the end, we solve
# the problem of routing /any/request/to/any/resource/ as a GET of a static file
# because it is mounted at /.
app.mount(AppStaticFilesConfig.SOURCE_DIRECTORY,
          StaticFiles(directory=AppStaticFilesConfig.DIRECTORY, html=True),
          name=AppStaticFilesConfig.NAME)
