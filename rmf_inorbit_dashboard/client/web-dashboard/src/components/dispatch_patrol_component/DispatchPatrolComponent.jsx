import React, { useState } from 'react';
import Box from '@mui/material/Box';
import Paper from '@mui/material/Paper';
import axios from 'axios';

import { MapContainer } from '../../containers/Map.container';

import DispatchPatrolForm from '../dispatch_patrol_form/DispatchPatrolForm';

export default function SimplePaper() {
  const { vertices } = MapContainer.useContainer();
  const [apiResponse, setApiResponse] = useState();
  const [requestError, setRequestError] = useState();

  const validVertices = vertices.filter((vertex) => vertex[2].name);

  const sendPatrolRequest = async (taskDescription) => {
    try {
      const URL = 'http://127.0.0.1:8001/submit_task/';
      const apiResponse = await axios.post(URL, taskDescription);
      setApiResponse({ ...apiResponse });
    } catch (error) {
      if (Array.isArray(error.response.data.detail)) {
        setRequestError({ ...error.response.data.detail[0] });
      } else {
        setRequestError({ msg: error.response.data.detail });
      }
    }
  };

  if (validVertices.length <= 0) {
    return null;
  }

  return (
    <Box
      sx={{
        display: 'flex',
        flexWrap: 'wrap',
      }}
    >
      <Paper elevation={8}>
        <DispatchPatrolForm
          validVertices={validVertices}
          sendPatrolRequest={sendPatrolRequest}
          apiResponse={apiResponse}
          setApiResponse={setApiResponse}
          requestError={requestError}
          setRequestError={setRequestError}
        />
      </Paper>
    </Box>
  );
}

