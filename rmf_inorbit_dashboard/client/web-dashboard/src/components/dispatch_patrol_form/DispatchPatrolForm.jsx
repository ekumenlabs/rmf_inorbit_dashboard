import React, { useState, useRef, useEffect } from 'react';

import {
  Alert,
  Button,
  FormControl,
  Grid,
  InputLabel,
  MenuItem,
  Select,
  TextField,
  Typography,
} from '@mui/material';

function DispatchPatrolForm(props) {
  const {
    validVertices,
    sendPatrolRequest,
    apiResponse,
    setApiResponse,
    requestError,
    setRequestError,
  } = props;

  const validVerticesNamesArray = validVertices.map((vertex) => vertex[2].name);

  const selectableStartVertices = useRef(validVerticesNamesArray);
  const selectableEndVertices = useRef(validVerticesNamesArray);

  const [startVertex, setStartVertex] = useState('');
  const [startVertexError, setStartVertexError] = useState(false);

  const [endVertex, setEndVertex] = useState('');
  const [endVertexError, setEndVertexError] = useState(false);

  const [iterations, setIterations] = useState('1');
  const [iterationsError, setIterationsError] = useState(false);

  const [priority, setPriority] = useState('0');
  const [priorityError, setPriorityError] = useState(false);

  const [startDelay, setStartDelay] = useState('0');
  const [startDelayError, setStartDelayError] = useState(false);

  const [disableSubmitButton, setDisableSubmitButton] = useState(false);

  useEffect(() => {
    cleanForm();
  }, [apiResponse, requestError]);

  const cleanForm = () => {
    setTimeout(() => {
      setStartVertex('');
      setEndVertex('');
      setIterations('1');
      setPriority('0');
      setStartDelay('0');
      setApiResponse(undefined);
      setRequestError(undefined);
      setDisableSubmitButton(false);
    }, '4000');
  };

  const handleSubmit = async (event) => {
    event.preventDefault();

    setStartVertexError(false);
    setEndVertexError(false);
    setIterationsError(false);
    setPriorityError(false);
    setStartDelayError(false);

    if (!startVertex) {
      setStartVertexError(true);
      return;
    }
    if (!endVertex) {
      setEndVertexError(true);
      return;
    }
    if (!iterations || parseInt(iterations) <= 0 || !Number.isInteger(parseFloat(iterations))) {
      setIterationsError(true);
      return;
    }
    if (!priority || parseInt(priority) < 0 || !Number.isInteger(parseFloat(priority))) {
      setPriorityError(true);
      return;
    }
    if (!startDelay || parseInt(startDelay) < 0 || !Number.isInteger(parseFloat(startDelay))) {
      setStartDelayError(true);
      return;
    }

    setDisableSubmitButton(true);

    const taskDescription = {
      start_time: parseInt(startDelay),
      priority: parseInt(priority),
      description: {
        start: startVertex,
        finish: endVertex,
        loop_num: parseInt(iterations),
      },
    };

    sendPatrolRequest(taskDescription);
  };

  const handleStarVertexSelection = (event) => {
    setStartVertex(event.target.value);
    selectableEndVertices.current = validVerticesNamesArray.filter((vertex) => vertex !== event.target.value);
  };

  const handleEndVertexSelection = (event) => {
    setEndVertex(event.target.value);
    selectableStartVertices.current = validVerticesNamesArray.filter((vertex) => vertex !== event.target.value);
  };

  return (
    <form autoComplete="off" onSubmit={handleSubmit} noValidate>
      <Grid container spacing={2} padding={2}>
        <Typography spacing={2} padding={2} paddingBottom={0}>
          Select the start and destination vertices of the patrol.
        </Typography>

        <Grid item xs={12}>
          <FormControl fullWidth>
            <InputLabel id="select-start-vertex-label">Start</InputLabel>
            <Select
              labelId="select-start-vertex-label"
              id="select_start_vertex"
              value={startVertex}
              onChange={handleStarVertexSelection}
              fullWidth
              label="Start"
              error={startVertexError}
            >
              {selectableStartVertices.current.map((vertex) => (
                <MenuItem
                  key={vertex}
                  value={vertex}
                >
                  {vertex}
                </MenuItem>
              ))}
            </Select>
          </FormControl>

          {startVertexError
            ? (<Alert severity="error">Please select a starting point</Alert>)
            : null}
        </Grid>

        <Grid item xs={12}>
          <FormControl fullWidth>
            <InputLabel id="select-destination-vertex-label">Destination</InputLabel>
            <Select
              labelId="select-destination-vertex-label"
              id="select_start_vertex"
              value={endVertex}
              onChange={handleEndVertexSelection}
              fullWidth
              label="Destination"
              error={endVertexError}
            >
              {selectableEndVertices.current.map((vertex) => (
                <MenuItem
                  key={vertex}
                  value={vertex}
                >
                  {vertex}
                </MenuItem>
              ))}
            </Select>
          </FormControl>

          {endVertexError
            ? (<Alert severity="error">Please select a destination point</Alert>)
            : null}
        </Grid>

        <Grid item xs={12}>
          <hr />
        </Grid>

        <Typography spacing={2} padding={2} paddingBottom={0}>
          Modify the default values for Iterations, Priority and Start delay if needed.
        </Typography>

        <Grid item xs={12}>
          <TextField
            label="Iterations"
            onChange={(event) => setIterations(event.target.value)}
            required
            variant="outlined"
            color="primary"
            type="number"
            fullWidth
            value={iterations}
            error={iterationsError}
          />

          {iterationsError
            ? (<Alert severity="error">Please select the number of iterations, default 1. Have to be Integer.</Alert>)
            : null}
        </Grid>

        <Grid item xs={12}>
          <TextField
            label="Priority"
            onChange={(event) => setPriority(event.target.value)}
            required
            variant="outlined"
            color="primary"
            type="number"
            fullWidth
            value={priority}
            error={priorityError}
          />

          {priorityError
            ? (<Alert severity="error">Please select a natural integer number. Default 0</Alert>)
            : null}
        </Grid>

        <Grid item xs={12}>
          <TextField
            label="Start delay"
            onChange={(event) => setStartDelay(event.target.value)}
            required
            variant="outlined"
            color="primary"
            type="number"
            fullWidth
            value={startDelay}
            error={startDelayError}
          />

          {startDelayError
            ? (<Alert severity="error">Please select a natural integer number. Default 0</Alert>)
            : null}
        </Grid>

        <Grid item xs={12}>
          <Button
            variant="contained"
            color="primary"
            type="submit"
            fullWidth
            disabled={disableSubmitButton}
          >
            Submit task
          </Button>
        </Grid>

        <Grid item xs={12}>
          {apiResponse ? (<Alert severity="success">!! {apiResponse.data.message}</Alert>) : null}
          {requestError ? (<Alert severity="error">{`(message: ${requestError.msg},  type: ${requestError.type})`}</Alert>) : null}
        </Grid>
      </Grid>
    </form>
  );
}

export default DispatchPatrolForm;
