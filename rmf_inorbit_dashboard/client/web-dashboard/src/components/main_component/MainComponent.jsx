import React, { useState, useEffect } from 'react';
import Box from '@mui/material/Box';
import Grid from '@mui/material/Grid';

import { handleParseFiles } from '../../hooks/useFileInputParser';
import {
  handleParseTaskStateUpdate,
  handleParseLogUpdate,
  handleLogs,
} from '../../hooks/useWebSocketMessagesParsers';
import { RobotsContainer } from '../../containers/Robots.container';
import { MenusContainer } from '../../containers/Menus.container';
import { MapContainer } from '../../containers/Map.container';

import AdapterConfigDialog from '../adapter_config_dialog/AdapterConfigDialog';
import Map from '../map/Map';
import WidgetsAccordion from '../accordion_component/AccordionComponent';

export default function MainComponent() {
  const [ws, setWs] = useState(null);
  const [wsRestart, setWsRestart] = useState(0);
  const {
    setRobots,
    runningTasks,
    setRunningTasks,
  } = RobotsContainer.useContainer();
  const { apiKey, setApiKey } = MenusContainer.useContainer();
  const { setLanes, setVertices } = MapContainer.useContainer();
  const wsUrl = 'ws://localhost:8000';

  // Connect to websockets.
  useEffect(() => {
    const wsClient = new WebSocket(wsUrl);

    wsClient.onopen = () => {
      // eslint-disable-next-line no-console
      console.log('<< The web socket is open >>');
      setWs(wsClient);
    };

    wsClient.onclose = () => {
      // eslint-disable-next-line no-console
      console.log('<< The web socket is closed >>');
      setTimeout(() => {
        setWsRestart((prev) => prev + 1);
      }, 3000);
    };
  }, [wsRestart]);

  useEffect(() => {
    if (ws) {
      ws.onmessage = (event) => {
        try {
          addObjectIfDifferent(JSON.parse(event.data));

        } catch (error) {
          handleLogs(`The web socket received and discarded an invalid message. Invalid message == ${event.data}`);
        }
      };
    }
  }, [ws, runningTasks]);

  const addObjectIfDifferent = (message) => {
    const newRunningTasks = [...runningTasks];

    if (message.type === 'task_state_update') {
      const parsedTask = handleParseTaskStateUpdate(message);
      if (parsedTask) {
        insertNewTaskMessage(parsedTask, newRunningTasks, message);
      }

    } else if (message.type === 'task_log_update') {
      const parsedLogsArray = handleParseLogUpdate(message);
      if (parsedLogsArray.length > 0) {
        parsedLogsArray.forEach((log) => insertNewLogMessage(log));
      }

    } else {
      handleLogs(`unknown or unused message type == ${message.type}`);
    }
  };

  const insertNewTaskMessage = (parsedTask, newRunningTasks, message) => {
    if (message.data.booking.id && typeof message.data.booking.id === 'string') {
      const runningTaskIndex = newRunningTasks.findIndex((task) => task.id === message.data.booking.id);
      if (runningTaskIndex === -1) {
        newRunningTasks.push(parsedTask);

      } else if (newRunningTasks[runningTaskIndex].status !== parsedTask.status) {
        parsedTask.phases = newRunningTasks[runningTaskIndex].phases;
        parsedTask.start_time = newRunningTasks[runningTaskIndex].start_time;
        newRunningTasks.splice(runningTaskIndex, 1, parsedTask);

      } else {
        handleLogs(
          `from insertNewTaskMessage. The new parsed task status message equals the stored one, ergo is not inserted in the component state.
           parsed task == ${JSON.stringify(parsedTask)}`
        );
        return;
      }

      setRunningTasks(newRunningTasks);
    }
  };

  const insertNewLogMessage = (log) => {
    const nextRunningTasks = runningTasks.map((task) => {
      if (task.id !== log.task_id) {
        return task;
      }
      const newLog = {
        msg: log.msg,
        time_stamp: log.time_stamp,
        id: log.id,
      };
      return {
        ...task,
        phases: [newLog, ...task.phases]
      };
    });
    setRunningTasks(nextRunningTasks);
  };

  const handleDialogResponse = async (validationResponse) => {
    const appData = await handleParseFiles(
      validationResponse.adapterConfig,
      validationResponse.navGraph,
      validationResponse.values.apiKey
    );

    setRobots(appData.robots);
    setApiKey(appData.apiKey);
    setLanes(appData.lanes);
    setVertices(appData.vertices);
  };

  if (!apiKey) {
    return (
      <AdapterConfigDialog
        handleDialogResponse={handleDialogResponse}
      />
    );
  }

  return (
    <Box sx={{ flexGrow: 1, backgroundColor: '#e7eef6' }}>
      <Grid container>
        <Grid item xs={8}>
          <Map />
        </Grid>
        <Grid item xs={4}>
          <WidgetsAccordion />
        </Grid>
      </Grid>
    </Box>
  );
}
