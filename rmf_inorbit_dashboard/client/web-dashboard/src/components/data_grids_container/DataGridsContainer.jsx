import React, { useState, useEffect } from 'react';

import IconButton from '@mui/material/IconButton';
import Typography from '@mui/material/Typography';
import SwapVerticalCircleOutlinedIcon from '@mui/icons-material/SwapVerticalCircleOutlined';

import TaskDataGrid from '../data_grid_component/DataGridComponent';
import LogsTableDataGrid from '../logs-table-component/LogsTableDataGrid';
import { RobotsContainer } from '../../containers/Robots.container';

export default function DataGridsContainer() {
  const {
    runningTasks,
  } = RobotsContainer.useContainer();

  const [selectedTaskNameforLogs, setSelectedTaskNameforLogs] = useState({ name: '' });
  const [requestedLogs, setRequestedLogs] = useState([]);
  const [tasksTableOpen, setTasksTableOpen] = useState(true);

  useEffect(() => {
    if (runningTasks.length === 0 || !selectedTaskNameforLogs) {
      return;
    }
    const requestedLogs = runningTasks.filter((task) => task.id === selectedTaskNameforLogs.name);

    if (requestedLogs.length > 0 && requestedLogs[0].phases) {
      setRequestedLogs([...requestedLogs[0].phases]);
    }
  }, [runningTasks, selectedTaskNameforLogs]);

  const togleTables = () => {
    setTasksTableOpen(!tasksTableOpen);
  };

  const handleRowClick = (params) => {
    setRequestedLogs([]);
    setSelectedTaskNameforLogs({ name: params.row.id });
    setTasksTableOpen(!tasksTableOpen);
  };

  return (
    <>
      <div style={{ display: 'flex', justifyContent: 'space-between', padding: '1em 1em 0' }}>
        {
          selectedTaskNameforLogs.name && !tasksTableOpen
            ? <Typography sx={{ width: '70%', flexShrink: 0 }}>Logs for task: {selectedTaskNameforLogs.name}</Typography>
            : <span />
        }
        <IconButton
          size={selectedTaskNameforLogs.name ? 'large' : ''}
          color="secondary"
          onClick={() => togleTables()}
          disabled={!selectedTaskNameforLogs.name}
        >
          <SwapVerticalCircleOutlinedIcon fontSize="inherit" />
        </IconButton>
      </div>

      {
        tasksTableOpen
          ? <TaskDataGrid runningTasks={runningTasks} handleRowClick={handleRowClick} />
          : null
      }

      {
        !tasksTableOpen
          ? <LogsTableDataGrid requestedLogs={requestedLogs} />
          : null
      }
    </>
  );
}
