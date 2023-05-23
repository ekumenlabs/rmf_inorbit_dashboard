import { useState } from 'react';
import { createContainer } from 'unstated-next';

export const RobotsContainer = createContainer(() => {
  const [robots, setRobots] = useState([]);
  const [selectedRobot, setSelectedRobot] = useState({});
  const [runningTasks, setRunningTasks] = useState([]);

  return {
    robots,
    setRobots,
    selectedRobot,
    setSelectedRobot,
    runningTasks,
    setRunningTasks,
  };
});
