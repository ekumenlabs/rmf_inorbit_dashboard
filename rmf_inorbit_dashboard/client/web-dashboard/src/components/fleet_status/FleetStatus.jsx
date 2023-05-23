import React, { useEffect } from 'react';
import Box from '@mui/material/Box';

import { MenusContainer } from '../../containers/Menus.container';
import { RobotsContainer } from '../../containers/Robots.container';

function FleetStatus() {
  const { robots, setSelectedRobot } = RobotsContainer.useContainer();
  const { apiKey } = MenusContainer.useContainer();

  useEffect(() => {
    window.addEventListener('message', (event) => {
      const iframeFleetStatus = document.getElementById('iframe_fleet_status');
      const iframeVitals = document.getElementById('iframe_vitals');
      const iframeMap = document.getElementById('iframe_map');

      if (event.source.window === iframeFleetStatus.contentWindow && event.data.inOrbit && event.data.source !== 'react-devtools-bridge') {
        setSelectedRobot({ robotId: event.data.context.robot.robotId });

        const robotId = event.data.context && event.data.context.robot && event.data.context.robot.robotId;

        if (robotId) {
          iframeVitals.contentWindow.postMessage({ inOrbit: true, writeContext: { slot: 'robot', prop: 'robotId', value: robotId } }, '*');
          iframeMap.contentWindow.postMessage({ inOrbit: true, writeContext: { slot: 'robot', prop: 'robotId', value: robotId } }, '*');
        }
      }
    });
  }, []);

  if (robots.length === 0 || !apiKey) {
    return null;
  }

  return (
    <Box sx={{ flexGrow: 1 }}>
      <iframe
        src={`https://embeds.inorbit.ai/embeds/D5NfCR53bt7g43gY9/ucMfxcXq7uY3v9DitubM?ctx=()&appKey=${apiKey}`}
        width="580"
        height="437"
        id="iframe_fleet_status"
      />
    </Box>
  );
}

export default FleetStatus;
