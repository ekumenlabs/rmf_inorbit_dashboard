import React from 'react';
import Box from '@mui/material/Box';

import { MenusContainer } from '../../containers/Menus.container';
import { RobotsContainer } from '../../containers/Robots.container';


function Vitals() {
  const { robots } = RobotsContainer.useContainer();
  const { apiKey } = MenusContainer.useContainer();

  if (robots.length === 0 || !apiKey) {
    return null;
  }

  return (
    <Box sx={{ flexGrow: 1 }}>
      <iframe
        src={`https://embeds.inorbit.ai/embeds/D5NfCR53bt7g43gY9/mXOHeB40bK4e4hipE4TM?
        ctx=()&appKey=${apiKey}`}
        width="580"
        height="565"
        id="iframe_vitals"
      />
    </Box>
  );
}

export default Vitals;
