import React from 'react';
import Box from '@mui/material/Box';

import { MenusContainer } from '../../containers/Menus.container';


function Map() {
  const { apiKey } = MenusContainer.useContainer();

  if (!apiKey) {
    return null;
  }

  return (
    <Box sx={{ flexGrow: 1, position: 'relative' }}>
      <iframe
        src={`https://embeds.inorbit.ai/embeds/D5NfCR53bt7g43gY9/M8xr67fIqVrJx1GfoN8a?ctx=()&appKey=${apiKey}`}
        width="1250"
        height="900"
        id="iframe_map"
      />
    </Box>
  );
}

export default Map;
