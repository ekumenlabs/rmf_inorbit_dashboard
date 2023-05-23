import { useState } from 'react';

import { createContainer } from 'unstated-next';

export const MapContainer = createContainer(() => {
  const [vertices, setVertices] = useState([]);
  const [lanes, setLanes] = useState([]);

  return {
    vertices,
    setVertices,
    lanes,
    setLanes,
  };
});
