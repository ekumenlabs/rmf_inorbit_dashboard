import React from 'react';

import { MenusContainer } from './Menus.container';
import { RobotsContainer } from './Robots.container';
import { MapContainer } from './Map.container';

// The order here is important. Each provider can inherit from the provider before it
const providers = [
  MenusContainer.Provider,
  MapContainer.Provider,
  RobotsContainer.Provider,
];

export const ContainerProviders = React.memo(({ children }) => {
  // Put the entire app in the lowest part of the chain
  let current = children;

  // Iteratively build the provider contexts (reversed)
  providers.reverse().forEach((Tag) => {
    current = <Tag>{current}</Tag>;
  });

  return current;
});
