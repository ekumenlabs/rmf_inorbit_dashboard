import { useState } from 'react';

import { createContainer } from 'unstated-next';

export const MenusContainer = createContainer(() => {
  const [openConfigurationFileDialog, setConfigurationFileDialogOpen] = useState(true);
  const [apiKey, setApiKey] = useState();

  return {
    apiKey,
    setApiKey,
    openConfigurationFileDialog,
    setConfigurationFileDialogOpen,
  };
});
