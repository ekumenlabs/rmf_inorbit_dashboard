import React from 'react';
import ReactDOM from 'react-dom/client';

import App from './App';
import './index.css';
import { ContainerProviders } from './containers';

ReactDOM.createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    <ContainerProviders>
      <App />
    </ContainerProviders>
  </React.StrictMode>
);
