import React from 'react';

import ResponsiveAppBar from './components/top_bar/TopBar';
import MainComponent from './components/main_component/MainComponent';

function App() {

  return (
    <div className="App">
      <ResponsiveAppBar />
      <MainComponent />
    </div>
  );
}

export default App;
