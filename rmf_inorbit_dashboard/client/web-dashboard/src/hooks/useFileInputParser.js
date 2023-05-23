/* eslint-disable no-alert */

const handleParseFiles = async (adapterConfig, navGraph, apiKey) => {
  try {
    const appData = {
      robots: [],
      vertices: [],
      lanes: [],
      apiKey,
    };

    for (const key in adapterConfig.robots) {

      if (Object.prototype.hasOwnProperty.call(adapterConfig.robots, key)) {
        const robot = {
          robot_name: key,
          robot_id: adapterConfig.robots[key].robot_config.robot_id,
        };

        appData.robots.push(robot);
      }
    }

    /**
     * Dev Note (quiquequique)
     * At this stage, We use only the first one if there is more than one level in the represented world and discard the others.
     */
    const { levels } = navGraph;
    const firstLevelKeyName = Object.keys(levels)[0];
    appData.vertices = levels[firstLevelKeyName].vertices;
    appData.lanes = levels[firstLevelKeyName].lanes;

    return appData;

  } catch (error) {

    alert(`An error occurred while parsing your configuration file. Please provide a YAML file with a valid schema.
    Error: ${error.message}`);
  }
};

export { handleParseFiles };
