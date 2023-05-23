import moment from 'moment';

const handleLogs = (error) => {
  // eslint-disable-next-line no-console
  console.log(`Could not correctly parse the message and compose the object, error: ${error}`);
};

const handleParseTaskStateUpdate = (message) => {
  const taskObject = {};
  try {
    taskObject.assigned_to = message.data.assigned_to.name;
    taskObject.id = message.data.booking.id;
    taskObject.status = message.data.status;
    taskObject.start_time = moment(message.data.unix_millis_start_time).format('YY/MM/DD HH:mm:ss');
    taskObject.phases = [];
  } catch (error) {
    handleLogs(`from parseTaskStateUpdate error: ${error}`);
  }
  return taskObject;
};

const handleParseLogUpdate = (message) => {
  const hasPhases = (message) => Object.keys(message.data.phases).length !== 0 && message.data.phases.constructor === Object;

  const notAllPhasesAreEmptyObjects = (message) => {
    for (const key in message.data.phases) {
      if (Object.prototype.hasOwnProperty.call(message.data.phases, key)) {
        if (Object.keys(message.data.phases[key]).length !== 0 && message.data.phases[key].constructor === Object) {
          return true;
        }
      }
    }
  };

  const isEmptyObject = (obj) => Object.keys(obj).length === 0;

  if (!hasPhases(message) || !notAllPhasesAreEmptyObjects(message)) {
    handleLogs(`from parseLogUpdate invalid message == ${JSON.parse(message)}`);
    return;
  }

  const notEmptyMessages = Object.entries(message.data.phases).filter((phase) => !isEmptyObject(phase[1].events));

  if (notEmptyMessages.length === 0) {
    handleLogs('message have only empty phases');
    return;
  }

  const eventsAsArray = Object.values(notEmptyMessages[0][1].events);

  const parsedLogsArray = eventsAsArray[0].map((logEvent) => {
    const fromLogMessageExtractedData = {
      task_id: message.data.task_id,
      id: crypto.randomUUID(),
      msg: logEvent.text,
      time_stamp: moment(logEvent.unix_millis_time).format('YY/MM/DD HH:mm:ss'),
    };
    return fromLogMessageExtractedData;
  });

  return parsedLogsArray;
};

export {
  handleParseTaskStateUpdate,
  handleParseLogUpdate,
  handleLogs,
};
