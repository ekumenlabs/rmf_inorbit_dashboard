import axios from 'axios';
import { parse } from 'yaml';
import { z } from 'zod';

const lastValidatedAdapterConfigObj = {
  currentValue: undefined,
};

const lastValidatedNavGraphObj = {
  currentValue: undefined,
};

const handleFormValidation = async (values) => {
  const apiKeyValidationSchema = z.object({
    apiKey: z.string().min(16, { message: 'provide an api-key with sixteen characters' }).max(16),
  });

  const adapterConfigBasicSchema = z.object({
    map: z.object({}),
    rmf_fleet: z.object({}),
    robots: z.object({}),
  });

  const oneRobotSchema = z.object({
    rmf_config: z.object({
      charger: z.object({ waypoint: z.string().min(1) }),
      robot_state_update_frequency: z.number(),
      start: z.object({}),
    }),
    robot_config: z.object({
      actions: z.object({}),
      battery_charge_attribute_id: z.string().min(1),
      charging_status_attribute_id: z.string().min(1),
      max_delay: z.number(),
      robot_id: z.string().min(1),
    }),
  });

  const navGraphSchema = z.object({
    building_name: z.string(),
    levels: z.object({}),
  }).strict();

  const oneLevelSchema = z.object({
    lanes: z.array(z.array(z.unknown())),
    vertices: z.array(z.array(z.unknown())),
  }).strict();

  const errors = {
    apiKey: '',
    adapter_config_file: '',
    nav_graph_file: '',
  };

  if (values) {
    try {
      const apiKeyValidationResult = apiKeyValidationSchema.safeParse({ apiKey: values.apiKey });

      if (!apiKeyValidationResult.success) {
        errors.apiKey = apiKeyValidationResult.error.issues[0].message;
      }

      const convertYamlToObject = async (yamlFileId) => {
        const uploadFileElement = document.getElementById(yamlFileId);
        const blob = new Blob(uploadFileElement.files, { type: 'text/yaml' });
        return parse(await blob.text());
      };

      if (values.adapter_config_file) {
        const adapterConfig = await convertYamlToObject('adapter_config_file');

        const basicYamlValidationResult = adapterConfigBasicSchema.safeParse({ ...adapterConfig });

        if (!basicYamlValidationResult.success) {
          errors.adapter_config_file = `error in schema, Message: (${basicYamlValidationResult.error.issues[0].message})
          Path: (${basicYamlValidationResult.error.issues[0].path[0]})`;
        }

        const { robots } = adapterConfig;
        for (const key in robots) {
          if (Object.prototype.hasOwnProperty.call(adapterConfig.robots, key)) {

            const robotValidation = oneRobotSchema.safeParse({ ...robots[key] });

            if (!robotValidation.success) {
              errors.adapter_config_file = `error in the robots configuration (${robotValidation.error.issues[0].message})
              at path (${robotValidation.error.issues[0].path[0]}/${robotValidation.error.issues[0].path[1]})`;
            }
          }
        }

        if (!errors.adapter_config_file) {
          // The adapter config file is valid.
          lastValidatedAdapterConfigObj.currentValue = adapterConfig;
        }

      } else {
        errors.adapter_config_file = 'yaml file is required';
      }

      if (values.nav_graph_file) {
        const navGraph = await convertYamlToObject('nav_graph_file');

        const navGraphBasicValidationResult = navGraphSchema.safeParse({ ...navGraph });

        if (!navGraphBasicValidationResult.success) {
          errors.nav_graph_file = `error in schema, Message: (${navGraphBasicValidationResult.error.issues[0].message})
          Path: (${navGraphBasicValidationResult.error.issues[0].path[0]})`;
        } else {
          const { levels } = navGraph;
          // Only the first level will be validated. If there are more levels, they will be omitted from the code.
          const firstLevelKeyName = Object.keys(levels)[0];
          const levelValidation = oneLevelSchema.safeParse({ ...levels[firstLevelKeyName] });

          if (!levelValidation.success) {
            errors.nav_graph_file = `error in Level schema, Message: (${levelValidation.error.issues[0].message})
            Path: (${levelValidation.error.issues[0].path[0]})`;
          } else {
            // The nav graph file is valid.
            lastValidatedNavGraphObj.currentValue = navGraph;
          }
        }
      } else {
        errors.nav_graph_file = 'yaml file is required';
      }

      if (!errors.apiKey && !errors.adapter_config_file) {

        /**
         * Dev notes (quiquequique):
         * There is no specific endpoint in InOrbit API to check the validity of the key.
         * We call the /robots endpoint and only use the error if there is any.
         */
        try {
          const apiTestUrl = 'https://api.inorbit.ai/robots';
          const apiKeyInorbitValidationResult = await axios.get(apiTestUrl, { headers: { 'x-auth-inorbit-app-key': `${values.apiKey}` } });

        } catch (error) {
          errors.apiKey = `invalid API KEY, with error: ${error.response.data.error}`;
        }
      }

      for (const key in errors) {
        if (Object.prototype.hasOwnProperty.call(errors, key)) {
          if (errors[key]) {
            return errors;
          }
        }
      }

      return undefined;

    } catch (error) {
      // eslint-disable-next-line no-alert
      alert(error.message);
    }
  }
};

export { handleFormValidation, lastValidatedAdapterConfigObj, lastValidatedNavGraphObj };
