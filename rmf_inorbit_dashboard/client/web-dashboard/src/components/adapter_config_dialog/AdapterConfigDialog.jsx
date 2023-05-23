import React, { useRef } from 'react';
import './AdapterConfigDialog.css';


import {
  Alert,
  Button,
  Dialog,
  DialogContent,
  Grid,
  styled,
  TextField,
  Typography,
} from '@mui/material';

import {
  ErrorMessage,
  Formik,
  Form,
  Field,
  useField,
} from 'formik';

import { MenusContainer } from '../../containers/Menus.container';
import { handleFormValidation, lastValidatedAdapterConfigObj, lastValidatedNavGraphObj } from '../../hooks/useFormValidator';

const BootstrapDialog = styled(Dialog)(({ theme }) => ({
  '& .MuiDialogContent-root': {
    padding: theme.spacing(2),
  },
  '& .MuiDialogActions-root': {
    padding: theme.spacing(1),
  },
}));


export default function AdapterConfigDialog({ handleDialogResponse }) {
  const adapter_config_file_ref = useRef(null);
  const nav_graph_file_ref = useRef(null);
  const { openConfigurationFileDialog, setConfigurationFileDialogOpen } = MenusContainer.useContainer();


  return (
    <div>
      <BootstrapDialog
        aria-labelledby="customized-dialog-title"
        open={openConfigurationFileDialog}
      >
        <DialogContent dividers>
          <Typography gutterBottom variant="h6">
            Use the buttons below to choose and upload the configuration YAML files from your PC that matches the required schemas.
          </Typography>

          <Formik
            initialValues={{
              apiKey: '',
              adapter_config_file: '',
              nav_graph_file: '',
            }}

            validate={ async (values) => handleFormValidation(values)}

            onSubmit={async (values) => {
              const adapterConfig = lastValidatedAdapterConfigObj.currentValue;
              const navGraph = lastValidatedNavGraphObj.currentValue;
              handleDialogResponse({
                values,
                adapterConfig,
                navGraph,
              });
              setTimeout(() => {
                setConfigurationFileDialogOpen(false);
              }, 400);
            }}
          >
            {({ isSubmitting }) => (
              <Form>
                <Grid container spacing={2}>
                  <Grid item xs={12}>
                    <Typography>
                      Upload the adapter configuration file:
                    </Typography>

                    <FileUploadInput
                      name="adapter_config_file"
                      adapter_config_file_ref={adapter_config_file_ref}
                      id="adapter_config_file"
                      multiple={false}
                    />
                  </Grid>

                  <Grid item xs={12}>
                    <Typography>
                      Upload the navigation graph file:
                    </Typography>

                    <FileUploadInput
                      name="nav_graph_file"
                      nav_graph_file_ref={nav_graph_file_ref}
                      id="nav_graph_file"
                      multiple={false}
                    />
                  </Grid>

                  <Grid item xs={12}>
                    <Field
                      name="apiKey"
                      type="password"
                      as={TextField}
                      variant="outlined"
                      color="primary"
                      label="Please add your api-key"
                      fullWidth
                    />

                    <ErrorMessage name="apiKey">
                      { (msg) => <Alert severity="error">{msg}</Alert> }
                    </ErrorMessage>
                  </Grid>

                  <Grid item xs={12}>
                    <Button type="submit" fullWidth variant="contained" disabled={isSubmitting} >
                      submit
                    </Button>
                  </Grid>
                </Grid>
              </Form>
            )}

          </Formik>

        </DialogContent>
      </BootstrapDialog>
    </div>
  );
}

const FileUploadInput = ({ ...props }) => {
  const [field, meta] = useField(props);
  return (
    <div>
      <input type="file" accept=".yaml,.yml" {...field} {...props} />
      {meta.touched && meta.error ? (
        <Alert severity="error">{meta.error}</Alert>
      ) : null}
    </div>
  );
};
