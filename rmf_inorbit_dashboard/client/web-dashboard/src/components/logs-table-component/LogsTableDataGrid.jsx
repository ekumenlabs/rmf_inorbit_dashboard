import React from 'react';
import Box from '@mui/material/Box';
import { DataGrid } from '@mui/x-data-grid';


const columns = [
  { field: 'time_stamp',
    headerName: 'Time-stamp',
    width: 160,
    headerClassName: 'theme-header',
    sortable: false,
  },
  {
    field: 'msg',
    headerName: 'Log message',
    width: 422,
    editable: false,
    headerClassName: 'theme-header',
    sortable: false,
  },
];

export default function LogsTableDataGrid({ requestedLogs }) {

  return (
    <Box
      sx={{
        height: 400,
        width: '100%',
        '& .theme-header': {
          backgroundColor: '#1565c0',
          color: '#fff'
        },
      }}
    >
      <DataGrid
        rows={requestedLogs}
        columns={columns}
        initialState={{
          pagination: {
            paginationModel: {
              pageSize: 5,
            },
          },
        }}
        pageSizeOptions={[5]}
        checkboxSelection={false}
        disableRowSelectionOnClick
      />
    </Box>
  );
}
