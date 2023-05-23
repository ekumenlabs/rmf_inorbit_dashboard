import React from 'react';
import Box from '@mui/material/Box';
import { DataGrid } from '@mui/x-data-grid';

const columns = [
  { field: 'id',
    headerName: 'Task ID',
    width: 160,
    headerClassName: 'theme-header',
    cellClassName: 'theme-cell',
  },
  {
    field: 'assigned_to',
    headerName: 'Assigned',
    width: 130,
    editable: false,
    headerClassName: 'theme-header',
  },
  {
    field: 'start_time',
    headerName: 'Created at',
    width: 160,
    editable: false,
    headerClassName: 'theme-header',
    cellClassName: 'theme-cell',
  },
  {
    field: 'status',
    headerName: 'Status',
    width: 132,
    editable: false,
    headerClassName: 'theme-header',
  },
];

export default function TasksDataGrid({ runningTasks, handleRowClick }) {
  return (
    <Box
      sx={{
        height: 400,
        width: '100%',
        '& .theme-header': {
          backgroundColor: '#1565c0',
          color: '#fff'
        },
        '& .theme-cell': {
          backgroundColor: '#bdbdbd'
        }
      }}
    >
      <DataGrid
        rows={runningTasks}
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
        onRowClick={handleRowClick} { ...runningTasks }
      />
    </Box>
  );
}

