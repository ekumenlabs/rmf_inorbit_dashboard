import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// https://vitejs.dev/config/
export default defineConfig({
	plugins: [react()],
	build: {
		// We are dumping the distribution outside the root directory.
		// This flag allows to override the contents without a warning.
		emptyOutDir: true,
		// Distribution directory.
		outDir: '../../../static',
	}
});
