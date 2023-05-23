from glob import glob
import os
import subprocess
from setuptools import setup, find_packages

package_name = 'rmf_inorbit_dashboard'

# Base list of data_files
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name),
        glob('launch/*launch.[pxy][yma]*')),
]

def package_files(data_files, directory_list):
    """
    Packages files recursively from directory_list and appends it to data_files.
    """
    paths_dict = {}

    for directory in directory_list:
        for (path, _, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

class NPMBuilder:
    """
    This class cleans and builds the static web assets.
    """
    CREATE_STATIC_DIR_CMD='mkdir -p static'
    CREATE_STATIC_DIR_PATH='.'
    NPM_RUN_BUILD_CMD='npm run build'
    NPM_DIR_PATH='rmf_inorbit_dashboard/client/web-dashboard'
    CLEAR_STATIC_DIR_CMD='rm -rf static'
    CLEAR_DIR_PATH='.'

    def _run(self, cmd, cwd):
        process = subprocess.run(cmd, cwd=cwd, shell=True, text=True,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT)
        if process.returncode != 0:
            if process.stdout:
                print(process.stdout)
            if process.stderr:
                print(process.stderr)
            print('error: "{}" failed with: {}'.format(cmd, process.returncode))
        return process.returncode

    def build(self):
        # Remove the static asset directory.
        error_code = self._run(NPMBuilder.CLEAR_STATIC_DIR_CMD,
                               NPMBuilder.CLEAR_DIR_PATH)
        if error_code != 0:
            return error_code

        # Create the static asset directory.
        error_code = self._run(NPMBuilder.CREATE_STATIC_DIR_CMD,
                               NPMBuilder.CREATE_STATIC_DIR_PATH)
        if error_code != 0:
            return error_code

        # Run npm build command.
        error_code = self._run(NPMBuilder.NPM_RUN_BUILD_CMD,
                               NPMBuilder.NPM_DIR_PATH)
        return error_code

def build_and_package_data_files(data_files, directory_list):
    """
    Builds the static files and packages all the data files
    to be installed together with this package.

    npm build cannot be injected as another build stage because
    it generates files that within the setup file. We need to know
    the list of files to be distributed before we call setup() to
    be passed in as part of 'data_files'.
    """
    error_code = NPMBuilder().build()
    if error_code != 0:
        print('Failed to build the static assets.')
        exit(error_code)

    return package_files(data_files, directory_list)

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=build_and_package_data_files(data_files, ['static/']),
    install_requires=['setuptools'],
    zip_safe=True,
    author="Tomás Badenes tomasbadenes@gmail.com",
    maintainer='Julian Cerruti',
    maintainer_email='julian@inorbit.ai',
    description='REST and WS API designed to be consumed from the InOrbit/Ekumen RMF dashboard',
    license='3-Clause BSD License',
    entry_points={
        'console_scripts': [
            'ws_server=rmf_inorbit_dashboard.server.ws_server:main'
        ],
    },
)
