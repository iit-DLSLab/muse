# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/home/ynistico-iit.local/miniforge3/envs/muse/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/home/ynistico-iit.local/miniforge3/envs/muse/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install;/home/ynistico-iit.local/miniforge3/envs/muse'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3.11/site-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/devel/.private/state_estimator/env.sh')

output_filename = '/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/state_estimator/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
