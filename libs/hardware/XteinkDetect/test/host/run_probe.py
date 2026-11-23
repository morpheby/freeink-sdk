#!/usr/bin/env python3
"""Compile the real detector for C3 and S3 with GPIO/time stubs."""
from pathlib import Path
import subprocess
import tempfile

here = Path(__file__).resolve().parent
hardware = here.parents[2]
with tempfile.TemporaryDirectory() as directory:
    for name, flags in [('c3', ['FREEINK_DEVICE_X3', 'FREEINK_DEVICE_X4']),
                        ('pro', ['FREEINK_DEVICE_X4PRO', 'FREEINK_CAPTURE_PANEL_MTP'])]:
        binary = str(Path(directory)/name)
        subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=address,undefined',
                        *['-D'+flag+'=1' for flag in flags],
                        '-I'+str(here/'stubs'),
                        '-I'+str(hardware/'BoardConfig/include'),
                        '-I'+str(here.parents[1]/'include'),
                        str(here/'test_display_probe.cpp'),
                        str(here.parents[1]/'src/XteinkDetect.cpp'), '-o', binary],check=True)
        subprocess.run([binary],check=True)
