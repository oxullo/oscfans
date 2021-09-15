from PyInstaller.utils.hooks import collect_dynamic_libs

# from pathlib import Path
#
# import glfw
#
# glfw_lib = Path(glfw.__path__[0]) / 'libglfw.3.dylib'
#
# binaries=[(glfw_lib, '.')]

binaries = collect_dynamic_libs('glfw', '.')
