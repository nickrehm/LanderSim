import PyInstaller.__main__

# this just generates an executable for the landerSim.py application so you don't have to hae a python environment setup

options = [
    '--onefile',    # Generate a single executable file
    'landerSim.py'     
]

PyInstaller.__main__.run(options)