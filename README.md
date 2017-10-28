# malmo-project

## Dependencies

To run this project you will need:
* [Python 2.7 64-bit](https://www.python.org/downloads/release/python-2713/)
* [Malmo 0.30](https://github.com/Microsoft/malmo/releases)
* Setup `AIMA_PATH`, `MALMO_ROOT`, `MALMO_XSD_PATH` & `JAVA_HOME`

## Possible issues and resolution
* If you're getting DDL %1 load failed, you probably have a 32-bit version of python, you can check this in a shell by typing `python --version`
* If `import MalmoPython` is failing, there is an path issue. A workaround for this is to copy MalmoPython.pyp & MalmoPython.lib over to your working directory
