
PYTHON_PATH=`python -c "import sysconfig; print(sysconfig.get_path('include'))"`
PYTHON_LIBPATH=`python -c "import sysconfig; print(sysconfig.get_path('stdlib'))"`

export CPATH=$PYTHON_PATH:$CPATH
export LD_LIBRARY_PATH=$PYTHON_LIBPATH:$LD_LIBRARY_PATH
