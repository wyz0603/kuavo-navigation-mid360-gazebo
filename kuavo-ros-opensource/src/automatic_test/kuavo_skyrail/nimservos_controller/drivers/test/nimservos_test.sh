export LD_LIBRARY_PATH=../lib/python-can-3.3.4/can/interfaces/bmcan:$LD_LIBRARY_PATH
export PYTHONPATH=..:../lib/python-can-3.3.4:$PYTHONPATH
export LD_NIMSERVOS_SDK_PATH=../src:$LD_NIMSERVOS_SDK_PATH
python3 nimservos_test.py
# python3 my_ruiwo.py
