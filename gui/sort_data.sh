cd $1
mkdir datalogs
mkdir commandlogs
mkdir seriallogs
mkdir pythonlogs
mv *_datalog* datalogs
mv *_command_log* commandlogs
mv *_serial_log* seriallogs
mv *_python_log* pythonlogs