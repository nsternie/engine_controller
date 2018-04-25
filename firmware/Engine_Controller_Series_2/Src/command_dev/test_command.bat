gcc -c command.c --std=c99 -gstabs
gcc -c command_test.c --std=c99 -gstabs
gcc -o command_test command_test.o command.o
REM gdb command_test.exe
command_test.exe