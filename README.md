# thymio-test

for now ```nonblocking_comm_move.py``` is best at communication and movement.
We use threading to receive messages and use select library to have non-blocking I/O
and so non-blocking keybord input and get control back on the main thread. 