# marble_status_monitor
It's kinda fragile right now but it works. Currently working on making this more robust, modular and will add some useful features so check in later. I'll also add better instructions and maybe an example.

This package will monitor topic publishing rates specified by a given config file. At a specified rate (default is 1 Hz), the number of topics publishing at the correct rates will be printed on the terminal. If there are topics not being published correctly, then those topic names will be printed. 
