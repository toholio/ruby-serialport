require 'mkmf'

have_header("termios.h")
$CFLAGS += " -D_XOPENSOURCE"
create_makefile("serialport")
#with_config(debug)
