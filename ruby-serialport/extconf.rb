require 'mkmf'

exit(1) if not have_header("termios.h") or not have_header("unistd.h")
printf("checking for OS... ")
STDOUT.flush
os = /-([a-z]+)[0-9]*.*/.match(RUBY_PLATFORM)[1]
puts(os)
$CFLAGS += " -D#{os}"
create_makefile("serialport")
