require 'mkmf'

printf("checking for OS... ")
STDOUT.flush
os = /-([a-z]+)[0-9]*.*/.match(RUBY_PLATFORM)[1]
puts(os)
$CFLAGS += " -D#{os}"
create_makefile("serialport")
#with_config(debug)
