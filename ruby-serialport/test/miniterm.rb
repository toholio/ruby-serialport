require "serialport"


if ARGV.size < 4
  STDERR.print <<EOF
  Usage: ruby #{$0} num_port bps nbits stopb
EOF
  exit(1)
end

sp = SerialPort.new(ARGV[0].to_i, ARGV[1].to_i, ARGV[2].to_i, ARGV[3].to_i, SerialPort::NONE)

while (l = STDIN.gets) do
  sp.write(l.sub("\n", "\r")); sleep 1
  print sp.read(sp.dispo).sub("\r", "\n")
end
sp.close
