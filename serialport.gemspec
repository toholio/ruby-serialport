SPEC = Gem::Specification.new do |s|
  s.name = 'serialport'
  s.version = '0.7.2'
  s.summary  = 'Library for using RS-232 serial ports.'
  s.description = 'SerialPort is a Ruby library that provides a class for using RS-232 serial ports.'
  s.files = [ 'CHANGELOG',
              'MANIFEST',
              'README',
              'serialport.gemspec',
              'extconf.rb',
              'lib/serialport.rb',
              'src/posix_serialport_impl.c',
              'src/serialport.c',
              'src/serialport.h',
              'src/win_serialport_impl.c' ]
  s.test_files = [ 'test/miniterm.rb' ]
  s.extensions << 'extconf.rb'
  s.has_rdoc = true
  s.extra_rdoc_files = [ 'README', 'src/serialport.c', 'src/serialport.h' ]
  s.rdoc_options = [ '--main', 'README' ]
  s.authors = ['Guillaume Pierronnet', 'Alan Stern', 'Daniel E. Shipton', 'Tobin Richard']
  s.email = 'tobin.richard@gmail.com'
  s.homepage = 'http://github.com/toholio/ruby-serialport/'
end
