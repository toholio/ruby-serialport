SPEC = Gem::Specification.new do |s|
   s.name = 'serialport'
   s.version = "0.7.1"
   s.summary  = "Library for using RS-232 serial ports."
   s.description = "SerialPort is a Ruby library that provides a class for using RS-232 serial ports."
   s.files = Dir.glob("{doc,src,lib,test}/**/*").delete_if { |item| item.include?( ".svn" ) }
   s.files.concat [ "README", "CHANGELOG" ]
   s.extensions << 'extconf.rb'
   s.has_rdoc = true
   s.extra_rdoc_files = [ "README", "src/serialport.c", "src/serialport.h" ]
   s.rdoc_options = [ "--main", "README" ]
   s.authors = ["Guillaume Pierronnet", "Alan Stern", "Daniel E. Shipton", "Tobin Richard"]
   s.email = "tobin.richard@gmail.com"
   s.homepage = "http://ruby-serialport.rubyforge.org"
end
