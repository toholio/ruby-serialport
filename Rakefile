require 'rubygems'
Gem::manage_gems
require 'rake/gempackagetask'

spec = Gem::Specification.new do |s|
   s.platform = Gem::Platform::CURRENT
   s.name = 'ruby-serialport'
   s.version = "0.7.0"
   s.summary = "Ruby/SerialPort is a Ruby library that provides a class for using RS-232 serial ports."

   s.files = Dir.glob("{doc,src,lib,test}/**/*").delete_if { |item| item.include?( ".svn" ) }
   s.files.concat [ "README", "CHANGELOG" ]
   s.extensions << 'extconf.rb'
   s.has_rdoc = true
   s.extra_rdoc_files = [ "README", "src/serialport.c", "src/serialport.h" ]
   s.rdoc_options = [ "--main", "README" ]
   s.authors = ["Guillaume Pierronnet", "Alan Stern", "Daniel E. Shipton"]
   s.email = "daniel.shipton.oss@gmail.com"
   s.homepage = "http://ruby-serialport.rubyforge.org"
end

Rake::GemPackageTask.new(spec) do |pkg|
   pkg.need_tar = true
end

task :default => "pkg/#{spec.name}-#{spec.version}-#{spec.platform}.gem" do
   puts " #{spec.name} => pkg/#{spec.name}-#{spec.version}-#{spec.platform}.gem generated"
end
