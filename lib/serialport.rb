require "serialport.so"

class SerialPort
   private_class_method(:create)

   def SerialPort::new(port, *params)
      sp = create(port)
      begin
         sp.set_modem_params(*params)
      rescue
         sp.close
         raise
      end
      return sp
   end

   def SerialPort::open(port, *params)
      sp = create(port)
      begin
         sp.set_modem_params(*params)
         if (block_given?)
            yield sp
            sp.close
            return nil
         end
      rescue
         sp.close
         raise
      end
      return sp
   end
end
