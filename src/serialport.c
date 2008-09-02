/* Ruby/SerialPort $Id$
 * Guillaume Pierronnet <moumar@netcourrier.com>
 * Alan Stern <stern@rowland.harvard.edu>
 * Daniel E. Shipton <dshipton@redshiptechnologies.com>
 *
 * This code is hereby licensed for public consumption under either the
 * GNU GPL v2 or greater.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * For documentation on serial programming, see the excellent:
 * "Serial Programming Guide for POSIX Operating Systems"
 * written Michael R. Sweet.
 * http://www.easysw.com/~mike/serial/
 */

#include "serialport.h"

VALUE cSerialPort; /* serial port class */

VALUE sBaud, sDataBits, sStopBits, sParity; /* strings */
VALUE sRts, sDtr, sCts, sDsr, sDcd, sRi;

static VALUE sp_create(class, _port)
   VALUE class, _port;
{
   return sp_create_impl(class, _port);
}

static VALUE sp_set_modem_params(argc, argv, self)
   int argc;
   VALUE *argv, self;
{
   return sp_set_modem_params_impl(argc, argv, self);
}

static VALUE sp_break(self, time)
   VALUE self, time;
{
   return sp_break_impl(self, time);
}

static VALUE sp_get_dtr(self)
   VALUE self;
{
   return sp_get_dtr_impl(self);
}

static VALUE sp_get_flow_control(self)
   VALUE self;
{
   return sp_get_flow_control_impl(self);
}

static VALUE sp_get_read_timeout(self)
   VALUE self;
{
   return sp_get_read_timeout_impl(self);
}

static VALUE sp_get_rts(self)
   VALUE self;
{
   return sp_get_rts_impl(self);
}

static VALUE sp_get_write_timeout(self)
   VALUE self;
{
   return sp_get_write_timeout_impl(self);
}

static VALUE sp_set_dtr(self, val)
{
   return sp_set_dtr_impl(self, val);
}

static VALUE sp_set_flow_control(self, val)
{
   return sp_set_flow_control_impl(self, val);
}

static VALUE sp_set_read_timeout(self, val)
{
   return sp_set_read_timeout_impl(self, val);
}

static VALUE sp_set_rts(self, val)
{
   return sp_set_rts_impl(self, val);
}

static VALUE sp_set_write_timeout(self, val)
{
   return sp_set_write_timeout_impl(self, val);
}

static void get_modem_params(self, mp)
   VALUE self;
   struct modem_params *mp;
{
   get_modem_params_impl(self, mp);
}

static VALUE sp_set_data_rate(self, data_rate)
   VALUE self, data_rate;
{
   VALUE argv[4];

   argv[0] = data_rate;
   argv[1] = argv[2] = argv[3] = Qnil;
   sp_set_modem_params(4, argv, self);

   return data_rate;
}

static VALUE sp_set_data_bits(self, data_bits)
   VALUE self, data_bits;
{
   VALUE argv[4];

   argv[1] = data_bits;
   argv[0] = argv[2] = argv[3] = Qnil;
   sp_set_modem_params(4, argv, self);

   return data_bits;
}

static VALUE sp_set_stop_bits(self, stop_bits)
   VALUE self, stop_bits;
{
   VALUE argv[4];

   argv[2] = stop_bits;
   argv[0] = argv[1] = argv[3] = Qnil;
   sp_set_modem_params(4, argv, self);

   return stop_bits;
}

static VALUE sp_set_parity(self, parity)
   VALUE self, parity;
{
   VALUE argv[4];

   argv[3] = parity;
   argv[0] = argv[1] = argv[2] = Qnil;
   sp_set_modem_params(4, argv, self);

   return parity;
}

static VALUE sp_get_data_rate(self)
   VALUE self;
{
   struct modem_params mp;

   get_modem_params(self, &mp);

   return INT2FIX(mp.data_rate);
}

static VALUE sp_get_data_bits(self)
   VALUE self;
{
   struct modem_params mp;

   get_modem_params(self, &mp);

   return INT2FIX(mp.data_bits);
}

static VALUE sp_get_stop_bits(self)
   VALUE self;
{
   struct modem_params mp;

   get_modem_params(self, &mp);

   return INT2FIX(mp.stop_bits);
}

static VALUE sp_get_parity(self)
   VALUE self;
{
   struct modem_params mp;

   get_modem_params(self, &mp);

   return INT2FIX(mp.parity);
}

static VALUE sp_get_modem_params(self)
   VALUE self;
{
   struct modem_params mp;
   VALUE hash;

   get_modem_params(self, &mp);

   hash = rb_hash_new();

   rb_hash_aset(hash, sBaud, INT2FIX(mp.data_rate));
   rb_hash_aset(hash, sDataBits, INT2FIX(mp.data_bits));
   rb_hash_aset(hash, sStopBits, INT2FIX(mp.stop_bits));
   rb_hash_aset(hash, sParity, INT2FIX(mp.parity));

   return hash;
}

static VALUE sp_get_cts(self)
   VALUE self;
{
   struct line_signals ls;

   get_line_signals_helper(self, &ls);

   return INT2FIX(ls.cts);
}

static VALUE sp_get_dsr(self)
   VALUE self;
{
   struct line_signals ls;

   get_line_signals_helper(self, &ls);

   return INT2FIX(ls.dsr);
}

static VALUE sp_get_dcd(self)
   VALUE self;
{
   struct line_signals ls;

   get_line_signals_helper(self, &ls);

   return INT2FIX(ls.dcd);
}

static VALUE sp_get_ri(self)
   VALUE self;
{
   struct line_signals ls;

   get_line_signals_helper(self, &ls);

   return INT2FIX(ls.ri);
}

static VALUE sp_signals(self)
   VALUE self;
{
   struct line_signals ls;
   VALUE hash;

   get_line_signals_helper(self, &ls);

   hash = rb_hash_new();

#if !(defined(OS_MSWIN) || defined(OS_BCCWIN))
   rb_hash_aset(hash, sRts, INT2FIX(ls.rts));
   rb_hash_aset(hash, sDtr, INT2FIX(ls.dtr));
#endif
   rb_hash_aset(hash, sCts, INT2FIX(ls.cts));
   rb_hash_aset(hash, sDsr, INT2FIX(ls.dsr));
   rb_hash_aset(hash, sDcd, INT2FIX(ls.dcd));
   rb_hash_aset(hash, sRi, INT2FIX(ls.ri));

   return hash;
}

void Init_serialport()
{
   sBaud = rb_str_new2("baud");
   sDataBits = rb_str_new2("data_bits");
   sStopBits = rb_str_new2("stop_bits");
   sParity = rb_str_new2("parity");
   sRts = rb_str_new2("rts");
   sDtr = rb_str_new2("dtr");
   sCts = rb_str_new2("cts");
   sDsr = rb_str_new2("dsr");
   sDcd = rb_str_new2("dcd");
   sRi = rb_str_new2("ri");

   rb_gc_register_address(&sBaud);
   rb_gc_register_address(&sDataBits);
   rb_gc_register_address(&sStopBits);
   rb_gc_register_address(&sParity);
   rb_gc_register_address(&sRts);
   rb_gc_register_address(&sDtr);
   rb_gc_register_address(&sCts);
   rb_gc_register_address(&sDsr);
   rb_gc_register_address(&sDcd);
   rb_gc_register_address(&sRi);

   cSerialPort = rb_define_class("SerialPort", rb_cIO);
   rb_define_singleton_method(cSerialPort, "create", sp_create, 1);

   rb_define_method(cSerialPort, "get_modem_params", sp_get_modem_params, 0);
   rb_define_method(cSerialPort, "set_modem_params", sp_set_modem_params, -1);
   rb_define_method(cSerialPort, "modem_params", sp_get_modem_params, 0);
   rb_define_method(cSerialPort, "modem_params=", sp_set_modem_params, -1);
   rb_define_method(cSerialPort, "baud", sp_get_data_rate, 0);
   rb_define_method(cSerialPort, "baud=", sp_set_data_rate, 1);
   rb_define_method(cSerialPort, "data_bits", sp_get_data_bits, 0);
   rb_define_method(cSerialPort, "data_bits=", sp_set_data_bits, 1);
   rb_define_method(cSerialPort, "stop_bits", sp_get_stop_bits, 0);
   rb_define_method(cSerialPort, "stop_bits=", sp_set_stop_bits, 1);
   rb_define_method(cSerialPort, "parity", sp_get_parity, 0);
   rb_define_method(cSerialPort, "parity=", sp_set_parity, 1);

   rb_define_method(cSerialPort, "flow_control=", sp_set_flow_control, 1);
   rb_define_method(cSerialPort, "flow_control", sp_get_flow_control, 0);

   rb_define_method(cSerialPort, "read_timeout", sp_get_read_timeout, 0);
   rb_define_method(cSerialPort, "read_timeout=", sp_set_read_timeout, 1);
   rb_define_method(cSerialPort, "write_timeout", sp_get_write_timeout, 0);
   rb_define_method(cSerialPort, "write_timeout=", sp_set_write_timeout, 1);

   rb_define_method(cSerialPort, "break", sp_break, 1);

   rb_define_method(cSerialPort, "signals", sp_signals, 0);
   rb_define_method(cSerialPort, "get_signals", sp_signals, 0);
   rb_define_method(cSerialPort, "rts", sp_get_rts, 0);
   rb_define_method(cSerialPort, "rts=", sp_set_rts, 1);
   rb_define_method(cSerialPort, "dtr", sp_get_dtr, 0);
   rb_define_method(cSerialPort, "dtr=", sp_set_dtr, 1);
   rb_define_method(cSerialPort, "cts", sp_get_cts, 0);
   rb_define_method(cSerialPort, "dsr", sp_get_dsr, 0);
   rb_define_method(cSerialPort, "dcd", sp_get_dcd, 0);
   rb_define_method(cSerialPort, "ri", sp_get_ri, 0);

   rb_define_const(cSerialPort, "NONE", INT2FIX(NONE));
   rb_define_const(cSerialPort, "HARD", INT2FIX(HARD));
   rb_define_const(cSerialPort, "SOFT", INT2FIX(SOFT));

   rb_define_const(cSerialPort, "SPACE", INT2FIX(SPACE));
   rb_define_const(cSerialPort, "MARK", INT2FIX(MARK));
   rb_define_const(cSerialPort, "EVEN", INT2FIX(EVEN));
   rb_define_const(cSerialPort, "ODD", INT2FIX(ODD));

   rb_define_const(cSerialPort, "VERSION", rb_str_new2(RUBY_SERIAL_PORT_VERSION));
}
