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

#ifndef _RUBY_SERIAL_PORT_H_
#define _RUBY_SERIAL_PORT_H_

#define RUBY_SERIAL_PORT_VERSION   "0.7.0"

#include <ruby.h>    /* ruby inclusion */
#include <rubyio.h>  /* ruby io inclusion */

struct modem_params
{
   int data_rate;
   int data_bits;
   int stop_bits;
   int parity;
};

struct line_signals
{
   int rts;
   int dtr;
   int cts;
   int dsr;
   int dcd;
   int ri;
};

#define NONE   0
#define HARD   1
#define SOFT   2

#if defined(OS_MSWIN) || defined(OS_BCCWIN)
   #define SPACE  SPACEPARITY
   #define MARK   MARKPARITY
   #define EVEN   EVENPARITY
   #define ODD    ODDPARITY
#else
   #define SPACE  0
   #define MARK   0
   #define EVEN   1
   #define ODD    2
#endif

static VALUE sBaud, sDataBits, sStopBits, sParity; /* strings */
static VALUE sRts, sDtr, sCts, sDsr, sDcd, sRi;

/* Implementation specific functions. */
VALUE sp_create_impl(VALUE class, VALUE _port);
VALUE sp_set_modem_params_impl(int argc, VALUE *argv, VALUE self);
void get_modem_params_impl(VALUE self, struct modem_params *mp);
VALUE sp_set_flow_control_impl(VALUE self, VALUE val);
VALUE sp_get_flow_control_impl(VALUE self);
VALUE sp_set_read_timeout_impl(VALUE self, VALUE val);
VALUE sp_get_read_timeout_impl(VALUE self);
VALUE sp_set_write_timeout_impl(VALUE self, VALUE val);
VALUE sp_get_write_timeout_impl(VALUE self);
VALUE sp_break_impl(VALUE self, VALUE time);
void get_line_signals_helper_impl(VALUE obj, struct line_signals *ls);
VALUE set_signal_impl(VALUE obj, VALUE val, int sig);
VALUE sp_set_rts_impl(VALUE self, VALUE val);
VALUE sp_set_dtr_impl(VALUE self, VALUE val);
VALUE sp_get_rts_impl(VALUE self);
VALUE sp_get_dtr_impl(VALUE self);

#endif
