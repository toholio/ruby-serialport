/* Ruby/SerialPort $Id$
 * Guillaume Pierronnet <moumar@netcourrier.com>
 *
 * This code is hereby licensed for public consumption under either the
 * GNU GPL v2 or greater, or Larry Wall's Artistic license - your choice.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Lot of documentation used in the development ( perhaps all :) comes from the 
 * "Serial Programming Guide for POSIX Operating Systems"
 * written Michael R. Sweet. Special thans to him :)
 * get it at: http://www.easysw.com/~mike/serial/
 */


#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <ruby.h>    /* ruby inclusion */

#define VERSION "0.1"

#if defined(linux)
#define CNEW_RTSCTS CRTSCTS
#endif
#if 0
#if defined(linux)
# include <linux/cdrom.h>

#elif defined(sun) && defined(unix) && defined(__SVR4)
# include <sys/cdio.h>

#elif defined(__FreeBSD__)
#include <sys/cdio.h>

#elif defined(__OpenBSD__) || defined(__NetBSD__)
#include <sys/cdio.h>
#else
# error "Your OS isn't supported yet."
#endif	/* os selection */
#endif
typedef struct _SP {
  int fd;	/* file descriptor */
  struct termios *params;  /* parameters for the port */
  struct termios *old_params;  /* old_parameters */
} SP;

VALUE cSerialPort; /* serial port class */
VALUE eSerialPort; /* serial port error class */ 

static SP *get_sp(obj)
  VALUE obj;
{
  SP *sp;
  Data_Get_Struct(obj, SP, sp);
  return sp;
}

static void free_sp(sp)
  SP *sp;
{
  free(sp->params);

  /* restore old values */
  tcsetattr(sp->fd, TCSADRAIN, sp->old_params);
  free(sp->old_params);
  free(sp);
  return;
}

static VALUE sp_init(self, _num_port, _data_rate, _data_bits, _parity, _stop_bits, _flow_control)
  VALUE self, _num_port, _data_rate, _data_bits, _parity, _stop_bits, _flow_control;
{
  SP *sp;
  char *ports[] = { "/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3" };
  int num_port;
  int data_bits = CS8;


  num_port = FIX2INT(_num_port);
  sp = (SP*)malloc(sizeof(SP));
  
  sp->fd = open(ports[num_port], O_RDWR | O_NOCTTY | O_NDELAY);

  if (sp->fd == -1)
    rb_raise(eSerialPort, "Failed to open port %d", num_port);

  fcntl(sp->fd, F_SETFL, 0); /* enable blocking read */

  sp->old_params = (struct termios*)malloc(sizeof(struct termios));
  tcgetattr(sp->fd, sp->old_params); /* save current parameters */
		  
  sp->params = (struct termios*)malloc(sizeof(struct termios));
  tcgetattr(sp->fd, sp->params);

  cfsetispeed(sp->params, FIX2INT(_data_rate));
  cfsetospeed(sp->params, FIX2INT(_data_rate));

  /*
   * Enable the receiver and set localmode
   */
  sp->params->c_cflag |= (CLOCAL | CREAD);
  
  /*
   * Set up parity
   */

  switch(FIX2INT(_parity)) {
    case 0: /* NONE */
    case 1: /* SPACE */
      sp->params->c_cflag &= ~PARENB;
      sp->params->c_cflag &= ~CSTOPB;
      break;
    
    case 2: /* EVEN */
      sp->params->c_cflag |= PARENB;
      sp->params->c_cflag &= ~PARODD;
      sp->params->c_cflag &= ~CSTOPB;
      break;

    case 3: /* ODD */
      sp->params->c_cflag |= PARENB;
      sp->params->c_cflag |= PARODD;
      sp->params->c_cflag &= ~CSTOPB;
      break;
  } 

  /*
   * Set up character size
   */

  switch(FIX2INT(_data_bits)) {
    case 5:
      data_bits = CS5;
      break;
    case 6:
      data_bits = CS6;
      break;
    case 7:
      data_bits = CS7;
      break;
  }
  sp->params->c_cflag &= ~CSIZE;
  sp->params->c_cflag |= data_bits;

  /*
   * Set up flow control
   */
  /* disable hardware flow control */
  sp->params->c_cflag &= ~CNEW_RTSCTS;
  
  if ( FIX2INT(_flow_control) == 1) ;
    sp->params->c_cflag |= CNEW_RTSCTS;

  tcsetattr(sp->fd, TCSANOW, sp->params);
  return Data_Wrap_Struct(self, 0, free_sp, sp);
}

static VALUE sp_close(self)
  VALUE self;
{
  SP *sp;

  sp = get_sp(self);
  close(sp->fd);
  
  return Qnil;
}

void Init_serialport() {
  
  eSerialPort = rb_define_class("SerialPortError", rb_eStandardError); 
  cSerialPort = rb_define_class("SerialPort", rb_cObject);
  rb_define_method(cSerialPort, "new", sp_init, 6);
  rb_define_method(cSerialPort, "close", sp_close, 0);
  rb_define_const(cSerialPort, "B50", INT2FIX(B50));
  rb_define_const(cSerialPort, "B75", INT2FIX(B75));
  rb_define_const(cSerialPort, "B110", INT2FIX(B110));
  rb_define_const(cSerialPort, "B134", INT2FIX(B134));
  rb_define_const(cSerialPort, "B150", INT2FIX(B150));
  rb_define_const(cSerialPort, "B200", INT2FIX(B200));
  rb_define_const(cSerialPort, "B300", INT2FIX(B300));
  rb_define_const(cSerialPort, "B600", INT2FIX(B600));
  rb_define_const(cSerialPort, "B1200", INT2FIX(B1200));
  rb_define_const(cSerialPort, "B1800", INT2FIX(B1800));
  rb_define_const(cSerialPort, "B2400", INT2FIX(B2400));
  rb_define_const(cSerialPort, "B4800", INT2FIX(B4800));
  rb_define_const(cSerialPort, "B9600", INT2FIX(B9600));
  rb_define_const(cSerialPort, "B19200", INT2FIX(B19200));
  rb_define_const(cSerialPort, "B38400", INT2FIX(B38400));
  rb_define_const(cSerialPort, "B57600", INT2FIX(B57600));
  /*rb_define_const(cSerialPort, "B76800", INT2FIX(B76800));*/
  rb_define_const(cSerialPort, "B115200", INT2FIX(B115200));

  rb_define_const(cSerialPort, "NONE", INT2FIX(0));
  rb_define_const(cSerialPort, "SPACE", INT2FIX(1));
  rb_define_const(cSerialPort, "EVEN", INT2FIX(2));
  rb_define_const(cSerialPort, "ODD", INT2FIX(3));

}
