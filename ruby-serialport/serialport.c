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
 * For documentation on serial programming, see the excellent:
 * "Serial Programming Guide for POSIX Operating Systems"
 * written Michael R. Sweet.
 * http://www.easysw.com/~mike/serial/
 */


#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <ruby.h>    /* ruby inclusion */

#define VERSION "0.1"

#if 0
#define CHECK_OPENED(sp) if(sp->fd == -1) rb_raise(eSerialPort, "Serial port is not opened.")
#endif
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
VALUE eSerialPortTimeout; /* serial port timeout error class */ 

static SP *get_sp(obj)
  VALUE obj;
{
  SP *sp;
  Data_Get_Struct(obj, SP, sp);
  if (sp->fd == -1)
    rb_raise(eSerialPort, "Serial port is not opened.");
  return sp;
}

static void close_sp(sp)
  SP *sp;
{
  tcsetattr(sp->fd, TCSADRAIN, sp->old_params);
  close(sp->fd);
  sp->fd = -1;
}

static void free_sp(sp)
  SP *sp;
{
  if(sp->fd != -1)
    close_sp(sp);    
  free(sp->params);
  free(sp->old_params);
  free(sp);
  return;
}

static void termios_setspeed(t_ios, speed)
  struct termios *t_ios;
  speed_t speed;
{
      cfsetispeed(t_ios, speed);
      cfsetospeed(t_ios, speed);
}

static VALUE sp_init(self, _num_port, _data_rate, _data_bits, _parity, _stop_bits, _flow_control, _timeout)
  VALUE self, _num_port, _data_rate, _data_bits, _parity, _stop_bits, _flow_control;
{
  SP *sp;
#if defined(linux)
  char *ports[] = { "/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3" };
#endif
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
  switch(FIX2INT(_data_rate)) {
    case 50: termios_setspeed(sp->params, FIX2INT(B50)); break;

    case 75: termios_setspeed(sp->params, FIX2INT(B75)); break; 

    case 110: termios_setspeed(sp->params, FIX2INT(B110)); break;

    case 134: termios_setspeed(sp->params, FIX2INT(B134)); break;

    case 150: termios_setspeed(sp->params, FIX2INT(B150)); break;

    case 200: termios_setspeed(sp->params, FIX2INT(B200)); break;

    case 300: termios_setspeed(sp->params, FIX2INT(B300)); break;

    case 600: termios_setspeed(sp->params, FIX2INT(B600)); break;

    case 1200: termios_setspeed(sp->params, FIX2INT(B1200)); break;

    case 1800: termios_setspeed(sp->params, FIX2INT(B1800)); break;

    case 2400: termios_setspeed(sp->params, FIX2INT(B2400)); break;

    case 4800: termios_setspeed(sp->params, FIX2INT(B4800)); break;

    case 9600: termios_setspeed(sp->params, FIX2INT(B9600)); break;

    case 19200: termios_setspeed(sp->params, FIX2INT(B19200)); break;

    case 38400: termios_setspeed(sp->params, FIX2INT(B38400)); break;

    case 57600: termios_setspeed(sp->params, FIX2INT(B57600)); break;

/*    case 76800: termios_setspeed(sp->params, FIX2INT(B76800)); break;
 */
    case 115200: termios_setspeed(sp->params, FIX2INT(B115200)); break;

    default:
      close_sp(sp);
      rb_raise(rb_eArgError, "Data rate is not supported.");
      break;
  }
  /*
   * Enable the receiver and set localmode
   */
  sp->params->c_cflag |= (CLOCAL | CREAD);
  
  /*
   * Set up parity
   */

  if (!strcmp(STR2CSTR(_parity), "none") ||
    !strcnmp(STR2CSTR(_parity), "space")) {
      sp->params->c_cflag &= ~PARENB;
      sp->params->c_cflag &= ~CSTOPB;
  }
  else if (!strcmp(STR2CSTR(_parity), "even")) {
      sp->params->c_cflag |= PARENB;
      sp->params->c_cflag &= ~PARODD;
      sp->params->c_cflag &= ~CSTOPB;
  }
  else if (!strcmp(STR2CSTR(_parity), "odd")) {
      sp->params->c_cflag |= PARENB;
      sp->params->c_cflag |= PARODD;
      sp->params->c_cflag &= ~CSTOPB;
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

  /*
   * Set up input type
   */

  /* raw */
  sp->params->c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
 
  /* canonical (line-oriented) */
  
  /* sp->params->c_lflag |= (ICANON | ECHO | ECHOE); */

  sp->params->c_cc[VMIN]  = 0;
  sp->params->c_cc[VTIME] = FIX2INT(_timeout);

  tcsetattr(sp->fd, TCSANOW, sp->params);
  return Data_Wrap_Struct(self, 0, free_sp, sp);
}

static VALUE sp_close(self)
  VALUE self;
{
  SP *sp;

  sp = get_sp(self);
  close_sp(sp);
  return Qnil;
}

static VALUE sp_read(self)
  VALUE self;
{
  SP *sp;
  char str[255];
  int ret;  


  sp = get_sp(self);
  ret = read(sp->fd, &str, 255);
  if ( ret < 0 )
    rb_raise(eSerialPort, "Failed to read.");
  else if (ret == 0)
    rb_raise(eSerialPortTimeout, "Timeout elapsed.");
 
  return rb_str_new(str, ret);

/*  char buf;  
  char *str;
  int ret;
  int size = 0;
  VALUE final_str;
#define STR_SIZE 255

  sp = get_sp(self);
  str = (char*)malloc(STR_SIZE);
retry:
  ret = read(sp->fd, &buf, 1);
  if(ret == 1) {
    *str++ = buf;
    size++;
    if (size < STR_SIZE)
      goto retry;
  }
  
  final_str = rb_str_new(str, size);
  free(str);
  return(final_str);
*/
}

static VALUE sp_write(self, str)
  VALUE self, str;
{
  SP *sp;
  char *src;
  long size_src;
  int ret;

  sp = get_sp(self);
  src = RSTRING(str)->ptr;
  size_src = RSTRING(str)->len;
  ret = write(sp->fd, src, size_src);  
  if (ret < 0)
    rb_raise(eSerialPort, "Error writing port.");
  return INT2FIX(ret);
}

void Init_serialport() {
  
  eSerialPort = rb_define_class("SerialPortError", rb_eStandardError);
  eSerialPortTimeout = rb_define_class("SerialPortTimeout", eSerialPort);
  cSerialPort = rb_define_class("SerialPort", rb_cObject);
  rb_define_singleton_method(cSerialPort, "new", sp_init, 7);
  rb_define_method(cSerialPort, "close", sp_close, 0);
  rb_define_method(cSerialPort, "read", sp_read, 0);
  rb_define_method(cSerialPort, "write", sp_write, 1);
}
