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

#define _ALL_SOURCE	1
#define VERSION 	"0.1"

#define NONE 		0
#define HARD 		1
#define SOFT 		2

#define SPACE		0
#define MARK		0
#define EVEN		1
#define ODD		2


#include <stdio.h>   /* Standard input/output definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>

#include <ruby.h>    /* ruby inclusion */
#include <rubyio.h>  /* ruby io inclusion */

#ifdef CRTSCTS
#define HAVE_FLOWCONTROL_HARD 1
#else
#undef HAVE_FLOWCONTROL_HARD
#endif

VALUE cSerialPort; /* serial port class */

static int
sp_get_fd(obj)
  VALUE obj;
{
  OpenFile *fptr;

  GetOpenFile(obj, fptr);

  return (fileno(fptr->f));
}

static VALUE
sp_new(class, _port, _data_rate, _data_bits, _stop_bits, _parity)
  VALUE class, _port, _data_rate, _data_bits, _parity, _stop_bits;
{
  OpenFile *fp;
  int fd;
  int num_port;
  char *port;
  char *ports[] = { 
#if defined(linux)
  "/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3"
#elif defined(freebsd) || defined(netbsd) || defined(openbsd)
  "/dev/cuaa0", "/dev/cuaa1", "/dev/cuaa2", "/dev/cuaa3", 
#elif defined(solaris)
  "/dev/ttya", "/dev/ttyb", "/dev/ttyc", "/dev/ttyd"
#elif defined(aix)
  "/dev/tty0", "/dev/tty1", "/dev/tty2", "/dev/tty3"
#elif defined(irix)
  "/dev/ttyf1", "/dev/ttyf2", "/dev/ttyf3", "/dev/ttyf4"
#endif
  };
  struct termios params;
  int data_bits;
  int data_rate;
  
  NEWOBJ(sp, struct RFile);
  
  Check_Type(_data_rate, T_FIXNUM);
  Check_Type(_data_bits, T_FIXNUM);
  Check_Type(_stop_bits, T_FIXNUM);
  Check_Type(_parity, T_FIXNUM);

  OBJSETUP(sp, class, T_FILE);
  MakeOpenFile(sp, fp);

  switch(TYPE(_port)) {
    case T_FIXNUM:
      num_port = FIX2INT(_port);
      port = ports[num_port];
      break;

    case T_STRING:
      port = RSTRING(_port)->ptr;
      break;

    default:
      rb_raise(rb_eTypeError, "wrong argument type");
      break;
  }
  
  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

  if (fd == -1)
    rb_sys_fail(port);

  fp->f = rb_fdopen(fd, "r+");
  fp->mode = FMODE_READWRITE|FMODE_SYNC;
 
  switch(FIX2INT(_data_rate)) {
    case 50:    data_rate = B50; break;
    case 75:    data_rate = B75; break; 
    case 110:   data_rate = B110; break;
    case 134:   data_rate = B134; break;
    case 150:   data_rate = B150; break;
    case 200:   data_rate = B200; break;
    case 300:   data_rate = B300; break;
    case 600:   data_rate = B600; break;
    case 1200:  data_rate = B1200; break;
    case 1800:  data_rate = B1800; break;
    case 2400:  data_rate = B2400; break;
    case 4800:  data_rate = B4800; break;
    case 9600:  data_rate = B9600; break;
    case 19200: data_rate = B19200; break;
    case 38400: data_rate = B38400; break;
#ifdef B57600
    case 57600: data_rate = B57600; break;
#endif
#ifdef B76800
    case 76800: data_rate = B76800; break;
#endif
#ifdef B115200
    case 115200: data_rate = B115200; break;
#endif
#ifdef B230400
    case 230400: data_rate = B230400; break;
#endif

    default:
      close(fd);
      rb_raise(rb_eArgError, "unknown baud rate");
      break;
  }

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
    case 8:
      data_bits = CS8;
      break;
    default:
      close(fd);
      rb_raise(rb_eArgError, "unknown character size");
      break;
  }
  params.c_cflag = data_bits | CLOCAL | CREAD;

  switch(FIX2INT(_parity)) {
    case EVEN:
      params.c_cflag |= PARENB;
      params.c_cflag &= ~PARODD;
      break;

    case ODD:
      params.c_cflag |= PARENB;
      params.c_cflag |= PARODD;
      break;

    case NONE:
      params.c_cflag &= ~PARENB;
      break;

    default:
      close(fd);
      rb_raise(rb_eArgError, "unknown parity");
      break;
      
  }
  /*
   * Set up stop bits
   */
  switch(FIX2INT(_stop_bits)) {
    case 1:
      params.c_cflag &= ~CSTOPB;
      break;
    case 2:
      params.c_cflag |= CSTOPB;
      break;
    default:
      close(fd);
      rb_raise(rb_eArgError, "unknown number of stop bits");
    break;
  }

  params.c_oflag = 0;
  params.c_lflag = 0; 
  cfsetispeed(&params, data_rate);
  cfsetospeed(&params, data_rate);

  params.c_cc[VMIN] = 1;
  params.c_cc[VTIME] = 0;


  /* enable blocking read */
  fcntl(fd, F_SETFL, 0);
/*  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK); */
  tcsetattr(fd, TCSANOW, &params);
#if 0
  tcsetattr(fd, TCSANOW, &params);
  params.c_cflag = CS8 | CLOCAL | CREAD;
  params.c_oflag = 0;
  params.c_lflag = 0;
  cfsetspeed(&params, data_rate);
  tcsetattr(fd, TCSAFLUSH, &params);
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);
#endif

  return (VALUE)sp;
}

static VALUE sp_set_flow_control(self, val)
  VALUE self, val;
{
  int fd;
  int flowc;
  struct termios params;

  Check_Type(val, T_FIXNUM);

  fd = sp_get_fd(self);
  tcgetattr(fd, &params);

  flowc = FIX2INT(val);
  if ( flowc == NONE ) {
#ifdef HAVE_FLOWCONTROL_HARD
    params.c_cflag &= ~CRTSCTS;
#endif
    params.c_iflag &= ~(IXON | IXOFF | IXANY);
  }
  else {
    if ( flowc & HARD )
#ifdef HAVE_FLOWCONTROL_HARD
      params.c_cflag |= CRTSCTS;
#else
      rb_raise(rb_eIOError, "Hardware flow control not supported");
#endif
    if ( flowc & SOFT )
      params.c_iflag |= (IXON | IXOFF | IXANY);
  }   

  tcsetattr(fd, TCSANOW, &params);
  return val;
}
static VALUE sp_get_flow_control(self)
  VALUE self;
{
  int ret;
  int fd;
  struct termios params;

  fd = sp_get_fd(self);
  tcgetattr(fd, &params);
  ret = 0;
#ifdef HAVE_FLOWCONTROL_HARD
  if ( params.c_cflag & CRTSCTS)
    ret += HARD;
#endif
  if ( params.c_iflag & (IXON | IXOFF | IXANY))
    ret += SOFT;
 
  return INT2FIX(ret);
}

/* break */
static VALUE sp_break(self, time)
  VALUE self, time;
{
  int fd;
 
  Check_Type(time, T_FIXNUM);

  fd = sp_get_fd(self);
  tcsendbreak(fd, FIX2INT(time));
  return Qnil;
}
static int get_signal(obj, sig)
  VALUE obj;
  int sig;
{
  int fd, status;
  fd = sp_get_fd(obj);
  ioctl(fd, TIOCMGET, &status);
  
  return ( sig & status ? 1 : 0 );
} 

static VALUE set_signal(obj, val, sig)
  VALUE obj,val;
  int sig;
{
  int status;
  int fd;
  int set;

  Check_Type(val, T_FIXNUM);
  fd = sp_get_fd(obj);
  ioctl(fd, TIOCMGET, &status);
  set = FIX2INT(val);
  if (set == 0)
    status &= ~sig;
  else if (set == 1)
    status |= sig;
  else
    rb_raise(rb_eArgError, "Invalid value");
  ioctl(fd, TIOCMSET, &status);
  return INT2FIX(get_signal(obj, sig));
}

/* RTS */
static VALUE sp_set_rts(self, val)
  VALUE self, val;
{
    return set_signal(self, val, TIOCM_RTS);
}

static VALUE sp_get_rts(self)
  VALUE self;
{
  return INT2FIX(get_signal(self, TIOCM_RTS));
}

/* DTR */
static VALUE sp_set_dtr(self, val)
  VALUE self, val;
{
  return set_signal(self, val, TIOCM_DTR);
}

static VALUE sp_get_dtr(self)
  VALUE self;
{
  return INT2FIX(get_signal(self, TIOCM_DTR));
}
/* CTS */
static VALUE sp_get_cts(self)
  VALUE self;
{
  return INT2FIX(get_signal(self, TIOCM_CTS));
}
/* DSR */
static VALUE sp_get_dsr(self)
  VALUE self;
{
  return INT2FIX(get_signal(self, TIOCM_DSR));
}
/* DCD */
static VALUE sp_get_dcd(self)
  VALUE self;
{
  return INT2FIX(get_signal(self, TIOCM_CD));
}
/* RI */
static VALUE sp_get_ri(self)
  VALUE self;
{
  return INT2FIX(get_signal(self, TIOCM_RI));
}

static VALUE
sp_signals(self)
  VALUE self;
{
  VALUE hash;

  hash = rb_hash_new();
  rb_hash_aset(hash, rb_str_new2("rts"), sp_get_rts(self));
  rb_hash_aset(hash, rb_str_new2("dtr"), sp_get_dtr(self));
  rb_hash_aset(hash, rb_str_new2("cts"), sp_get_cts(self));
  rb_hash_aset(hash, rb_str_new2("dsr"), sp_get_dsr(self));
  rb_hash_aset(hash, rb_str_new2("dcd"), sp_get_dcd(self));
  rb_hash_aset(hash, rb_str_new2("ri"), sp_get_ri(self));
  return hash;
}
/*
static VALUE
sp_dispo(self)
  VALUE self;
{
  int fd, ret;
  fd = sp_get_fd(self);

  ioctl(fd, FIONREAD, &ret);

  return INT2FIX(ret);
}
*/
void Init_serialport() {
  
  cSerialPort = rb_define_class("SerialPort", rb_cIO);
  rb_define_singleton_method(cSerialPort, "new", sp_new, 5);

  rb_define_method(cSerialPort, "flow_control=", sp_set_flow_control, 1);
  rb_define_method(cSerialPort, "flow_control", sp_get_flow_control, 0);

/*  rb_define_method(cSerialPort, "dispo", sp_dispo, 0);*/

  rb_define_method(cSerialPort, "break", sp_break, 1);
 
  rb_define_method(cSerialPort, "signals", sp_signals, 0);
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

}
