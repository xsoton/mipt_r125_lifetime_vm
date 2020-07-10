#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <stdarg.h>

#include <gpib/ib.h>

// === config
#define HANTEK_TMC "/dev/usbtmc0"
#define PPS_GPIB_NAME "AKIP-1142/3G"
#define VM_GPIB_NAME "AKIP-V7-78/1"
#define IF_GPIB_NAME "GPIB-USB-HS"

// === pps
#define PPS_TIMEOUT_S 1

// === vm
#define VM_TIMEOUT_S 1
#define VM_POINTS 2000

// === osc
#define OSC_TIMEOUT_S 1
#define OSC_TRIGGER_DELAY_US 100

// === common
#define REPETITIONS 10

// === threads
void *commander(void *);
void *worker(void *);

// === utils
int get_run();
void set_run(int run_new);
double get_time();

int gpib_open(const char *name);
int gpib_close(int dev);
int gpib_read(int dev, char *buf, size_t buf_length);
int gpib_write(int dev, const char *str);
int gpib_print(int dev, const char *format, ...);
void gpib_print_error(int dev);

int usbtmc_open(const char *name);
int usbtmc_close(int dev);
int usbtmc_read(int dev, char *buf, size_t buf_length);
int usbtmc_write(int dev, const char *str);
int usbtmc_print(int dev, const char *format, ...);

// === global variables
char dir_name[200];
pthread_rwlock_t run_lock;
int run;
const char *experiment_name;

// === program entry point
int main(int argc, char const *argv[])
{
	int ret = 0;
	int status;

	time_t start_time;
	struct tm start_time_struct;

	pthread_t t_commander;
	pthread_t t_worker;

	// === check input parameters
	if (argc < 2)
	{
		fprintf(stderr, "# E: Usage: lifetime <experiment_name>\n");
		ret = -1;
		goto main_exit;
	}
	experiment_name = argv[1];

	// === get start time of experiment
	start_time = time(NULL);
	localtime_r(&start_time, &start_time_struct);

	// === we need actual information w/o buffering
	setlinebuf(stdout);
	setlinebuf(stderr);

	// === initialize run state variable
	pthread_rwlock_init(&run_lock, NULL);
	run = 1;

	// === create dirictory in "20191012_153504_<experiment_name>" format
	snprintf(dir_name, 100, "%04d-%02d-%02d_%02d-%02d-%02d_%s",
		start_time_struct.tm_year + 1900,
		start_time_struct.tm_mon + 1,
		start_time_struct.tm_mday,
		start_time_struct.tm_hour,
		start_time_struct.tm_min,
		start_time_struct.tm_sec,
		experiment_name
	);
	status = mkdir(dir_name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	if (status == -1)
	{
		fprintf(stderr, "# E: unable to create experiment directory (%s)\n", strerror(errno));
		ret = -2;
		goto main_exit;
	}

	// === now start threads
	pthread_create(&t_commander, NULL, commander, NULL);
	pthread_create(&t_worker, NULL, worker, NULL);

	// === and wait ...
	pthread_join(t_worker, NULL);

	// === cancel commander thread becouse we don't need it anymore
	// === and wait for cancelation finish
	pthread_cancel(t_commander);
	pthread_join(t_commander, NULL);
	printf("\n");

	main_exit:
	return ret;
}

// === commander function
void *commander(void *arg)
{
	(void) arg;

	char str[100];
	char *s;
	int ccount;

	while(get_run())
	{
		printf("> ");

		s = fgets(str, 100, stdin);
		if (s == NULL)
		{
			fprintf(stderr, "# E: Exit\n");
			set_run(0);
			break;
		}

		switch(str[0])
		{
			case 'h':
				printf(
					"Help:\n"
					"\th -- this help;\n"
					"\tq -- exit the program;\n");
				break;
			case 'q':
				set_run(0);
				break;
			default:
				ccount = strlen(str)-1;
				fprintf(stderr, "# E: Unknown command (%.*s)\n", ccount, str);
				break;
		}
	}

	return NULL;
}

// === worker function
void *worker(void *arg)
{
	(void) arg;

	int r;

	int osc_fd;
	int pps_fd;
	int vm_fd;
	int if_fd;

	char filename_lt[100];
	char gnuplot_cmd[100];
	FILE *lt_fp;
	FILE *gp;
	char buf[100];

	char *rbuf;
	const size_t rbufsize = 2000000;

	rbuf = (char *) malloc(rbufsize);
	if (rbuf == NULL)
	{
		fprintf(stderr, "# E: Unable to allocate %ld bytes (%s)\n", rbufsize, strerror(errno));
		goto worker_nomem;
	}

	// === first we are connecting to instruments
	r = osc_fd = usbtmc_open(HANTEK_TMC);  if(r == -1) goto worker_open_hantek;
	r = pps_fd = gpib_open(PPS_GPIB_NAME); if(r == -1) goto worker_pps_ibfind;
	r = vm_fd  = gpib_open(VM_GPIB_NAME);  if(r == -1) goto worker_vm_ibfind;
	r = if_fd  = gpib_open(IF_GPIB_NAME);  if(r == -1) goto worker_if_ibfind;

	// === init pps
	gpib_write(pps_fd, "output 0");
	gpib_write(pps_fd, "instrument:nselect 1");
	gpib_write(pps_fd, "voltage:limit 11V");
	gpib_write(pps_fd, "voltage 10.0");
	gpib_write(pps_fd, "current 0.1");
	gpib_write(pps_fd, "channel:output 1");
	gpib_write(pps_fd, "instrument:nselect 2");
	gpib_write(pps_fd, "voltage:limit 5.5V");
	gpib_write(pps_fd, "voltage 5.0");
	gpib_write(pps_fd, "current 0.15");
	gpib_write(pps_fd, "channel:output 1");
	gpib_write(pps_fd, "instrument:nselect 1");

	sleep(PPS_TIMEOUT_S);

	// === init vm
	gpib_write(vm_fd, "function \"voltage:dc\"");
	gpib_write(vm_fd, "voltage:dc:range:auto off");
	gpib_write(vm_fd, "voltage:dc:range 10");
	gpib_write(vm_fd, "voltage:dc:resolution 1e-5");
	gpib_write(vm_fd, "voltage:dc:nplcycles 0.02");
	gpib_write(vm_fd, "trigger:source immediate");
	gpib_write(vm_fd, "trigger:delay:auto off");
	gpib_write(vm_fd, "trigger:delay 0");
	gpib_write(vm_fd, "trigger:count 1");
	gpib_write(vm_fd, "data:feed rdg_store, \"calculate\"");
	gpib_print(vm_fd, "sample:count %d", VM_POINTS);

	sleep(VM_TIMEOUT_S);

	gpib_print_error(vm_fd);

	// === init osc
	usbtmc_write(osc_fd, "dds:switch 0");
	usbtmc_write(osc_fd, "channel1:display off");
	usbtmc_write(osc_fd, "channel2:display off");
	usbtmc_write(osc_fd, "channel3:display off");
	usbtmc_write(osc_fd, "channel4:display off");
	usbtmc_write(osc_fd, "dds:type dc");
	usbtmc_write(osc_fd, "dds:offset 0");
	usbtmc_write(osc_fd, "dds:wave:mode off");
	usbtmc_write(osc_fd, "dds:burst:switch off");
	usbtmc_write(osc_fd, "dds:switch 1");

	sleep(OSC_TIMEOUT_S);

	ibconfig(if_fd, IbcAUTOPOLL, 1);

	// === create log file
	snprintf(filename_lt, 100, "%s/lifetime.dat", dir_name);

	lt_fp = fopen(filename_lt, "w+");
	if(lt_fp == NULL)
	{
		fprintf(stderr, "# E: Unable to open file \"%s\" (%s)\n", filename_lt, strerror(errno));
		goto worker_lt_fopen;
	}
	setlinebuf(lt_fp);

	// === write vac header
	r = fprintf(lt_fp,
		"# mipt_r125_lifetime_vm\n"
		"# Decay curves\n"
		"# Experiment name \"%s\"\n"
		"#\n"
		"# Columns:\n"
		"# 1 - index\n"
		"# 2 - time, s\n"
		"# 3 - sample voltage, V\n"
		"# 4 - sample current, A\n"
		"# 5 - laser voltage, V\n"
		"# 6 - laser current, A\n"
		"# 7 - measurement duration, A\n",
		experiment_name
	);
	if(r < 0)
	{
		fprintf(stderr, "# E: Unable to print to file \"%s\" (%s)\n", filename_lt, strerror(r));
		goto worker_lt_header;
	}

	// === open gnuplot
	snprintf(gnuplot_cmd, 100, "gnuplot > %s/gnuplot.log 2>&1", dir_name);
	gp = popen(gnuplot_cmd, "w");
	if (gp == NULL)
	{
		fprintf(stderr, "# E: unable to open gnuplot pipe (%s)\n", strerror(errno));
		goto worker_gp_popen;
	}
	setlinebuf(gp);

	// === prepare gnuplot
	r = fprintf(gp,
		"set xlabel \"Time, s\"\n"
		"set ylabel \"Voltage, V\"\n"
	);
	if(r < 0)
	{
		fprintf(stderr, "# E: Unable to print to gp (%s)\n", strerror(r));
		goto worker_gp_settings;
	}

	// === let the action begins!
	for (int lt_index = 0; lt_index < REPETITIONS; ++lt_index)
	{
		double lt_time;
		double sample_voltage;
		double sample_current;
		double laser_voltage;
		double laser_current;
		char   buf[100];

		if (get_run())
		{
			lt_time = get_time();
			if (lt_time < 0)
			{
				fprintf(stderr, "# E: Unable to get time\n");
				set_run(0);
				break;
			}

			gpib_write(pps_fd, "measure:voltage:all?");
			gpib_read(pps_fd, buf, 100);
			sscanf(buf, "%lf, %lf", &sample_voltage, &laser_voltage);

			gpib_write(pps_fd, "measure:current:all?");
			gpib_read(pps_fd, buf, 100);
			sscanf(buf, "%lf, %lf", &sample_current, &laser_current);

			for (int falling = 0; falling < 2; ++falling)
			{
				FILE *curve_fp;
				// char *b;
				char filename_curve[100];
				double t1, t2;

				// === crate curve file
				snprintf(filename_curve, 100, "%s/curve_%d.%d.dat", dir_name, lt_index, falling);
				curve_fp = fopen(filename_curve, "w+");
				if(curve_fp == NULL)
				{
					fprintf(stderr, "# E: Unable to open file \"%s\" (%s)\n", filename_curve, strerror(errno));
					set_run(0);
					continue;
				}
				setlinebuf(curve_fp);

				// === write curve header
				r = fprintf(curve_fp,
					"# mipt_r125_lifetime_vm\n"
					"# Decay curve\n"
					"# Experiment name \"%s\"\n"
					"# Columns:\n"
					"# 1 - index\n"
					"# 2 - value, double\n",
					experiment_name
				);

				memset(rbuf, 0, rbufsize);

				// ibcmd(if_fd, DCL);
				// usleep(1000);
				gpib_write(vm_fd, "*cls");
				usleep(1000);
				gpib_write(vm_fd, "*sre 16");

				t1 = get_time();

				gpib_write(vm_fd, "init");

				usleep(OSC_TRIGGER_DELAY_US);

				if (falling == 0) usbtmc_write(osc_fd, "dds:offset 3.5");
				else              usbtmc_write(osc_fd, "dds:offset 0");

				fprintf(stderr, "# [debug] wait...\n");

				gpib_write(vm_fd, "fetch?");
				r = ibwait(if_fd, SRQI | TIMO);

				t2 = get_time();

				if     (r | SRQI) fprintf(stderr, "SRQI\n");
				else if(r | TIMO) fprintf(stderr, "TIMO\n");

				gpib_read(vm_fd, rbuf, rbufsize);

				for (int i = 0, j = 0; i < VM_POINTS; ++i)
				{
					char *b = rbuf + j;
					for (; j < rbufsize; ++j)
					{
						if ((rbuf[j] == ',') || (rbuf[j] == 0))
						{
							rbuf[j] = 0;
							fprintf(curve_fp, "%d\t%s\n", i, b);
							j++;
							break;
						}
					}
				}

				fprintf(curve_fp, "# duration = %le\n", t2 - t1);

				gpib_print_error(vm_fd);

				// r = fprintf(gp,
				// 	"set title \"i = %d.%d, t = %.3lf s, U = %.3lf V, I = %.3lf A, Ul = %.3lf V, Il = %.3lf A\"\n"
				// 	"plot \"%s\" u ($1/%le):($2*%le) w l lw 1 title \"freq = %.1lf Hz, duty = %d %%\"\n",
				// 	lt_index, attempt, lt_time, sample_voltage, sample_current, laser_voltage, laser_current,
				// 	filename_curve, sampling_rate, 10*OSC_YSCALE_V/256.0, freq[lt_index], OSC_DUTY
				// );
				// if(r < 0)
				// {
				// 	fprintf(stderr, "# E: Unable to print to gp (%s)\n", strerror(r));
				// 	set_run(0);
				// }

				r = fclose(curve_fp);
				if (r == EOF)
				{
					fprintf(stderr, "# E: Unable to close file \"%s\" (%s)\n", filename_curve, strerror(errno));
				}
			}

			r = fprintf(lt_fp, "%d\t%le\t%.3le\t%.3le\t%.3le\t%.3le\t%le\n",
				lt_index,
				lt_time,
				sample_voltage,
				sample_current,
				laser_voltage,
				laser_current,
				0.0
			);
			if(r < 0)
			{
				fprintf(stderr, "# E: Unable to print to file \"%s\" (%s)\n", filename_lt, strerror(r));
				set_run(0);
				break;
			}
		}
		else
		{
			break;
		}
	}

	gpib_write(pps_fd, "output 0");
	gpib_write(pps_fd, "voltage 0");

	usbtmc_write(osc_fd, "dds:offset 0");
	usbtmc_write(osc_fd, "dds:switch 0");

	gpib_write(pps_fd, "system:beeper");

	worker_gp_settings:

	r = pclose(gp);
	if (r == -1)
	{
		fprintf(stderr, "# E: Unable to close gnuplot pipe (%s)\n", strerror(errno));
	}
	worker_gp_popen:


	worker_lt_header:

	r = fclose(lt_fp);
	if (r == EOF)
	{
		fprintf(stderr, "# E: Unable to close file \"%s\" (%s)\n", filename_lt, strerror(errno));
	}
	worker_lt_fopen:

	worker_if_ibfind:

	gpib_close(vm_fd);
	worker_vm_ibfind:

	gpib_close(pps_fd);
	worker_pps_ibfind:

	usbtmc_close(osc_fd);
	worker_open_hantek:

	free(rbuf);
	worker_nomem:

	return NULL;
}

// === utils
int get_run()
{
	int run_local;
	pthread_rwlock_rdlock(&run_lock);
		run_local = run;
	pthread_rwlock_unlock(&run_lock);
	return run_local;
}

void set_run(int run_new)
{
	pthread_rwlock_wrlock(&run_lock);
		run = run_new;
	pthread_rwlock_unlock(&run_lock);
}

double get_time()
{
	static int first = 1;
	static struct timeval t_first = {0};
	struct timeval t = {0};
	double ret;
	int r;

	if (first == 1)
	{
		r = gettimeofday(&t_first, NULL);
		if (r == -1)
		{
			fprintf(stderr, "# E: unable to get time (%s)\n", strerror(errno));
			ret = -1;
		}
		else
		{
			ret = 0.0;
			first = 0;
		}
	}
	else
	{
		r = gettimeofday(&t, NULL);
		if (r == -1)
		{
			fprintf(stderr, "# E: unable to get time (%s)\n", strerror(errno));
			ret = -2;
		}
		else
		{
			ret = (t.tv_sec - t_first.tv_sec) * 1e6 + (t.tv_usec - t_first.tv_usec);
			ret /= 1e6;
		}
	}

	return ret;
}

// === GPIB ===

int gpib_open(const char *name)
{
	int r;

	r = ibfind(name);
	if (r == -1)
	{
		fprintf(stderr, "# E: unable to open gpib (ibsta = %d, iberr = %d)\n", ibsta, iberr);
	}

	return r;
}

int gpib_close(int dev)
{
	int r;
	int ret = 0;

	r = ibclr(dev);
	if (r & 0x8000)
	{
		ret = -1;
		fprintf(stderr, "# E: unable to clr gpib (ibsta = %d, iberr = %d)\n", ibsta, iberr);
	}

	r = gpib_write(dev, "*rst");
	if (r == -1)
	{
		ret = r;
	}

	sleep(1);

	r = ibloc(dev);
	if (r & 0x8000)
	{
		ret = -1;
		fprintf(stderr, "# E: unable to loc gpib (ibsta = %d, iberr = %d)\n", ibsta, iberr);
	}

	return ret;
}

int gpib_read(int dev, char *buf, size_t buf_length)
{
	int r;
	int ret = 0;

	r = ibrd(dev, buf, buf_length);
	if (r & 0x8000)
	{
		ret = -1;
		fprintf(stderr, "# E: unable to write to gpib (ibsta = %d, iberr = %d)\n", ibsta, iberr);
	}
	else
	{
		ret = ibcnt;
		if (ibcnt < buf_length)
		{
			buf[ibcnt] = 0;
		}
	}

	return ret;
}

int gpib_write(int dev, const char *str)
{
	int r;
	int ret = 0;

	r = ibwrt(dev, str, strlen(str));
	if (r & 0x8000)
	{
		ret = -1;
		fprintf(stderr, "# E: unable to write to gpib (ibsta = %d, iberr = %d)\n", ibsta, iberr);
	}
	else
	{
		ret = ibcnt;
	}

	return ret;
}

int gpib_print(int dev, const char *format, ...)
{
	int r;
	va_list args;
	char buf[100];
	const size_t bufsize = 100;

	va_start(args, format);
	r = vsnprintf(buf, bufsize, format, args);
	if (r < 0)
	{
		fprintf(stderr, "# E: unable to printf to buffer (%s)\n", strerror(errno));
		goto gpib_print_vsnprintf;
	}
	r = gpib_write(dev, buf);
	gpib_print_vsnprintf:
	va_end(args);

	return r;
}

void gpib_print_error(int dev)
{
	char buf[100] = {0};
	gpib_write(dev, "system:error?");
	gpib_read(dev, buf, 100);
	fprintf(stderr, "# [debug] error = %s\n", buf);
}

// === USBTMC ===

int usbtmc_open(const char *name)
{
	int r;

	r = open(name, O_RDWR);
	if (r == -1)
	{
		fprintf(stderr, "# E: unable to open usbtmc \"%s\" (%s)\n", name, strerror(errno));
	}

	return r;
}

int usbtmc_close(int dev)
{
	int r;

	usbtmc_write(dev, "*rst");

	r = close(dev);
	if (r == -1)
	{
		fprintf(stderr, "# E: unable to close usbtmc (%s)\n", strerror(errno));
	}

	return r;
}

int usbtmc_read(int dev, char *buf, size_t buf_length)
{
	int r;

	r = read(dev, buf, buf_length);
	if (r == -1)
	{
		fprintf(stderr, "# E: unable to read from usbtmc (%s)\n", strerror(errno));
	}

	return r;
}

int usbtmc_write(int dev, const char *str)
{
	int r;

	r = write(dev, str, strlen(str));
	if (r == -1)
	{
		fprintf(stderr, "# E: unable to write to usbtmc \"%s\" (%s)\n", str, strerror(errno));
	}

	return r;
}

int usbtmc_print(int dev, const char *format, ...)
{
	int r;
	va_list args;

	va_start(args, format);
	r = vdprintf(dev, format, args);
	if (r < 0)
	{
		fprintf(stderr, "# E: unable to printf to usbtmc (%s)\n", strerror(errno));
	}
	va_end(args);

	return r;
}
