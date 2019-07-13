
#include "bmi160.hpp"

#include <px4_config.h>
#include <px4_getopt.h>

/** driver 'main' command */
extern "C" { __EXPORT int bmi160_main(int argc, char *argv[]); }

/**
 * Local functions in support of the shell command.
 */
namespace bmi160
{

BMI160	*g_dev_int; // on internal bus
BMI160	*g_dev_ext; // on external bus

void	start(bool, enum Rotation);
void	stop(bool);
void	info(bool);
void	regdump(bool);
void	testerror(bool);
void	usage();


/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)
{
	BMI160 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr != nullptr)
		/* if already started, the still command succeeded */
	{
		errx(0, "already started");
	}

	/* create the driver */
	if (external_bus) {
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_BMI)
		*g_dev_ptr = new BMI160(PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_BMI, rotation);
#else
		errx(0, "External SPI not available");
#endif

	} else {
#if defined(PX4_SPIDEV_BMI)
		*g_dev_ptr = new BMI160(PX4_SPI_BUS_SENSORS, PX4_SPIDEV_BMI, rotation);
#else
		errx(0, "No Internal SPI CS");
#endif
	}

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}

	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}

	exit(0);
fail:

	if (*g_dev_ptr != nullptr) {
		delete (*g_dev_ptr);
		*g_dev_ptr = nullptr;
	}

	errx(1, "driver start failed");
}

void
stop(bool external_bus)
{
	BMI160 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr != nullptr) {
		delete *g_dev_ptr;
		*g_dev_ptr = nullptr;

	} else {
		/* warn, but not an error */
		warnx("already stopped.");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info(bool external_bus)
{
	BMI160 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_info();

	exit(0);
}

/**
 * Dump the register information
 */
void
regdump(bool external_bus)
{
	BMI160 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		errx(1, "driver not running");
	}

	printf("regdump @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_registers();

	exit(0);
}


/**
 * deliberately produce an error to test recovery
 */
void
testerror(bool external_bus)
{
	BMI160 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		errx(1, "driver not running");
	}

	(*g_dev_ptr)->test_error();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'stop', 'regdump', 'testerror'");
	warnx("options:");
	warnx("    -X    (external bus)");
	warnx("    -R rotation");
}

} // namespace

int
bmi160_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool external_bus = false;
	enum Rotation rotation = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "XR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			bmi160::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		bmi160::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		bmi160::start(external_bus, rotation);
	}

	if (!strcmp(verb, "stop")) {
		bmi160::stop(external_bus);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		bmi160::info(external_bus);
	}

	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump")) {
		bmi160::regdump(external_bus);
	}

	if (!strcmp(verb, "testerror")) {
		bmi160::testerror(external_bus);
	}

	bmi160::usage();
	return -1;
}
