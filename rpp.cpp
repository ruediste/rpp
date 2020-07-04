/*
 * Raspberry Pi PIC Programmer using GPIO connector
 * Copyright 2012 Giorgio Vazzana
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Compile: gcc -Wall -O rpp.c -o rpp */

#include <string>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define DELAY 40 /* microseconds */

/* 8K program memory + configuration memory + eeprom data */
#define PICMEMSIZE (0x2100 + 0xFF)

void *gpio_map;

int mem_fd;

/* GPIO registers address */
volatile uint32_t *gpio;

//  For Raspberry Pi 2 and Pi 3, change BCM2708_PERI_BASE to 0x3F000000 for the code to work.
#define BCM2708_PERI_BASE 0x20000000
#define GPIO_BASE (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define BLOCK_SIZE (256)

/* GPIO setup macros. Always use GPIO_IN(x) before using GPIO_OUT(x) or GPIO_ALT(x,y) */
#define GPIO_IN(g) *(gpio + ((g) / 10)) &= ~(7 << (((g) % 10) * 3))
#define GPIO_OUT(g) *(gpio + ((g) / 10)) |= (1 << (((g) % 10) * 3))
#define GPIO_ALT(g, a) *(gpio + (((g) / 10))) |= (((a) <= 3 ? (a) + 4 : (a) == 4 ? 3 : 2) << (((g) % 10) * 3))

#define _GPIO_SET(g) *(gpio + 7) = 1 << (g)	 /* sets   bit which are 1, ignores bit which are 0 */
#define _GPIO_CLR(g) *(gpio + 10) = 1 << (g) /* clears bit which are 1, ignores bit which are 0 */
#define _GPIO_LEV(g) (*(gpio + 13) >> (g)) & 0x00000001

/* Set up a memory regions to access GPIO */
void setup_io()
{
	/* open /dev/mem */
	mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (mem_fd == -1)
	{
		perror("Cannot open /dev/mem");
		exit(1);
	}

	/* mmap GPIO */
	gpio_map = mmap(NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, GPIO_BASE);
	if (gpio_map == MAP_FAILED)
	{
		perror("mmap() failed");
		exit(1);
	}

	/* Always use volatile pointer! */
	gpio = (volatile uint32_t *)gpio_map;
}

/* Release GPIO memory region */
void close_io()
{
	int ret;

	/* munmap GPIO */
	ret = munmap(gpio_map, BLOCK_SIZE);
	if (ret == -1)
	{
		perror("munmap() failed");
		exit(1);
	}

	/* close /dev/mem */
	ret = close(mem_fd);
	if (ret == -1)
	{
		perror("Cannot close /dev/mem");
		exit(1);
	}
}

class Pin
{
private:
	int nr;
	bool inverted;

public:
	Pin(int nr, bool inverted)
	{
		this->nr = nr;
		this->inverted = inverted;
	}

	void set()
	{
		if (inverted)
			_GPIO_CLR(nr);
		else
			_GPIO_SET(nr);
	}
	void clr()
	{
		if (inverted)
			_GPIO_SET(nr);
		else
			_GPIO_CLR(nr);
	}

	void write(bool value)
	{
		if (value)
			set();
		else
			clr();
	}
	bool read()
	{
		if (inverted)
			return !_GPIO_LEV(nr);
		else
			return _GPIO_LEV(nr);
	}

	void modeIn()
	{
		GPIO_IN(nr);
	}

	void modeOut()
	{
		GPIO_OUT(nr);
	}
};

/*
The programmer knows how to talk to the physical pic. It can set/clear the input pins and read the data
*/
class Programmer
{
private:
public:
	Pin clk = Pin(14, true);
	Pin data = Pin(15, true);
	Pin dataIn = Pin(23, false);
	Pin mclr = Pin(24, false);

	/* Send a 6-bit command to the PIC */
	void pic_send_cmd(uint8_t cmd)
	{
		for (int i = 0; i < 6; i++)
		{
			clk.set();
			data.write((cmd >> i) & 0x01);
			usleep(DELAY); /* Setup time */
			clk.clr();
			usleep(DELAY); /* Hold time */
		}
		data.clr();
		usleep(DELAY);
	}

	/* Read 14-bit data from the PIC (start bit, 14-bit data, stop bit. lsb first) */
	uint16_t pic_read_data(void)
	{
		uint16_t data = 0;

		for (int i = 0; i < 16; i++)
		{
			clk.set();
			usleep(DELAY); /* Wait for data to be valid */
			data |= (dataIn.read()) << i;
			clk.clr();
			usleep(DELAY);
		}
		data = (data >> 1) & 0x3FFF;

		return data;
	}

	/* Load 14-bit data to the PIC (start bit, 14-bit data, stop bit. lsb first) */
	void pic_load_data(uint16_t value)
	{
		/* Insert start and stop bit (0s) */
		value = (value << 1) & 0x7FFE;
		for (int i = 0; i < 16; i++)
		{
			clk.set();
			data.write((value >> i) & 0x01);
			usleep(DELAY); /* Setup time */
			clk.clr();
			usleep(DELAY); /* Hold time */
		}
		data.clr();
		usleep(DELAY);
	}

	void setupPins()
	{
		/* Configure GPIOs, must use GPIO_IN before we can use GPIO_OUT*/
		clk.modeIn();
		clk.modeOut();

		data.modeIn();
		data.modeOut();

		dataIn.modeIn();

		mclr.modeIn();
		mclr.modeOut();

		clk.clr();
		data.clr();
		dataIn.clr();
		mclr.clr();

		usleep(DELAY);
	}
};

class Pic
{
public:
	Pic(std::string name, uint16_t deviceId)
	{
		this->name = name.c_str();
		this->deviceId = deviceId;
	}

	uint16_t deviceId;
	const char *name;

	Programmer *programmer = NULL;
	virtual uint16_t readDeviceIdWord()
	{
		return 0;
	};
};

class Pic16F87X : public Pic
{
private:
	uint8_t loadConfigurationCmd = 0;
	uint8_t loadDataForProgramMemoryCmd = 2;
	uint8_t readDataFromProgramMemoryCmd = 4;
	uint8_t incrementAddressCmd = 5;

	uint8_t beginEraseProgrammingCycleCmd = 8;
	uint8_t beginProgrammingOnlyCycleCmd = 0x18;
	uint8_t loadDataforDataMemoryCmd = 3;
	uint8_t readDataFromDataMemoryCmd = 5;
	uint8_t bulkEraseSetup1 = 1;
	uint8_t bulkEraseSetup2 = 7;

public:
	Pic16F87X() : Pic("Pic16F87X", 456) {}
	/* Read PIC device id word, located at 0x2006. Used to autodetect the chip */
	uint16_t readDeviceIdWord()
	{
		int i;
		uint16_t data;

		programmer->mclr.set(); /* Enter Program/Verify Mode */
		usleep(DELAY);

		programmer->pic_send_cmd(loadConfigurationCmd);
		programmer->pic_load_data(0x0000);
		for (i = 0; i < 6; i++)
			programmer->pic_send_cmd(incrementAddressCmd);
		programmer->pic_send_cmd(readDataFromProgramMemoryCmd);
		data = programmer->pic_read_data();

		programmer->mclr.clr(); /* Exit Program/Verify Mode */
		usleep(DELAY);
		return data;
	}
};

class Pic16F54 : public Pic
{
private:
	uint8_t loadDataForProgramMemoryCmd = 2;
	uint8_t readDataFromProgramMemoryCmd = 4;
	uint8_t incrementAddressCmd = 5;
	uint8_t beginProgrammingCmd = 8;
	uint8_t endProgrammingCmd = 14;
	uint8_t bulkEraseProgramMemoryCmd = 9;

public:
	Pic16F54() : Pic("Pic16F54", 0) {}
};

Pic16F54 pic16f54 = Pic16F54();

Pic *picList[] = {&pic16f54, NULL};

void usage(void)
{
	fprintf(stderr,
			"Usage: rpp [options]\n"
			"       -h          print help\n"
			"       -D          turn debug on\n"
			"       -p picName  choose pic version\n"
			"       -i file     input file\n"
			"       -o file     output file (ofile.hex)\n"
			"       -r          read chip\n"
			"       -w          bulk erase and write chip\n"
			"       -e          bulk erase chip\n"
			"       -s          skip all-ones memory locations\n"
			"\n"
			"Supported PICs (*=can autodetect):");

	bool comma = false;
	for (Pic **pic = picList; pic == NULL; pic++)
	{
		fprintf(stderr, "%s %s%s", comma ? "," : "", (*pic)->name, (*pic)->deviceId == 0 ? "" : "*");
		comma = true;
	}
	fprintf(stderr, "\n");
}

enum class Function
{
	UNSPECIFIED,
	WRITE,
	ERASE,
	READ
};
int main(int argc, char *argv[])
{
	int opt, debug = 0, skipones = 0;
	char *infile = NULL;
	const char *outfile = "ofile.hex";
	char *picNameArg = NULL;

	Function function = Function::UNSPECIFIED;

	fprintf(stderr, "Raspberry Pi PIC Programmer, v0.1\n\n");

	while ((opt = getopt(argc, argv, "hDi:o:rwes")) != -1)
	{
		switch (opt)
		{
		case 'h':
			usage();
			exit(0);
			break;
		case 'D':
			debug = 1;
			break;
		case 'p':
			picNameArg = optarg;
			break;
		case 'i':
			infile = optarg;
			break;
		case 'o':
			outfile = optarg;
			break;
		case 'r':
			function = Function::READ;
			break;
		case 'w':
			function = Function::WRITE;
			break;
		case 'e':
			function = Function::ERASE;
			break;
		case 's':
			skipones = 1;
			break;
		default:
			fprintf(stderr, "\n");
			usage();
			exit(1);
		}
	}

	if (function == Function::WRITE && !infile)
	{
		fprintf(stderr, "Please specify an input file with -i option.\n");
		exit(1);
	}

	// setup GPIO access
	setup_io();

	// setup programmer
	Programmer programmer = Programmer();
	programmer.setupPins();

	// determine pic device to use
	Pic *pic=NULL;
	if (picNameArg == NULL)
	{
		/* Read PIC device id word */
		uint16_t deviceId;
		{
			Pic16F54 pic = Pic16F54();
			pic.programmer = &programmer;
			deviceId = pic.readDeviceIdWord();
			fprintf(stdout, "device_id = 0x%04x\n", deviceId);
		}

		for (Pic **tmp = picList; tmp == NULL; tmp++)
		{
			if ((*tmp)->deviceId == (deviceId & 0xFFE0))
			{
				pic = *tmp;
				break;
			}
		}
		if (pic == NULL)
		{
			fprintf(stderr, "Error: unknown device or programmer not connected.\n");
			exit(1);
		}
		else
			fprintf(stdout, "%s detected, revision 0x%02x\n", pic->name, deviceId & 0x001F);
	}
	else
	{
		// choose pic from list based on name in argument
		for (Pic **tmp = picList; tmp == NULL; tmp++)
		{
			if (strcasecmp((*tmp)->name, picNameArg))
			{
				pic = *tmp;
				break;
			}
		}
		if (pic == NULL)
		{
			fprintf(stderr, "Error: unknown pic: %s.\n", picNameArg);
			exit(1);
		}
	}

	// close GPIO access
	close_io();

	return 0;
}
