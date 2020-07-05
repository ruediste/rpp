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

#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <algorithm>
#include <time.h>
#include <errno.h>

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

bool debug = false;

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
	double sleepFactor = 1;
	Pin clk = Pin(14, true);
	Pin data = Pin(15, true);
	Pin dataIn = Pin(23, false);
	Pin mclr = Pin(24, false);

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

	void sleep(long nseconds)
	{
		timespec t = timespec();
		t.tv_sec = 0;
		t.tv_nsec = nseconds * sleepFactor;

		int ret;
		while ((ret = nanosleep(&t, &t)) == -1 && errno == EINTR)
		{
			if (errno != EINTR)
				break;
		};
		if (ret == -1)
		{
			perror("Error during Sleep");
			exit(1);
		}
	}
};

#define NS(t) (t)
#define US(t) NS(t * 1000)
#define MS(t) US(t * 1000)

class HexFile
{
public:
	HexFile()
	{
		data = (uint16_t *)calloc(PICMEMSIZE, sizeof(*data));
		filled = (uint8_t *)calloc(PICMEMSIZE, sizeof(*filled));
	}

	~HexFile()
	{
		free(data);
		free(filled);
	}
	uint16_t program_memory_used_cells;
	uint16_t program_memory_max_used_address;

	uint8_t has_configuration_data;
	uint8_t has_eeprom_data;

	uint16_t *data;	 /* 14-bit data */
	uint8_t *filled; /* 1 if this cell is used */
};

class PicHelper
{

public:
	/* Data setup time befor clk down*/
	long Tset1;
	/* Data hold time after clk down*/
	long Thld1;
	/* Data input not driven to next clock input (delay required between command/data or command/command)*/
	long Tdly1;
	/* Delay between clk down to clk up of next command or data*/
	long Tdly2;
	/* Clk up to data out valid (during Read Data)*/
	long Tdly3;
	PicHelper(long Tset1, long Thld1, long Tdly1, long Tdly2, long Tdly3)
	{
		this->Tset1 = Tset1;
		this->Thld1 = Thld1;
		this->Tdly1 = Tdly1;
		this->Tdly2 = Tdly2;
		this->Tdly3 = Tdly3;
	}
	Programmer *programmer;

	/* Send a 6-bit command to the PIC */
	void sendCmd(uint8_t cmd)
	{
		for (int i = 0; i < 6; i++)
		{
			programmer->clk.set();
			programmer->data.write((cmd >> i) & 0x01);
			programmer->sleep(Tset1);
			programmer->clk.clr();
			programmer->sleep(Thld1);
		}
		programmer->data.clr();
		programmer->sleep(std::max(Tdly1, Tdly2));
	}

	/* Read 14-bit data from the PIC (start bit, 14-bit data, stop bit. lsb first) */
	uint16_t readData(void)
	{
		uint16_t data = 0;

		for (int i = 0; i < 16; i++)
		{
			programmer->clk.set();
			programmer->sleep(Tdly3);
			data |= (programmer->dataIn.read()) << i;
			programmer->clk.clr();
			programmer->sleep(Thld1);
		}
		data = (data >> 1) & 0x3FFF;

		return data;
	}

	/* Load 14-bit data to the PIC (start bit, 14-bit data, stop bit. lsb first) */
	void loadData(uint16_t value)
	{
		/* Insert start and stop bit (0s) */
		value = (value << 1) & 0x7FFE;
		for (int i = 0; i < 16; i++)
		{
			programmer->clk.set();
			programmer->data.write((value >> i) & 0x01);
			programmer->sleep(Tset1);
			programmer->clk.clr();
			programmer->sleep(Thld1);
		}
		programmer->data.clr();
		programmer->sleep(std::max(Tdly1, Tdly2));
	}
};

class Pic
{
public:
	Pic(const char *name, uint16_t deviceId)
	{
		this->name = name;
		this->deviceId = deviceId;
	}

	uint16_t deviceId;
	const char *name;

	Programmer *programmer = NULL;

	virtual void setProgrammer(Programmer *programmer)
	{
		this->programmer = programmer;
	}
	virtual uint16_t readDeviceIdWord()
	{
		return 0;
	};

	virtual void write(HexFile *hex)
	{
		fprintf(stderr, "Writing PIC not supported");
		exit(1);
	}
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

	PicHelper helper;

public:
	Pic16F87X() : Pic("Pic16F87X", 456), helper(0, 0, 0, 0, 0) {}

	void setProgrammer(Programmer *programmer)
	{
		Pic::setProgrammer(programmer);
		helper.programmer = programmer;
	}

	/* Read PIC device id word, located at 0x2006. Used to autodetect the chip */
	uint16_t readDeviceIdWord()
	{
		int i;
		uint16_t data;

		programmer->mclr.set(); /* Enter Program/Verify Mode */
		usleep(DELAY);

		helper.sendCmd(loadConfigurationCmd);
		helper.loadData(0x0000);
		for (i = 0; i < 6; i++)
			helper.sendCmd(incrementAddressCmd);
		helper.sendCmd(readDataFromProgramMemoryCmd);
		data = helper.readData();

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
	long Thld0 = US(5);
	long Tera = MS(10);
	long Treset = MS(10);
	long Tprog;
	PicHelper helper;

public:
	Pic16F54() : Pic("Pic16F54", 0), helper(NS(100), NS(100), US(1), US(1), NS(80)) {}

	void setProgrammer(Programmer *programmer)
	{
		Pic::setProgrammer(programmer);
		helper.programmer = programmer;
	}

	/* Bulk erase the chip */
	void bulkErase()
	{
		fprintf(stderr, "Bulk erasing program memory and configuration word...\n");

		programmer->mclr.set();
		programmer->sleep(Thld0);
		helper.sendCmd(bulkEraseProgramMemoryCmd);
		programmer->sleep(Tera);
		programmer->mclr.clr();
		programmer->sleep(Treset);
	}

	/* Bulk erase the chip, and then write contents of the .hex file to the PIC */
	void write(HexFile *hex)
	{
		/* Write Program Memory */
		programmer->mclr.set();
		programmer->sleep(Thld0);

		fprintf(stdout, "Bulk erasing chip...\n");
		helper.sendCmd(bulkEraseProgramMemoryCmd);
		programmer->sleep(Tera);

		fprintf(stdout, "Writing chip...\n");
		helper.sendCmd(incrementAddressCmd);

		unsigned int addr;
		for (addr = 0; addr <= hex->program_memory_max_used_address; addr++)
		{
			if (hex->filled[addr])
			{
				helper.sendCmd(loadDataForProgramMemoryCmd);
				helper.loadData(hex->data[addr]);
				helper.sendCmd(beginProgrammingCmd);
				programmer->sleep(Tprog);

				helper.sendCmd(readDataFromProgramMemoryCmd);
				uint16_t data = helper.readData();

				if (debug)
					fprintf(stderr, "  addr = 0x%04X  written = 0x%04X  read = 0x%04X\n", addr, hex->data[addr], data);

				if (hex->data[addr] != data)
				{
					fprintf(stderr, "Error: addr = 0x%04X, written = 0x%04X, read = 0x%04X\n",  addr, hex->data[addr],  data);
					exit(1);
				}
			}
			helper.sendCmd(incrementAddressCmd);
		}

		programmer->mclr.clr();
		programmer->sleep(Treset);
	}
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
	for (Pic **pic = picList; *pic == NULL; pic++)
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

/* Read a file in Intel HEX 16-bit format and return a pointer to the picmemory
   struct on success, or NULL on error */
struct HexFile *read_inhx16(char *infile)
{
	FILE *fp;
	int linenum;
	char line[256], *ptr;
	size_t linelen;
	int nread;

	uint16_t i;
	uint8_t start_code;
	uint8_t byte_count;
	uint16_t address;
	uint8_t record_type;
	uint16_t data;
	uint8_t checksum_calculated;
	uint8_t checksum_read;

	struct HexFile *pm;

	fp = fopen(infile, "r");
	if (fp == NULL)
	{
		fprintf(stderr, "Error: cannot open source file %s.\n", infile);
		return NULL;
	}

	pm = new HexFile();
	if (pm)
	{
		pm->data = (uint16_t *)calloc(PICMEMSIZE, sizeof(*pm->data));
		pm->filled = (uint8_t *)calloc(PICMEMSIZE, sizeof(*pm->filled));
	}
	if (!pm || !pm->data || !pm->filled)
	{
		fprintf(stderr, "Error: calloc() failed.\n");
		return NULL;
	}

	fprintf(stderr, "Reading hex file...\n");

	linenum = 0;
	while (1)
	{
		ptr = fgets(line, 256, fp);

		if (ptr != NULL)
		{
			linenum++;
			linelen = strlen(line);
			if (debug)
			{
				fprintf(stderr, "  line %d (%zd bytes): '", linenum, linelen);
				for (i = 0; i < linelen; i++)
				{
					if (line[i] == '\n')
						fprintf(stderr, "\\n");
					else if (line[i] == '\r')
						fprintf(stderr, "\\r");
					else
						fprintf(stderr, "%c", line[i]);
				}
				fprintf(stderr, "'\n");
			}

			start_code = line[0];
			if (start_code != ':')
			{
				fprintf(stderr, "Error: invalid start code.\n");
				delete (pm);
				return NULL;
			}

			nread = sscanf(&line[1], "%2hhx", &byte_count);
			if (nread != 1)
			{
				fprintf(stderr, "Error: cannot read byte count.\n");
				delete pm;
				return NULL;
			}
			if (debug)
				fprintf(stderr, "  byte_count  = 0x%02X\n", byte_count);

			nread = sscanf(&line[3], "%4hx", &address);
			if (nread != 1)
			{
				fprintf(stderr, "Error: cannot read address.\n");
				delete pm;
				return NULL;
			}
			if (debug)
				fprintf(stderr, "  address     = 0x%04X\n", address);

			nread = sscanf(&line[7], "%2hhx", &record_type);
			if (nread != 1)
			{
				fprintf(stderr, "Error: cannot read record type.\n");
				delete pm;
				return NULL;
			}
			if (debug)
				fprintf(stderr, "  record_type = 0x%02X (%s)\n", record_type, record_type == 0 ? "data" : (record_type == 1 ? "EOF" : "Unknown"));
			if (record_type != 0 && record_type != 1)
			{
				fprintf(stderr, "Error: unknown record type.\n");
				delete pm;
				return NULL;
			}

			checksum_calculated = byte_count;
			checksum_calculated += (address >> 8) & 0xFF;
			checksum_calculated += address & 0xFF;
			checksum_calculated += record_type;

			for (i = 0; i < byte_count; i++)
			{
				nread = sscanf(&line[9 + 4 * i], "%4hx", &data);
				if (nread != 1)
				{
					fprintf(stderr, "Error: cannot read data.\n");
					delete pm;
					return NULL;
				}
				if (debug)
					fprintf(stderr, "  data        = 0x%04X\n", data);
				checksum_calculated += (data >> 8) & 0xFF;
				checksum_calculated += data & 0xFF;

				if (address + i < 0x2000)
				{
					pm->program_memory_used_cells += 1;
					pm->program_memory_max_used_address = address + i;
				}
				else if (0x2000 <= address + i && address + i < 0x2008)
					pm->has_configuration_data = 1;
				else if (address + i >= 0x2100)
					pm->has_eeprom_data = 1;

				pm->data[address + i] = data;
				pm->filled[address + i] = 1;
			}

			checksum_calculated = (checksum_calculated ^ 0xFF) + 1;

			nread = sscanf(&line[9 + 4 * i], "%2hhx", &checksum_read);
			if (nread != 1)
			{
				fprintf(stderr, "Error: cannot read checksum.\n");
				delete pm;
				return NULL;
			}
			if (debug)
				fprintf(stderr, "  checksum    = 0x%02X\n", checksum_read);

			if (checksum_calculated != checksum_read)
			{
				fprintf(stderr, "Error: checksum does not match.\n");
				delete pm;
				return NULL;
			}

			if (debug)
				fprintf(stderr, "\n");

			if (record_type == 1)
				break;
		}
		else
		{
			fprintf(stderr, "Error: unexpected EOF.\n");
			delete pm;
			return NULL;
		}
	}

	fclose(fp);

	return pm;
}

int main(int argc, char *argv[])
{
	int opt, skipones = 0;
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
			debug = true;
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
	Pic *pic = NULL;
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

	pic->setProgrammer(&programmer);

	switch (function)
	{
	case Function::WRITE:
	{
		fprintf(stdout, "Writing PIC");
		HexFile *file = read_inhx16(infile);
		if (file == NULL)
			exit(1);
		pic->write(file);
	}
	default:
	{
		fprintf(stderr, "Please select a function");
	}
	}
	// close GPIO access
	close_io();

	return 0;
}
