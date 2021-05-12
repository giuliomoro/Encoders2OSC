#include "src/MCP23017.h"
#include <libraries/Encoder/Encoder.h>
#include <libraries/UdpClient/UdpClient.h>
#include <oscpkt.hh>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <thread>
#include <vector>
#include <string>

std::string gOscAddress = "/encoder"; // OSC address. Message format: <address> <encoderId> <encoderValue>
std::string gDestinationIp = "192.168.7.1"; // 192.168.7.1 is the host computer (if it's a Mac or Linux, it would be 192.168.6.1 if it's Windows).
int gDestinationPort = 5555;

std::vector<Encoder> encoders;
std::vector<std::array<uint8_t, 2>> pinsPairs = {
	{{0, 1}},
	{{3, 2}},
	{{9, 8}},
	{{12, 13}},
};

int i2cBus = 1;
std::vector<uint8_t> i2cAddresses = {
	0x21,
	0x22,
	0x26,
	0x27,
};

std::vector<uint16_t> oldGpios;
std::vector<uint16_t> gpios;
std::vector<MCP23017> mcps;
int gStop = 0;

static void makeBinaryString(char* str, uint16_t data)
{
	for(unsigned int n = 0; n < sizeof(data) * 8; ++n, ++str)
		*str = (data & (1 << n)) ? '1' : '0';
	*str = '\0';
}

static void processEnc()
{
	static std::vector<uint16_t> oldGpios = gpios;
	for(unsigned int m = 0; m < mcps.size(); ++m)
	{
		MCP23017& mcp = mcps[m];
		uint16_t& oldGpio = oldGpios[m];
		uint16_t& gpio = gpios[m];
		// we always read from INTCAP (the state of the GPIO when the
		// interrupt was triggered). This resets the interrupt and
		// allows us to see which pin toggled first (useful for encoders!)
		gpio = mcp.readINTCAPAB();
		for(unsigned int n = 0; n < pinsPairs.size(); ++n)
		{
			auto& pins = pinsPairs[n % pinsPairs.size()];
			encoders[n + m * pinsPairs.size()].process(gpio & (1 << pins[0]), gpio & (1 << pins[1]));
		}
		if(oldGpio != gpio)
		{
			// we caught the device as an interrupt had just occurred.
			// add a little "debouncing" delay before reading again
			// TODO: do not hold back reading back other devices while debouncing one device
			usleep(1000);
			// read and ignore in order to clear any interrupt due to bounces that
			// may have occurred in the meantime.
			mcp.readINTCAP(0);
		}
	}
}

static void printEnc(UdpClient* socket)
{
	int ss = 20;
	char stars[ss + 1];
	char spaces[ss + 1];
	memset(stars, '*', ss);
	memset(spaces, ' ', ss);
	spaces[ss] = stars[ss] = '\0';
	std::vector<int> oldRots(encoders.size());
	std::vector<uint16_t> oldGpios = gpios;
	oscpkt::PacketWriter pw;
	while(!gStop){
		pw.init();
		//pw.startBundle();
		unsigned int numMsg = 0;
		for(unsigned int m = 0; m < mcps.size(); ++m)
		{
			uint16_t& gpio = gpios[m];
			uint16_t& oldGpio = oldGpios[m];
			bool shouldPrint = false;
			if(gpio != oldGpio)
				shouldPrint = true;
			oldGpio = gpio;
			unsigned int encStart = m * pinsPairs.size();
			unsigned int encEnd = (1 + m) * pinsPairs.size();
			for(unsigned int n = encStart; n < encEnd; ++n)
			{
				int& oldRot = oldRots[n];
				int rot = encoders[n].get();
				if(oldRot != rot)
				{
					shouldPrint = true;
					oscpkt::Message msg(gOscAddress);
					pw.addMessage(msg.pushInt32(int32_t(n)).pushInt32(int32_t(rot)));
					printf("%s %u %d\n", gOscAddress.c_str(), n, rot);
					++numMsg;
				}
				oldRot = rot;
			}
			if(shouldPrint)
			{
				char str[17];
				makeBinaryString(str, gpio);
				printf("[%d] %s: ", m, str);
				for(unsigned int n = encStart; n < encEnd; ++n)
				{
					int rot = encoders[n].get();
					int numStars = rot + 1;
					while(numStars < 0)
						numStars += ss;
					numStars %= ss;
					printf("%4d %.*s%.*s", rot, numStars, stars, ss - numStars, spaces);
				}
				printf("\n");
			}
		}
		if(numMsg)
		{
			//if(pw.endBundle().isOk())
			{
				socket->send((void*)pw.packetData(), pw.packetSize());
			}
		}
		usleep(10000);
	}
}

// Handle Ctrl-C by requesting that the threads stop
void interrupt_handler(int var)
{
	gStop = true;
}

int main(int argc, char** argv)
{
	UdpClient socket;
	if(!socket.setup(gDestinationPort, gDestinationIp.c_str()))
	{
		fprintf(stderr, "Unable to send to %s:%d\n", gDestinationIp.c_str(), gDestinationPort);
		return 1;
	}
	mcps.reserve(i2cAddresses.size()); // ensure no allocation happens in the below loop.
	for(unsigned int c = 0; c < i2cAddresses.size(); ++c)
	{
		mcps.emplace_back(i2cBus, i2cAddresses[c]);
		if(!mcps.back().openI2C())
		{
			fprintf(stderr, "Failed to open device on bus %d, address %#x\n", i2cBus, i2cAddresses[c]);
			mcps.erase(mcps.end() - 1, mcps.end());
			continue;
		}
		MCP23017& mcp = mcps.back();
		gpios.push_back(0);
		for(unsigned int n = 0; n < 16; ++n)
		{
			mcp.pinMode(n, MCP23017::INPUT);
			mcp.pullUp(n, MCP23017::HIGH);  // turn on a 100K pullup internally
			// we set up interrupts so we can read the INTCAP register
			mcp.setupInterrupts(true, false, MCP23017::HIGH);
			for(unsigned int n = 0; n < 16; ++n)
				mcp.setupInterruptPin(n, MCP23017::CHANGE);
		}
	}
	if(!mcps.size())
	{
		fprintf(stderr, "No device detected\n");
		return 1;
	} else {
		printf("%d encoder boards detected\n", mcps.size());
	}
	for(unsigned int n = 0; n < mcps.size() * pinsPairs.size(); ++n)
	{
		encoders.push_back({0, Encoder::ACTIVE_HIGH});
	}
	signal(SIGINT, interrupt_handler);
	signal(SIGTERM, interrupt_handler);
	std::thread printThread(printEnc, &socket);
	while(!gStop)
	{
		processEnc();
	}
	if(printThread.joinable())
		printThread.join();
	return 0;
}
