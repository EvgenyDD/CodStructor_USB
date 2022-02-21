#include <cstdio>
#include <iostream>
#include <string>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial_port/serial_interface.h"

#if 0
#include "serial_port/serial.cpp"
#endif

void enumerate_ports()
{
	std::vector<PortInfo> devices_found = serial_port_get_list();

	std::vector<PortInfo>::iterator iter = devices_found.begin();

	while(iter != devices_found.end())
	{
		PortInfo device = *iter++;

		printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
			   device.hardware_id.c_str());
	}
}

// int run(int argc, char **argv)
//{
//   if(argc < 2) {
//       print_usage();
//     return 0;
//   }

//  // Argument 1 is the serial port or enumerate flag
//  string port(argv[1]);

//  if( port == "-e" ) {
//      enumerate_ports();
//      return 0;
//  }
//  else if( argc < 3 ) {
//      print_usage();
//      return 1;
//  }

//  // Argument 2 is the baudrate
//  unsigned long baud = 0;
//#if defined(WIN32) && !defined(__MINGW32__)
//  sscanf_s(argv[2], "%lu", &baud);
//#else
//  sscanf(argv[2], "%lu", &baud);
//#endif

//  // port, baudrate, timeout in milliseconds
//  Serial my_serial(port, baud, Timeout::simpleTimeout(1000));

//  cout << "Is the serial port open?";
//  if(my_serial.isOpen())
//    cout << " Yes." << endl;
//  else
//    cout << " No." << endl;

//  // Get the Test string
//  int count = 0;
//  string test_string;
//  if (argc == 4) {
//    test_string = argv[3];
//  } else {
//    test_string = "Testing.";
//  }

//  // Test the timeout, there should be 1 second between prints
//  cout << "Timeout == 1000ms, asking for 1 more byte than written." << endl;
//  while (count < 10) {
//    size_t bytes_wrote = my_serial.write(test_string);

//    string result = my_serial.read(test_string.length()+1);

//    cout << "Iteration: " << count << ", Bytes written: ";
//    cout << bytes_wrote << ", Bytes read: ";
//    cout << result.length() << ", String read: " << result << endl;

//    count += 1;
//  }

//  // Test the timeout at 250ms
//  my_serial.setTimeout(Timeout::max(), 250, 0, 250, 0);
//  count = 0;
//  cout << "Timeout == 250ms, asking for 1 more byte than written." << endl;
//  while (count < 10) {
//    size_t bytes_wrote = my_serial.write(test_string);

//    string result = my_serial.read(test_string.length()+1);

//    cout << "Iteration: " << count << ", Bytes written: ";
//    cout << bytes_wrote << ", Bytes read: ";
//    cout << result.length() << ", String read: " << result << endl;

//    count += 1;
//  }

//  // Test the timeout at 250ms, but asking exactly for what was written
//  count = 0;
//  cout << "Timeout == 250ms, asking for exactly what was written." << endl;
//  while (count < 10) {
//    size_t bytes_wrote = my_serial.write(test_string);

//    string result = my_serial.read(test_string.length());

//    cout << "Iteration: " << count << ", Bytes written: ";
//    cout << bytes_wrote << ", Bytes read: ";
//    cout << result.length() << ", String read: " << result << endl;

//    count += 1;
//  }

//  // Test the timeout at 250ms, but asking for 1 less than what was written
//  count = 0;
//  cout << "Timeout == 250ms, asking for 1 less than was written." << endl;
//  while (count < 10) {
//    size_t bytes_wrote = my_serial.write(test_string);

//    string result = my_serial.read(test_string.length()-1);

//    cout << "Iteration: " << count << ", Bytes written: ";
//    cout << bytes_wrote << ", Bytes read: ";
//    cout << result.length() << ", String read: " << result << endl;

//    count += 1;
//  }

//  return 0;
//}

int main(int argc, char **argv)
{
	std::vector<PortInfo> devices_found = serial_port_get_list();
	std::vector<PortInfo>::iterator iter = devices_found.begin();
	printf("Avail. COM devices: \n");
	while(iter != devices_found.end())
	{
		PortInfo device = *iter++;
		printf("\t%s\t%s\t%s\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str());
	}

	serial_port_t p = {0};
	serial_port_init(&p, "/dev/ttyS6",
							   115200,
							   eightbits,
							   parity_odd,
							   stopbits_one,
							   flowcontrol_none);
	int sts = serial_port_open(&p);
	printf("Open sts: %d\n", sts);
	sts = serial_port_write(&p, {9, 0,1,2,3});
	printf("Wr sts: %d\n", sts);
	std::vector<uint8_t> b;
	sts = serial_port_read(&p, &b);
	printf("Rd sts: %d %d\n", sts, b.size());
	for(auto i : b)
	{
		printf(">%d\n", i);
	}
}
