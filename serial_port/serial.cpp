#include <chrono>
#include <thread>

#ifdef __cplusplus
#define CAST(t, v) (static_cast<t>(v))
#else
#define CAST(t, v) ((t)(v))
#endif

#if defined(_WIN32)

#include "serial_interface.h"
#include <sstream>
#include <stdio.h>

//#define SERIAL_OVERLAP_MODE

static std::wstring _prefix_port_if_needed(const std::wstring *input)
{
	static std::wstring windows_com_port_prefix = L"\\\\.\\";
	if(input->compare(windows_com_port_prefix) != 0) return windows_com_port_prefix + *input;
	return *input;
}

void serial_port_setTimeout(serial_port_t *sp, Timeout *timeout)
{
	sp->timeout_ = *timeout;
	if(sp->is_open_)
	{
		serial_port_reconfigure(sp);
	}
}

Timeout serial_port_getTimeout(serial_port_t *sp)
{
	return sp->timeout_;
}

void serial_port_setBaudrate(serial_port_t *sp, unsigned long baudrate)
{
	sp->baudrate_ = baudrate;
	if(sp->is_open_)
	{
		serial_port_reconfigure(sp);
	}
}

unsigned long serial_port_getBaudrate(serial_port_t *sp)
{
	return sp->baudrate_;
}

void serial_port_setBytesize(serial_port_t *sp, bytesize_t bytesize)
{
	sp->bytesize_ = bytesize;
	if(sp->is_open_)
	{
		serial_port_reconfigure(sp);
	}
}

bytesize_t serial_port_getBytesize(serial_port_t *sp)
{
	return sp->bytesize_;
}

void serial_port_setParity(serial_port_t *sp, parity_t parity)
{
	sp->parity_ = parity;
	if(sp->is_open_)
	{
		serial_port_reconfigure(sp);
	}
}

parity_t serial_port_getParity(serial_port_t *sp)
{
	return sp->parity_;
}

void serial_port_setStopbits(serial_port_t *sp, stopbits_t stopbits)
{
	sp->stopbits_ = stopbits;
	if(sp->is_open_)
	{
		serial_port_reconfigure(sp);
	}
}

stopbits_t serial_port_getStopbits(serial_port_t *sp)
{
	return sp->stopbits_;
}

void serial_port_setFlowcontrol(serial_port_t *sp, flowcontrol_t flowcontrol)
{
	sp->flowcontrol_ = flowcontrol;
	if(sp->is_open_)
	{
		serial_port_reconfigure(sp);
	}
}

flowcontrol_t serial_port_getFlowcontrol(serial_port_t *sp)
{
	return sp->flowcontrol_;
}

void serial_port_init(serial_port_t *sp,
					  const std::string &port,
					  unsigned long baudrate,
					  bytesize_t bytesize,
					  parity_t parity,
					  stopbits_t stopbits,
					  flowcontrol_t flowcontrol)
{
	sp->port_ = std::wstring(port.begin(), port.end());
	sp->fd_ = INVALID_HANDLE_VALUE;
	sp->is_open_ = false;
	sp->baudrate_ = baudrate;
	sp->parity_ = parity;
	sp->bytesize_ = bytesize;
	sp->stopbits_ = stopbits;
	sp->flowcontrol_ = flowcontrol;
	sp->read_mutex = CreateMutex(nullptr, false, nullptr);
	sp->write_mutex = CreateMutex(nullptr, false, nullptr);
}

void seral_port_deinit(serial_port_t *sp)
{
	serial_port_close(sp);
	CloseHandle(sp->read_mutex);
	CloseHandle(sp->write_mutex);
}

int serial_port_open(serial_port_t *sp)
{
	if(sp->port_.empty())
	{
		printf("Empty port is invalid.\n");
		return 1;
	}
	if(sp->is_open_ == true)
	{
		printf("Serial port already open.\n");
		return 2;
	}

	// See: https://github.com/wjwwood/serial/issues/84
	std::wstring port_with_prefix = _prefix_port_if_needed(&sp->port_);
	LPCWSTR lp_port = port_with_prefix.c_str();
#ifdef SERIAL_OVERLAP_MODE
	sp->fd_ = CreateFileW(lp_port,
						  GENERIC_READ | GENERIC_WRITE,
						  0,
						  nullptr,
						  OPEN_EXISTING,
						  FILE_FLAG_OVERLAPPED,
						  nullptr);
#else
	sp->fd_ = CreateFileW(lp_port,
						  GENERIC_READ | GENERIC_WRITE,
						  0,
						  nullptr,
						  OPEN_EXISTING,
						  FILE_ATTRIBUTE_NORMAL,
						  nullptr);
#endif

	if(sp->fd_ == INVALID_HANDLE_VALUE)
	{
		DWORD create_file_err = GetLastError();
		switch(create_file_err)
		{
		case ERROR_FILE_NOT_FOUND:
			printf("[E]\tSpecified port '%s' does not exist", serial_port_getPort(sp).c_str());
			return 3;

		case ERROR_ACCESS_DENIED:
			printf("[E]\tAccess denied when opening '%s' serial port", serial_port_getPort(sp).c_str());
			return 4;

		default:
			printf("[E]\tUnknown error (%ld) opening the '%s' serial port", create_file_err, serial_port_getPort(sp).c_str());
			return 5;
		}
	}

	serial_port_reconfigure(sp);
	sp->is_open_ = true;
	return 0;
}

int serial_port_reconfigure(serial_port_t *sp)
{
	if(sp->fd_ == INVALID_HANDLE_VALUE) return 1;

	DCB dcbSerialParams;
	memset(&dcbSerialParams, 0, sizeof(DCB));
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if(!GetCommState(sp->fd_, &dcbSerialParams))
	{
		printf("[E]\tError getting the serial port state\n");
		return 2;
	}

	// setup baud rate
	switch(sp->baudrate_)
	{
	case 110: dcbSerialParams.BaudRate = CBR_110; break;
	case 300: dcbSerialParams.BaudRate = CBR_300; break;
	case 600: dcbSerialParams.BaudRate = CBR_600; break;
	case 1200: dcbSerialParams.BaudRate = CBR_1200; break;
	case 2400: dcbSerialParams.BaudRate = CBR_2400; break;
	case 4800: dcbSerialParams.BaudRate = CBR_4800; break;
	case 9600: dcbSerialParams.BaudRate = CBR_9600; break;
	case 14400: dcbSerialParams.BaudRate = CBR_14400; break;
	case 19200: dcbSerialParams.BaudRate = CBR_19200; break;
	case 57600: dcbSerialParams.BaudRate = CBR_57600; break;
	case 38400: dcbSerialParams.BaudRate = CBR_38400; break;
	case 115200: dcbSerialParams.BaudRate = CBR_115200; break;
	case 128000: dcbSerialParams.BaudRate = CBR_128000; break;
	case 256000: dcbSerialParams.BaudRate = CBR_256000; break;
	default: dcbSerialParams.BaudRate = sp->baudrate_; break;
	}
	switch(sp->bytesize_)
	{
	case fivebits: dcbSerialParams.ByteSize = 5; break;
	case sixbits: dcbSerialParams.ByteSize = 6; break;
	case sevenbits: dcbSerialParams.ByteSize = 7; break;
	default:
	case eightbits: dcbSerialParams.ByteSize = 8; break;
	}

	switch(sp->stopbits_)
	{
	case stopbits_two: break;
	case stopbits_one_point_five: dcbSerialParams.StopBits = ONE5STOPBITS; break;
	default:
	case stopbits_one: dcbSerialParams.StopBits = ONESTOPBIT; break;
	}

	switch(sp->parity_)
	{
	case parity_even: dcbSerialParams.Parity = EVENPARITY; break;
	case parity_odd: dcbSerialParams.Parity = ODDPARITY; break;
	case parity_mark: dcbSerialParams.Parity = MARKPARITY; break;
	case parity_space: dcbSerialParams.Parity = SPACEPARITY; break;
	default:
	case parity_none: dcbSerialParams.Parity = NOPARITY; break;
	}

	switch(sp->flowcontrol_)
	{
	case flowcontrol_software:
		dcbSerialParams.fOutxCtsFlow = false;
		dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
		dcbSerialParams.fOutX = true;
		dcbSerialParams.fInX = true;
		break;
	case flowcontrol_hardware:
		dcbSerialParams.fOutxCtsFlow = true;
		dcbSerialParams.fRtsControl = RTS_CONTROL_HANDSHAKE;
		dcbSerialParams.fOutX = false;
		dcbSerialParams.fInX = false;
		break;
	default:
	case flowcontrol_none:
		dcbSerialParams.fOutxCtsFlow = false;
		dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
		dcbSerialParams.fOutX = false;
		dcbSerialParams.fInX = false;
		break;
	}

	if(!SetCommState(sp->fd_, &dcbSerialParams))
	{
		CloseHandle(sp->fd_);
		printf("[E]\tError setting serial port settings\n");
		return 3;
	}

	// Setup timeouts
	COMMTIMEOUTS timeouts;
	memset(&timeouts, 0, sizeof(COMMTIMEOUTS));
	timeouts.ReadIntervalTimeout = sp->timeout_.inter_byte_timeout;
	timeouts.ReadTotalTimeoutConstant = sp->timeout_.read_timeout_constant;
	timeouts.ReadTotalTimeoutMultiplier = sp->timeout_.read_timeout_multiplier;
	timeouts.WriteTotalTimeoutConstant = sp->timeout_.write_timeout_constant;
	timeouts.WriteTotalTimeoutMultiplier = sp->timeout_.write_timeout_multiplier;

#ifdef SERIAL_OVERLAP_MODE
	timeouts.ReadIntervalTimeout = 0;
	timeouts.ReadTotalTimeoutConstant = 0;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 0;
	timeouts.WriteTotalTimeoutMultiplier = 0;
#endif

	if(!SetCommTimeouts(sp->fd_, &timeouts))
	{
		printf("[E]\tError setting timeouts\n");
		return 4;
	}

#ifdef SERIAL_OVERLAP_MODE
	if(!SetCommMask(sp->fd_, EV_RXCHAR))
	{
		printf("[E]\tError setting CommMask\n");
		return 5;
	}

	memset(&sp->fd_overlap_read, 0, sizeof(OVERLAPPED));
	memset(&sp->fd_overlap_write, 0, sizeof(OVERLAPPED));
	memset(&sp->fd_overlap_evt, 0, sizeof(OVERLAPPED));

	sp->fd_overlap_read.hEvent = CreateEvent(nullptr, true, false, nullptr);
	sp->fd_overlap_write.hEvent = CreateEvent(nullptr, true, false, nullptr);
	sp->fd_overlap_evt.hEvent = CreateEvent(nullptr, true, false, nullptr);
	if(!sp->fd_overlap_read.hEvent || !sp->fd_overlap_write.hEvent || !sp->fd_overlap_evt.hEvent)
	{
		printf("[E]\tError creating overlap objects\n");
		return 6;
	}

	PurgeComm(sp->fd_, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_TXABORT | PURGE_RXABORT);
#endif
	return 0;
}

int serial_port_close(serial_port_t *sp)
{
	if(sp->is_open_ == true)
	{
		if(sp->fd_ != INVALID_HANDLE_VALUE)
		{
			if(CloseHandle(sp->fd_) == 0)
			{
				printf("[E]\tError while closing serial port, %s\n", GetLastError());
				return 1;
			}
			sp->fd_ = INVALID_HANDLE_VALUE;
		}
		sp->is_open_ = false;
	}
	return 0;
}

size_t serial_port_available(serial_port_t *sp)
{
	if(!sp->is_open_) return 0;

	COMSTAT cs;
	if(!ClearCommError(sp->fd_, nullptr, &cs))
	{
		printf("[E]\tError while checking status of the serial port, %s\n", GetLastError());
	}
	return CAST(size_t, cs.cbInQue);
}

static size_t _serial_port_read(serial_port_t *sp, std::vector<uint8_t> *buf)
{
	if(!sp->is_open_) return 0;

#ifdef SERIAL_OVERLAP_MODE
	DWORD bytes_to_read = 0;
	DWORD err;
	COMSTAT stat;

	if(!sp->evt_waiting)
	{
		sp->evt_mask = EV_RXCHAR;
		if(!WaitCommEvent(sp->fd_, &sp->evt_mask, &sp->fd_overlap_evt))
		{
			if(GetLastError() != ERROR_IO_PENDING)
			{
				printf("[E]WaitCommEvent failed\n");
				return 0;
			}
			else
			{
				sp->evt_waiting = true;
			}
		}
		else
		{
			ClearCommError(sp->fd_, &err, &stat);
			bytes_to_read = stat.cbInQue;
			sp->evt_waiting = false;
		}
	}

	if(sp->evt_waiting)
	{
		switch(WaitForSingleObject(sp->fd_overlap_evt.hEvent, 10 /* ms */))
		{
		case WAIT_OBJECT_0:
			GetOverlappedResult(sp->fd_, &sp->fd_overlap_evt, &sp->evt_mask_len, false);
			ResetEvent(sp->fd_overlap_evt.hEvent);
			ClearCommError(sp->fd_, &err, &stat);
			bytes_to_read = stat.cbInQue;
			sp->evt_waiting = false;
			break;

		case WAIT_TIMEOUT: return 0;

		default:
			printf("[E]\tGetOverlappedResult failed\n");
			sp->evt_waiting = false;
			return 0;
		}
	}

	if(bytes_to_read)
	{
		buf.resize(bytes_to_read);
		if(!ReadFile(sp->fd_, buf.data(), bytes_to_read, &sp->l, &sp->fd_overlap_read))
		{
			printf("[E]ReadFile failed to read buffered data, %s\n", GetLastError());
			return 0;
		}
		if(sp->l != bytes_to_read)
		{
			printf("[E]\tRead file length mismatch\n");
		}
		return bytes_to_read;
	}
	return 0;
#else
	DWORD bytes_read;
	size_t av = serial_port_available(sp);
	buf->resize(av);
	if(!ReadFile(sp->fd_, buf->data(), CAST(DWORD, av), &bytes_read, nullptr))
	{
		printf("[E]ReadFile failed to read buffered data, %s\n", GetLastError());
		return 0;
	}
	return bytes_read;
#endif
}

static size_t _serial_port_write(serial_port_t *sp, const uint8_t *data, size_t length)
{
	if(sp->is_open_ == false) return 0;
	DWORD bytes_written;
#ifdef SERIAL_OVERLAP_MODE
	if(!WriteFile(sp->fd_, data, static_cast<DWORD>(length), &bytes_written, &sp->fd_overlap_write))
	{
		switch(WaitForSingleObject(sp->fd_overlap_write.hEvent, INFINITE))
		{
		case WAIT_OBJECT_0:
			GetOverlappedResult(sp->fd_, &sp->fd_overlap_write, &bytes_written, false);
			ResetEvent(&sp->fd_overlap_write.hEvent);
			break;

		default:
		case WAIT_TIMEOUT:
			printf("[E]WriteFile failed, %s\n", GetLastError());
			return 0;
		}
	}
#else
	if(!WriteFile(sp->fd_, data, static_cast<DWORD>(length), &bytes_written, nullptr))
	{
		return 0;
	}
#endif
	return bytes_written;
}

static void _serial_port_setPort(serial_port_t *sp, const std::string *port)
{
	sp->port_ = std::wstring(port->begin(), port->end());
}

std::string serial_port_getPort(serial_port_t *sp)
{
	return std::string(sp->port_.begin(), sp->port_.end());
}

static void _serial_port_flush(serial_port_t *sp)
{
	if(sp->is_open_) FlushFileBuffers(sp->fd_);
}

static void _serial_port_flushInput(serial_port_t *sp)
{
	if(sp->is_open_) PurgeComm(sp->fd_, PURGE_RXCLEAR);
}

static void _serial_port_flushOutput(serial_port_t *sp)
{
	if(sp->is_open_) PurgeComm(sp->fd_, PURGE_TXCLEAR);
}

void serial_port_setBreak(serial_port_t *sp, bool level)
{
	if(sp->is_open_ == false) return;
	EscapeCommFunction(sp->fd_, level ? SETBREAK : CLRBREAK);
}

void serial_port_setRTS(serial_port_t *sp, bool level)
{
	if(sp->is_open_ == false) return;
	EscapeCommFunction(sp->fd_, level ? SETRTS : CLRRTS);
}

void serial_port_setDTR(serial_port_t *sp, bool level)
{
	if(sp->is_open_ == false) return;
	EscapeCommFunction(sp->fd_, level ? SETDTR : CLRDTR);
}

bool serial_port_waitForChange(serial_port_t *sp)
{
	if(sp->is_open_ == false) return false;
	DWORD dwCommEvent;

	if(!SetCommMask(sp->fd_, EV_CTS | EV_DSR | EV_RING | EV_RLSD))
	{
		printf("[E]\tError setting communications mask\n");
		return false;
	}

	if(!WaitCommEvent(sp->fd_, &dwCommEvent, nullptr))
	{
		printf("[E]\tAn error occurred waiting for the event\n");
		return false;
	}
	return true;
}

bool serial_port_getDSR(serial_port_t *sp)
{
	if(sp->is_open_ == false) return false;
	DWORD dwModemStatus;
	if(!GetCommModemStatus(sp->fd_, &dwModemStatus))
	{
		printf("[E]\tError getting the status of the DSR line\n");
		return false;
	}
	return (MS_DSR_ON & dwModemStatus) != 0;
}

bool serial_port_getRI(serial_port_t *sp)
{
	if(sp->is_open_ == false) return false;
	DWORD dwModemStatus;
	if(!GetCommModemStatus(sp->fd_, &dwModemStatus))
	{
		printf("[E]\tError getting the status of the RI line\n");
		return false;
	}
	return (MS_RING_ON & dwModemStatus) != 0;
}

bool serial_port_getCD(serial_port_t *sp)
{
	if(sp->is_open_ == false) return false;
	DWORD dwModemStatus;
	if(!GetCommModemStatus(sp->fd_, &dwModemStatus))
	{
		printf("[E]\tError getting the status of the CD line\n");
	}
	return (MS_RLSD_ON & dwModemStatus) != 0;
}

int serial_port_readLock(serial_port_t *sp)
{
	if(WaitForSingleObject(sp->read_mutex, INFINITE) != WAIT_OBJECT_0)
	{
		printf("[E]\tError claiming read mutex\n");
		return 1;
	}
	return 0;
}

int serial_port_readUnlock(serial_port_t *sp)
{
	if(!ReleaseMutex(sp->read_mutex))
	{
		printf("[E]\tError releasing read mutex\n");
		return 1;
	}
	return 0;
}

int serial_port_writeLock(serial_port_t *sp)
{
	if(WaitForSingleObject(sp->write_mutex, INFINITE) != WAIT_OBJECT_0)
	{
		printf("[E]\tError claiming write mutex\n");
		return 1;
	}
	return 0;
}

int serial_port_writeUnlock(serial_port_t *sp)
{
	if(!ReleaseMutex(sp->write_mutex))
	{
		printf("[E]\tError releasing write mutex\n");
		return 1;
	}
	return 0;
}

#endif // #if defined(_WIN32)

#if !defined(_WIN32)

#include <errno.h>
#include <fcntl.h>
#include <paths.h>
#include <pthread.h>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/param.h>
#include <sys/signal.h>
#include <sysexits.h>
#include <termios.h>
#include <unistd.h>

#if defined(__linux__)
#include <linux/serial.h>
#endif

#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#ifdef __MACH__
#include <AvailabilityMacros.h>
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#include "impl.hpp"

#ifndef TIOCINQ
#ifdef FIONREAD
#define TIOCINQ FIONREAD
#else
#define TIOCINQ 0x541B
#endif
#endif

#if defined(MAC_OS_X_VERSION_10_3) && (MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_3)
#include <IOKit/serial/ioss.h>
#endif

MillisecondTimer::MillisecondTimer(const uint32_t millis)
	: expiry(timespec_now())
{
	int64_t tv_nsec = expiry.tv_nsec + (millis * 1e6);
	if(tv_nsec >= 1e9)
	{
		int64_t sec_diff = tv_nsec / static_cast<int>(1e9);
		expiry.tv_nsec = tv_nsec % static_cast<int>(1e9);
		expiry.tv_sec += sec_diff;
	}
	else
	{
		expiry.tv_nsec = tv_nsec;
	}
}

int64_t
MillisecondTimer::remaining()
{
	timespec now(timespec_now());
	int64_t millis = (expiry.tv_sec - now.tv_sec) * 1e3;
	millis += (expiry.tv_nsec - now.tv_nsec) / 1e6;
	return millis;
}

timespec
MillisecondTimer::timespec_now()
{
	timespec time;
#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
	clock_serv_t cclock;
	mach_timespec_t mts;
	host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
	clock_get_time(cclock, &mts);
	mach_port_deallocate(mach_task_self(), cclock);
	time.tv_sec = mts.tv_sec;
	time.tv_nsec = mts.tv_nsec;
#else
	clock_gettime(CLOCK_MONOTONIC, &time);
#endif
	return time;
}

timespec
timespec_from_ms(const uint32_t millis)
{
	timespec time;
	time.tv_sec = millis / 1e3;
	time.tv_nsec = (millis - (time.tv_sec * 1e3)) * 1e6;
	return time;
}

Serial::SerialImpl::SerialImpl(const string &port, unsigned long baudrate,
							   bytesize_t bytesize,
							   parity_t parity, stopbits_t stopbits,
							   flowcontrol_t flowcontrol)
	: port_(port), fd_(-1), is_open_(false), xonxoff_(false), rtscts_(false),
	  baudrate_(baudrate), parity_(parity),
	  bytesize_(bytesize), stopbits_(stopbits), flowcontrol_(flowcontrol)
{
	pthread_mutex_init(&this->read_mutex, NULL);
	pthread_mutex_init(&this->write_mutex, NULL);
	if(port_.empty() == false)
		open();
}

void serial_port_deinit(serial_port_t *sp)
{
	_serial_port_close(sp);
	pthread_mutex_destroy(&sp->read_mutex);
	pthread_mutex_destroy(&sp->write_mutex);
}

void serial_port_open(serial_port_t *sp)
{
	if(port_.empty())
	{
		throw invalid_argument("Empty port is invalid.");
	}
	if(is_open_ == true)
	{
		throw SerialException("Serial port already open.");
	}

	fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

	if(fd_ == -1)
	{
		switch(errno)
		{
		case EINTR:
			// Recurse because this is a recoverable error.
			open();
			return;
		case ENFILE:
		case EMFILE:
			THROW(IOException, "Too many file handles open.");
		default:
			THROW(IOException, errno);
		}
	}

	reconfigurePort();
	is_open_ = true;
}

void serial_por_reconfigurePort(serial_port_t *sp)
{
	if(sp->fd_ == -1)
	{
		// Can only operate on a valid file descriptor
		THROW(IOException, "Invalid file descriptor, is the serial port open?");
	}

	struct termios options; // The options for the file descriptor

	if(tcgetattr(fd_, &options) == -1)
	{
		THROW(IOException, "::tcgetattr");
	}

	// set up raw mode / no echo / binary
	options.c_cflag |= (tcflag_t)(CLOCAL | CREAD);
	options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
									ISIG | IEXTEN); //|ECHOPRT

	options.c_oflag &= (tcflag_t) ~(OPOST);
	options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);
#ifdef IUCLC
	options.c_iflag &= (tcflag_t)~IUCLC;
#endif
#ifdef PARMRK
	options.c_iflag &= (tcflag_t)~PARMRK;
#endif

	// setup baud rate
	bool custom_baud = false;
	speed_t baud;
	switch(baudrate_)
	{
#ifdef B0
	case 0: baud = B0; break;
#endif
#ifdef B50
	case 50: baud = B50; break;
#endif
#ifdef B75
	case 75: baud = B75; break;
#endif
#ifdef B110
	case 110: baud = B110; break;
#endif
#ifdef B134
	case 134: baud = B134; break;
#endif
#ifdef B150
	case 150: baud = B150; break;
#endif
#ifdef B200
	case 200: baud = B200; break;
#endif
#ifdef B300
	case 300: baud = B300; break;
#endif
#ifdef B600
	case 600: baud = B600; break;
#endif
#ifdef B1200
	case 1200: baud = B1200; break;
#endif
#ifdef B1800
	case 1800: baud = B1800; break;
#endif
#ifdef B2400
	case 2400: baud = B2400; break;
#endif
#ifdef B4800
	case 4800: baud = B4800; break;
#endif
#ifdef B7200
	case 7200: baud = B7200; break;
#endif
#ifdef B9600
	case 9600: baud = B9600; break;
#endif
#ifdef B14400
	case 14400: baud = B14400; break;
#endif
#ifdef B19200
	case 19200: baud = B19200; break;
#endif
#ifdef B28800
	case 28800: baud = B28800; break;
#endif
#ifdef B57600
	case 57600: baud = B57600; break;
#endif
#ifdef B76800
	case 76800: baud = B76800; break;
#endif
#ifdef B38400
	case 38400: baud = B38400; break;
#endif
#ifdef B115200
	case 115200: baud = B115200; break;
#endif
#ifdef B128000
	case 128000: baud = B128000; break;
#endif
#ifdef B153600
	case 153600: baud = B153600; break;
#endif
#ifdef B230400
	case 230400: baud = B230400; break;
#endif
#ifdef B256000
	case 256000: baud = B256000; break;
#endif
#ifdef B460800
	case 460800: baud = B460800; break;
#endif
#ifdef B500000
	case 500000: baud = B500000; break;
#endif
#ifdef B576000
	case 576000: baud = B576000; break;
#endif
#ifdef B921600
	case 921600: baud = B921600; break;
#endif
#ifdef B1000000
	case 1000000: baud = B1000000; break;
#endif
#ifdef B1152000
	case 1152000: baud = B1152000; break;
#endif
#ifdef B1500000
	case 1500000: baud = B1500000; break;
#endif
#ifdef B2000000
	case 2000000: baud = B2000000; break;
#endif
#ifdef B2500000
	case 2500000: baud = B2500000; break;
#endif
#ifdef B3000000
	case 3000000: baud = B3000000; break;
#endif
#ifdef B3500000
	case 3500000: baud = B3500000; break;
#endif
#ifdef B4000000
	case 4000000: baud = B4000000; break;
#endif
	default:
		custom_baud = true;
		// OS X support
#if defined(MAC_OS_X_VERSION_10_4) && (MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_4)
		// Starting with Tiger, the IOSSIOSPEED ioctl can be used to set arbitrary baud rates
		// other than those specified by POSIX. The driver for the underlying serial hardware
		// ultimately determines which baud rates can be used. This ioctl sets both the input
		// and output speed.
		speed_t new_baud = static_cast<speed_t>(baudrate_);
		if(-1 == ioctl(fd_, IOSSIOSPEED, &new_baud, 1))
		{
			THROW(IOException, errno);
		}
		// Linux Support
#elif defined(__linux__) && defined(TIOCSSERIAL)
		struct serial_struct ser;

		if(-1 == ioctl(fd_, TIOCGSERIAL, &ser))
		{
			THROW(IOException, errno);
		}

		// set custom divisor
		ser.custom_divisor = ser.baud_base / static_cast<int>(baudrate_);
		// update flags
		ser.flags &= ~ASYNC_SPD_MASK;
		ser.flags |= ASYNC_SPD_CUST;

		if(-1 == ioctl(fd_, TIOCSSERIAL, &ser))
		{
			THROW(IOException, errno);
		}
#else
		throw invalid_argument("OS does not currently support custom bauds");
#endif
	}
	if(custom_baud == false)
	{
#ifdef _BSD_SOURCE
		::cfsetspeed(&options, baud);
#else
		::cfsetispeed(&options, baud);
		::cfsetospeed(&options, baud);
#endif
	}

	// setup char len
	options.c_cflag &= (tcflag_t)~CSIZE;
	if(bytesize_ == eightbits)
		options.c_cflag |= CS8;
	else if(bytesize_ == sevenbits)
		options.c_cflag |= CS7;
	else if(bytesize_ == sixbits)
		options.c_cflag |= CS6;
	else if(bytesize_ == fivebits)
		options.c_cflag |= CS5;
	else
		throw invalid_argument("invalid char len");
	// setup stopbits
	if(stopbits_ == stopbits_one)
		options.c_cflag &= (tcflag_t) ~(CSTOPB);
	else if(stopbits_ == stopbits_one_point_five)
		// ONE POINT FIVE same as TWO.. there is no POSIX support for 1.5
		options.c_cflag |= (CSTOPB);
	else if(stopbits_ == stopbits_two)
		options.c_cflag |= (CSTOPB);
	else
		throw invalid_argument("invalid stop bit");
	// setup parity
	options.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
	if(parity_ == parity_none)
	{
		options.c_cflag &= (tcflag_t) ~(PARENB | PARODD);
	}
	else if(parity_ == parity_even)
	{
		options.c_cflag &= (tcflag_t) ~(PARODD);
		options.c_cflag |= (PARENB);
	}
	else if(parity_ == parity_odd)
	{
		options.c_cflag |= (PARENB | PARODD);
	}
#ifdef CMSPAR
	else if(parity_ == parity_mark)
	{
		options.c_cflag |= (PARENB | CMSPAR | PARODD);
	}
	else if(parity_ == parity_space)
	{
		options.c_cflag |= (PARENB | CMSPAR);
		options.c_cflag &= (tcflag_t) ~(PARODD);
	}
#else
	// CMSPAR is not defined on OSX. So do not support mark or space parity.
	else if(parity_ == parity_mark || parity_ == parity_space)
	{
		throw invalid_argument("OS does not support mark or space parity");
	}
#endif // ifdef CMSPAR
	else
	{
		throw invalid_argument("invalid parity");
	}
	// setup flow control
	if(flowcontrol_ == flowcontrol_none)
	{
		xonxoff_ = false;
		rtscts_ = false;
	}
	if(flowcontrol_ == flowcontrol_software)
	{
		xonxoff_ = true;
		rtscts_ = false;
	}
	if(flowcontrol_ == flowcontrol_hardware)
	{
		xonxoff_ = false;
		rtscts_ = true;
	}
	// xonxoff
#ifdef IXANY
	if(xonxoff_)
		options.c_iflag |= (IXON | IXOFF); //|IXANY)
	else
		options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
#else
	if(xonxoff_)
		options.c_iflag |= (IXON | IXOFF);
	else
		options.c_iflag &= (tcflag_t) ~(IXON | IXOFF);
#endif
		// rtscts
#ifdef CRTSCTS
	if(rtscts_)
		options.c_cflag |= (CRTSCTS);
	else
		options.c_cflag &= (unsigned long)~(CRTSCTS);
#elif defined CNEW_RTSCTS
	if(rtscts_)
		options.c_cflag |= (CNEW_RTSCTS);
	else
		options.c_cflag &= (unsigned long)~(CNEW_RTSCTS);
#else
#error "OS Support seems wrong."
#endif

	// http://www.unixwiz.net/techtips/termios-vmin-vtime.html
	// this basically sets the read call up to be a polling read,
	// but we are using select to ensure there is data available
	// to read before each call, so we should never needlessly poll
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 0;

	// activate settings
	::tcsetattr(fd_, TCSANOW, &options);

	// Update byte_time_ based on the new settings.
	uint32_t bit_time_ns = 1e9 / baudrate_;
	byte_time_ns_ = bit_time_ns * (1 + bytesize_ + parity_ + stopbits_);

	// Compensate for the stopbits_one_point_five enum being equal to int 3,
	// and not 1.5.
	if(stopbits_ == stopbits_one_point_five)
	{
		byte_time_ns_ += ((1.5 - stopbits_one_point_five) * bit_time_ns);
	}
}

void serial_port_close(serial_port_t *sp)
{
	if(!sp->is_open_) return;
	if(sp->fd_ != -1)
	{
		int ret;
		ret = ::close(sp->fd_);
		if(ret == 0)
		{
			sp->fd_ = -1;
		}
		else
		{
			THROW(IOException, errno);
		}
	}
	sp->is_open_ = false;
}

size_t serial_port_available(serial_port_t *sp)
{
	if(!sp->is_open_) return 0;
	int count = 0;
	if(-1 == ioctl(sp->fd_, TIOCINQ, &count))
	{
		THROW(IOException, errno);
	}
	else
	{
		return CAST(size_t, count);
	}
}

bool serial_port_waitReadable(serial_port_t *sp, uint32_t timeout)
{
	// Setup a select call to block for serial data or a timeout
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(fd_, &readfds);
	timespec timeout_ts(timespec_from_ms(timeout));
	int r = pselect(fd_ + 1, &readfds, NULL, NULL, &timeout_ts, NULL);
	if(r < 0)
	{
		// Select was interrupted
		if(errno == EINTR) return false;
		// Otherwise there was some error
		THROW(IOException, errno);
	}
	// Timeout occurred
	if(r == 0)
	{
		return false;
	}
	// This shouldn't happen, if r > 0 our fd has to be in the list!
	if(!FD_ISSET(fd_, &readfds))
	{
		THROW(IOException, "select reports ready to read, but our fd isn't"
						   " in the list, this shouldn't happen!");
	}
	// Data available to read.
	return true;
}

void serial_port_waitByteTimes(serial_port_t *sp, size_t count)
{
	timespec wait_time = {0, CAST(long, byte_time_ns_ *count)};
	pselect(0, NULL, NULL, NULL, &wait_time, NULL);
}

size_t _serial_port_read(serial_port_t *sp, uint8_t *buf, size_t size)
{
	// If the port is not open, throw
	if(!sp->is_open_) return 0;
	size_t bytes_read = 0;

	// Calculate total timeout in milliseconds t_c + (t_m * N)
	long total_timeout_ms = timeout_.read_timeout_constant;
	total_timeout_ms += timeout_.read_timeout_multiplier * static_cast<long>(size);
	MillisecondTimer total_timeout(total_timeout_ms);

	// Pre-fill buffer with available bytes
	{
		ssize_t bytes_read_now = ::read(fd_, buf, size);
		if(bytes_read_now > 0)
		{
			bytes_read = bytes_read_now;
		}
	}

	while(bytes_read < size)
	{
		int64_t timeout_remaining_ms = total_timeout.remaining();
		if(timeout_remaining_ms <= 0)
		{
			// Timed out
			break;
		}
		// Timeout for the next select is whichever is less of the remaining
		// total read timeout and the inter-byte timeout.
		uint32_t timeout = std::min(static_cast<uint32_t>(timeout_remaining_ms),
									timeout_.inter_byte_timeout);
		// Wait for the device to be readable, and then attempt to read.
		if(waitReadable(timeout))
		{
			// If it's a fixed-length multi-byte read, insert a wait here so that
			// we can attempt to grab the whole thing in a single IO call. Skip
			// this wait if a non-max inter_byte_timeout is specified.
			if(size > 1 && timeout_.inter_byte_timeout == Timeout::max())
			{
				size_t bytes_available = available();
				if(bytes_available + bytes_read < size)
				{
					waitByteTimes(size - (bytes_available + bytes_read));
				}
			}
			// This should be non-blocking returning only what is available now
			//  Then returning so that select can block again.
			ssize_t bytes_read_now =
				::read(fd_, buf + bytes_read, size - bytes_read);
			// read should always return some data as select reported it was
			// ready to read when we get to this point.
			if(bytes_read_now < 1)
			{
				// Disconnected devices, at least on Linux, show the
				// behavior that they are always ready to read immediately
				// but reading returns nothing.
				throw SerialException("device reports readiness to read but "
									  "returned no data (device disconnected?)");
			}
			// Update bytes_read
			bytes_read += static_cast<size_t>(bytes_read_now);
			// If bytes_read == size then we have read everything we need
			if(bytes_read == size)
			{
				break;
			}
			// If bytes_read < size then we have more to read
			if(bytes_read < size)
			{
				continue;
			}
			// If bytes_read > size then we have over read, which shouldn't happen
			if(bytes_read > size)
			{
				throw SerialException("read over read, too many bytes where "
									  "read, this shouldn't happen, might be "
									  "a logical error!");
			}
		}
	}
	return bytes_read;
}

size_t _serial_port_write(serial_port_t *sp, const uint8_t *data, size_t length)
{
	if(sp->is_open_ == false) return 0;
	fd_set writefds;
	size_t bytes_written = 0;

	// Calculate total timeout in milliseconds t_c + (t_m * N)
	long total_timeout_ms = timeout_.write_timeout_constant;
	total_timeout_ms += timeout_.write_timeout_multiplier * static_cast<long>(length);
	MillisecondTimer total_timeout(total_timeout_ms);

	bool first_iteration = true;
	while(bytes_written < length)
	{
		int64_t timeout_remaining_ms = total_timeout.remaining();
		// Only consider the timeout if it's not the first iteration of the loop
		// otherwise a timeout of 0 won't be allowed through
		if(!first_iteration && (timeout_remaining_ms <= 0))
		{
			// Timed out
			break;
		}
		first_iteration = false;

		timespec timeout(timespec_from_ms(timeout_remaining_ms));

		FD_ZERO(&writefds);
		FD_SET(fd_, &writefds);

		// Do the select
		int r = pselect(fd_ + 1, NULL, &writefds, NULL, &timeout, NULL);

		// Figure out what happened by looking at select's response 'r'
		/** Error **/
		if(r < 0)
		{
			// Select was interrupted, try again
			if(errno == EINTR)
			{
				continue;
			}
			// Otherwise there was some error
			THROW(IOException, errno);
		}
		/** Timeout **/
		if(r == 0)
		{
			break;
		}
		/** Port ready to write **/
		if(r > 0)
		{
			// Make sure our file descriptor is in the ready to write list
			if(FD_ISSET(fd_, &writefds))
			{
				// This will write some
				ssize_t bytes_written_now =
					::write(fd_, data + bytes_written, length - bytes_written);
				// write should always return some data as select reported it was
				// ready to write when we get to this point.
				if(bytes_written_now < 1)
				{
					// Disconnected devices, at least on Linux, show the
					// behavior that they are always ready to write immediately
					// but writing returns nothing.
					throw SerialException("device reports readiness to write but "
										  "returned no data (device disconnected?)");
				}
				// Update bytes_written
				bytes_written += static_cast<size_t>(bytes_written_now);
				// If bytes_written == size then we have written everything we need to
				if(bytes_written == length)
				{
					break;
				}
				// If bytes_written < size then we have more to write
				if(bytes_written < length)
				{
					continue;
				}
				// If bytes_written > size then we have over written, which shouldn't happen
				if(bytes_written > length)
				{
					throw SerialException("write over wrote, too many bytes where "
										  "written, this shouldn't happen, might be "
										  "a logical error!");
				}
			}
			// This shouldn't happen, if r > 0 our fd has to be in the list!
			THROW(IOException, "select reports ready to write, but our fd isn't"
							   " in the list, this shouldn't happen!");
		}
	}
	return bytes_written;
}

void serial_port_setPort(serial_port_t *sp, const std::string &port)
{
	sp->port_ = port;
}

std::string _serial_port_getPort(serial_port_t *sp) { return sp->port_; }

void serial_port_flush(serial_port_t *sp)
{
	if(sp->is_open_ == false) return;
	tcdrain(sp->fd_);
}

void _serial_port_flushInput(serial_port_t *sp)
{
	if(sp->is_open_ == false) return;
	tcflush(sp->fd_, TCIFLUSH);
}

void _serial_port_flushOutput(serial_port_t *sp)
{
	if(sp->is_open_ == false) return;
	tcflush(sp->fd_, TCOFLUSH);
}

void serial_port_sendBreak(serial_port_t *sp, int duration)
{
	if(sp->is_open_ == false) return;
	tcsendbreak(sp->fd_, static_cast<int>(duration / 4));
}

void serial_port_setBreak(serial_port_t *sp, bool level)
{
	if(sp->is_open_ == false) return;
	if(level)
	{
		if(-1 == ioctl(sp->fd_, TIOCSBRK))
		{
			stringstream ss;
			ss << "setBreak failed on a call to ioctl(TIOCSBRK): " << errno << " " << strerror(errno);
			throw(SerialException(ss.str().c_str()));
		}
	}
	else
	{
		if(-1 == ioctl(sp->fd_, TIOCCBRK))
		{
			stringstream ss;
			ss << "setBreak failed on a call to ioctl(TIOCCBRK): " << errno << " " << strerror(errno);
			throw(SerialException(ss.str().c_str()));
		}
	}
}

void serial_port_setRTS(serial_port_t *sp, bool level)
{
	if(sp->is_open_ == false) return;
	int command = TIOCM_RTS;
	if(level)
	{
		if(-1 == ioctl(sp->fd_, TIOCMBIS, &command))
		{
			stringstream ss;
			ss << "setRTS failed on a call to ioctl(TIOCMBIS): " << errno << " " << strerror(errno);
			throw(SerialException(ss.str().c_str()));
		}
	}
	else
	{
		if(-1 == ioctl(sp->fd_, TIOCMBIC, &command))
		{
			stringstream ss;
			ss << "setRTS failed on a call to ioctl(TIOCMBIC): " << errno << " " << strerror(errno);
			throw(SerialException(ss.str().c_str()));
		}
	}
}

void serial_port_setDTR(serial_port_t *sp, bool level)
{
	if(sp->is_open_ == false) return;
	int command = TIOCM_DTR;
	if(level)
	{
		if(-1 == ioctl(sp->fd_, TIOCMBIS, &command))
		{
			stringstream ss;
			ss << "setDTR failed on a call to ioctl(TIOCMBIS): " << errno << " " << strerror(errno);
			throw(SerialException(ss.str().c_str()));
		}
	}
	else
	{
		if(-1 == ioctl(sp->fd_, TIOCMBIC, &command))
		{
			stringstream ss;
			ss << "setDTR failed on a call to ioctl(TIOCMBIC): " << errno << " " << strerror(errno);
			throw(SerialException(ss.str().c_str()));
		}
	}
}

bool serial_port_waitForChange(serial_port_t *sp)
{
#ifndef TIOCMIWAIT
	while(sp->is_open_ == true)
	{
		int status;
		if(-1 == ioctl(sp->fd_, TIOCMGET, &status))
		{
			stringstream ss;
			ss << "waitForChange failed on a call to ioctl(TIOCMGET): " << errno << " " << strerror(errno);
			throw(SerialException(ss.str().c_str()));
		}
		if(0 != (status & TIOCM_CTS) || 0 != (status & TIOCM_DSR) || 0 != (status & TIOCM_RI) || 0 != (status & TIOCM_CD))
		{
			return true;
		}

		std::this_thread::sleep_for(std::chrono::microseconds(1000));
	}
	return false;
#else
	int command = (TIOCM_CD | TIOCM_DSR | TIOCM_RI | TIOCM_CTS);
	if(-1 == ioctl(sp->fd_, TIOCMIWAIT, &command))
	{
		stringstream ss;
		ss << "waitForDSR failed on a call to ioctl(TIOCMIWAIT): "
		   << errno << " " << strerror(errno);
		throw(SerialException(ss.str().c_str()));
	}
	return true;
#endif
}

bool serial_port_getCTS(serial_port_t *sp)
{
	if(sp->is_open_ == false) return false;
	int status;
	if(-1 == ioctl(sp->fd_, TIOCMGET, &status))
	{
		stringstream ss;
		ss << "getCTS failed on a call to ioctl(TIOCMGET): " << errno << " " << strerror(errno);
		throw(SerialException(ss.str().c_str()));
	}
	return 0 != (status & TIOCM_CTS);
}

bool serial_port_getDSR(serial_port_t *sp)
{
	if(sp->is_open_ == false) return false;
	int status;
	if(-1 == ioctl(sp->fd_, TIOCMGET, &status))
	{
		stringstream ss;
		ss << "getDSR failed on a call to ioctl(TIOCMGET): " << errno << " " << strerror(errno);
		throw(SerialException(ss.str().c_str()));
	}
	return 0 != (status & TIOCM_DSR);
}

bool serial_port_getRI(serial_port_t *sp)
{
	if(sp->is_open_ == false) return false;
	int status;
	if(-1 == ioctl(sp->fd_, TIOCMGET, &status))
	{
		stringstream ss;
		ss << "getRI failed on a call to ioctl(TIOCMGET): " << errno << " " << strerror(errno);
		throw(SerialException(ss.str().c_str()));
	}
	return 0 != (status & TIOCM_RI);
}

bool serial_port_getCD(serial_port_t *sp)
{
	if(sp->is_open_ == false) return false;
	int status;
	if(-1 == ioctl(sp->fd_, TIOCMGET, &status))
	{
		stringstream ss;
		ss << "getCD failed on a call to ioctl(TIOCMGET): " << errno << " " << strerror(errno);
		throw(SerialException(ss.str().c_str()));
	}
	return 0 != (status & TIOCM_CD);
}

void serial_port_readLock(serial_port_t *sp)
{
	int result = pthread_mutex_lock(&sp->read_mutex);
	if(result)
	{
		THROW(IOException, result);
	}
}

void serial_port_readUnlock(serial_port_t *sp)
{
	int result = pthread_mutex_unlock(&sp->read_mutex);
	if(result)
	{
		THROW(IOException, result);
	}
}

void serial_port_writeLock(serial_port_t *sp)
{
	int result = pthread_mutex_lock(&sp->write_mutex);
	if(result)
	{
		THROW(IOException, result);
	}
}

void serial_port_writeUnlock(serial_port_t *sp)
{
	int result = pthread_mutex_unlock(&sp->write_mutex);
	if(result)
	{
		THROW(IOException, result);
	}
}

size_t _serial_port_read(serial_port_t *sp, std::vector<uint8_t> *buf)
{
	auto count = read(sp->buf_rx, sizeof(sp->buf_rx));
	buf->assign(sp->buf_rx, sp->buf_rx + count);
	return count;
}

#endif // !defined(_WIN32)

#include <algorithm>

//#if !defined(_WIN32) && !defined(__OpenBSD__) && !defined(__FreeBSD__)
//#include <alloca.h>
//#endif

//#if defined (__MINGW32__)
//#define alloca __builtin_alloca
//#endif

#include "serial_interface.h"

size_t serial_port_read(serial_port_t *sp, std::vector<uint8_t> *buffer)
{
	serial_port_readLock(sp);
	size_t readed = _serial_port_read(sp, buffer);
	serial_port_readUnlock(sp);
	return readed;
}

size_t serial_port_write(serial_port_t *sp, const std::vector<uint8_t> data)
{
	serial_port_readLock(sp);
	serial_port_writeLock(sp);
	size_t writed = _serial_port_write(sp, &data[0], data.size());
	serial_port_writeUnlock(sp);
	serial_port_readUnlock(sp);
	return writed;
}

void serial_port_setPort(serial_port_t *sp, const std::string *port)
{
	serial_port_readLock(sp);
	serial_port_writeLock(sp);
	bool was_open = sp->is_open_;
	if(was_open) serial_port_close(sp);
	_serial_port_setPort(sp, port);
	if(was_open) serial_port_open(sp);
	serial_port_writeUnlock(sp);
	serial_port_readUnlock(sp);
}

void serial_port_flush(serial_port_t *sp)
{
	serial_port_readLock(sp);
	serial_port_writeLock(sp);
	_serial_port_flush(sp);
	serial_port_writeUnlock(sp);
	serial_port_readUnlock(sp);
}

void serial_port_flushInput(serial_port_t *sp)
{
	serial_port_readLock(sp);
	_serial_port_flushInput(sp);
	serial_port_readUnlock(sp);
}

void serial_port_flushOutput(serial_port_t *sp)
{
	serial_port_writeLock(sp);
	_serial_port_flushOutput(sp);
	serial_port_writeUnlock(sp);
}

/// === PORT LIST === ///

#if defined(_WIN32)
// Convert a wide Unicode string to an UTF8 string
std::string utf8_encode(const std::wstring &wstr)
{
	int size_needed = WideCharToMultiByte(CP_UTF8, 0, &wstr[0], CAST(int, wstr.size()), nullptr, 0, nullptr, nullptr);
	std::string strTo(CAST(size_t, size_needed), 0);
	WideCharToMultiByte(CP_UTF8, 0, &wstr[0], CAST(int, wstr.size()), &strTo[0], size_needed, nullptr, nullptr);
	return strTo;
}
std::vector<PortInfo> serial_port_get_list(void)
{
	std::vector<PortInfo> devices_found;
	HDEVINFO device_info_set = SetupDiGetClassDevs(CAST(const GUID *, &GUID_DEVCLASS_PORTS), nullptr, nullptr, DIGCF_PRESENT);

	unsigned int device_info_set_index = 0;
	SP_DEVINFO_DATA device_info_data;

	device_info_data.cbSize = sizeof(SP_DEVINFO_DATA);

	while(SetupDiEnumDeviceInfo(device_info_set, device_info_set_index, &device_info_data))
	{
		device_info_set_index++;

		// Get port name
		HKEY hkey = SetupDiOpenDevRegKey(device_info_set, &device_info_data, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);

		TCHAR port_name[port_name_max_length];
		DWORD port_name_length = port_name_max_length;

		LONG return_code = RegQueryValueEx(hkey, _T("PortName"), nullptr, nullptr, (LPBYTE)port_name, &port_name_length);

		RegCloseKey(hkey);

		if(return_code != EXIT_SUCCESS) continue;

		if(port_name_length > 0 && port_name_length <= port_name_max_length)
			port_name[port_name_length - 1] = '\0';
		else
			port_name[0] = '\0';

		if(_tcsstr(port_name, _T("LPT")) != nullptr) continue; // Ignore parallel ports

		// Get port friendly name
		TCHAR friendly_name[friendly_name_max_length];
		DWORD friendly_name_actual_length = 0;

		BOOL got_friendly_name = SetupDiGetDeviceRegistryProperty(
			device_info_set,
			&device_info_data,
			SPDRP_FRIENDLYNAME,
			nullptr,
			(PBYTE)friendly_name,
			friendly_name_max_length,
			&friendly_name_actual_length);

		if(got_friendly_name == true && friendly_name_actual_length > 0)
			friendly_name[friendly_name_actual_length - 1] = '\0';
		else
			friendly_name[0] = '\0';

		// Get hardware ID
		TCHAR hardware_id[hardware_id_max_length];
		DWORD hardware_id_actual_length = 0;

		BOOL got_hardware_id = SetupDiGetDeviceRegistryProperty(
			device_info_set,
			&device_info_data,
			SPDRP_HARDWAREID,
			nullptr,
			(PBYTE)hardware_id,
			hardware_id_max_length,
			&hardware_id_actual_length);

		if(got_hardware_id == true && hardware_id_actual_length > 0)
			hardware_id[hardware_id_actual_length - 1] = '\0';
		else
			hardware_id[0] = '\0';

#ifdef UNICODE
		std::string portName = utf8_encode(port_name);
		std::string friendlyName = utf8_encode(friendly_name);
		std::string hardwareId = utf8_encode(hardware_id);
#else
		std::string portName = port_name;
		std::string friendlyName = friendly_name;
		std::string hardwareId = hardware_id;
#endif

		PortInfo port_entry;
		port_entry.port = portName;
		port_entry.description = friendlyName;
		port_entry.hardware_id = hardwareId;
		devices_found.push_back(port_entry);
	}

	SetupDiDestroyDeviceInfoList(device_info_set);

	return devices_found;
}
#endif // #if defined(_WIN32)

#if defined(__linux__) || defined(__CYGWIN__)
static std::vector<std::string> glob_(const vector<string> &patterns)
{
	std::vector<string> paths_found;

	if(patterns.size() == 0) return paths_found;

	glob_t glob_results;

	int glob_retval = glob(patterns[0].c_str(), 0, nullptr, &glob_results);

	std::vector<string>::const_iterator iter = patterns.begin();

	while(++iter != patterns.end())
	{
		glob_retval = glob(iter->c_str(), GLOB_APPEND, nullptr, &glob_results);
	}

	for(int path_index = 0; path_index < (int)glob_results.gl_pathc; path_index++)
	{
		paths_found.push_back(glob_results.gl_pathv[path_index]);
	}
	globfree(&glob_results);
	return paths_found;
}

static std::string basename(const string &path)
{
	size_t pos = path.rfind("/");
	if(pos == std::string::npos) return path;
	return string(path, pos + 1, string::npos);
}

static std::string dirname(const string &path)
{
	size_t pos = path.rfind("/");
	if(pos == std::string::npos) return path;
	if(pos == 0) return "/";
	return string(path, 0, pos);
}

static bool path_exists(const string &path)
{
	struct stat sb;

	if(stat(path.c_str(), &sb) == 0)
		return true;

	return false;
}

static std::string realpath_(const string &path)
{
	char *real_path = realpath(path.c_str(), nullptr);
	std::string result;
	if(real_path != nullptr)
	{
		result = real_path;
		free(real_path);
	}
	return result;
}

static std::string usb_sysfs_friendly_name(const string &sys_usb_path)
{
	unsigned int device_number = 0;
	std::istringstream(read_line(sys_usb_path + "/devnum")) >> device_number;
	std::string manufacturer = read_line(sys_usb_path + "/manufacturer");
	std::string product = read_line(sys_usb_path + "/product");
	std::string serial = read_line(sys_usb_path + "/serial");
	if(manufacturer.empty() && product.empty() && serial.empty()) return "";
	return format("%s %s %s", manufacturer.c_str(), product.c_str(), serial.c_str());
}

static std::vector<std::string> get_sysfs_info_(const string &device_path)
{
	std::string device_name = basename(device_path);
	std::string friendly_name;
	std::string hardware_id;
	std::string sys_device_path = format("/sys/class/tty/%s/device", device_name.c_str());

	if(device_name.compare(0, 6, "ttyUSB") == 0)
	{
		sys_device_path = dirname(dirname(realpath_(sys_device_path)));

		if(path_exists(sys_device_path))
		{
			friendly_name = usb_sysfs_friendly_name(sys_device_path);
			hardware_id = usb_sysfs_hw_string(sys_device_path);
		}
	}
	else if(device_name.compare(0, 6, "ttyACM") == 0)
	{
		sys_device_path = dirname(realpath_(sys_device_path));

		if(path_exists(sys_device_path))
		{
			friendly_name = usb_sysfs_friendly_name(sys_device_path);
			hardware_id = usb_sysfs_hw_string(sys_device_path);
		}
	}
	else
	{
		// Try to read ID string of PCI device
		string sys_id_path = sys_device_path + "/id";
		if(path_exists(sys_id_path)) hardware_id = read_line(sys_id_path);
	}

	if(friendly_name.empty()) friendly_name = device_name;
	if(hardware_id.empty()) hardware_id = "n/a";

	std::vector<string> result;
	result.push_back(friendly_name);
	result.push_back(hardware_id);
	return result;
}

static std::string read_line(const string &file)
{
	std::ifstream ifs(file.c_str(), ifstream::in);
	std::string line;
	if(ifs)
	{
		getline(ifs, line);
	}
	return line;
}

static std::string format(const char *format, ...)
{
	va_list ap;
	size_t buffer_size_bytes = 256;
	std::string result;
	char *buffer = (char *)malloc(buffer_size_bytes);

	if(buffer == nullptr) return result;

	bool done = false;
	unsigned int loop_count = 0;

	while(!done)
	{
		va_start(ap, format);

		int return_value = vsnprintf(buffer, buffer_size_bytes, format, ap);

		if(return_value < 0)
		{
			done = true;
		}
		else if(return_value >= (int)buffer_size_bytes)
		{
			// Realloc and try again.
			buffer_size_bytes = return_value + 1;
			char *new_buffer_ptr = (char *)realloc(buffer, buffer_size_bytes);

			if(new_buffer_ptr == nullptr)
			{
				done = true;
			}
			else
			{
				buffer = new_buffer_ptr;
			}
		}
		else
		{
			result = buffer;
			done = true;
		}

		va_end(ap);

		if(++loop_count > 5) done = true;
	}
	free(buffer);
	return result;
}

static std::string usb_sysfs_hw_string(const string &sysfs_path)
{
	string serial_number = read_line(sysfs_path + "/serial");

	if(serial_number.length() > 0)
	{
		serial_number = format("SNR=%s", serial_number.c_str());
	}

	std::string vid = read_line(sysfs_path + "/idVendor");
	std::string pid = read_line(sysfs_path + "/idProduct");

	return format("USB VID:PID=%s:%s %s", vid.c_str(), pid.c_str(), serial_number.c_str());
}

std::vector<PortInfo> serial_port_get_list(void)
{
	vector<PortInfo> results;

	vector<string> search_globs;
	search_globs.push_back("/dev/ttyACM*");
	search_globs.push_back("/dev/ttyS*");
	search_globs.push_back("/dev/ttyUSB*");
	search_globs.push_back("/dev/tty.*");
	search_globs.push_back("/dev/cu.*");

	vector<string> devices_found = glob_(search_globs);

	vector<string>::iterator iter = devices_found.begin();

	while(iter != devices_found.end())
	{
		std::string device = *iter++;
		vector<string> sysfs_info = get_sysfs_info_(device);
		string friendly_name = sysfs_info[0];
		string hardware_id = sysfs_info[1];

		PortInfo device_entry;
		device_entry.port = device;
		device_entry.description = friendly_name;
		device_entry.hardware_id = hardware_id;
		results.push_back(device_entry);
	}
	return results;
}

#endif // defined(__linux__)