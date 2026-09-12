#pragma once
// ===========================================================================
// SerialTransport — USB/UART transport for hardware-in-the-loop (HITL).
//
// Deliberately mirrors the *method surface* of SocketTransport
// (open/close/is_open/is_connected/send/recv/config) so a firmware bridge
// that talks to SITL over a socket can talk to a physical flight controller
// over USB with no change to its send/parse logic — only the transport type
// differs.
//
// Use: a physical PX4 board enumerates as a CDC-ACM serial device
//   Linux:   /dev/ttyACM0   macOS: /dev/cu.usbmodemXXXX   Windows: COM3
// For USB-ACM the baud rate is nominally ignored by the device, but a valid
// speed is still programmed into termios/DCB; telemetry radios DO honour it.
//
// Non-blocking by contract: recv() returns 0 when no bytes are available.
// ===========================================================================

#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#else
#  include <termios.h>
#  include <fcntl.h>
#  include <unistd.h>
#  include <errno.h>
#  include <sys/ioctl.h>
#endif

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <mutex>

namespace dronesim::sitl {

// ---------------------------------------------------------------------------
// SerialTransport
// ---------------------------------------------------------------------------
class SerialTransport {
public:
    struct Config {
        std::string port{};            // "/dev/ttyACM0", "COM3", ...
        uint32_t    baud{921600};      // ignored by USB-ACM; honoured by UART
        int         recv_buf_bytes{65536};   // accepted for parity; advisory
        int         send_buf_bytes{65536};
    };

    explicit SerialTransport(Config cfg) noexcept : _cfg(std::move(cfg)) {}
    ~SerialTransport() { close(); }

    SerialTransport(const SerialTransport&)            = delete;
    SerialTransport& operator=(const SerialTransport&) = delete;

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------
    [[nodiscard]] bool open() noexcept {
#ifdef _WIN32
        return _open_win32();
#else
        return _open_posix();
#endif
    }

    void close() noexcept {
#ifdef _WIN32
        if (_h != INVALID_HANDLE_VALUE) { ::CloseHandle(_h); _h = INVALID_HANDLE_VALUE; }
#else
        if (_fd >= 0) { ::close(_fd); _fd = -1; }
#endif
    }

    // A serial link has no peer handshake: "connected" == "port is open".
    // Deciding *when* to start streaming HIL (after the autopilot is
    // identified) is the router's job, not the transport's.
    [[nodiscard]] bool is_open() const noexcept {
#ifdef _WIN32
        return _h != INVALID_HANDLE_VALUE;
#else
        return _fd >= 0;
#endif
    }
    [[nodiscard]] bool is_connected() const noexcept { return is_open(); }

    // -----------------------------------------------------------------------
    // send — writes all bytes; returns false on unrecoverable error
    // -----------------------------------------------------------------------
    bool send(const uint8_t* data, size_t len) noexcept {
        if (!is_open()) return false;
        std::lock_guard<std::mutex> lk(_send_mtx);
        size_t sent = 0;
        while (sent < len) {
#ifdef _WIN32
            DWORD wrote = 0;
            if (!::WriteFile(_h, data + sent, static_cast<DWORD>(len - sent),
                             &wrote, nullptr))
                return false;
            if (wrote == 0) return false;
            sent += wrote;
#else
            ssize_t n = ::write(_fd, data + sent, len - sent);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue; // drain
                if (errno == EINTR) continue;
                return false;
            }
            sent += static_cast<size_t>(n);
#endif
        }
        return true;
    }

    bool send(const std::vector<uint8_t>& buf) noexcept {
        return send(buf.data(), buf.size());
    }

    // -----------------------------------------------------------------------
    // recv — non-blocking. Returns byte count (0 = nothing yet, <0 = error).
    // -----------------------------------------------------------------------
    int recv(uint8_t* buf, int max_len) noexcept {
        if (!is_open()) return -1;
#ifdef _WIN32
        DWORD got = 0;
        if (!::ReadFile(_h, buf, static_cast<DWORD>(max_len), &got, nullptr))
            return -1;
        return static_cast<int>(got); // COMMTIMEOUTS set for immediate return
#else
        ssize_t n = ::read(_fd, buf, static_cast<size_t>(max_len));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
            if (errno == EINTR) return 0;
            return -1;
        }
        return static_cast<int>(n); // 0 is legitimate (no data) for a tty
#endif
    }

    const Config& config() const noexcept { return _cfg; }

private:
    Config     _cfg;
    std::mutex _send_mtx;

#ifdef _WIN32
    // -----------------------------------------------------------------------
    // Win32 COM path. API-accurate; compiled/validated on Windows only.
    // -----------------------------------------------------------------------
    HANDLE _h{INVALID_HANDLE_VALUE};

    bool _open_win32() noexcept {
        std::string name = _cfg.port;
        if (name.rfind("\\\\.\\", 0) != 0) name = "\\\\.\\" + name; // COM10+ needs prefix
        _h = ::CreateFileA(name.c_str(), GENERIC_READ | GENERIC_WRITE, 0,
                           nullptr, OPEN_EXISTING, 0, nullptr);
        if (_h == INVALID_HANDLE_VALUE) return false;

        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);
        if (!::GetCommState(_h, &dcb)) { close(); return false; }
        dcb.BaudRate = _cfg.baud;
        dcb.ByteSize = 8;
        dcb.Parity   = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary  = TRUE;
        dcb.fOutxCtsFlow = FALSE; dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl  = DTR_CONTROL_ENABLE;
        dcb.fRtsControl  = RTS_CONTROL_ENABLE;
        dcb.fInX = FALSE; dcb.fOutX = FALSE;
        if (!::SetCommState(_h, &dcb)) { close(); return false; }

        // Non-blocking reads: return immediately with whatever is buffered.
        COMMTIMEOUTS to{};
        to.ReadIntervalTimeout         = MAXDWORD;
        to.ReadTotalTimeoutConstant    = 0;
        to.ReadTotalTimeoutMultiplier  = 0;
        to.WriteTotalTimeoutConstant   = 100;
        to.WriteTotalTimeoutMultiplier = 0;
        if (!::SetCommTimeouts(_h, &to)) { close(); return false; }

        ::SetupComm(_h, static_cast<DWORD>(_cfg.recv_buf_bytes),
                        static_cast<DWORD>(_cfg.send_buf_bytes));
        ::PurgeComm(_h, PURGE_RXCLEAR | PURGE_TXCLEAR);
        return true;
    }
#else
    // -----------------------------------------------------------------------
    // POSIX termios path (Linux + macOS).
    // -----------------------------------------------------------------------
    int _fd{-1};

    static speed_t _to_speed(uint32_t baud) noexcept {
        switch (baud) {
            case 57600:   return B57600;
            case 115200:  return B115200;
            case 230400:  return B230400;
#ifdef B460800
            case 460800:  return B460800;
#endif
#ifdef B500000
            case 500000:  return B500000;
#endif
#ifdef B921600
            case 921600:  return B921600;
#endif
#ifdef B1500000
            case 1500000: return B1500000;
#endif
            default:      return B115200; // safe fallback (USB-ACM ignores it)
        }
    }

    bool _open_posix() noexcept {
        _fd = ::open(_cfg.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (_fd < 0) return false;

        termios tio{};
        if (::tcgetattr(_fd, &tio) != 0) { close(); return false; }

        // Raw mode: no canonical processing, no echo, 8N1, no flow control.
        cfmakeraw(&tio);
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~CRTSCTS;
        tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        tio.c_cflag |= CS8;
        tio.c_cc[VMIN]  = 0;   // non-blocking: return whatever is available
        tio.c_cc[VTIME] = 0;

        const speed_t sp = _to_speed(_cfg.baud);
        cfsetispeed(&tio, sp);
        cfsetospeed(&tio, sp);

        if (::tcsetattr(_fd, TCSANOW, &tio) != 0) { close(); return false; }
        ::tcflush(_fd, TCIOFLUSH);
        return true;
    }
#endif
};

} // namespace dronesim::sitl
