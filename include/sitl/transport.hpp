#pragma once
// ===========================================================================
// ITransport — thin abstraction so a firmware bridge can run over either a
// socket (SITL) or a serial link (HITL) with no change to its send/parse
// logic. Adapters wrap the concrete transports, which keep their existing
// value-typed API for the paths that don't need polymorphism (ArduPilot JSON,
// Betaflight MSP).
//
// Per-tick cost is a handful of virtual calls against the physics step — not
// per byte — so the dispatch is negligible.
//
// Adapters own their transport via unique_ptr, which lets an already-open
// transport be handed to a bridge without reopening it (important for a live
// USB board: reopening would reset the CDC-ACM link and drop the HITL session).
// ===========================================================================
#include "sitl/socket_transport.hpp"
#include "sitl/serial_transport.hpp"
#include <cstdint>
#include <cstddef>
#include <memory>
#include <vector>

namespace dronesim::sitl {

// ---------------------------------------------------------------------------
struct ITransport {
    virtual ~ITransport() = default;

    virtual bool     open()                          noexcept = 0;
    virtual void     close()                         noexcept = 0;
    [[nodiscard]] virtual bool is_open()       const noexcept = 0;
    [[nodiscard]] virtual bool is_connected()  const noexcept = 0;
    virtual bool     send(const uint8_t* data, size_t len) noexcept = 0;
    virtual int      recv(uint8_t* buf, int max_len)      noexcept = 0;

    // Port for status/display: socket returns local_port; serial returns 0.
    [[nodiscard]] virtual uint16_t port_hint() const noexcept = 0;

    // Convenience overload shared by all transports.
    bool send(const std::vector<uint8_t>& v) noexcept { return send(v.data(), v.size()); }
};

// ---------------------------------------------------------------------------
// SocketTransportAdapter — wraps the existing SocketTransport (SITL paths)
// ---------------------------------------------------------------------------
class SocketTransportAdapter final : public ITransport {
public:
    explicit SocketTransportAdapter(SocketTransport::Config cfg)
        : _t(std::make_unique<SocketTransport>(std::move(cfg))) {}
    explicit SocketTransportAdapter(std::unique_ptr<SocketTransport> t) noexcept
        : _t(std::move(t)) {}

    bool open()                    noexcept override { return _t->open(); }
    void close()                   noexcept override { _t->close(); }
    bool is_open()           const noexcept override { return _t->is_open(); }
    bool is_connected()      const noexcept override { return _t->is_connected(); }
    bool send(const uint8_t* d, size_t n) noexcept override { return _t->send(d, n); }
    int  recv(uint8_t* b, int m)          noexcept override { return _t->recv(b, m); }
    uint16_t port_hint()     const noexcept override { return _t->config().local_port; }

    [[nodiscard]] SocketTransport& inner() noexcept { return *_t; }

private:
    std::unique_ptr<SocketTransport> _t;
};

// ---------------------------------------------------------------------------
// SerialTransportAdapter — wraps SerialTransport (HITL over USB/UART)
// ---------------------------------------------------------------------------
class SerialTransportAdapter final : public ITransport {
public:
    explicit SerialTransportAdapter(SerialTransport::Config cfg)
        : _t(std::make_unique<SerialTransport>(std::move(cfg))) {}
    explicit SerialTransportAdapter(std::unique_ptr<SerialTransport> t) noexcept
        : _t(std::move(t)) {}

    bool open()                    noexcept override { return _t->open(); }
    void close()                   noexcept override { _t->close(); }
    bool is_open()           const noexcept override { return _t->is_open(); }
    bool is_connected()      const noexcept override { return _t->is_connected(); }
    bool send(const uint8_t* d, size_t n) noexcept override { return _t->send(d, n); }
    int  recv(uint8_t* b, int m)          noexcept override { return _t->recv(b, m); }
    uint16_t port_hint()     const noexcept override { return 0; }

    [[nodiscard]] SerialTransport& inner() noexcept { return *_t; }

private:
    std::unique_ptr<SerialTransport> _t;
};

} // namespace dronesim::sitl
