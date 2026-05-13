#include "WhisperMessage.h"
#include <iostream>
#include <span>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

bool send_all(int socket, std::span<const std::byte> data)
{
    size_t total_sent = 0;
    
    while (total_sent < data.size()) {
        const void *current_ptr = data.data() + total_sent;
        size_t bytes_remaining = data.size() - total_sent;
        ssize_t sent = send(socket, current_ptr, bytes_remaining, 0);
        if (sent == -1) {
            if (errno == EINTR)
                continue;
            std::cerr << "Send error: " << strerror(errno) << std::endl;
            return false;
        }

        total_sent += static_cast<size_t>(sent);
    }

    return true;
}

bool recv_all(int socket, std::span<std::byte> data)
{
    size_t total_received = 0;

    while (total_received < data.size()) {
        void *current_ptr = data.data() + total_received;
        size_t bytes_remaining = data.size() - total_received;
        ssize_t received = recv(socket, current_ptr, bytes_remaining, 0);
        if (received == 0) {
            std::cerr << "Connection closed by peer before buffer was filled." << std::endl;
            return false;
        }
        if (received == -1) {
            if (errno == EINTR)
                continue;
            std::cerr << "Recv error: " << strerror(errno) << std::endl;
            return false;
        }
        total_received += static_cast<size_t>(received);
    }

    return true;
}

class WhisperClient
{
public:
    WhisperClient(const std::string &server_ip, int port) : server_ip_(server_ip), port_(port)
    {
        socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_ < 0)
            throw std::runtime_error("cannot create socket");

        sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);
        inet_pton(AF_INET, server_ip.c_str(), &serv_addr.sin_addr);

        if (connect(socket_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
            throw std::runtime_error("connection failed");
    }

    ~WhisperClient()
    {
        close(socket_);
    }

    bool command(
        WhisperMessage req,
        WhisperMessage rsp
    ) {
        char buffer[sizeof(WhisperMessage)];
        req.serializeTo(buffer);

        if (not send_all(socket_, std::as_bytes(std::span(buffer))))
            return false;

        if (not recv_all(socket_, std::as_writable_bytes(std::span(buffer))))
            return false;

        rsp = WhisperMessage::deserializeFrom(buffer);
        return true;
    }

    bool step(uint32_t hart)
    {
        WhisperMessage msg(hart, Step);

        char buffer[sizeof(WhisperMessage)];
        msg.serializeTo(buffer);

        if (not send_all(socket_, std::as_bytes(std::span(buffer))))
            return false;

        if (not recv_all(socket_, std::as_writable_bytes(std::span(buffer))))
            return false;

        msg = WhisperMessage::deserializeFrom(buffer);
        nchanges_ = msg.value;
        return true;
    }

    bool get_change(uint32_t hart, WhisperMessage &change)
    {
        WhisperMessage msg(hart, Change);

        char buffer[sizeof(WhisperMessage)];
        msg.serializeTo(buffer);

        if (not send_all(socket_, std::as_bytes(std::span(buffer))))
            return false;

        if (not recv_all(socket_, std::as_writable_bytes(std::span(buffer))))
            return false;

        change = WhisperMessage::deserializeFrom(buffer);
        return true;
    }

    bool get_changes(uint32_t hart, std::vector<WhisperMessage> &changes)
    {
        changes.clear();
        for (; nchanges_; nchanges_--) {
            WhisperMessage change;
            get_change(hart, change);
            changes.push_back(change);
        }
        return true;
    }

    bool peek(
        uint32_t hart,
		uint32_t resource,
        uint64_t address,
        uint32_t size,
		uint64_t &value
    ) {
        WhisperMessage msg(hart, Peek, resource, address, value, size, 0, 0);

        char buffer[sizeof(WhisperMessage)];
        msg.serializeTo(buffer);

        if (not send_all(socket_, std::as_bytes(std::span(buffer))))
            return false;

        if (not recv_all(socket_, std::as_writable_bytes(std::span(buffer))))
            return false;

        msg = WhisperMessage::deserializeFrom(buffer);
        value = msg.value;
        return true;
    }

    bool poke(
        uint32_t hart,
		uint32_t resource,
        uint64_t address,
		uint64_t value,
        uint32_t size,
        uint64_t tag = 0,
		uint64_t time = 0
    ) {
        WhisperMessage msg(hart, Poke, resource, address, value, size, tag, time);

        char buffer[sizeof(WhisperMessage)];
        msg.serializeTo(buffer);

        if (not send_all(socket_, std::as_bytes(std::span(buffer))))
            return false;

        if (not recv_all(socket_, std::as_writable_bytes(std::span(buffer))))
            return false;

        msg = WhisperMessage::deserializeFrom(buffer);
        return true;
    }

private:
    std::string server_ip_;
    int port_;
    int socket_;
    int nchanges_;
};

int main(int argc, char *argv[])
{
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " PORT\n";
        return -1;
    }

    const int port = atoi(argv[1]);
    WhisperClient client("127.0.0.1", port);
    uint32_t hart = 0;

    client.step(hart);

    std::vector<WhisperMessage> changes;
    client.get_changes(hart, changes);
    for (const auto &change : changes) {
        std::cout << char(change.resource) << " " << change.address << " " << change.value << "\n";
    }

    client.poke(hart, 'r', 1, 0xdeadbeef, 8);

    uint64_t value;
    char resource = 'r';
    uint64_t address = 1;
    client.peek(hart, resource, 1, 8, value);
    std::cout << std::hex << resource << " " << address << " " << value << std::dec << "\n";

    return 0;
}
