#include "TcpConnection.hpp"

TcpConnection::TcpConnection(int port) : port(port), socket(io_context) {
    boost::asio::ip::tcp::acceptor acceptor(io_context, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port));
    std::cout << "Waiting for connection on port " << port << "..." << std::endl;
    acceptor.accept(socket);  // Accept connection on socket
    std::cout << "Connected!" << std::endl;
}

int TcpConnection::getPort() const {
    return port;
}

void TcpConnection::sendFrame(const cv::Mat& frame) {
    std::vector<uchar> buffer;
    cv::imencode(".jpg", frame, buffer);
    int frameSize = buffer.size();

    // Convert frameSize to network byte order (big endian)
    int netFrameSize = htonl(frameSize);  // Use htonl for 32-bit integers

    // Send frame size first (now in network byte order)
    boost::asio::write(this->socket, boost::asio::buffer(&netFrameSize, sizeof(netFrameSize)));

    // Send the frame data
    boost::asio::write(this->socket, boost::asio::buffer(buffer.data(), buffer.size()));
}

std::string TcpConnection::receiveStringData() {
    boost::asio::streambuf buf;
    std::string receivedData;

    try {
        boost::asio::read_until(socket, buf, "\n");

        std::istream stream(&buf);
        std::getline(stream, receivedData);
    } catch (std::exception& e) {
        std::cerr << "Error receiving data: " << e.what() << std::endl;
    }

    return receivedData;
}


void TcpConnection::sendStringData(const std::string& data) {
    try {
        boost::asio::write(this->socket, boost::asio::buffer(data));
    } catch (const std::exception& e) {
        std::cerr << "Error sending string over TCP: " << e.what() << std::endl;
    }
}
