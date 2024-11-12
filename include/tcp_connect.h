#ifndef TCPCONN_H
#define TCPCONN_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include <arpa/inet.h>  // For htonl function on Linux

class TcpConnection
{
    int port;
    boost::asio::io_context io_context;
    boost::asio::ip::tcp::socket socket;
public:
    TcpConnection(int port = 9999);
    int getPort();
    void sendFrame(const cv::Mat& frame);
};

TcpConnection::TcpConnection(int port) : port(port), socket(io_context){
    
    boost::asio::ip::tcp::acceptor acceptor(io_context, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port));
    std::cout << "Waiting for connection on port " << port << "..." << std::endl;
    acceptor.accept(socket);  // Accept connection on socket
    std::cout << "Connected!" << std::endl;
}

int TcpConnection::getPort(){
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

#endif