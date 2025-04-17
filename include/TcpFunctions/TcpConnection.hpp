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
    explicit TcpConnection(int port);
    int getPort() const;
    void sendFrame(const cv::Mat& frame);
    std::string receiveData();
};

#endif
