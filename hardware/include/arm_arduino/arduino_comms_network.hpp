#ifndef ARM_ARDUINO_ARDUINO_COMMS_HPP
#define ARM_ARDUINO_ARDUINO_COMMS_HPP

#include <sstream>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>
#include <netdb.h>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <mutex>
#include <vector>

#define A_10 -57.296
#define B_10 90.0

bool ConnectToServer(int &skt, int portno, std::string ip_addr)
{
    struct sockaddr_in serv_addr;
    struct hostent *server;

    skt = socket(AF_INET, SOCK_STREAM, 0);
    if (skt < 0)
    {
        printf("ERROR opening socket");
        return false;
    }
    server = gethostbyname(ip_addr.c_str());
    if (server == NULL)
    {
        fprintf(stderr,"ERROR, no such host\n");
        return false;
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(skt,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    {
        printf("ERROR connecting");
        return false;
    }
    return true;
}

bool ReadNetworkMessage(const int &socket, std::string &msg)
{
    try
    {
        char buffer[1024];
        bzero(buffer, 1024);

        int n = read(socket, buffer, 1024);
        if(n<=0)
        {
          printf("ERROR reading to socket");
          return false;
        }
        msg = buffer;
    }
    catch(...)
    {
      return false;
    }
}

bool SendNetworkMessage(const int &socket, const std::string &msg)
{
  try
  {
    int n = send(socket, msg.c_str(), msg.length(), MSG_NOSIGNAL);
    if (n < 0)
    {
      printf("ERROR writing to socket");
      return false;
    }
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return false;
  }

  return true;
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(std::string ip_addr, int32_t network_port_number)
  {
    m_ip_addr = ip_addr;
    m_network_port_number = network_port_number;

    ConnectToServer(m_socket, m_network_port_number, m_ip_addr);
  }

  void disconnect()
  {
    close(m_socket);
  }

  bool connected() const
  {
    if(m_socket<0)
    {
      return false;
    }
    return true;
  }

  std::string send_msg_get_response(const std::string &msg_to_send, bool print_output = false)
  {
    std::string response;
    std::string msg = "send_get_response|" + msg_to_send;

    bool res = SendNetworkMessage(m_socket, msg);
    res = ReadNetworkMessage(m_socket, response);
    if (print_output)
    {
      std::cout << "Sent: " << msg << " Recv: " << response << std::endl;
    }

    return response;
  }

  void send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    std::string msg = "send|" + msg_to_send;
    SendNetworkMessage(m_socket, msg);
  }

  void send_empty_msg()
  {
    send_msg("\r");
  }

  std::vector<double> ConvertAngles(const std::vector<double> &joint_positions)
  {
    std::vector<double> angles;
    angles.resize(7);

    if (count==0)
    {
      for(int i = 0; i < 7; i++)
      {
        init_angle[i] = joint_positions[i];
      }
    }

    for(int i = 0; i < 6; i++)
    {
      angles[i] = (long)((joint_positions[i]-init_angle[i])*stepsPerRad[i]);
    }
    angles[1] *= -1.0;
    angles[2] *= -1.0;
    angles[4] *= -1.0;
    
    double x = ((joint_positions[6]-init_angle[6])*stepsPerRad[6]);
    double y = A_10*x+B_10;
    angles[6] = y;
    count=1;
    return angles;
  }

  void set_joint_values(const std::vector<double> &joint_positions)
  {
    std::vector<double> angles = ConvertAngles(joint_positions);
    std::stringstream ss;
    ss << "$" << angles[0];
    for(int i = 1; i < angles.size(); i++)
    {
      ss << "," << angles[i];
    }
    ss << '\n';
    // std::string s = send_msg_get_response(ss.str());
    send_msg(ss.str());
  }

  // void set_joint_values(const std::vector<double> &joint_positions)
  // {
  //   std::stringstream ss;
  //   ss << joint_positions[0];
  //   for(int i = 1; i < joint_positions.size(); i++)
  //   {
  //     ss << " " << joint_positions[i];
  //   }
  //   send_msg(ss.str());
  // }

private:
    int m_socket=-1;
    std::string m_ip_addr;
    int32_t m_network_port_number;
    int count = 0;
    double init_angle[7] = {0,0,0,0,0,0,0}; 
    float stepsPerRad[7] = {10185.916357881,5602.253996835,22190.746351098,1018.591635788,5653.183578624,1018.591635788,1.0};  // microsteps/revolution (using 16ths) from observation, for each motor
};

#endif // ARM_ARDUINO_ARDUINO_COMMS_HPP