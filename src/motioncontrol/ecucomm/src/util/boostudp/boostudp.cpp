/*
  This file is part of asio-udp-device, a class wrapper to
  use the boost::asio udp functionality.

  asio-udp-device is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Nathan Michael, Oct. 2011
*/

#include <iostream>
#include <boost/version.hpp>
#include "boostudp.h"

using namespace std;

using boost::asio::ip::udp;
//初始化
BoostUdp::BoostUdp()
{
  isopen = false;
  socket = NULL;
  bytes_transferred = 0;
}

BoostUdp::BoostUdp(const string local_ip,unsigned int local_port)
{
  isopen = false;
  socket = NULL;
  bytes_transferred = 0;

  open(local_ip,local_port);//打开端口号和ip地址
}

BoostUdp::BoostUdp(const string local_ip,unsigned int local_port,
                             const string &ip_address,
                             unsigned int remote_port)
{
  isopen = false;
  socket = NULL;
  bytes_transferred = 0;

  open(local_ip,local_port, ip_address, remote_port);
}

BoostUdp::~BoostUdp()
{
  if (isopen)
    close();

  if (socket!=NULL)
    delete socket;

}

void BoostUdp::open(const string local_ip,unsigned int local_port)
{
  udp::endpoint local_endpoint(boost::asio::ip::address::from_string(local_ip), local_port);

  if (!isopen)
    {
      socket = new udp::socket(io_service, local_endpoint);
#if BOOST_VERSION >= 104700
      socket->non_blocking(true);
#else
      boost::asio::socket_base::non_blocking_io non_blocking_command(true);
      socket->io_control(non_blocking_command);
#endif

      if (!socket->is_open())
        throw runtime_error("Failed to open socket");

      isopen = true;
    }
}

void BoostUdp::connectRemoteEndpoint(
                         const string &remote_ip_address,
                         unsigned int remote_port)
{

    send_remote_endpoint=udp::endpoint(boost::asio::ip::address::from_string(remote_ip_address), remote_port);

      try
        {
          socket->connect(send_remote_endpoint);
        }
      catch (std::exception e)
        {
          cerr << "Failed to connect to remote socket" << endl;
          throw;
        }

}

void BoostUdp::open(const string local_ip,unsigned int local_port,
                         const string &remote_ip_address,
                         unsigned int remote_port)
{
  udp::endpoint local_endpoint(boost::asio::ip::address::from_string(local_ip), local_port);
  
  send_remote_endpoint=udp::endpoint(boost::asio::ip::address::from_string(remote_ip_address), remote_port);

  if (!isopen)
    {
      socket = new udp::socket(io_service, local_endpoint);
#if BOOST_VERSION >= 104700
      socket->non_blocking(true);
#else
      boost::asio::socket_base::non_blocking_io non_blocking_command(true);
      socket->io_control(non_blocking_command);
#endif


      try
        {
          socket->connect(send_remote_endpoint);
        }
      catch (std::exception e)
        {
          cerr << "Failed to connect to remote socket" <<e.what()<< endl;
          throw;
        }

      if (!socket->is_open())
        throw runtime_error("Failed to opensocket");


      isopen = true;
    }
}

void BoostUdp::close()
{
  if (isopen)
    {
      if (socket != 0)
        socket->close();
      isopen = false;
    }
}


void BoostUdp::setReadCallback(const boost::function<void (const char*, size_t)>& handler)
{
  read_callback = handler;
  //std::cout<<"setReadCallBack!"<<std::endl;
}

bool BoostUdp::send(const vector<char>& msg)
{
  if (!isopen)
    return false;

  try
    {
      socket->send(boost::asio::buffer(&(msg[0]), msg.size()));
    }
  catch (const boost::system::system_error& err)
    {
      cerr << "Failed to send! "<<err.what() << endl;
      //throw;
    }

  return true;
}

bool BoostUdp::send(const string& msg)
{
  if (!isopen)
    return false;

  try
    {
      socket->send(boost::asio::buffer(msg.c_str(), msg.size()));
    }
  catch (const boost::system::system_error& err)
    {
      cerr << "Failed to send! "<<err.what()<< endl;
      //throw;
    }

  return true;
}

bool BoostUdp::send(char*  msg,size_t len)
{
  if (!isopen)
    return false;

  try
    {
      socket->send(boost::asio::buffer(msg, len));
    }
  catch (const boost::system::system_error& err)
    {
     // cerr << "Failed to send! "<<err.what() << endl;
    }

  return true;
}



void BoostUdp::receive()
{

//udp::endpoint   remote_endpoint;
  bytes_transferred=0;
  if (socket->available() > 0)
    {
      try
        {
          bytes_transferred = socket->receive_from(boost::asio::buffer(read_msg, MAX_READ_LENGTH),receive_remote_endpoint);
//          std:: cout<<" bytes_transferred="<< bytes_transferred<<std::endl;
//         char *str ;
//         str = read_msg;
//         str[bytes_transferred] = '\0';
//         std:: cout<<" data="<< str<<std::endl;
        }
      catch (boost::system::system_error& err)
        {
          std::cerr << "Error: " << err.what() << std::endl;
        }
    }


  if (!read_callback.empty() && (bytes_transferred > 0))
  {
     // std::cout<<"from:"<<remote_endpoint.address().to_string().c_str()<<" "
     //                <<remote_endpoint.port()<<std::endl;
    std::cout<<"callback"<<std::endl;
    read_callback(const_cast<char *>(read_msg), bytes_transferred);
  }
}

void BoostUdp::getReiceiveAddr(std::string &ip_address,unsigned int& remote_port)
{
    ip_address=receive_remote_endpoint.address().to_string();
    remote_port=receive_remote_endpoint.port();
}

void BoostUdp::getMessage(char*  msg,size_t& len)
{

      len=bytes_transferred;
      memcpy(msg,read_msg,len);
    //msg=read_msg;
    //std::cout<<(int)read_msg[0]<<" "<<(int)msg[0]<<std::endl;
   //std::cout<<"len="<<len<<endl;
}
