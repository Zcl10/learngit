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

#ifndef __BOOSTUDP__
#define __BOOSTUDP__

#include <string>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>

#define MAX_READ_LENGTH 10010
  using boost::asio::ip::udp;
class BoostUdp
{
 public:
  BoostUdp();
  BoostUdp(const std::string local_ip,unsigned int local_port);
  BoostUdp(const std::string local_ip,unsigned int local_port,
                const std::string &ip_address,
                unsigned int remote_port);
  ~BoostUdp();

  void open(const std::string local_ip,unsigned int local_port);
  void open(const std::string local_ip,unsigned int local_port,
            const std::string &ip_address,
            unsigned int remote_port);
  void connectRemoteEndpoint(
                         const std::string &remote_ip_address,
                         unsigned int remote_port);
  void close();

  void receive();
  void setReadCallback(const boost::function<void (const char*, size_t)>& handler);

  bool send(const std::string& msg);
  bool send(const std::vector<char>& msg);
  bool send(char*  msg,size_t len);
  void getMessage(char*  msg,size_t& len);
  void getReiceiveAddr(std::string &ip_address,unsigned int& remote_port);
private:
	 size_t bytes_transferred;
	 char read_msg[MAX_READ_LENGTH];

  udp::endpoint   send_remote_endpoint;
  udp::endpoint   receive_remote_endpoint;
  bool isopen;

  boost::asio::io_service io_service;
  boost::asio::ip::udp::socket* socket;

  boost::function<void (const char*, size_t)> read_callback;
 

};
#endif
