/* Descriptions for this sample code.
1. How to compile this sample code.
gcc -o a boost_server.cc -D__SAMPLE_MAIN__ -lboost_system-mt -lboost_thread-mt

2. Realized points.
 a. Server function
 b. Separated Server Framework and Child Session
 c. Realized the separation with template of C++
*/
#include <set>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

using boost::asio::ip::tcp;

//////////////////////////////////////////////////////////////////
// Sample_Child_Session : Copy this class to make new child_session
struct Sample_Child_Session
{
  //////////////////////////////////////////////////////////////////
  // Sample_Child_Session.Variables
  boost::asio::io_service       _ioservice;
  boost::asio::ip::tcp::socket  _socket;
  boost::asio::streambuf        _buffer;
  boost::thread                 _thread;
  // Add your own variables here

  //////////////////////////////////////////////////////////////////
  // Sample_Child_Session.Constructor
  Sample_Child_Session(void) : _socket(_ioservice) {}

  //////////////////////////////////////////////////////////////////
  // Sample_Child_Session.Child_Session_Loop_Core
  void Child_Session_Loop_Core(void)
  {
    boost::system::error_code err;
//std::cout << "Child_Session_Loop a [" << this << "]" << std::endl;
    boost::asio::read_until(_socket, _buffer , "\r\n", err);
//std::cout << "Child_Session_Loop b [" << this << "]" << std::endl;
/* SAMPLE CODE : Display received data for debug
    std::iostream st(&_buffer);
    std::stringstream s;
    s << st.rdbuf();
    std::cout << CString(s.str().c_str()) << std::endl;
    st << s.str() << std::endl;
*/
// SAMPLE CODE : Sendback received data for debug
    boost::asio::write(_socket, _buffer);
//
//std::cout << "Child_Session_Loop c [" << this << "]" << std::endl;
//std::cout << "Child_Session_Loop d [" << this << "]" << std::endl;
  }

  //////////////////////////////////////////////////////////////////
  // Sample_Child_Session.Accept_Process
  void Accept_Process(void)
  {
//std::cout << "Accept_Process a [" << this << "]" << std::endl;
// SAMPLE CODE : Sendback Accepted Acknowledgment for debug
    boost::asio::streambuf  ack_comment;
    std::iostream st(&ack_comment);
    st << "+ -- Accepted ["  << this << "]" << std::endl;;
    boost::asio::write(_socket, ack_comment);
//std::cout << "Accept_Process b [" << this << "]" << std::endl;
    while(1)
      Child_Session_Loop_Core();
//std::cout << "Accept_Process c [" << this << "]" << std::endl;
  }
};

//////////////////////////////////////////////////////////////////
// Server_Framework
template <typename Child_Session> class Server_Framework
{
  //////////////////////////////////////////////////////////////////
  // Server_Framework.Variables
private:
  boost::asio::io_service         _ioservice;
  boost::asio::ip::tcp::acceptor  _acceptor;
  std::set<Child_Session*>        _Child_Session_list;
  Child_Session                  *_new_Child_Sessionp;
  boost::thread                   _thread;
public:

  //////////////////////////////////////////////////////////////////
  // Server_Framework.Make_A_Child_Session_and_Accept_Loop
  void Make_A_Child_Session_and_Accept_Loop(void)
  {
    while(1)
    {
      _new_Child_Sessionp = new Child_Session;
      _Child_Session_list.insert(_new_Child_Sessionp);
std::cout << "Made a child session [" << _new_Child_Sessionp<<"]"<<std::endl;
      boost::system::error_code err;
      _acceptor.accept(_new_Child_Sessionp->_socket, err);
std::cout << "After accept\n";
      _new_Child_Sessionp->_thread = boost::thread(
                             boost::bind(&Child_Session::Accept_Process
                                         , _new_Child_Sessionp));
std::cout << "After thread\n";
    }
  }

  //////////////////////////////////////////////////////////////////
  // Server_Framework.Remove_Child_Session
  void Remove_Child_Session(Child_Session *Child_Sessionp)
  {
    delete Child_Sessionp;
    _Child_Session_list.erase(Child_Sessionp);
  }

  //////////////////////////////////////////////////////////////////
  // Server_Framework.Constractor
  Server_Framework(unsigned short port) try
    :_acceptor(_ioservice
        , boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
    , _new_Child_Sessionp(NULL)
  {
    _thread = boost::thread(
        boost::bind(&Server_Framework::Make_A_Child_Session_and_Accept_Loop
                          , this));
  }
  catch(std::exception const &err)
  {
    std::cerr << err.what() << std::endl;
  }

  //////////////////////////////////////////////////////////////////
  // Server_Framework.Destractor
  ~Server_Framework()
  {
    _thread.join();
    for(typename std::set<Child_Session*>::iterator 
      i = _Child_Session_list.begin(); i != _Child_Session_list.end(); i++)
        delete *i;
  }
};

#ifdef __SAMPLE_MAIN__
int main(void)
{
  Server_Framework<Sample_Child_Session> sf(3000);
  while(1)
    sleep(10);
  return 0;
}
#endif
