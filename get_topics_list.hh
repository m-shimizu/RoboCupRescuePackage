// Desctiption: 
// This program's function is getting topics list from running gazebo, 
//   and find a topic name including some keywords which you wanna get.
// In USARGazebo.cc, you can see a sample code for using this program.
//   To find the sample code in USARGazebo.cc, search "TopicsList".
//
// Author : Masaru Shimizu
// E-Mail : shimizu@sist.chukyo-u.ac.jp
// Date   : 5.2015
//

#ifndef GETPCLST_HH
#define GETPCLST_HH

#include <set>
#include <iostream>

//////////////////////////////////////////////////////////////////
// Topics List 
struct TopicsList
{
  //////////////////////////////////////////////////////////////////
  // TopicsList.Variables
  int                              _already_called_get_topics;
  std::string                      _data;
  gazebo::msgs::Packet             _packet;
  gazebo::msgs::Request            _request;
  gazebo::transport::ConnectionPtr _connection;
  gazebo::msgs::GzString_V         _topics;
  std::set<char*>                  _topics_list;

  //////////////////////////////////////////////////////////////////
  // TopicsList.Constructor
  TopicsList(void) : _data(), _packet(), _request()
                      , _topics(), _already_called_get_topics(0)
                      , _topics_list()
  {
    _topics_list.clear();
  }

  //////////////////////////////////////////////////////////////////
  // TopicsList.Destructor
  ~TopicsList()
  {
    Clear_topics_list();
  }

  //////////////////////////////////////////////////////////////////
  // Return the number of topics in _topics_list
  int Size(void)
  {
    return _topics_list.size();
  }

  //////////////////////////////////////////////////////////////////
  // TopicsList.Add a topics to _topics_list
  void Add_a_topics(const char* _a_topic)
  {
    char* _newp;
    _newp = new char[strlen(_a_topic)+2];
    strcpy(_newp, _a_topic);
    _topics_list.insert(_newp);
  }

  //////////////////////////////////////////////////////////////////
  // TopicsList.Clear _topics_list
  void Clear_topics_list(void)
  {
    if(0 == Size())
      return;
    for(typename std::set<char*>::iterator i = _topics_list.begin()
      ; i != _topics_list.end(); i++)
        delete *i;
    _topics_list.clear();
  }

  //////////////////////////////////////////////////////////////////
  // TopicsList.Get_Topics_List
  // SAMPLE CODE to get a list of topics from 
  //   https://bitbucket.org/osrf/gazebo/src/5eed0402ea08b3dff261d25ef8da82db426bbab6/tools/gz_topic.cc?at=default#cl-87
  void Get_Topics_List(void)
  {
    _connection = gazebo::transport::connectToMaster();
    _request.set_id(0);
    _request.set_request("get_topics");
    _connection->EnqueueMsg(gazebo::msgs::Package("request",_request),true);
    _connection->Read(_data);
    _packet.ParseFromString(_data);
    _topics.ParseFromString(_packet.serialized_data());
    _connection.reset();
    _already_called_get_topics = 1;
    Clear_topics_list();
    for(int i = 0; i < _topics.data_size(); ++i)
    {
      std::stringstream _s(_topics.data(i));
      Add_a_topics((const char*)_s.str().c_str());
    }
// Codes for debug
/*std::cout << "_topics size = " << _topics.data_size() << std::endl;
Disp_Topics();
std::cout << "_topics_list size = " << Size() << std::endl;
Disp_topics_list();*/
  }

  //////////////////////////////////////////////////////////////////
  // TopicsList.Refresh_Topics_List
  void Refresh_Topics_List(void)
  {
    Get_Topics_List();
  }

  //////////////////////////////////////////////////////////////////
  // TopicsList.Filter : remove topic entries which does not have _keyword
  //                       from _topics_list
  int Filter(const char* _keyword = NULL)
  {
//    if(1 != _already_called_get_topics)
//      Get_Topics_List();
    if(0 == Size())
      return 0;
    if(NULL == _keyword || (NULL != _keyword && 0 == _keyword[0]))
      return Size();
    for(typename std::set<char*>::iterator i = _topics_list.begin()
      ; i != _topics_list.end();)
    {
      if(NULL == strcasestr(*i, _keyword))
      {
        delete *i;
        _topics_list.erase(*i);
        i = _topics_list.begin();
        continue;
      }
      i++;
    }
//std::cout << "Remain = " << Size() << std::endl;
    return Size();
  }

  //////////////////////////////////////////////////////////////////
  // TopicsList.Search_n : Search about 3 keywords in _topics_list,
  //  and return _num'th string's pointer
  //  (The first hitted string's _num is 0).
  const char* Search_n(int _num, const char* _keyword1 = NULL, 
                           const char* _keyword2 = NULL,
                           const char* _keyword3 = NULL)
  {
//    if(1 != _already_called_get_topics)
//      Get_Topics_List();
    if(0 == Size())
      return NULL;
    for(typename std::set<char*>::iterator i = _topics_list.begin()
      ; i != _topics_list.end(); i++)
      if(NULL == _keyword3 || (NULL != _keyword3 && 0 == _keyword3[0]) || 
         (NULL != _keyword3 && 0 != _keyword3[0]
          && NULL != strcasestr(*i, _keyword3)))
        if(NULL == _keyword2 || (NULL != _keyword2 && 0 == _keyword2[0]) || 
           (NULL != _keyword2 && 0 != _keyword2[0]
            && NULL != strcasestr(*i, _keyword2)))
          /* Memo for Debug
          if(NULL == _keyword1 || (NULL != _keyword1 && 0 == _keyword1[0])||
             (NULL != _keyword1 && 0 != _keyword1[0]
              && NULL != strcasestr(*i, _keyword1)))  */
          if((NULL != _keyword1 && 0 != _keyword1[0]
              && NULL != strcasestr(*i, _keyword1)))
            if(0 == _num--)
              return *i;
    return NULL;
  }

  //////////////////////////////////////////////////////////////////
  // TopicsList.Search : Search about 3 keywords in _topics_list,
  //  and return the first hitted string's pointer.
  const char* Search(const char* _keyword1 = NULL, 
                     const char* _keyword2 = NULL,
                     const char* _keyword3 = NULL)
  {
    Search_n(0, _keyword1, _keyword2, _keyword3);
  }

  //////////////////////////////////////////////////////////////////
  // TopicsList.Disp_Topics
  void Disp_topics(void)
  {
    for (int i = 0; i < _topics.data_size(); ++i)
    {
      std::cout << _topics.data(i) << std::endl;
    }
  }

  //////////////////////////////////////////////////////////////////
  // TopicsList.Disp_Topics_List
  void Disp_topics_list(void)
  {
    if(0 == Size())
      return;
    for(typename std::set<char*>::iterator i = _topics_list.begin()
      ; i != _topics_list.end(); i++)
        std::cout <<  *i << std::endl;
  }
};

#endif
