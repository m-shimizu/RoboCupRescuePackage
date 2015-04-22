// Desctiption: 
// This program's function is breaking an USAR command string into some parameters consisted from a name and a value string.
// At the bottom of thie file, you can see a sample code for using this program.
//
// Author : Masaru Shimizu
// E-Mail : shimizu@sist.chukyo-u.ac.jp
// Date   : 4.2015
//

#ifndef BUCIP_HH
#define BUCIP_HH

#include <set>
#include <iostream>

//////////////////////////////////////////////////////////////////
// Means of error codes
enum Break_USAR_Command_Into_Params_Error_Code
{
  BIE_GOOD         =  0,
  BIE_NO_COMMAND   = -1,
  BIE_BROKEN_BRACE = -2
};

//////////////////////////////////////////////////////////////////
// Definition of a data structure for having a parameter
struct UC_Param
{
  //////////////////////////////////////////////////////////////////
  // Variables
  char _Name[20];
  char _Value[100];

  //////////////////////////////////////////////////////////////////
  // Display a parameter consisted from a Name and a Value string
  void Disp(void) { std::cout << _Name << " " << _Value << std::endl; }
};

#ifndef _MIN
#define _MIN(X,Y) (((X)<(Y))?(X):(Y))
#endif

//////////////////////////////////////////////////////////////////
// Definition of breaking an usar command string into some parameters
struct Break_USAR_Command_Into_Params
{

  //////////////////////////////////////////////////////////////////
  // Variables
  int                 _Err_code;
  UC_Param*           _ucpp;
  char*               _cmdbuf;
  std::set<UC_Param*> _UC_Param_list;

  //////////////////////////////////////////////////////////////////
  // The destructor
  ~Break_USAR_Command_Into_Params() 
  {
    if(NULL != _cmdbuf)
      delete _cmdbuf; 
    Remove_all();
  }

  //////////////////////////////////////////////////////////////////
  // Do something to initialize. 
  void Init(void) { _UC_Param_list.clear(); }

  //////////////////////////////////////////////////////////////////
  // The Constructor
  Break_USAR_Command_Into_Params(char* cb) : _cmdbuf(NULL), _Err_code(BIE_GOOD)
  {
    Init();
    if(0 < strlen(cb))
    {
      _cmdbuf = new char[strlen(cb)+1]; 
      strcpy(_cmdbuf, cb);
      Break_Into();
    }
    else
      _Err_code = BIE_NO_COMMAND;
  }

  //////////////////////////////////////////////////////////////////
  // Return an error code in a processing of constructor
  int Error_code(void)
  {
    return _Err_code;
  }

  //////////////////////////////////////////////////////////////////
  // Search Name and return Value
  char* Search(const char* name){ return Search((char*)name); }
  char* Search(char* name)
  {
    if(0 == Size())
      return NULL;
    for(typename std::set<UC_Param*>::iterator
      i = _UC_Param_list.begin(); i != _UC_Param_list.end(); i++)
    {
      UC_Param* upp = *i;
      if(0 == strncasecmp(upp->_Name, name, strlen(name)))
        return upp->_Value;
    }
    return NULL;
  }

  //////////////////////////////////////////////////////////////////
  // Return the number of pairs of brace
  int Size(void)
  {
    return _UC_Param_list.size();
  }

  //////////////////////////////////////////////////////////////////
  // Display all pairs of name and value
  void Disp(void)
  {
    std::cout << "The number of elements : " << Size() << std::endl;
    if(0 == Size())
      return;
    for(typename std::set<UC_Param*>::iterator
      i = _UC_Param_list.begin(); i != _UC_Param_list.end(); i++)
    {
      UC_Param* upp = *i;
      upp->Disp();
    }
  }

  //////////////////////////////////////////////////////////////////
  // Break an usar command string into params
  void Break_Into(void)
  {
//    int      _in_brace = 0, _after_space = 0, _Name_len, _Value_len;
    enum BI_STATE
    {
      BI_SEARCH_LEFT_BRACE  = 0,
      BI_SEARCH_NAME_TOP    = 1,
      BI_SEARCH_SPACE       = 2,
      BI_SEARCH_VALUE_TOP   = 3,
      BI_SEARCH_RIGHT_BRACE = 4
    };
    int      BI_state = BI_SEARCH_LEFT_BRACE;
    char*    _Name_Start = NULL;
    char*    _Value_Start = NULL;
    int      _Name_len, _Value_len;
    UC_Param _tmp;
    for(int i = 0; _cmdbuf[i] != 0; i++)
    {
      switch(BI_state)
      {
        case BI_SEARCH_LEFT_BRACE :  
               if('{' == _cmdbuf[i])
                 BI_state = BI_SEARCH_NAME_TOP;
               break;
        case BI_SEARCH_NAME_TOP : 
               if(!isspace(_cmdbuf[i]))
               {
                 _Name_Start = &_cmdbuf[i];
                 BI_state = BI_SEARCH_SPACE;
               }
               break;
        case BI_SEARCH_SPACE : 
               if(isspace(_cmdbuf[i]))
               {
                 _Name_len = (int)(&_cmdbuf[i] - _Name_Start);
                 strncpy(_tmp._Name, _Name_Start
                                    , _MIN(_Name_len, sizeof(_tmp._Name)-1));
                 _tmp._Name[_MIN(_Name_len, sizeof(_tmp._Name)-1)] = 0;
                 BI_state = BI_SEARCH_VALUE_TOP;
               }
               break;
        case BI_SEARCH_VALUE_TOP : 
               if(!isspace(_cmdbuf[i]))
               {
                 _Value_Start = &_cmdbuf[i];
                 BI_state = BI_SEARCH_RIGHT_BRACE;
               }
               break;
        case BI_SEARCH_RIGHT_BRACE : 
               if('}' == _cmdbuf[i])
               {
                 _Value_len = (int)(&_cmdbuf[i] - _Value_Start);
                 strncpy(_tmp._Value, _Value_Start
                                 , _MIN(_Value_len, sizeof(_tmp._Value)-1));
                 _tmp._Value[_MIN(_Value_len, sizeof(_tmp._Value)-1)] = 0;
                 Add_Param(_tmp);
                 _Name_Start = _Value_Start = NULL;
                 BI_state = BI_SEARCH_LEFT_BRACE;
               }
               break;
      }
    }
    if(BI_SEARCH_LEFT_BRACE != BI_state)
    {
      _Err_code = BIE_BROKEN_BRACE;
      Remove_all();
      return;
    }
  }

  //////////////////////////////////////////////////////////////////
  // .Make_A_Child_Session_and_Accept_Loop
  void Add_Param(UC_Param& up)
  {
    UC_Param* _newp;
    _newp = new UC_Param;
    *_newp = up;
    _UC_Param_list.insert(_newp);
  }
  void Remove_all(void)
  {
    if(0 == Size())
      return;
    for(typename std::set<UC_Param*>::iterator
      i = _UC_Param_list.begin(); i != _UC_Param_list.end(); i++)
        delete *i;
    Init();
  }
};

#ifdef __SAMPLE_CODE_FOR_USING_Break_USAR_Command_Into_Params__
  //  You can see a whole class code as UC_INIT in USARGazebo.cc
  int UC_INIT::read_params_from_usar_command(void)
  {
    float                        x = 0, y = 0, z = 0, r = 0, p = 0, yaw = 0;
    char*                        rtn;
    Break_USAR_Command_Into_Params BUCIP(_parent.ucbuf);
    BUCIP.Disp();
    if(BIE_GOOD != BUCIP.Error_code())
      return -2;
    rtn = BUCIP.Search("Location");
    if(NULL != rtn)
      sscanf(rtn, "%f,%f,%f", &x, &y, &z);
    rtn = BUCIP.Search("Rotation");
    if(NULL != rtn)
      sscanf(rtn, "%f,%f,%f", &r, &p, &yaw);
    rtn = BUCIP.Search("ClassName");
    if(NULL != rtn)
    {
      rtn = BUCIP.Search("Name");
      if(NULL != rtn)
        record_robot_param(BUCIP.Search("Name"), BUCIP.Search("ClassName")
          ,x, y, z, r, p, yaw);
      else
        record_robot_param(BUCIP.Search("ClassName"), BUCIP.Search("ClassName")
          ,x, y, z, r, p, yaw);
      return 1;
    }
    return -1;
//  Sample command : INIT {ClassName pioneer3at_with_sensors}{Name Robo_A}{Location 1,-2,0}{Rotation 0,0,0}{Start Point1}
  }
#endif

#endif
