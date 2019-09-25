//======================================================================
// Author	: Qi Wang
// Email	: wangqi.dream@gmail.com
// Version	:
// Copyright	:
// Descriptoin	:
//======================================================================



#ifndef XMLCONF_HH
#define XMLCONF_HH

#include "tinyxml2.h"
#include <string>
#include <iostream>
#include <stdexcept>
using namespace tinyxml2;
class XmlConf
{
public:
     XmlConf();
    virtual ~XmlConf();
    bool LoadConf(const char*);
    void String(std::string&);
    void Print();
    bool Parse(const char*,const char*);
    bool Parse(const char* module_name);
    bool SaveConf(const char*);

    bool SetSystemParam(const char*,const char*);
    bool SetSystemParam(const char*,double);
    //bool AddSystemParam(const char*, const char*);

    bool GetSystemParam(const char*, float&);
    bool GetSystemParam(const char*, double&);
    bool GetSystemParam(const char*, bool&);
    bool GetSystemParam(const char*, int&);
    bool GetSystemParam(const char*, int64_t&);
    bool GetSystemParam(const char*, std::string&);
    bool GetSystemParam(const char*, unsigned int&);

    bool GetModuleParam(const char*, float&);
    bool GetModuleParam(const char*, double&);
    bool GetModuleParam(const char*, bool&);
    bool GetModuleParam(const char*, int&);
    bool GetModuleParam(const char*, unsigned int&);
    bool GetModuleParam(const char*, int64_t&);
    bool GetModuleParam(const char*, std::string&);

    const char* GetModuleName();

private:
    /* data */
    XMLDocument doc;
    XMLNode* system;
    XMLNode* module;
};


#endif /* ifndef XMLCONF_HH */
