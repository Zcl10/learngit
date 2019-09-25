#include "xmlconf.h"
#include <sstream>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

XmlConf::XmlConf() {
    system=NULL;
    module=NULL;
}

XmlConf::~XmlConf() {

    doc.Clear();
}

void XmlConf::Print() {
    XMLPrinter printer;
    doc.Accept(&printer);
    std::cout<<printer.CStr()<<std::endl;
}


const char* XmlConf::GetModuleName()
{
    if(module==NULL)
	return NULL;
    return module->Value();
}
bool XmlConf::SetSystemParam(const char* entry, const char* value)
{
    //XMLText node_new(value);

    system->FirstChildElement(entry)->SetText(value);
    //XMLNode* node_old=system->FirstChildElement(entry)->FirstChild();
    //system->FirstChildElement(entry)->ReplaceChild(node_old,node_new);


    return true;
}

bool XmlConf::SetSystemParam(const char* entry, double value)
{
    char time[20];
    sprintf(time,"%.6f",value);
    return SetSystemParam(entry,time);
}

//bool XmlConf::AddSystemParam(const char* entry, const char* value)
//{
    //XMLElement node_new(entry);
    //if(system->InsertEndChild(node_new)==NULL)
    //{
	//SetSystemParam(entry,"dfas");
	//return false;
    //}
    //else
    //{
	//SetSystemParam(entry,value);
	//return true;
    //}
//}
bool XmlConf::SaveConf(const char* path) {
   return doc.SaveFile(path);
}
bool XmlConf::LoadConf(const char* xml) {
    doc.Clear();
    system=NULL;
    module=NULL;

    bool ret=true;
    ret = doc.LoadFile(xml);
    if (ret ) {
      return false;
    }

    XMLNode* root= doc.FirstChildElement("root");
    if(root==NULL)
	return false;

    system= root->FirstChildElement("system");
    if(system==NULL)
	return false;

   return true;
}

void XmlConf::String(std::string& xml) {
    XMLPrinter printer;
    doc.Accept(&printer);
    xml=printer.CStr();
}

bool XmlConf::Parse(const char* xml,const char* module_name) {
    doc.Clear();
    system=NULL;
    module=NULL;
    doc.Parse(xml);//, NULL, XML_DEFAULT_ENCODING);

    XMLNode* root= doc.FirstChildElement("root");
    if(root==NULL)
	return false;

    system= root->FirstChildElement("system");
    if(system==NULL)
	return false;

    module= root->FirstChildElement(module_name);
    if(module==NULL)
	return false;

    return true;
}

bool XmlConf::Parse(const char* module_name) {
    //doc.Clear();
    system=NULL;
    module=NULL;
    //doc.Parse(xml, NULL, XML_DEFAULT_ENCODING);

    XMLNode* root= doc.FirstChildElement("root");
    if(root==NULL)
	return false;

    system= root->FirstChildElement("system");
    if(system==NULL)
	return false;

    module= root->FirstChildElement(module_name);
    if(module==NULL)
	return false;

    return true;
}

bool XmlConf::GetSystemParam(const char* param_name, float& param) {
    const char* param_str;
    if( system->FirstChildElement(param_name) !=NULL &&
	    system->FirstChildElement(param_name)->GetText()!=NULL)
	param_str = system->FirstChildElement(param_name)->GetText();
    else
	return false;

    try {
	    XMLUtil::ToFloat(param_str,&param);
    }
    catch (const std::invalid_argument& ia) {
	std::cerr << "Invalid config : " <<param_name << '\n';
	return false;
    }

    return true;
}

bool XmlConf::GetSystemParam(const char* param_name, double& param) {
    const char* param_str;
    if( system->FirstChildElement(param_name) !=NULL &&
	    system->FirstChildElement(param_name)->GetText()!=NULL)
	param_str = system->FirstChildElement(param_name)->GetText();
    else
	return false;

    try {
        XMLUtil::ToDouble(param_str,&param);
    }
    catch (const std::invalid_argument& ia) {
	std::cerr << "Invalid config : " <<param_name << '\n';
	return false;
    }

    return true;
}

bool XmlConf::GetSystemParam(const char* param_name, bool& param) {
    const char* param_str;
    if( system->FirstChildElement(param_name) !=NULL &&
	    system->FirstChildElement(param_name)->GetText()!=NULL)
	param_str = system->FirstChildElement(param_name)->GetText();
    else
	return false;

    try {
        XMLUtil::ToBool(param_str,&param);
    }
    catch (const std::invalid_argument& ia) {
	std::cerr << "Invalid config : " <<param_name << '\n';
	return false;
    }

	return true;
}

bool XmlConf::GetSystemParam(const char* param_name, int& param) {
    const char*  param_str;
    if( system->FirstChildElement(param_name) !=NULL &&
	    system->FirstChildElement(param_name)->GetText()!=NULL)
	param_str = system->FirstChildElement(param_name)->GetText();
    else
	return false;

    try {
	    XMLUtil::ToInt(param_str,&param);
    }
    catch (const std::invalid_argument& ia) {
	std::cerr << "Invalid config : " <<param_name << '\n';
	return false;
    }

    return true;
}

bool XmlConf::GetSystemParam(const char* param_name, unsigned int& param) {
    const char* param_str;
    if( system->FirstChildElement(param_name) !=NULL &&
	    system->FirstChildElement(param_name)->GetText()!=NULL)
	param_str = system->FirstChildElement(param_name)->GetText();
    else
	return false;

    try {
        XMLUtil::ToUnsigned(param_str,&param);
    }
    catch (const std::invalid_argument& ia) {
	std::cerr << "Invalid config : " <<param_name << '\n';
	return false;
    }

    return true;
}

bool XmlConf::GetSystemParam(const char* param_name, int64_t& param) {
    const char* param_str;
    if( system->FirstChildElement(param_name) !=NULL &&
	    system->FirstChildElement(param_name)->GetText()!=NULL)
	param_str = system->FirstChildElement(param_name)->GetText();
    else
	return false;

    try {
        XMLUtil::ToInt64(param_str,&param);
    }
    catch (const std::invalid_argument& ia) {
	std::cerr << "invalid config : " <<param_name << '\n';
	return false;
    }

    return true;
}

bool XmlConf::GetSystemParam(const char* param_name, std::string& param) {
    if( system->FirstChildElement(param_name) !=NULL &&
	    system->FirstChildElement(param_name)->GetText()!=NULL)
	param = system->FirstChildElement(param_name)->GetText();
    else
	return false;

    return true;
}
//-------------------------------


bool XmlConf::GetModuleParam(const char* param_name, float& param) {

    const char *param_str;
    if( module->FirstChildElement(param_name) !=NULL &&
	    module->FirstChildElement(param_name)->GetText()!=NULL)
	param_str = module->FirstChildElement(param_name)->GetText();
    else
	return false;

    try {
        XMLUtil::ToFloat(param_str,&param);
    }
    catch (const std::invalid_argument& ia) {
	std::cerr << "Invalid config : " <<param_name << '\n';
	return false;
    }

    return true;
}


bool XmlConf::GetModuleParam(const char* param_name, double& param) {
    const char* param_str;
    if( module->FirstChildElement(param_name) !=NULL &&
	    module->FirstChildElement(param_name)->GetText()!=NULL)
	param_str = module->FirstChildElement(param_name)->GetText();
    else
	return false;

    try {
	    XMLUtil::ToDouble(param_str,&param);
    }
    catch (const std::invalid_argument& ia) {
	std::cerr << "Invalid config : " <<param_name << '\n';
	return false;
    }

    return true;
}

bool XmlConf::GetModuleParam(const char* param_name, bool& param) {
    const char* param_str;
    if( module->FirstChildElement(param_name) !=NULL &&
	    module->FirstChildElement(param_name)->GetText()!=NULL)
	param_str = module->FirstChildElement(param_name)->GetText();
    else
	return false;
    //std::cout<<"fdsaf"<<std::endl;

    try {
	    XMLUtil::ToBool(param_str,&param);
    }
    catch (const std::invalid_argument& ia) {
	std::cerr << "Invalid config : " <<param_name << '\n';
	return false;
    }

	return true;
}

bool XmlConf::GetModuleParam(const char* param_name, unsigned int& param) {
    const char* param_str;
    if( module->FirstChildElement(param_name) !=NULL &&
	    module->FirstChildElement(param_name)->GetText()!=NULL)
	param_str = module->FirstChildElement(param_name)->GetText();
    else
	return false;

    try {
	    XMLUtil::ToUnsigned(param_str,&param);
    }
    catch (const std::invalid_argument& ia) {
	std::cerr << "Invalid config : " <<param_name << '\n';
	return false;
    }

    return true;
}
bool XmlConf::GetModuleParam(const char* param_name, int& param) {
    const char* param_str;
    if( module->FirstChildElement(param_name) !=NULL &&
	    module->FirstChildElement(param_name)->GetText()!=NULL)
	param_str = module->FirstChildElement(param_name)->GetText();
    else
	return false;

    try {
	    XMLUtil::ToInt(param_str,&param);
    }
    catch (const std::invalid_argument& ia) {
	std::cerr << "Invalid config : " <<param_name << '\n';
	return false;
    }

    return true;
}

bool XmlConf::GetModuleParam(const char* param_name, int64_t& param) {
    const char* param_str;
    if( module->FirstChildElement(param_name) !=NULL &&
	    module->FirstChildElement(param_name)->GetText()!=NULL)
	param_str = module->FirstChildElement(param_name)->GetText();
    else
	return false;

    try {
	    XMLUtil::ToInt64(param_str,&param);
    }
    catch (const std::invalid_argument& ia) {
	std::cerr << "Invalid config : " <<param_name << '\n';
	return false;
    }

    return true;
}

bool XmlConf::GetModuleParam(const char* param_name, std::string& param) {
    if( module->FirstChildElement(param_name) !=NULL &&
	    module->FirstChildElement(param_name)->GetText()!=NULL)
	param = module->FirstChildElement(param_name)->GetText();
    else
	return false;

    return true;
}
