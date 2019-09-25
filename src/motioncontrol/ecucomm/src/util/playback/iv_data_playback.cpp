#include "iv_data_playback.h"
#include <glog/logging.h>
IvDataPlayback::IvDataPlayback() {
    playback_on=false;
    record_on=false;
    file_invalid=true;
    initial_timeout=0.1;
}

IvDataPlayback::~IvDataPlayback() {
    //save_file<<line;
    //save_file.flush();
    //line.clear();

    if(play_file.is_open())
    {
	play_file.close();
    }

    if(save_file.is_open())
    {
	std::cout<<"save file is closed"<<std::endl;
	save_file.close();
    }
}


void IvDataPlayback::TimeSetup( double st, double at) {
    playback_start_at=st-at;
    start_time=st;
}


bool IvDataPlayback::SetRecordDir(std::string& p) {
    fs::path dir(p);
    if(fs::create_directory(dir))
	return true;

    if(fs::exists(dir)==true)
	return true;
    else
	return false;
}

bool IvDataPlayback::SaveSinglefile(int index,void* buf, long size)
{
    static std::stringstream stream;
    static std::string name;
    stream.clear();
    name.clear();
    stream<<index;
    stream>>name;
    SaveSinglefile(name,buf,size);
    return true;
}

bool IvDataPlayback::ReadSinglefile(int index,void* buf, long size)
{
    static std::stringstream stream;
    static std::string name;
    stream.clear();
    name.clear();
    stream<<index;
    stream>>name;
    ReadSinglefile(name,buf,size);
    return true;
}

bool IvDataPlayback::SaveSinglefile(std::string name,void* buf, long size)
{
    name=record_path+name;
    if(outfile.is_open()==true)
	outfile.close();
    outfile.open(name.c_str(),std::ofstream::binary);
    outfile.write((char*)buf,size);
    outfile.close();
    return true;
}
bool IvDataPlayback::ReadSinglefile(std::string name,void* buf, long size)
{
    name=playback_path+name;
    if(infile.is_open()==true)
	infile.close();
    infile.open(name.c_str(),std::ifstream::binary);
    infile.read((char*)buf,size);
    infile.close();
    return true;
}
const char* IvDataPlayback::GetPlaybackPath()
{
    return playback_path.c_str();
}
const char* IvDataPlayback::GetRecordPath()
{
    return record_path.c_str();
}

bool IvDataPlayback::RecordIsOn()
{
    return record_on;
}

bool IvDataPlayback::PlaybackIsOn()
{
    return playback_on;
}

void IvDataPlayback::Setup(XmlConf& config)
{


    playback_on=false;
    record_on=false;

    bool sys_playback=false;
    bool module_playback=false;
    double at=0;
    playback_path.clear();


    if(!config.GetSystemParam("system_start_time",start_time))
    {
	std::cerr<<"system start time is incorrect"<<start_time<<std::endl;
    }

    //start_time = etime();    //jkj 2017/4/20

    if(!config.GetSystemParam("playback_on",sys_playback))
    {
	std::cerr<<"system playback_on is incorrect"<<std::endl;
    }
    if(!config.GetModuleParam("playback_on",module_playback))
    {
	std::cerr<<"module playback_on is incorrect"<<std::endl;
    }

    playback_on = sys_playback && module_playback;
    if(playback_on)
    {
	if(!config.GetSystemParam("playback_path",playback_path))
	{
	    std::cerr<<"playback path is incorrect"<<playback_path<<std::endl;
	}
	playback_path+="/";
	playback_path+=config.GetModuleName();
	playback_path+="/";
	SetPlaybackFile();

	if(!config.GetSystemParam("playback_start_at",at))
	{
	    std::cerr<<"playback start time is incorrect"<<playback_start_at<<std::endl;
	}
    }
    playback_start_at=start_time-at;

    bool sys_record=false;
    bool module_record=false;
    record_path.clear();

    if(!config.GetSystemParam("record_on",sys_record))
    {
	std::cerr<<"system record_on is incorrect"<<std::endl;
    }
    if(!config.GetModuleParam("record_on",module_record))
    {
	std::cerr<<"module record_on is incorrect"<<std::endl;
    }

    record_on = sys_record && module_record;
    if (record_on) {
	if(!config.GetSystemParam("record_path",record_path))
	{
	    std::cerr<<"record path is incorrect"<<std::endl;
	}
	if(SetRecordDir(record_path)==false)
	{
	    std::cerr<<"fails to setup the record path"<<std::endl;
	}else
	{
	    record_path+="/";
	    record_path+=config.GetModuleName();
	    record_path+="/";
	    SetRecordFile();
	}
    }

    if(!config.GetModuleParam("timeout",initial_timeout))
    {
	std::cerr<<"timeout is incorrect"<<std::endl;
	//initial_timeout=timer->timeout;
    }else
    {
	//timer->timeout=initial_timeout;
    }

}

double IvDataPlayback::SysTime()
{
    return ros::Time::now().toSec()-playback_start_at;
}

double IvDataPlayback::SysTime(double stamp)
{
    return stamp-playback_start_at;
}

double IvDataPlayback::NowTime(double stamp)
{
    return stamp+playback_start_at;
}


bool IvDataPlayback::SetPlaybackFile() {
    play_file_name=playback_path;
    play_file_name+="playback_data";

    if(play_file.is_open())
	play_file.close();

    play_file.open(play_file_name.c_str(),std::ios::in | std::ios::app);
    if ( (play_file.rdstate() & std::ifstream::failbit ) != 0 )
    {
	std::cerr << "Error opening playback file "<<play_file_name<<std::endl;
	return false;
    }
    while(play_file.eof()==false&&play_file.is_open())
    {
		play_file.getline(buf,2000);
		if(buf[0]=='%'||buf[0]==0)
		{
			continue;
		}

		next_line=buf;
		lines.push_back(next_line);
    }
    line_num = 0;
    return true;
}

bool IvDataPlayback::SetPlaybackFile(std::string& p) {
    play_file_name=p;
    if(play_file.is_open())
	play_file.close();

    play_file.open(play_file_name.c_str(),std::ios::in | std::ios::app);
    if ( (play_file.rdstate() & std::ifstream::failbit ) != 0 )
    {
	std::cerr << "Error opening playback file "<<play_file_name<<std::endl;
	return false;
    }
    return true;
}

bool IvDataPlayback::SetRecordFile() {
    save_file_name=record_path+"playback_data";

    fs::path dir(record_path);
    if(save_file.is_open())
	save_file.close();

    if(!fs::exists(dir))
    {
	fs::create_directory(dir);
	save_file.open(save_file_name.c_str(),std::ios::out | std::ios::app);
    }else
    {
	save_file.open(save_file_name.c_str(),std::ios::out | std::ios::app);
    }
    return true;
}

bool IvDataPlayback::SetRecordFile(std::string& p) {
    save_file_name=p;
    size_t pos=save_file_name.find_last_of('/');

    fs::path dir(save_file_name.substr(0,pos));
    if(save_file.is_open())
	save_file.close();

    if(!fs::exists(dir))
    {
	fs::create_directory(dir);
	save_file.open(save_file_name.c_str(),std::ios::out | std::ios::app);
    }else
    {
	save_file.open(save_file_name.c_str(),std::ios::out | std::ios::app);
    }
    return true;
}

void IvDataPlayback::BeginSaveLine() {
    //in order to calculate the computation load
    //data needs to be saved before it is writen in the line.

    //if( line.size() > 204800 )//2K
    //{
	save_file<<line;
	save_file.flush();
	line.clear();
    //}

    SetPrecision(5);
}

void IvDataPlayback::EndSaveLine() {
    save_stream.clear();
    save_stream<<(ros::Time::now().toSec()-start_time);
    tmp.clear();
    save_stream>>tmp;
    line=tmp+'\t'+line+'\n';
}

void IvDataPlayback::BeginSaveTitle() {
	std::ostringstream tmp11;//qjy,0508
    line.clear();
    save_stream.clear();
    line+="%playback_timestamp";
    line+=tmp;
    tmp11<<(ros::Time::now().toSec()-1494211680.0);//qjy,0508,以2017年5月8日为起点。
    line+=tmp11.str();
    line+='\t';
}

void IvDataPlayback::EndSaveTitle() {
    line+='\n';
    //save_file<<line;
    //line.clear();
}


//IvDataPlayback& IvDataPlayback::operator<<(float data) {
    //save_stream.clear();
    //save_stream<<data;
    //tmp.clear();
    //save_stream>>tmp;
    //line+=tmp;
    //line+='\t';
    //return *this;
//}

//IvDataPlayback& IvDataPlayback::operator<<(double data) {

    //save_stream.clear();
    //save_stream<<data;
    //tmp.clear();
    //save_stream>>tmp;
    //line+=tmp;
    //line+='\t';
    //return *this;
//}

//IvDataPlayback& IvDataPlayback::operator<<(int data) {

    //save_stream.clear();
    //save_stream<<data;
    //tmp.clear();
    //save_stream>>tmp;
    //line+=tmp;
    //line+='\t';
    //return *this;
//}


//IvDataPlayback& IvDataPlayback::operator<<(long data) {

    //save_stream.clear();
    //save_stream<<data;
    //tmp.clear();
    //save_stream>>tmp;
    //line+=tmp;
    //line+='\t';
    //return *this;
//}

//IvDataPlayback& IvDataPlayback::operator<<(bool data) {

    //save_stream.clear();
    //save_stream<<data;
    //tmp.clear();
    //save_stream>>tmp;
    //line+=tmp;
    //line+='\t';
    //return *this;
//}

//IvDataPlayback& IvDataPlayback::operator<<(std::string data) {

    //line+=data;
    //line+='\t';
    //return *this;
//}

//IvDataPlayback& IvDataPlayback::operator<<(char* data) {

    //line+=data;
    //line+='\t';
    //return *this;
//}

//IvDataPlayback& IvDataPlayback::operator<<(char data)
//{
    //line+=data;
    //line+='\t';
    //return *this;
//}

//IvDataPlayback& IvDataPlayback::operator<<(unsigned char data)
//{
    //line+=data;
    //line+='\t';
    //return *this;
//}

void IvDataPlayback::SetFileLineNum(long num)
{
	line_num = num;
}
long IvDataPlayback::GetFileLineNum()
{
	return line_num;
}

//playback----------------------------------------------
bool IvDataPlayback::BeginLoadLine(int mode) {
    //double current_time = etime()-playback_start_at;;
    //std::cout<<timestamp<<" current time: "<<playback_start_at<<" "<<current_time<<std::endl;
    double current_time;
    while(line_num<lines.size())
    {


	next_line=lines[line_num];
	line_num++;
	play_stream.str("");
	play_stream.clear();
	play_stream<<next_line;
	play_stream>>timestamp;

//	LOG(INFO)<<"timestamp "<<timestamp;
	current_time = ros::Time::now().toSec()-playback_start_at;
	/*std::cout<<"current_time="<<current_time<<std::endl;
	std::cout<<"timestamp="<<timestamp<<std::endl;
	std::cout<<"playback_start_at="<<playback_start_at<<std::endl;*/
	if( current_time < timestamp|| mode)
	{
	    file_invalid=false;
	   return true;
	}
    }
    file_invalid=true;

    return false;
}

bool IvDataPlayback::EndLoadLine() {
    if(file_invalid==false)
    {
	double current_time = ros::Time::now().toSec()-playback_start_at;
	double delay = timestamp - current_time;
//	LOG(INFO)<<"timestamp "<<timestamp;
// 	LOG(INFO)<<"delay="<<delay;
	//std::cout<<"timeout:"<<timer->timeout<<std::endl;


	if(delay>0)
	  usleep(delay*1000000);
	current_time = ros::Time::now().toSec()-playback_start_at;
	//std::cout<<"current_time="<<current_time<<std::endl;
	//std::cout<<"timestamp="<<timestamp<<std::endl;
	//current_time = etime()-playback_start_at;
    }else
    {
//	timer->timeout=initial_timeout;
    }
    //std::cout<<timestamp<<" current time: "<<current_time<<std::endl;
    //double current_time;
    //while(play_file.eof()==false&&play_file.is_open())
    //{
	//play_file.getline(buf,1000);
	//if(buf[0]=='%'||buf[0]==0)
	//{
	    //continue;
	//}

	//next_line=buf;
	//play_stream.str("");
	//play_stream.clear();
	//play_stream<<next_line;
	//play_stream>>timestamp;

	//current_time = etime()-playback_start_at;
	//if( current_time > timestamp )
	//{
	    //continue;
	//}else
	//{
	    //std::cout<<timestamp<<std::endl;
	    //timer->timeout = timestamp - current_time;
	    //timer->sync();
	    //break;
	//}
    //}
    return true;
}
//IvDataPlayback& IvDataPlayback::operator>>(float& data)
//{
    //play_stream>>data;
    //return *this;
//}

//IvDataPlayback& IvDataPlayback::operator>>(double& data)
//{
    //play_stream>>data;
    //return *this;
//}

//IvDataPlayback& IvDataPlayback::operator>>(int& data)
//{
    //play_stream>>data;
    //return *this;
//}

//IvDataPlayback& IvDataPlayback::operator>>(long& data)
//{
    //play_stream>>data;
    //return *this;
//}

//IvDataPlayback& IvDataPlayback::operator>>(std::string& data)
//{
    //play_stream>>data;
    //return *this;
//}

//IvDataPlayback& IvDataPlayback::operator>>(bool& data)
//{
    //play_stream>>data;
    //return *this;
//}

//IvDataPlayback& IvDataPlayback::operator>>(char& data)
//{
    //play_stream>>data;
    //return *this;
//}

//IvDataPlayback& IvDataPlayback::operator>>(unsigned char& data)
//{
    //play_stream>>data;
    //return *this;
//}

void IvDataPlayback::SetPrecision(int p) {
    save_stream<<std::setiosflags(std::ios::fixed)<<std::setprecision(p);
}


const char* IvDataPlayback::MakeRecordFileName(int index, const char* suffix)
{
    recordfile_index_stream.str("");
    recordfile_index_stream.clear();
    recordfile_name.clear();

    recordfile_index_stream<<index;
    recordfile_index_stream>>recordfile_name;
    recordfile_name+=suffix;

    return recordfile_name.c_str();
}



















