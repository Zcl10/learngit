//======================================================================
// Author	: Qi Wang
// Email	: wangqi.dream@gmail.com
// Version	:
// Copyright	:
// Descriptoin	:
//======================================================================
#ifndef IV_DATA_PLAYBACK_HH
#define IV_DATA_PLAYBACK_HH


#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include "util/xmlconf/xmlconf.h"
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
namespace fs = boost::filesystem;

class IvDataPlayback
{
    public:
	IvDataPlayback ();
	virtual ~IvDataPlayback ();

	bool SetRecordFile(std::string&);
	bool SetRecordFile();
	bool SetRecordDir(std::string&);

	void BeginSaveLine();
	void EndSaveLine();

	void BeginSaveTitle();
	void EndSaveTitle();

	bool SetPlaybackFile(std::string&);
	bool SetPlaybackFile();
	void SetFileLineNum(long num);
	long GetFileLineNum();
	bool BeginLoadLine(int mode=0);
	bool EndLoadLine();

	double SysTime();
	double SysTime(double);
	double NowTime(double);
	void TimeSetup( double,double);

	const char* MakeRecordFileName(int, const char*);


	//bool
	template <typename T>
	    IvDataPlayback& operator<<(T);

	template <typename T>
	    IvDataPlayback& operator>>(T&);

	void SetPrecision(int);

	void Setup(XmlConf&);
	bool RecordIsOn();
	bool PlaybackIsOn();
	const char* GetRecordPath();
	const char* GetPlaybackPath();

	bool SaveSinglefile(std::string ,void* , long );
	bool ReadSinglefile(std::string ,void* , long );

	bool SaveSinglefile(int,void* , long );
	bool ReadSinglefile(int,void* , long );
    private:
	/* data */
	double start_time;
	double playback_start_at;
	//single file save;
	std::ifstream infile;
	std::ofstream outfile;

	//data save
	std::string save_file_name;
	std::string record_path;
	std::stringstream save_stream;
	std::fstream save_file;
	std::string line;
	std::string tmp;

	//data playback
	std::string play_file_name;
	std::string playback_path;
	std::stringstream play_stream;
	std::fstream play_file;
	std::string next_line;
	std::vector<std::string> lines;//jkj 增加逐帧调试
	long line_num = 0;   //jkj 增加逐帧调试
	double timestamp;

	char buf[2000];


	bool playback_on;
	bool record_on;
	bool file_invalid;
	double initial_timeout;


	std::stringstream recordfile_index_stream;
	std::string recordfile_name;
};

template<typename T>
IvDataPlayback& IvDataPlayback::operator>>(T& data)
{
    play_stream>>data;
    return *this;
}

template<typename T>
IvDataPlayback& IvDataPlayback::operator<<(T data) {

    save_stream.clear();
    save_stream<<data;
    tmp.clear();
    save_stream>>tmp;
    line+=tmp;
    line+='\t';
    return *this;
}
#endif /* ifndef IV_DATA_PLAYBACK_HH */
