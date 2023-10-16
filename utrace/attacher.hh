#ifndef _ATTACHER_HH__
#define _ATTACHER_HH__

#include <queue>
#include <string>
#include <mutex>
#include <utility>
#include <ctime>

//For simplicity, this watcher can only monitor ONE variable at ONE breakpoint for ONE process.
//Variable ,breakpoint and var's value are all prased as string. The analysis of the string-form val is taken as user's work.
//The watcher assumes that the user has root access and makes no mistake and the work never stop.
//More variables and more breakpoints can temporily be done by more watcher, and this work is taken as one of the future's goal. 

struct Stream;

class attacher{
    private:
        //for temporarily save the data
        Stream stream;
    public:
        using Var=std::string;
        using Val=std::pair<time_t,std::string>;
        using BreakPoint=std::string;
        attacher()=default;
        //filename for dbg info
        void attach_watch(int pid,BreakPoint breakPoint,Var var);
        Val event();
};

struct Stream{
    std::queue<attacher::Val> q;
    std::mutex lock;
};

#endif
