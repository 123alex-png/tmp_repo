#ifndef _ATTACHER_HH__
#define _ATTACHER_HH__

#include <queue>
#include <string>
#include <mutex>

class attacher{
    private:
        using Var=std::string;
        using Val=std::string;
        using BreakPoint=std::string;
        //for temporarily save the data
        struct {
            std::queue<Val> q;
            std::mutex lock;
        }stream;
        //for saving the Vars to be watched
        struct {
            Var var;
            BreakPoint breakPoint;
        }controler;
    public:
        attacher()=default;
        //filename for dbg info
        void attach_watch(int pid,BreakPoint breakPoint,Var var);
        Val event();
};

#endif
