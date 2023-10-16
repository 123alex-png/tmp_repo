#include "attacher.hh"
#include <thread>

static void gdb_run(attacher * self, int pid){
    
}

void attacher::attach_watch(int pid,BreakPoint breakPoint,Var var){
    std::thread t(gdb_run,this);
    return;
}

attacher::Val attacher::event(){
    stream.lock.lock();
    Val ans="";
    if(!stream.q.empty()){
        ans=stream.q.front();
        stream.q.pop();
    }
    stream.lock.unlock();
    return ans;
};