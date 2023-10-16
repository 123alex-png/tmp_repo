#include "attacher.hh"
#include <thread>
#include <unistd.h>
#include <assert.h>
#include <ctime>
using std::string;

static void gdb_run(std::queue<attacher::Val> * out, std::mutex * lock, int pid,string breakPoint, string var){
    int to_gdb[2],from_gdb[2];
    if(pipe(to_gdb)<0||pipe(from_gdb)<0) assert(0);
    pid_t p=fork();
    if(!p){//run gdb
        close(to_gdb[1]);close(from_gdb[0]);
        dup2(to_gdb[0],STDIN_FILENO);
        dup2(from_gdb[0],STDOUT_FILENO);
        char Pid[40];
        sprintf(Pid,"%d",pid);
        char * args[]={
            "gdb","-p",Pid
        };
        execv(args[0],args);
    }else{//communicate with gdb
        close(to_gdb[0]);close(from_gdb[1]);
        static char buffer[4096];
        int len=sprintf(buffer,"break %s\ncontinue\nprint %s",breakPoint.c_str(),var.c_str());
        write(to_gdb[1],buffer,len);
        while(1){
            do{
                read(from_gdb[0],buffer,1);
            }while(buffer[0]!='$');
            while(buffer[0]!='=') read(from_gdb[0],buffer,1);
            int pos=0;
            while(buffer[pos]!='\n') read(from_gdb[0],buffer+(++pos),1);
            attacher::Val val;
            val.first=time(NULL);
            for(int i=1;i<pos;i++) val.second+=buffer[pos];
            lock->lock();
            out->push(val);
            lock->unlock();
            write(to_gdb[1],"continue\n",10);
        }
    }
}

void attacher::attach_watch(int pid,BreakPoint breakPoint,Var var){
    std::thread t(gdb_run,&stream,&lock,pid,breakPoint,var);
    return;
}

attacher::Val attacher::event(){
    lock.lock();
    Val ans={0,""};
    if(!stream.empty()){
        ans=stream.front();
        stream.pop();
    }
    lock.unlock();
    return ans;
};
