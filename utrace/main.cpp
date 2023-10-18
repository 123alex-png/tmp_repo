// for Testing
#include "attacher.hh"
#include <iostream>
#include <unistd.h>

unsigned to_be_analysed(unsigned x){
    static unsigned number1=0,number2=0;
    number1+=x;number2++;
    return number1;
}

unsigned to_be_analyssed(unsigned x){
    static unsigned number1=0,number2=0;
    number1-=x;number2+=2u;
    return number2;
}

void func(){
    int s=0;
    puts("fork succ!");
    for(unsigned i=1;;i++){
        s+=to_be_analysed(i);
        s+=to_be_analyssed(i);
    }  
    exit(0);
}

int main(){
    attacher my;
    printf("Hello world!\n");
    pid_t pid=fork();
    if(!pid) func();
    printf("%d\n",pid);
    attacher::Config config={
            {"to_be_analysed",{"number1","number2"}},
            {"to_be_analyssed",{"number1","number2"}} 
        };
    my.attach_watch(pid,config);
    int i=0;
    while (1){
        i++;
        auto x=my.event();
        if(x.first==0) continue;
        std::cout<<x.first<<"-"<<i<<"-"<<x.second.first<<":";
        for(auto s:x.second.second) std::cout<<s<<" ";
        std::cout<<std::endl;
    }
}