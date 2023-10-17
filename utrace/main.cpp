// for Testing
#include "attacher.hh"
#include <iostream>
#include <unistd.h>

unsigned to_be_analyesd(unsigned x){
    static unsigned number=0;
    number+=x;
    return number;
}

void func(){
    int s=0;
    puts("fork succ!");
    for(unsigned i=1;;i++)
        s+=to_be_analyesd(i);
    exit(0);
}

int main(){
    attacher my;
    printf("Hello world!\n");
    pid_t pid=fork();
    if(!pid) func();
    printf("%d\n",pid);
    my.attach_watch(pid,"to_be_analyesd","number");
    while (1){
        auto x=my.event();
        if(x.first!=0) std::cout<<x.first<<" -- "<<"number"<<"="<<x.second<<std::endl;
    }
}