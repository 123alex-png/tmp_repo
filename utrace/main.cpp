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
    static int nr1=0,nrr2=0;
    nr1-=x;nrr2+=2u;
    return nrr2;
}

void func(){
    int s=0;
    puts("fork succ!");
    getchar();
    for(unsigned i=1;i<=200;i++){
        s+=to_be_analysed(2);
        s+=to_be_analyssed(2);
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
            {"to_be_analyssed",{"nr1","nrr2"}} 
        };
    my.attach_watch(pid,config);
    return 0;
}