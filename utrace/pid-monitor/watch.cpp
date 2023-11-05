#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <iostream>
#include <cstring>
#include <common.hpp>
#include <unordered_set>
#include <thread>

bool check(const string &filename){
    static char buf[1024];

    if (filename.find_first_not_of("0123456789")!=string::npos) 
        return false;
    FILE *fp=fopen(("/proc/"+filename+"/comm").c_str(), "r");
    if (fp==NULL) {
        perror("fopen");
        exit(1);
    }
    
    fgets(buf, 1024, fp);
    fclose(fp);
    string comm=buf;
    comm.erase(comm.find_last_not_of(" \n\r\t")+1);
    if (comm!=name) return false;

    fp=fopen(("/proc/"+filename+"/cmdline").c_str(), "r");
    if (fp==NULL) {
        perror("fopen");
        exit(1);
    }
    fgets(buf, 1024, fp);
    fclose(fp);
    string cmdline=buf;
    for (const auto &arg:arguments) 
        if (cmdline.find(arg)==string::npos) return false;
    
    static std::unordered_set<string> pids;
    if(pids.find(filename)==pids.end()){
        pids.insert(filename);
        return true;
    }else return false;
}

void watch(const string name) {
    while (true) {
        sleep(1);
        DIR *dir=opendir("/proc");
        if (dir==NULL) {
            perror("opendir");
            exit(1);
        }
        struct dirent *entry;
        while ((entry=readdir(dir))!=NULL) {
            if (entry->d_type==DT_DIR&&check(entry->d_name)) {
                std::cout << "pid: " << entry->d_name << std::endl;
                extern void run_monitor(const string &pid);
                std::thread t(run_monitor,entry->d_name);
                t.detach();
            }
        }
    }
}