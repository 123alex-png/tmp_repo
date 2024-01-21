#include <watch.hh>

using std::string;
void cmdRun(config* cfg, const std::vector<string>& argv) {
    socketClose sync(-1, false);
    runMonitor(cfg->getStreams(), "", argv, sync);
    return;
}

void pidAttach(config* cfg, int pid) {
    socketClose sync(-1, false);
    runMonitor(cfg->getStreams(), std::to_string(pid), std::vector<string>(),
               sync);
    return;
}
