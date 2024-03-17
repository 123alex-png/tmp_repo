#include <config/config.hh>
#include <config/coreutilConfig.hh>
#include <config/tarfileConfig.hh>
#include <config/whisperCppConfig.hh>
#include <connection/receive.hh>

// implementation of class config
config::config(output* out) : conn(NULL), trc(NULL), out(out) {}

void config::setConnect(connection* conn) { this->conn = conn; }

connection* config::getConnect() { return conn; }

void config::setTrace(trace* trc) { this->trc = trc; }

trace* config::getTrace() { return trc; }

output* config::getOutput() { return out; }

// implementation of class configInit

configInit::configInit(const std::string& configFile,
                       const std::string& portFile, output* out) {
    if (configFile.empty()) {
        conf = new config(out);

        trace* trc = new trace({}, true, out, trace::dbgType::GDB);
        conf->setTrace(trc);

        connection* conn = nullptr;
        try {
            int port = std::stoi(portFile);
            conn = new receive(trc, 1, port);
        } catch (const std::invalid_argument& e) {
            conn = new receive(trc, 1, portFile);
        }
        conf->setConnect(conn);

    } else {
        int id = std::stoi(configFile);
        switch (id) {
        case 1:
            conf = new tarfileConfig(out, portFile);
            break;
        case 2:
            conf = new coreutilConfig(out, portFile);
            break;
        case 3:
            conf = new whisperCppConfig(out, portFile);
            break;
        default:
            throw std::invalid_argument("Invalid config file id");
        }
    }
}

config* configInit::getConfig() { return conf; }
