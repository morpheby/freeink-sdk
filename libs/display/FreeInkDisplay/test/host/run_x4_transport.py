#!/usr/bin/env python3
from pathlib import Path
import tempfile, subprocess, shutil
HERE=Path(__file__).resolve().parent
with tempfile.TemporaryDirectory() as d:
    root=Path(d);(root/'include').mkdir();(root/'src/bus').mkdir(parents=True)
    shutil.copy(HERE.parents[1]/'include/Uc8279X4Transport.h',root/'include')
    shutil.copy(HERE.parents[1]/'include/Uc8279X4PanelPower.h',root/'include')
    (root/'src/bus/EpdBus.h').write_text('''#pragma once
#include <cstdint>
#include <vector>
namespace freeink { class EpdBus { public:
struct Write{uint8_t cmd;std::vector<uint8_t> data;};std::vector<Write> writes;
bool busy=false;uint32_t bytes=0;
void reset(uint16_t){} void cmd(uint8_t c){writes.push_back({c,{}});}
void data(uint8_t v){writes.back().data.push_back(v);}
void data(const uint8_t*p,uint16_t n){for(uint16_t i=0;i<n;++i)data(p[i]);}
struct Transaction { EpdBus& bus; void writeBytes(const uint8_t*p,uint16_t n){bus.bytes+=n;bus.writes.back().data.insert(bus.writes.back().data.end(),p,p+n);} void end(){} };
Transaction beginTxn(){return {*this};}
bool isBusy()const{return busy;}
}; }
''')
    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(root/'include'),str(HERE/'test_x4_transport.cpp'),'-o',str(root/'test')],check=True)
    subprocess.run([str(root/'test')],check=True)
