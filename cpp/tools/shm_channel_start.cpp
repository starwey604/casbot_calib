#include <casbot_calib/shm/channel.hpp>
#include <iostream>
#include <string>
int main(int argc,char** argv) {
  if(argc!=3||std::string(argv[1])!="--fresh-master") {
    std::cerr<<"Usage: shm_channel_start --fresh-master /SHM_NAME\n"
      <<"Explicitly initializes zero queues/zero semaphores after exclusive master startup.\n"
      <<"No motor command is enqueued. Does not reset queues or repair mutexes.\n";
    return 2;
  }
  try {
    casbot::shm::Channel channel(argv[2]);channel.activate_fresh_master();
    std::cout<<"Fresh-master handshake completed: back/front/start released once; no command frame sent.\n";
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
