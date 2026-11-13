#include "trical_stream.hpp"
#include <iostream>

int main() {
  TricalStream s;TricalStream::Joints q{},lo{},hi{};lo.fill(-2);hi.fill(2);
  auto packet=[](int generation,int action,double seconds,double a,double b){
    std::vector<double>x(31);x[0]=generation;x[1]=action;x[2]=seconds;
    for(int j=0;j<14;++j){x[3+j]=a;x[17+j]=b;}return x;};
  const auto reject=[&](auto x,std::int64_t now){
    try{s.accept(x,now,q,lo,hi,.5,.005);}catch(const std::exception&){return;}
    throw std::runtime_error("Invalid stream accepted");};
  reject(packet(1,1,1,0,.1),1); // first frame must hold actual start
  s.accept(packet(1,0,0,0,0),1,q,lo,hi,.5,.005);
  reject(packet(1,0,0,0,0),2); // no replay
  reject(packet(2,1,1,.1,.2),2); // discontinuous start
  reject(packet(2,1,.1,0,1),2); // velocity
  reject(packet(2,2,0,0,.1),2); // finish cannot move
  s.accept(packet(2,1,1,0,.1),10,q,lo,hi,.5,.005);
  auto r=s.row(500000010);
  if(std::abs(r[1]-.05)>1e-14||std::abs(r[15]-.21875)>1e-14)throw std::runtime_error("Wrong septic");
  reject(packet(3,0,0,.1,.1),500000010); // cannot replace active segment
  q.fill(.05);if(s.chord_distance(q)>1e-15)throw std::runtime_error("Chord monitor");
  q[3]+=.002;if(s.chord_distance(q)<.001)throw std::runtime_error("Tube monitor");
  r=s.row(2000000010);if(std::abs(r[1]-.1)>1e-14||r[15]!=0)throw std::runtime_error("Hold endpoint");
  s.accept(packet(3,2,0,.1,.1),2000000010,q,lo,hi,.5,.005);
  std::cout<<"TriCal stream cases passed\n";
}
