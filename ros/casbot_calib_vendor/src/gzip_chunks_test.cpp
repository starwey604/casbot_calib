#include "gzip_chunks.hpp"
#include <vector>
#include <iostream>
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  const std::filesystem::path root=argv[1];std::filesystem::remove_all(root);std::filesystem::create_directories(root);
  const std::string json="{\"receive_ns\":1000000000000000001,\"same\":true}",cdr("\0\xff\x01\0",4);
  std::string expected;
  {
    GzipChunks archive(root,50);
    for(int i=0;i<4;++i){auto batch=GzipChunks::frame(json,cdr.data(),cdr.size());expected+=batch;archive.enqueue(batch);if(!batch.empty())return 3;}
    archive.close();archive.check();
  }
  std::string actual;
  std::vector<std::filesystem::path> paths;
  for(const auto& p:std::filesystem::directory_iterator(root/"chunks"))paths.push_back(p.path());
  std::sort(paths.begin(),paths.end());
  for(const auto& path:paths) {
    if(path.extension()!=".gz"||path.string().find(".open.")!=std::string::npos)return 4;
    gzFile f=gzopen(path.c_str(),"rb");std::string data;char buffer[4096];int n;
    while((n=gzread(f,buffer,sizeof(buffer)))>0)data.append(buffer,n);
    if(n<0||gzclose(f)!=Z_OK||data.substr(0,7)!="CBRAW2\n")return 5;
    actual+=data.substr(7);
  }
  if(expected!=actual)return 6;
  std::filesystem::create_directory(root/"full");
  {GzipChunks full(root/"full",50,1);auto batch=expected;
    bool rejected=false;auto start=std::chrono::steady_clock::now();
    try{full.enqueue(batch);}catch(const std::ios_base::failure&){rejected=true;}
    if(!rejected||batch!=expected||std::chrono::steady_clock::now()-start>std::chrono::milliseconds(100))return 7;}
  std::filesystem::create_directory(root/"failure");
  {GzipChunks failed(root/"failure");std::filesystem::create_symlink("/dev/full",root/"failure/chunks/00000000.open.gz");
    auto batch=expected;failed.enqueue(batch);failed.close();bool rejected=false;
    try{failed.check();}catch(const std::ios_base::failure&){rejected=true;}
    if(!rejected||std::filesystem::exists(root/"failure/chunks/00000000.bin.gz"))return 8;}
  std::cout<<"lossless chunks, bounded queue, I/O failure prefix passed\n";
}
